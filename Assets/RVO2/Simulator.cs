// -----------------------------------------------------------------------
// <copyright file="Simulator.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Simulator.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

namespace RVO
{
    using System;
    using System.Collections.Generic;
    using Unity.Burst;
    using Unity.Collections;
    using Unity.Collections.LowLevel.Unsafe;
    using Unity.Jobs;
    using Unity.Jobs.LowLevel.Unsafe;
    using Unity.Mathematics;
    using UnityEngine;

    /// <summary>
    /// Defines the simulation.
    /// </summary>
    public sealed class Simulator : IDisposable
    {
        private NativeList<AgentData> agentDatas;
        private NativeList<Obstacle> obstacles;
        private KdTree kdTree;
        private float timeStep;

        private List<Agent> agents = new List<Agent>();
        private NativeParallelHashMap<int, int> agentIndexLookup;
        private NativeParallelMultiHashMap<int, int> obstacleIndexLookup;
        private int sid;

        private Agent defaultAgent;

        private int numWorkers;

        private JobHandle jobHandle;

        private float globalTime;
        private bool disposedValue;

        private bool agentTreeDirty;
        private bool obstacleTreeDirty;

        private bool scheduled;

        /// <summary>
        /// Initializes a new instance of the <see cref="Simulator"/> class.
        /// </summary>
        public Simulator()
        {
            this.agentDatas = new NativeList<AgentData>(8, Allocator.Persistent);
            this.obstacles = new NativeList<Obstacle>(8, Allocator.Persistent);
            this.kdTree = new KdTree(0, 0);
            this.agentIndexLookup = new NativeParallelHashMap<int, int>(8, Allocator.Persistent);
            this.obstacleIndexLookup = new NativeParallelMultiHashMap<int, int>(8, Allocator.Persistent);

            this.Clear();
        }

        /// <summary>
        /// Finalizes an instance of the <see cref="Simulator"/> class.
        /// </summary>
        ~Simulator()
        {
            this.Dispose(false);
        }

        /// <summary>
        /// Adds a new agent with default properties to the simulation.
        /// </summary>
        /// <param name="position">The two-dimensional starting position
        /// of this agent.</param>
        /// <returns>The number of the agent, or -1 when the agent defaults
        /// have not been set.</returns>
        public Agent AddAgent(float2 position)
        {
            return this.AddAgent(
                position,
                defaultAgent.neighborDist,
                defaultAgent.maxNeighbors,
                defaultAgent.timeHorizon,
                defaultAgent.timeHorizonObst,
                defaultAgent.radius,
                defaultAgent.maxSpeed,
                defaultAgent.velocity);
        }

        /// <summary>
        /// Adds a new obstacle to the simulation.
        /// </summary>
        /// <param name="vertices">List of the vertices of the polygonal obstacle
        /// in counterclockwise order.</param>
        /// <returns>The number of the first vertex of the obstacle, or -1 when
        /// the number of vertices is less than two.</returns>
        /// <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
        /// the environment, the vertices should be listed in clockwise order.
        /// </remarks>
        public int AddObstacle(IList<float2> vertices)
        {
            if (vertices.Count < 2)
            {
                return -1;
            }

            var obstacleId = ++this.sid;
            var startIndex = this.obstacles.Length;

            unsafe
            {
                for (var i = 0; i < vertices.Count; ++i)
                {
                    // NewObstacleVert will cause this.obstacles change
                    // so the old obstaclesPtr value became invalid and we have to assign again.
                    Obstacle* obstacle = this.NewObstacleVert(vertices[i], obstacleId);
                    var obstaclesPtr = (Obstacle*)this.obstacles.GetUnsafePtr();
                    var obstacleIndex = obstacle->id;

                    var isFirst = i == 0;
                    var isLast = i == vertices.Count - 1;

                    if (!isFirst)
                    {
                        obstacle->previousIndex = obstacleIndex - 1;
                        Obstacle* previous = obstaclesPtr + obstacle->previousIndex;
                        previous->nextIndex = obstacleIndex;
                    }

                    if (isLast)
                    {
                        obstacle->nextIndex = startIndex;
                        Obstacle* next = obstaclesPtr + startIndex;
                        next->previousIndex = startIndex;
                    }

                    obstacle->direction = math.normalize(vertices[isLast ? 0 : i + 1] - vertices[i]);

                    if (vertices.Count == 2)
                    {
                        obstacle->convex = true;
                    }
                    else
                    {
                        obstacle->convex = RVOMath.LeftOf(
                            vertices[isFirst ? vertices.Count - 1 : i - 1],
                            vertices[i],
                            vertices[isLast ? 0 : i + 1]) >= 0f;
                    }
                }
            }

            this.obstacleTreeDirty = true;

            return obstacleId;
        }

        public bool RemoveAgent(Agent agent)
        {
            if (agent == null)
                return false;

            return RemoveAgent(agent.id);
        }

        /// <summary>
        /// Remove a agent.
        /// </summary>
        /// <param name="agentId">The agent to remove.</param>
        /// <returns>If the agent was found and removed or not.</returns>
        public bool RemoveAgent(int agentId)
        {
            if (!this.agentIndexLookup.TryGetValue(agentId, out var index))
            {
                return false;
            }

            var lastIndex = this.agentDatas.Length - 1;
            var lastId = this.agentDatas[lastIndex].id;
            this.agentDatas.RemoveAtSwapBack(index);

            this.agents[index] = this.agents[lastIndex];
            this.agents.RemoveAt(lastIndex);

            this.agentIndexLookup.Remove(agentId);
            this.agentIndexLookup[lastId] = index;

            this.agentTreeDirty = true;

            return true;
        }

        /// <summary>
        /// Remove agentIds.
        /// </summary>
        /// <param name="agentIds">The agentIds to remove.</param>
        /// <returns>The number of key-value pairs that were found and removed.</returns>
        public int RemoveAgents(IEnumerable<int> agentIds)
        {
            var removed = 0;
            foreach (var agentId in agentIds)
            {
                if (!this.agentIndexLookup.TryGetValue(agentId, out var index))
                {
                    continue;
                }

                removed++;

                var lastIndex = this.agentDatas.Length - 1;
                var lastId = this.agentDatas[lastIndex].id;
                this.agentDatas.RemoveAtSwapBack(index);

                this.agents[index] = this.agents[lastIndex];
                this.agents.RemoveAt(lastIndex);

                this.agentIndexLookup.Remove(agentId);
                this.agentIndexLookup[lastId] = index;
            }

            this.agentTreeDirty = true;

            return removed;
        }

        /// <summary>
        /// Remove an obstacle.
        /// </summary>
        /// <param name="obstacleId">The obstacle to remove.</param>
        /// <returns>If the obstacle was found and removed or not.</returns>
        public bool RemoveObstacle(int obstacleId)
        {
            var buffer = new NativeList<int>(Allocator.Temp);
            this.obstacleIndexLookup.GetValuesForKey(obstacleId, ref buffer);
            buffer.Sort();

            if (buffer.IsCreated && buffer.Length > 0)
            {
                for (var i = buffer.Length - 1; i >= 0; --i)
                {
                    var index = buffer[i];
                    var lastIndex = this.obstacles.Length - 1;
                    var lastObstacle = this.obstacles[lastIndex];
                    var lastGroup = lastObstacle.obstacle;

                    var removingLast = index == lastIndex;
                    if (removingLast)
                    {
                        var lastObstacleNext = this.obstacles[lastObstacle.nextIndex];
                        lastObstacleNext.previousIndex = lastObstacle.previousIndex;
                        this.obstacles[lastObstacle.nextIndex] = lastObstacleNext;

                        var lastObstaclePrev = this.obstacles[lastObstacle.previousIndex];
                        lastObstaclePrev.nextIndex = lastObstacle.nextIndex;
                        this.obstacles[lastObstacle.previousIndex] = lastObstaclePrev;
                    }
                    else
                    {
                        var lastObstacleNext = this.obstacles[lastObstacle.nextIndex];
                        lastObstacleNext.previousIndex = index;
                        this.obstacles[lastObstacle.nextIndex] = lastObstacleNext;

                        var lastObstaclePrev = this.obstacles[lastObstacle.previousIndex];
                        lastObstaclePrev.nextIndex = index;
                        this.obstacles[lastObstacle.previousIndex] = lastObstaclePrev;
                    }

                    this.obstacleIndexLookup.Replace(lastGroup, lastIndex, index);
                    this.obstacles.RemoveAtSwapBack(index);
                }
            }

            buffer.Dispose();

            this.obstacleIndexLookup.Remove(obstacleId);

            this.obstacleTreeDirty = true;

            return true;
        }

        /// <summary>
        /// Ensure the simulator completes its current job
        /// so that the agent and obstacle data can be safely accessed.
        /// </summary>
        public void EnsureCompleted()
        {
            if (this.scheduled)
            {
                this.jobHandle.Complete();
                this.jobHandle = default;
                UpdateAgents();
                this.scheduled = false;
            }
        }

        /// <summary>
        /// Clears the simulation.
        /// </summary>
        public void Clear()
        {
            this.EnsureCompleted();

            if (this.agentDatas.IsCreated && this.agentDatas.Length > 0)
            {
                this.agentDatas.Clear();
            }

            this.defaultAgent = null;

            this.kdTree.Clear();

            if (this.obstacles.IsCreated && this.obstacles.Length > 0)
            {
                this.obstacles.Clear();
            }

            this.agents.Clear();

            this.globalTime = 0f;
            this.timeStep = 0.1f;

            this.SetNumWorkers(0);

            this.agentTreeDirty = false;
            this.obstacleTreeDirty = false;
        }

        unsafe void PrepareAgentData()
        {
            var arrayLength = this.agents.Count;
            AgentData* agentArray = agentDatas.GetUnsafePtr();
            for (int i = 0; i < arrayLength; i++)
            {
                var agent = this.agents[i];
                var agentData = agentArray + i;
                agentData->position = agent.position;
                agentData->prefVelocity = agent.prefVelocity;
                agentData->velocity = agent.velocity;
                agentData->maxNeighbors = agent.maxNeighbors;
                agentData->maxSpeed = agent.maxSpeed;
                agentData->neighborDist = agent.neighborDist;
                agentData->radius = agent.radius;
                agentData->timeHorizon = agent.timeHorizon;
                agentData->timeHorizonObst = agent.timeHorizonObst;
                agentData->weight = agent.weight;
                agentData->newVelocity = agent.newVelocity;
            }
        }

        unsafe void UpdateAgents()
        {
            var arrayLength = this.agents.Count;
            AgentData* agentArray = agentDatas.GetUnsafePtr();
            for (int i = 0; i < arrayLength; i++)
            {
                var agent = agents[i];
                var agentData = agentArray + i;
                agent.position= agentData->position;
                agent.velocity = agentData->velocity;
                agent.newVelocity = agentData->newVelocity;
            }
        }

        /// <summary>
        /// Performs a simulation step and updates the two-dimensional
        /// position and two-dimensional velocity of each agent.
        /// </summary>
        public void DoStep()
        {
            this.EnsureCompleted();

            this.EnsureObstacleTree();

            var arrayLength = this.agents.Count;

            EnsureTreeCapacity(ref this.kdTree, arrayLength);

            PrepareAgentData();

            // job0
            var buildJob = new BuildJob(this.kdTree, this.agentDatas.AsParallelReader());
            JobHandle jobHandle0 = buildJob.Schedule();

            // job1
            var innerLoop = Mathf.Max(arrayLength / Mathf.Max(this.numWorkers, 1), 1);
            var agentResult = new NativeArray<float2>(this.agents.Count, Allocator.TempJob);
            var computeJob = new ComputeJob(
                this.agentDatas.AsParallelReader(),
                this.obstacles.AsParallelReader(),
                this.kdTree.AsParallelReader(),
                this.timeStep,
                agentResult);
            JobHandle jobHandle1 = computeJob.Schedule(arrayLength, innerLoop, jobHandle0);

            // job2
            var updateJob = new UpdateJob(this.agentDatas.AsArray(), this.timeStep, agentResult);
            JobHandle jobHandle2 = updateJob.Schedule(arrayLength, innerLoop, jobHandle1);
            agentResult.Dispose(jobHandle2);

            this.jobHandle = jobHandle2;
            this.globalTime += this.timeStep;
            this.scheduled = true;
        }

        public bool TryGetAgent(int agentId, out Agent agent)
        {
            if(agentIndexLookup.TryGetValue(agentId,out int index))
            {
                agent = agents[index];
                return true;
            }

            agent = null;
            return false;
        }

        /// <summary>
        /// Returns the global time of the simulation.
        /// </summary>
        /// <returns>The present global time of the simulation (zero initially).
        /// </returns>
        public float GetGlobalTime()
        {
            return this.globalTime;
        }

        /// <summary>
        /// Returns the count of agentIds in the simulation.
        /// </summary>
        /// <returns>The count of agentIds in the simulation.</returns>
        public int GetNumAgents()
        {
            return this.agents.Count;
        }

        /// <summary>
        /// Returns the count of obstacle vertices in the simulation.
        /// </summary>
        /// <returns>The count of obstacle vertices in the simulation.</returns>
        public int GetNumObstacleVertices()
        {
            return this.obstacles.Length;
        }

        /// <summary>
        /// Returns the count of workers.
        /// </summary>
        /// <returns>The count of workers.</returns>
        public int GetNumWorkers()
        {
            return this.numWorkers;
        }

        /// <summary>
        /// Returns the two-dimensional position of a specified obstacle vertex.
        /// </summary>
        /// <param name="vertexId">The number of the obstacle vertex to be retrieved.</param>
        /// <returns>The two-dimensional position of the specified obstacle vertex.</returns>
        public float2 GetObstacleVertex(int vertexId)
        {
            return this.obstacles[vertexId].point;
        }

        /// <summary>
        /// Returns the number of the obstacle vertex succeeding the specified
        /// obstacle vertex in its polygon.
        /// </summary>
        /// <param name="vertexId">The number of the obstacle vertex whose successor
        /// is to be retrieved.</param>
        /// <returns>The number of the obstacle vertex succeeding the specified
        /// obstacle vertex in its polygon.</returns>
        public int GetNextObstacleVertexId(int vertexId)
        {
            return this.obstacles[vertexId].nextIndex;
        }

        /// <summary>
        /// Returns the number of the obstacle vertex preceding the specified
        /// obstacle vertex in its polygon.
        /// </summary>
        /// <param name="vertexId">The number of the obstacle vertex whose
        /// predecessor is to be retrieved.</param>
        /// <returns>The number of the obstacle vertex preceding the specified
        /// obstacle vertex in its polygon.</returns>
        public int GetPrevObstacleVertexId(int vertexId)
        {
            return this.obstacles[vertexId].previousIndex;
        }

        /// <summary>
        /// Returns the first vertex of one obstacle.
        /// </summary>
        /// <param name="obstacleId">The obstacle.</param>
        /// <returns>The first vertex of the obstacle.</returns>
        public int GetFirstObstacleVertexId(int obstacleId)
        {
            if (this.obstacleIndexLookup.TryGetFirstValue(obstacleId, out var result, out var _))
            {
                return result;
            }

            return -1;
        }

        /// <summary>
        /// Returns the time step of the simulation.
        /// </summary>
        ///
        /// <returns>The present time step of the simulation.</returns>
        public float GetTimeStep()
        {
            return this.timeStep;
        }

        /// <summary>
        /// Performs a visibility query between the two specified points
        /// with respect to the obstacles.
        /// </summary>
        /// <param name="point1">The first point of the query.</param>
        /// <param name="point2">The second point of the query.</param>
        /// <param name="radius">The minimal distance between the line connecting
        /// the two points and the obstacles in order for the points to be
        /// mutually visible (optional). Must be non-negative.</param>
        /// <returns>A boolean specifying whether the two points are mutually
        /// visible. Returns true when the obstacles have not been processed.
        /// </returns>
        public bool QueryVisibility(float2 point1, float2 point2, float radius)
        {
            this.EnsureCompleted();

            this.EnsureAgentTree();
            this.EnsureObstacleTree();

            unsafe
            {
                return this.kdTree.AsParallelReader()
                    .QueryVisibility(
                    point1,
                    point2,
                    radius,
                    (Obstacle*)this.obstacles.GetUnsafeReadOnlyPtr(),
                    this.obstacles.Length);
            }
        }

        /// <summary>
        /// Sets the default properties for any new agent that is added.
        /// </summary>
        /// <param name="neighborDist">The default maximum distance (center point
        /// to center point) to other agentIds a new agent takes into account in
        /// the navigation. The larger this number, the longer he running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The default maximum number of other agentIds
        /// a new agent takes into account in the navigation. The larger this
        /// number, the longer the running time of the simulation. If the number
        /// is too low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to other agentIds. The larger this number, the
        /// sooner an agent will respond to the presence of other agentIds, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="timeHorizonObst">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to obstacles. The larger this number, the
        /// sooner an agent will respond to the presence of obstacles, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="radius">The default radius of a new agent. Must be
        /// non-negative.</param>
        /// <param name="maxSpeed">The default maximum speed of a new agent. Must
        /// be non-negative.</param>
        /// <param name="velocity">The default initial two-dimensional linear
        /// velocity of a new agent.</param>
        public void SetAgentDefaults(
            float neighborDist,
            int maxNeighbors,
            float timeHorizon,
            float timeHorizonObst,
            float radius,
            float maxSpeed,
            float2 velocity)
        {
            this.defaultAgent = new Agent
            {
                maxNeighbors = maxNeighbors,
                maxSpeed = maxSpeed,
                neighborDist = neighborDist,
                radius = radius,
                timeHorizon = timeHorizon,
                timeHorizonObst = timeHorizonObst,
                velocity = velocity,
            };
        }

        /// <summary>
        /// Sets the global time of the simulation.
        /// </summary>
        /// <param name="globalTime">The global time of the simulation.</param>
        public void SetGlobalTime(float globalTime)
        {
            this.globalTime = globalTime;
        }

        /// <summary>
        /// Sets the number of workers.
        /// </summary>
        /// <param name="numWorkers">The number of workers.</param>
        public void SetNumWorkers(int numWorkers)
        {
            this.numWorkers = numWorkers;

            if (this.numWorkers <= 0)
            {
                this.numWorkers = JobsUtility.JobWorkerCount;
            }
        }

        /// <summary>
        /// Sets the time step of the simulation.
        /// </summary>
        /// <param name="timeStep">The time step of the simulation.
        /// Must be positive.</param>
        public void SetTimeStep(float timeStep)
        {
            this.timeStep = timeStep;
        }

        /// <summary>
        /// Queries the agent tree for agentIds within a specified radius of a given point.
        /// </summary>
        /// <param name="point">The center point of the query.</param>
        /// <param name="radius">The radius of the query.</param>
        /// <param name="result">The list to store the agentIds found.</param>
        /// <returns>The number of agentIds found within the specified radius.</returns>
        public int QueryAgent(float2 point, float radius, List<int> result)
        {
            if (result == null)
            {
                throw new ArgumentNullException(nameof(result));
            }

            this.EnsureCompleted();

            this.EnsureAgentTree();

            NativeArray<AgentData> agentsAsArray = this.agentDatas.AsArray();

            var buffer = new UnsafeList<AgentData>(8, Allocator.Temp);
            this.kdTree.AsParallelReader()
                .QueryAgentTree(
                in point,
                in radius,
                in agentsAsArray,
                ref buffer);

            result.Clear();
            foreach (var a in buffer)
            {
                result.Add(a.id);
            }

            buffer.Dispose();

            return result.Count;
        }

        /// <inheritdoc/>
        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Recursive method for building an agent k-D tree.
        /// </summary>
        /// <param name="kdTree">The k-D tree.</param>
        /// <param name="begin">The beginning agent k-D tree node node index.</param>
        /// <param name="end">The ending agent k-D tree node index.</param>
        /// <param name="nodeIndex">The current agent k-D tree node index.</param>
        /// <param name="agents">The array that holds the agent data.</param>
        /// <param name="agentsLength">The length for array <paramref name="agents"/>.</param>
        private static unsafe void BuildAgentTreeRecursive(
            ref KdTree kdTree,
            int begin,
            int end,
            int nodeIndex,
            AgentData* agents,
            int agentsLength)
        {
            var agentTreePtr = (KdTree.AgentTreeNode*)kdTree.agentTree.GetUnsafePtr();
            var agentIdsPtr = (int*)kdTree.agentIds.GetUnsafePtr();

            KdTree.AgentTreeNode* node = agentTreePtr + nodeIndex;
            node->begin = begin;
            node->end = end;
            AgentData* agentBegin = agents + agentIdsPtr[begin];
            node->minX = node->maxX = agentBegin->position.x;
            node->minY = node->maxY = agentBegin->position.y;

            for (var i = begin + 1; i < end; ++i)
            {
                AgentData* agentI = agents + agentIdsPtr[i];
                node->maxX = math.max(node->maxX, agentI->position.x);
                node->minX = math.min(node->minX, agentI->position.x);
                node->maxY = math.max(node->maxY, agentI->position.y);
                node->minY = math.min(node->minY, agentI->position.y);
            }

            if (end - begin <= KdTree.MaxLeafSize)
            {
                return;
            }

            // No leaf node.
            var isVertical = node->maxX - node->minX
                             > node->maxY - node->minY;
            var splitValue = 0.5f * (isVertical
                                 ? node->maxX + node->minX
                                 : node->maxY + node->minY);

            var left = begin;
            var right = end;

            while (left < right)
            {
                while (true)
                {
                    AgentData* agentLeft = agents + agentIdsPtr[left];
                    if (left < right
                        && (isVertical ? agentLeft->position.x : agentLeft->position.y) < splitValue)
                    {
                        ++left;
                    }
                    else
                    {
                        break;
                    }
                }

                while (true)
                {
                    AgentData* agentRight = agents + agentIdsPtr[right - 1];
                    if (right > left
                        && (isVertical ? agentRight->position.x : agentRight->position.y) >= splitValue)
                    {
                        --right;
                    }
                    else
                    {
                        break;
                    }
                }

                if (left >= right)
                {
                    continue;
                }

                var tempAgentIndex = agentIdsPtr[left];
                agentIdsPtr[left] = agentIdsPtr[right - 1];
                agentIdsPtr[right - 1] = tempAgentIndex;
                ++left;
                --right;
            }

            var leftSize = left - begin;

            if (leftSize == 0)
            {
                ++leftSize;
                ++left;
            }

            node->left = nodeIndex + 1;
            node->right = nodeIndex + (2 * leftSize);

            BuildAgentTreeRecursive(ref kdTree, begin, left, node->left, agents, agentsLength);
            BuildAgentTreeRecursive(ref kdTree, left, end, node->right, agents, agentsLength);
        }

        private static void EnsureTreeCapacity(ref KdTree kdTree, int agentCount)
        {
            if (kdTree.agentIds.Length == agentCount)
            {
                return;
            }

            kdTree.agentIds.Resize(agentCount, Allocator.Persistent);
            for (var i = 0; i < agentCount; ++i)
            {
                kdTree.agentIds[i] = i;
            }

            var agentTreeSize = 2 * agentCount;
            kdTree.agentTree.Resize(agentTreeSize, Allocator.Persistent);
            for (var i = 0; i < agentTreeSize; ++i)
            {
                kdTree.agentTree[i] = default;
            }
        }

        /// <summary>
        /// Builds an agent k-D tree.
        /// </summary>
        /// <param name="kdTree">The k-D Tree.</param>
        /// <param name="agents">The array that holds the agent data.</param>
        /// <param name="agentsLength">The length for array <paramref name="agents"/>.</param>
        private static unsafe void BuildAgentTree(
            ref KdTree kdTree,
            AgentData* agents,
            int agentsLength)
        {
            if (kdTree.agentIds.Length == 0)
            {
                return;
            }

            for (var i = 0; i < agentsLength; i++)
            {
                kdTree.agentIds[i] = i;
            }

            BuildAgentTreeRecursive(ref kdTree, 0, kdTree.agentIds.Length, 0, agents, agentsLength);
        }

        private unsafe Obstacle* NewObstacleVert(float2 point, int obstacleId)
        {
            var newIndex = this.obstacles.Length;
            var obstacleVert = new Obstacle(newIndex, point, obstacleId);
            this.obstacles.Add(obstacleVert);

            this.obstacleIndexLookup.Add(obstacleId, newIndex);

            return (Obstacle*)this.obstacles.GetUnsafePtr() + newIndex;
        }

        private unsafe void EnsureAgentTree()
        {
            if (this.agentTreeDirty)
            {
                EnsureTreeCapacity(ref this.kdTree, this.agentDatas.Length);
                var agentsReadonly = (AgentData*)this.agentDatas.GetUnsafeReadOnlyPtr();
                var agentsLength = this.agentDatas.Length;
                BuildAgentTree(ref this.kdTree, agentsReadonly, agentsLength);
                this.agentTreeDirty = false;
            }
        }

        private void EnsureObstacleTree()
        {
            if (this.obstacleTreeDirty)
            {
                this.BuildObstacleTree();
                this.obstacleTreeDirty = false;
            }
        }

        /// <summary>
        /// Adds a new agent to the simulation.
        /// </summary>
        /// <param name="position">The two-dimensional starting position of this
        /// agent.</param>
        /// <param name="neighborDist">The maximum distance (center point to
        /// center point) to other agentIds this agent takes into account in the
        /// navigation. The larger this number, the longer the running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The maximum number of other agentIds this
        /// agent takes into account in the navigation. The larger this number,
        /// the longer the running time of the simulation. If the number is too
        /// low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The minimal amount of time for which this
        /// agent's velocities that are computed by the simulation are safe with
        /// respect to other agentIds. The larger this number, the sooner this
        /// agent will respond to the presence of other agentIds, but the less
        /// freedom this agent has in choosing its velocities. Must be positive.
        /// </param>
        /// <param name="timeHorizonObst">The minimal amount of time for which
        /// this agent's velocities that are computed by the simulation are safe
        /// with respect to obstacles. The larger this number, the sooner this
        /// agent will respond to the presence of obstacles, but the less freedom
        /// this agent has in choosing its velocities. Must be positive.</param>
        /// <param name="radius">The radius of this agent. Must be non-negative.
        /// </param>
        /// <param name="maxSpeed">The maximum speed of this agent. Must be
        /// non-negative.</param>
        /// <param name="velocity">The initial two-dimensional linear velocity of
        /// this agent.</param>
        /// <returns>The number of the agent.</returns>
        public Agent AddAgent(
            float2 position,
            float neighborDist,
            int maxNeighbors,
            float timeHorizon,
            float timeHorizonObst,
            float radius,
            float maxSpeed,
            float2 velocity)
        {
            var newIndex = this.agentDatas.Length;
            var agentId = ++this.sid;
            var agentData = new AgentData(agentId);

            agentData.maxNeighbors = maxNeighbors;
            agentData.maxSpeed = maxSpeed;
            agentData.neighborDist = neighborDist;
            agentData.position = position;
            agentData.radius = radius;
            agentData.timeHorizon = timeHorizon;
            agentData.timeHorizonObst = timeHorizonObst;
            agentData.velocity = velocity;
            agentData.weight = 0.5f;

            var agent = new Agent(agentData);

            this.agentDatas.Add(agentData);
            this.agents.Add(agent);
            this.agentIndexLookup[agentId] = newIndex;

            this.agentTreeDirty = true;

            return agent;
        }

        /// <summary>
        /// Builds an obstacle k-D tree.
        /// </summary>
        private unsafe void BuildObstacleTree()
        {
            var kdTreeToBuild = this.kdTree;
            kdTreeToBuild.obstacleTreeNodes.Resize(0);

            var obstaclesLength = this.obstacles.Length;
            var obstacleIds = new NativeArray<int>(obstaclesLength, Allocator.Temp);
            var pointer = (int*)obstacleIds.GetUnsafePtr();

            for (var i = 0; i < obstaclesLength; ++i)
            {
                pointer[i] = i;
            }

            this.BuildObstacleTreeRecursive(ref kdTreeToBuild, pointer, obstaclesLength);
            this.kdTree = kdTreeToBuild;

            obstacleIds.Dispose();
        }

        /// <summary>
        /// Recursive method for building an obstacle k-D tree.
        /// </summary>
        /// <param name="kdTreeToBuild">The k-D tree.</param>
        /// <param name="obstacleIds">A pointer to the array holds obstacle Ids.</param>
        /// <param name="obstacleLength">Length of the array holds obstacle Ids.</param>
        /// <returns>An obstacle k-D tree node.</returns>
        private unsafe int BuildObstacleTreeRecursive(
            ref KdTree kdTreeToBuild,
            int* obstacleIds,
            in int obstacleLength)
        {
            if (obstacleLength == 0)
            {
                return -1;
            }

            var nodeIndex = kdTreeToBuild.NewObstacleTreeNode();
            var obstacleTreeNodesPtr = (KdTree.ObstacleTreeNode*)kdTreeToBuild.obstacleTreeNodes.GetUnsafePtr();
            KdTree.ObstacleTreeNode* node = obstacleTreeNodesPtr + nodeIndex;
            this.kdTree = kdTreeToBuild;

            var optimalSplit = 0;
            var minLeft = obstacleLength;
            var minRight = obstacleLength;

            var obstaclesPtr = (Obstacle*)this.obstacles.GetUnsafePtr();

            for (var i = 0; i < obstacleLength; ++i)
            {
                var leftSize = 0;
                var rightSize = 0;

                var obstacleI1Index = obstacleIds[i];
                Obstacle* obstacleI1 = obstaclesPtr + obstacleI1Index;
                var obstacleI2Index = obstacleI1->nextIndex;
                Obstacle* obstacleI2 = obstaclesPtr + obstacleI2Index;

                // Compute optimal split node.
                for (var j = 0; j < obstacleLength; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    var obstacleJ1Index = obstacleIds[j];
                    Obstacle* obstacleJ1 = obstaclesPtr + obstacleJ1Index;
                    var obstacleJ2Index = obstacleJ1->nextIndex;
                    Obstacle* obstacleJ2 = obstaclesPtr + obstacleJ2Index;

                    var j1LeftOfI = RVOMath.LeftOf(obstacleI1->point, obstacleI2->point, obstacleJ1->point);
                    var j2LeftOfI = RVOMath.LeftOf(obstacleI1->point, obstacleI2->point, obstacleJ2->point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        ++rightSize;
                    }
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    var bound1 = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                    var bound2 = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                    if (RVOMath.GreaterEqual(bound1, bound2))
                    {
                        break;
                    }
                }

                var bound1f = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                var bound2f = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                if (!RVOMath.Less(bound1f, bound2f))
                {
                    continue;
                }

                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }

            {
                // Build split node.
                var leftObstacles = new NativeArray<int>(minLeft, Allocator.Temp);
                var leftObstaclesPtr = (int*)leftObstacles.GetUnsafePtr();

                for (var n = 0; n < minLeft; ++n)
                {
                    leftObstaclesPtr[n] = -1;
                }

                var rightObstacles = new NativeArray<int>(minRight, Allocator.Temp);
                var rightObstaclesPtr = (int*)rightObstacles.GetUnsafePtr();

                for (var n = 0; n < minRight; ++n)
                {
                    rightObstaclesPtr[n] = -1;
                }

                var leftCounter = 0;
                var rightCounter = 0;
                var i = optimalSplit;

                var obstacleI1Index = obstacleIds[i];
                Obstacle* obstacleI1 = obstaclesPtr + obstacleI1Index;
                var obstacleI2Index = obstacleI1->nextIndex;
                Obstacle* obstacleI2 = obstaclesPtr + obstacleI2Index;

                for (var j = 0; j < obstacleLength; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    var obstacleJ1Index = obstacleIds[j];
                    Obstacle* obstacleJ1 = obstaclesPtr + obstacleJ1Index;
                    var obstacleJ2Index = obstacleJ1->nextIndex;
                    Obstacle* obstacleJ2 = obstaclesPtr + obstacleJ2Index;

                    var j1LeftOfI = RVOMath.LeftOf(obstacleI1->point, obstacleI2->point, obstacleJ1->point);
                    var j2LeftOfI = RVOMath.LeftOf(obstacleI1->point, obstacleI2->point, obstacleJ2->point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        leftObstaclesPtr[leftCounter++] = obstacleIds[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        rightObstaclesPtr[rightCounter++] = obstacleIds[j];
                    }
                    else
                    {
                        // Split obstacle j.
                        var dI2I1 = obstacleI2->point - obstacleI1->point;
                        var t = RVOMath.Det(dI2I1, obstacleJ1->point - obstacleI1->point)
                            / RVOMath.Det(dI2I1, obstacleJ1->point - obstacleJ2->point);

                        float2 splitPoint = obstacleJ1->point + (t * (obstacleJ2->point - obstacleJ1->point));

                        // NewObstacleVert will cause this.obstacles change
                        // so the old obstaclesPtr value became invalid and we have to assign again.
                        Obstacle* newObstacle = this.NewObstacleVert(splitPoint, obstacleJ1->obstacle);
                        obstaclesPtr = (Obstacle*)this.obstacles.GetUnsafePtr();

                        // 往NativeList中添加新元素可能导致NativeList扩容
                        // NativeList扩容后,指针指向了新地址
                        // 这时通过原指针计算出的其他指针还指向旧地址
                        // 需要更新这些指针
                        obstacleI1 = obstaclesPtr + obstacleI1Index;
                        obstacleI2 = obstaclesPtr + obstacleI2Index;
                        obstacleJ1 = obstaclesPtr + obstacleJ1Index;
                        obstacleJ2 = obstaclesPtr + obstacleJ2Index;

                        var newObstacleIndex = newObstacle->id;
                        newObstacle->previousIndex = obstacleJ1Index;
                        newObstacle->nextIndex = obstacleJ2Index;
                        newObstacle->convex = true;
                        newObstacle->direction = obstacleJ1->direction;

                        obstacleJ1->nextIndex = newObstacleIndex;
                        obstacleJ2->previousIndex = newObstacleIndex;

                        if (j1LeftOfI > 0f)
                        {
                            leftObstaclesPtr[leftCounter++] = obstacleJ1Index;
                            rightObstaclesPtr[rightCounter++] = newObstacleIndex;
                        }
                        else
                        {
                            rightObstaclesPtr[rightCounter++] = obstacleJ1Index;
                            leftObstaclesPtr[leftCounter++] = newObstacleIndex;
                        }
                    }
                }

                node->obstacleIndex = obstacleI1Index;

                var leftIndex = this.BuildObstacleTreeRecursive(
                    ref kdTreeToBuild,
                    (int*)leftObstacles.GetUnsafePtr(),
                    leftObstacles.Length);

                // NewObstacleTreeNode() in BuildObstacleTreeRecursive() will cause obstacleTreeNodes change
                // so the old obstacleTreeNodesPtr value became invalid and we have to assign again.
                obstacleTreeNodesPtr = (KdTree.ObstacleTreeNode*)kdTreeToBuild.obstacleTreeNodes.GetUnsafePtr();
                node = obstacleTreeNodesPtr + nodeIndex;
                node->leftIndex = leftIndex;

                var rightIndex = this.BuildObstacleTreeRecursive(
                    ref kdTreeToBuild,
                    (int*)rightObstacles.GetUnsafePtr(),
                    rightObstacles.Length);

                // NewObstacleTreeNode() in BuildObstacleTreeRecursive() will cause obstacleTreeNodes change
                // so the old obstacleTreeNodesPtr value became invalid and we have to assign again.
                obstacleTreeNodesPtr = (KdTree.ObstacleTreeNode*)kdTreeToBuild.obstacleTreeNodes.GetUnsafePtr();
                node = obstacleTreeNodesPtr + nodeIndex;
                node->rightIndex = rightIndex;

                leftObstacles.Dispose();
                rightObstacles.Dispose();

                return nodeIndex;
            }
        }

        private void Dispose(bool disposing)
        {
            if (!this.disposedValue)
            {
                if (disposing)
                {
                    // Managed state
                    this.Clear();
                }

                this.agentDatas.Dispose();
                this.obstacles.Dispose();

                this.kdTree.Dispose();

                this.agents.Clear();

                this.agentIndexLookup.Dispose();
                this.obstacleIndexLookup.Dispose();

                // Unmanaged resources
                this.disposedValue = true;
            }
        }

        [BurstCompile]
        private struct BuildJob : IJob
        {
            private KdTree kdTree;
            [ReadOnly]
            private NativeArray<AgentData>.ReadOnly agents;

            public BuildJob(KdTree kdTree, NativeArray<AgentData>.ReadOnly agents)
                : this()
            {
                this.kdTree = kdTree;
                this.agents = agents;
            }

            public unsafe void Execute()
            {
                BuildAgentTree(ref this.kdTree, (AgentData*)this.agents.GetUnsafeReadOnlyPtr(), this.agents.Length);
            }
        }

        [BurstCompile]
        private struct ComputeJob : IJobParallelFor
        {
            private readonly float timeStep;
            [WriteOnly]
            private NativeArray<float2> agentResult;
            [ReadOnly]
            private NativeArray<AgentData>.ReadOnly agents;
            [ReadOnly]
            private NativeArray<Obstacle>.ReadOnly obstacles;
            [ReadOnly]
            private KdTree.ReadOnly kdTree;

            public ComputeJob(
                NativeArray<AgentData>.ReadOnly agents,
                NativeArray<Obstacle>.ReadOnly obstacles,
                KdTree.ReadOnly kdTree,
                float timeStep,
                NativeArray<float2> agentResult)
                : this()
            {
                this.agents = agents;
                this.obstacles = obstacles;
                this.kdTree = kdTree;
                this.timeStep = timeStep;
                this.agentResult = agentResult;
            }

            public unsafe void Execute(int index)
            {
                var agentNeighbors = new UnsafeList<AgentData.Pair>(8, Allocator.Temp);
                var obstacleNeighbors = new UnsafeList<AgentData.Pair>(8, Allocator.Temp);

                var agentsPtr = (AgentData*)this.agents.GetUnsafeReadOnlyPtr();
                var obstaclesPtr = (Obstacle*)this.obstacles.GetUnsafeReadOnlyPtr();
                var agentsLength = this.agents.Length;
                var obstaclesLength = this.obstacles.Length;
                AgentData* agent = agentsPtr + index;

                agent->ComputeNeighbors(
                    in index,
                    in this.kdTree,
                    agentsPtr,
                    agentsLength,
                    obstaclesPtr,
                    obstaclesLength,
                    ref agentNeighbors,
                    ref obstacleNeighbors);
                agent->ComputeNewVelocity(
                    this.timeStep,
                    agentsPtr,
                    agentsLength,
                    obstaclesPtr,
                    obstaclesLength,
                    ref agentNeighbors,
                    ref obstacleNeighbors);
                this.agentResult[index] = agent->newVelocity;

                agentNeighbors.Dispose();
                obstacleNeighbors.Dispose();
            }
        }

        [BurstCompile]
        private struct UpdateJob : IJobParallelFor
        {
            private readonly float timeStep;
            [ReadOnly]
            private NativeArray<float2> agentResult;
            private NativeArray<AgentData> agents;

            public UpdateJob(
                NativeArray<AgentData> agents,
                float timeStep,
                NativeArray<float2> agentResult)
                : this()
            {
                this.agents = agents;
                this.timeStep = timeStep;
                this.agentResult = agentResult;
            }

            public void Execute(int index)
            {
                AgentData agent = this.agents[index];
                agent.newVelocity = this.agentResult[index];
                agent.Update(this.timeStep);
                this.agents[index] = agent;
            }
        }
    }
}
