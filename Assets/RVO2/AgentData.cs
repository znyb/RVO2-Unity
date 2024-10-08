// -----------------------------------------------------------------------
// <copyright file="Agent.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Agent.cs
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
    using Unity.Collections;
    using Unity.Collections.LowLevel.Unsafe;

#if RVO_FIXEDPOINT
    using fp = Deterministic.FixedPoint.fp;
    using float2 = Deterministic.FixedPoint.fp2;
    using math = Deterministic.FixedPoint.fixmath;
#else
    using Unity.Mathematics;
    using fp = System.Single;
#endif

    /// <summary>
    /// Defines an agent in the simulation.
    /// </summary>
    public struct AgentData
    {
        public readonly int id;
        
        public float2 position;
        public float2 prefVelocity;
        public float2 velocity;
        public int maxNeighbors;
        public fp maxSpeed;
        public fp neighborDist;
        public fp radius;
        public fp timeHorizon;
        public fp timeHorizonObst;
        public fp weight;
        public float2 newVelocity;

        internal AgentData(int id)
            : this()
        {
            this.id = id;
        }

        /// <summary>
        /// Computes the neighbors of this agent.
        /// </summary>
        internal unsafe void ComputeNeighbors(
            in int index,
            in KdTree.ReadOnly kdTree,
            AgentData* agents,
            int agentsLength,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> agentNeighbors,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            fp rangeSq = RVOMath.Square((this.timeHorizonObst * this.maxSpeed) + this.radius);
            fixed (AgentData* thisPtr = &this)
            {
                kdTree.ComputeObstacleNeighbors(
                    thisPtr,
                    rangeSq,
                    obstacles,
                    obstaclesLength,
                    ref obstacleNeighbors);
            }

            if (this.maxNeighbors > 0)
            {
                rangeSq = RVOMath.Square(this.neighborDist);
                fixed (AgentData* thisPtr = &this)
                {
                    kdTree.ComputeAgentNeighbors(
                        thisPtr,
                        ref rangeSq,
                        agents,
                        agentsLength,
                        ref agentNeighbors);
                }
            }
        }

        /// <summary>
        /// Computes the new velocity of this agent.
        /// </summary>
        internal unsafe void ComputeNewVelocity(
            fp timeStep,
            AgentData* agents,
            int agentsLength,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> agentNeighbors,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            UnsafeList<Line> orcaLines = new UnsafeList<Line>(8, Allocator.Temp);

            fp invTimeHorizonObst = RVOMath.One / this.timeHorizonObst;

            // Create obstacle ORCA lines.
            for (int i = 0; i < obstacleNeighbors.Length; ++i)
            {
                int obstacle1Index = obstacleNeighbors[i].index;
                Obstacle* obstacle1 = obstacles + obstacle1Index;
                int obstacle2Index = obstacle1->nextIndex;
                Obstacle* obstacle2 = obstacles + obstacle2Index;

                float2 relativePosition1 = obstacle1->point - this.position;
                float2 relativePosition2 = obstacle2->point - this.position;

                // Check if velocity obstacle of obstacle is already taken care
                // of by previously constructed obstacle ORCA lines.
                bool alreadyCovered = false;

                float2 relativeMove1 = invTimeHorizonObst * relativePosition1;
                float2 relativeMove2 = invTimeHorizonObst * relativePosition2;
                fp dist = invTimeHorizonObst * this.radius;

                for (int j = 0; j < orcaLines.Length; ++j)
                {
                    Line lineJ = orcaLines[j];
                    if (RVOMath.Det(relativeMove1 - lineJ.point, lineJ.direction) - dist >= -RVOMath.RVO_EPSILON &&
                        RVOMath.Det(relativeMove2 - lineJ.point, lineJ.direction) - dist >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                // Not yet covered. Check for collisions.
                fp distSq1 = math.lengthsq(relativePosition1);
                fp distSq2 = math.lengthsq(relativePosition2);

                fp radiusSq = RVOMath.Square(this.radius);

                float2 obstacleVector = obstacle2->point - obstacle1->point;
                fp s = math.dot(-relativePosition1, obstacleVector) / math.lengthsq(obstacleVector);
                fp distSqLine = math.lengthsq(-relativePosition1 - (s * obstacleVector));

                if (s < RVOMath.Zero && distSq1 <= radiusSq)
                {
                    // Collision with left vertex. Ignore if non-convex.
                    if (obstacle1->convex)
                    {
                        Line line = new Line(
                            new float2(RVOMath.Zero, RVOMath.Zero),
                            math.normalize(new float2(-relativePosition1.y, relativePosition1.x)));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s > RVOMath.One && distSq2 <= radiusSq)
                {
                    // Collision with right vertex. Ignore if non-convex or if
                    // it will be taken care of by neighboring obstacle.
                    if (obstacle2->convex && RVOMath.Det(relativePosition2, obstacle2->direction) >= RVOMath.Zero)
                    {
                        Line line = new Line(
                            new float2(RVOMath.Zero, RVOMath.Zero),
                            math.normalize(new float2(-relativePosition2.y, relativePosition2.x)));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s >= RVOMath.Zero && s <= RVOMath.One && distSqLine <= radiusSq)
                {
                    // Collision with obstacle segment.
                    Line line = new Line(
                        new float2(RVOMath.Zero, RVOMath.Zero),
                        -obstacle1->direction);
                    orcaLines.Add(line);

                    continue;
                }

                // No collision. Compute legs. When obliquely viewed, both legs
                // can come from a single vertex. Legs extend cut-off line when
                // non-convex vertex.
                float2 leftLegDirection, rightLegDirection;

                if (s < RVOMath.Zero && distSqLine <= radiusSq)
                {
                    // Obstacle viewed obliquely so that left vertex
                    // defines velocity obstacle.
                    if (!obstacle1->convex)
                    {
                        // Ignore obstacle.
                        continue;
                    }

                    obstacle2 = obstacle1;

                    fp leg1 = math.sqrt(distSq1 - radiusSq);

                    leftLegDirection = new float2(
                        (relativePosition1.x * leg1) - (relativePosition1.y * this.radius),
                        (relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                        / distSq1;
                    rightLegDirection = new float2(
                        (relativePosition1.x * leg1) + (relativePosition1.y * this.radius),
                        (-relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                        / distSq1;
                }
                else if (s > RVOMath.One && distSqLine <= radiusSq)
                {
                    // Obstacle viewed obliquely so that
                    // right vertex defines velocity obstacle.
                    if (!obstacle2->convex)
                    {
                        // Ignore obstacle.
                        continue;
                    }

                    obstacle1 = obstacle2;

                    fp leg2 = math.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new float2(
                        (relativePosition2.x * leg2) - (relativePosition2.y * this.radius),
                        (relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                        / distSq2;
                    rightLegDirection = new float2(
                        (relativePosition2.x * leg2) + (relativePosition2.y * this.radius),
                        (-relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                        / distSq2;
                }
                else
                {
                    // Usual situation.
                    if (obstacle1->convex)
                    {
                        fp leg1 = math.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new float2(
                            (relativePosition1.x * leg1) - (relativePosition1.y * this.radius),
                            (relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                            / distSq1;
                    }
                    else
                    {
                        // Left vertex non-convex; left leg extends cut-off line.
                        leftLegDirection = -obstacle1->direction;
                    }

                    if (obstacle2->convex)
                    {
                        fp leg2 = math.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new float2(
                            (relativePosition2.x * leg2) + (relativePosition2.y * this.radius),
                            (-relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                            / distSq2;
                    }
                    else
                    {
                        // Right vertex non-convex; right leg extends cut-off line.
                        rightLegDirection = obstacle1->direction;
                    }
                }

                // Legs can never point into neighboring edge when convex
                // vertex, take cutoff-line of neighboring edge instead. If
                // velocity projected on "foreign" leg, no constraint is added.
                int leftNeighborIndex = obstacle1->previousIndex;
                Obstacle* leftNeighbor = obstacles + leftNeighborIndex;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1->convex && RVOMath.Det(leftLegDirection, -leftNeighbor->direction) >= RVOMath.Zero)
                {
                    // Left leg points into obstacle.
                    leftLegDirection = -leftNeighbor->direction;
                    isLeftLegForeign = true;
                }

                if (obstacle2->convex && RVOMath.Det(rightLegDirection, obstacle2->direction) <= RVOMath.Zero)
                {
                    // Right leg points into obstacle.
                    rightLegDirection = obstacle2->direction;
                    isRightLegForeign = true;
                }

                // Compute cut-off centers.
                float2 leftCutOff = invTimeHorizonObst * (obstacle1->point - this.position);
                float2 rightCutOff = invTimeHorizonObst * (obstacle2->point - this.position);
                float2 cutOffVector = rightCutOff - leftCutOff;

                // Project current velocity on velocity obstacle.

                // Check if current velocity is projected on cutoff circles.
                bool same = obstacle1->id == obstacle2->id;
                fp t = same ? RVOMath.Half : math.dot(this.velocity - leftCutOff, cutOffVector)
                    / math.lengthsq(cutOffVector);
                fp tLeft = math.dot(this.velocity - leftCutOff, leftLegDirection);
                fp tRight = math.dot(this.velocity - rightCutOff, rightLegDirection);

                if ((t < RVOMath.Zero && tLeft < RVOMath.Zero) || (same && tLeft < RVOMath.Zero && tRight < RVOMath.Zero))
                {
                    // Project on left cut-off circle.
                    float2 unitW = math.normalize(this.velocity - leftCutOff);

                    Line line = new Line(
                        leftCutOff + (dist * unitW),
                        new float2(unitW.y, -unitW.x));
                    orcaLines.Add(line);

                    continue;
                }
                else if (t > RVOMath.One && tRight < RVOMath.Zero)
                {
                    // Project on right cut-off circle.
                    float2 unitW = math.normalize(this.velocity - rightCutOff);

                    Line line = new Line(
                        rightCutOff + (dist * unitW),
                        new float2(unitW.y, -unitW.x));
                    orcaLines.Add(line);

                    continue;
                }

                // Project on left leg, right leg, or cut-off line, whichever is
                // closest to velocity.
                fp distSqCutoff = (t < RVOMath.Zero || t > RVOMath.One || obstacle1->id == obstacle2->id)
                    ? RVOMath.Max
                    : math.lengthsq(this.velocity - (leftCutOff + (t * cutOffVector)));
                fp distSqLeft = tLeft < RVOMath.Zero ? RVOMath.Max
                    : math.lengthsq(this.velocity - (leftCutOff + (tLeft * leftLegDirection)));
                fp distSqRight = tRight < RVOMath.Zero ? RVOMath.Max
                    : math.lengthsq(this.velocity - (rightCutOff + (tRight * rightLegDirection)));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    // Project on cut-off line.
                    float2 direction = -obstacle1->direction;
                    Line line = new Line(
                        leftCutOff + (dist * new float2(-direction.y, direction.x)),
                        direction);
                    orcaLines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    // Project on left leg.
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    float2 direction = leftLegDirection;
                    Line line = new Line(
                        leftCutOff + (dist * new float2(-direction.y, direction.x)),
                        direction);
                    orcaLines.Add(line);

                    continue;
                }

                // Project on right leg.
                if (isRightLegForeign)
                {
                    continue;
                }

                float2 direction0 = -rightLegDirection;
                Line line0 = new Line(
                    rightCutOff + (dist * new float2(-direction0.y, direction0.x)),
                    direction0);
                orcaLines.Add(line0);
            }

            int numObstLines = orcaLines.Length;

            fp invTimeHorizon = RVOMath.One / this.timeHorizon;

            // Create agent ORCA lines.
            for (int i = 0; i < agentNeighbors.Length; ++i)
            {
                int otherIndex = agentNeighbors[i].index;
                AgentData* other = agents + otherIndex;

                float2 relativePosition = other->position - this.position;
                float2 relativeVelocity = this.velocity - other->velocity;
                fp distSq = math.lengthsq(relativePosition);
                fp combinedRadius = this.radius + other->radius;
                fp combinedRadiusSq = RVOMath.Square(combinedRadius);

                float2 point;
                float2 direction;
                float2 u;

                if (distSq > combinedRadiusSq)
                {
                    // No collision.
                    float2 w = relativeVelocity - (invTimeHorizon * relativePosition);

                    // Vector from cutoff center to relative velocity.
                    fp wLengthSq = math.lengthsq(w);
                    fp dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < RVOMath.Zero && RVOMath.Square(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        // Project on cut-off circle.
                        fp wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        direction = new float2(unitW.y, -unitW.x);
                        u = ((combinedRadius * invTimeHorizon) - wLength) * unitW;
                    }
                    else
                    {
                        // Project on legs.
                        fp leg = math.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.Det(relativePosition, w) > RVOMath.Zero)
                        {
                            // Project on left leg.
                            direction = new float2(
                                (relativePosition.x * leg) - (relativePosition.y * combinedRadius),
                                (relativePosition.x * combinedRadius) + (relativePosition.y * leg))
                                / distSq;
                        }
                        else
                        {
                            // Project on right leg.
                            direction = -new float2(
                                (relativePosition.x * leg) + (relativePosition.y * combinedRadius),
                                (-relativePosition.x * combinedRadius) + (relativePosition.y * leg))
                                / distSq;
                        }

                        fp dotProduct2 = math.dot(relativeVelocity, direction);
                        u = (dotProduct2 * direction) - relativeVelocity;
                    }
                }
                else
                {
                    // Collision. Project on cut-off circle of time timeStep.
                    fp invTimeStep = RVOMath.One / timeStep;

                    // Vector from cutoff center to relative velocity.
                    float2 w = relativeVelocity - (invTimeStep * relativePosition);

                    fp wLength = math.length(w);
                    float2 unitW = w / wLength;

                    direction = new float2(unitW.y, -unitW.x);
                    u = ((combinedRadius * invTimeStep) - wLength) * unitW;
                }

                point = this.velocity + (getAgentWeight(other->weight) * u);
                Line line = new Line(point, direction);
                orcaLines.Add(line);
            }

            int lineFail = this.LinearProgram2(
                orcaLines.Ptr,
                orcaLines.Length,
                this.maxSpeed,
                this.prefVelocity,
                false,
                ref this.newVelocity);

            if (lineFail < orcaLines.Length)
            {
                this.LinearProgram3(
                    orcaLines.Ptr,
                    orcaLines.Length,
                    numObstLines,
                    lineFail,
                    this.maxSpeed,
                    ref this.newVelocity);
            }

            orcaLines.Dispose();
        }

        internal fp getAgentWeight(fp otherWeight)
        {
            return weight / (weight + otherWeight);
        }

        /// <summary>
        /// Inserts an agent neighbor into the set of neighbors of this agent.
        /// </summary>
        /// <param name="agentIndex">A pointer to the agent to be inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        /// <param name="agents">The array that holds the agent data.</param>
        /// <param name="agentsLength">The length for array <paramref name="agents"/>.</param>
        /// <param name="agentNeighbors">The list to store the neighbor data.</param>
        internal unsafe void InsertAgentNeighbor(
            int agentIndex,
            ref fp rangeSq,
            AgentData* agents,
            int agentsLength,
            ref UnsafeList<Pair> agentNeighbors)
        {
            AgentData* agent = agents + agentIndex;
            if (this.id == agent->id)
            {
                return;
            }

            fp distSq = math.lengthsq(this.position - agent->position);

            if (distSq < rangeSq)
            {
                if (agentNeighbors.Length < this.maxNeighbors)
                {
                    agentNeighbors.Add(new Pair(distSq, agentIndex));
                }

                int i = agentNeighbors.Length - 1;

                while (i != 0 && distSq < agentNeighbors[i - 1].dist)
                {
                    agentNeighbors[i] = agentNeighbors[i - 1];
                    --i;
                }

                agentNeighbors[i] = new Pair(distSq, agentIndex);

                if (agentNeighbors.Length == this.maxNeighbors)
                {
                    rangeSq = agentNeighbors[agentNeighbors.Length - 1].dist;
                }
            }
        }

        /// <summary>
        /// Inserts a static obstacle neighbor into the set of neighbors of this agent.
        /// </summary>
        /// <param name="obstacleIndex">The number of the static obstacle to be inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        /// <param name="obstacles">The array that holds the obstacle verts.</param>
        /// <param name="obstaclesLength">The length for array <paramref name="obstacles"/>.</param>
        /// <param name="obstacleNeighbors">The list to store the neighbor dara.</param>
        internal unsafe void InsertObstacleNeighbor(
            int obstacleIndex,
            fp rangeSq,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            Obstacle* obstacle = obstacles + obstacleIndex;
            int nextObstacleIndex = obstacle->nextIndex;
            Obstacle* nextObstacle = obstacles + nextObstacleIndex;

            fp distSq = RVOMath.DistSqPointLineSegment(obstacle->point, nextObstacle->point, this.position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new Pair(distSq, obstacleIndex));

                int i = obstacleNeighbors.Length - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].dist)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }

                obstacleNeighbors[i] = new Pair(distSq, obstacleIndex);
            }
        }

        /// <summary>
        /// Updates the two-dimensional position and two-dimensional velocity of this agent.
        /// </summary>
        internal void Update(fp timeStep)
        {
            this.velocity = this.newVelocity;
            this.position += this.velocity * timeStep;
        }

        /// <summary>
        /// Solves a one-dimensional linear program on a specified line subject to
        /// linear constraints defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="lineNo">The specified line constraint.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        /// <returns>True if successful.</returns>
        private unsafe bool LinearProgram1(
            Line* lines,
            int linesLength,
            int lineNo,
            fp radius,
            float2 optVelocity,
            bool directionOpt,
            ref float2 result)
        {
            Line* lineNoPtr = lines + lineNo;
            fp dotProduct = math.dot(lineNoPtr->point, lineNoPtr->direction);
            fp discriminant = RVOMath.Square(dotProduct) + RVOMath.Square(radius) - math.lengthsq(lineNoPtr->point);

            if (discriminant < RVOMath.Zero)
            {
                // Max speed circle fully invalidates line lineNo.
                return false;
            }

            fp sqrtDiscriminant = math.sqrt(discriminant);
            fp tLeft = -dotProduct - sqrtDiscriminant;
            fp tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                Line* lineI = lines + i;
                fp denominator = RVOMath.Det(lineNoPtr->direction, lineI->direction);
                fp numerator = RVOMath.Det(lineI->direction, lineNoPtr->point - lineI->point);

                if (math.abs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    // Lines lineNo and i are (almost) parallel.
                    if (numerator < RVOMath.Zero)
                    {
                        return false;
                    }

                    continue;
                }

                fp t = numerator / denominator;

                if (denominator >= RVOMath.Zero)
                {
                    // Line i bounds line lineNo on the right.
                    tRight = math.min(tRight, t);
                }
                else
                {
                    // Line i bounds line lineNo on the left.
                    tLeft = math.max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                // Optimize direction.
                if (math.dot(optVelocity, lineNoPtr->direction) > RVOMath.Zero)
                {
                    // Take right extreme.
                    result = lineNoPtr->point + (tRight * lineNoPtr->direction);
                }
                else
                {
                    // Take left extreme.
                    result = lineNoPtr->point + (tLeft * lineNoPtr->direction);
                }
            }
            else
            {
                // Optimize closest point.
                fp t = math.dot(lineNoPtr->direction, optVelocity - lineNoPtr->point);

                if (t < tLeft)
                {
                    result = lineNoPtr->point + (tLeft * lineNoPtr->direction);
                }
                else if (t > tRight)
                {
                    result = lineNoPtr->point + (tRight * lineNoPtr->direction);
                }
                else
                {
                    result = lineNoPtr->point + (t * lineNoPtr->direction);
                }
            }

            return true;
        }

        /// <summary>
        /// Solves a two-dimensional linear program subject to linear constraints
        /// defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        /// <returns>The number of the line it fails on, and the number of lines if successful.</returns>
        private unsafe int LinearProgram2(
            Line* lines,
            int linesLength,
            fp radius,
            float2 optVelocity,
            bool directionOpt,
            ref float2 result)
        {
            if (directionOpt)
            {
                // Optimize direction. Note that the optimization velocity is of unit length in this case.
                result = optVelocity * radius;
            }
            else if (math.lengthsq(optVelocity) > RVOMath.Square(radius))
            {
                // Optimize closest point and outside circle.
                result = math.normalize(optVelocity) * radius;
            }
            else
            {
                // Optimize closest point and inside circle.
                result = optVelocity;
            }

            for (int i = 0; i < linesLength; ++i)
            {
                Line* lineI = lines + i;
                if (RVOMath.Det(lineI->direction, lineI->point - result) > RVOMath.Zero)
                {
                    // Result does not satisfy constraint i. Compute new optimal result.
                    float2 tempResult = result;
                    if (!this.LinearProgram1(
                        lines,
                        linesLength,
                        i,
                        radius,
                        optVelocity,
                        directionOpt,
                        ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return linesLength;
        }

        /// <summary>
        /// Solves a two-dimensional linear program subject to linear constraints
        /// defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="numObstLines">Count of obstacle lines.</param>
        /// <param name="beginLine">The line on which the 2-d linear program failed.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        private unsafe void LinearProgram3(
            Line* lines,
            int linesLength,
            int numObstLines,
            int beginLine,
            fp radius,
            ref float2 result)
        {
            fp distance = RVOMath.Zero;

            for (int i = beginLine; i < linesLength; ++i)
            {
                Line* lineI = lines + i;
                if (RVOMath.Det(lineI->direction, lineI->point - result) <= distance)
                {
                    continue;
                }

                // Result does not satisfy constraint of line i.
                UnsafeList<Line> projLines = new UnsafeList<Line>(numObstLines, Allocator.Temp);
                for (int ii = 0; ii < numObstLines; ++ii)
                {
                    projLines.Add(lines[ii]);
                }

                for (int j = numObstLines; j < i; ++j)
                {
                    Line* lineJ = lines + j;
                    float2 point;
                    float2 direction;

                    fp determinant = RVOMath.Det(lineI->direction, lineJ->direction);

                    if (math.abs(determinant) <= RVOMath.RVO_EPSILON)
                    {
                        // Line i and line j are parallel.
                        if (math.dot(lineI->direction, lineJ->direction) > RVOMath.Zero)
                        {
                            // Line i and line j point in the same direction.
                            continue;
                        }
                        else
                        {
                            // Line i and line j point in opposite direction.
                            point = weight * (lineI->point + lineJ->point);
                        }
                    }
                    else
                    {
                        point = lineI->point
                            + (RVOMath.Det(lineJ->direction, lineI->point - lineJ->point) / determinant * lineI->direction);
                    }

                    direction = math.normalize(lineJ->direction - lineI->direction);
                    Line line = new Line(point, direction);
                    projLines.Add(line);
                }

                float2 tempResult = result;
                if (this.LinearProgram2(
                    projLines.Ptr,
                    projLines.Length,
                    radius,
                    new float2(-lineI->direction.y, lineI->direction.x),
                    true,
                    ref result)
                    < projLines.Length)
                {
                    // This should in principle not happen. The result is by
                    // definition already in the feasible region of this
                    // linear program. If it fails, it is due to small
                    // floating point error, and the current result is kept.
                    result = tempResult;
                }

                distance = RVOMath.Det(lineI->direction, lineI->point - result);
            }
        }

        internal readonly struct Pair : IEquatable<Pair>
        {
            public readonly fp dist;
            public readonly int index;

            public Pair(fp dist, int index)
            {
                this.dist = dist;
                this.index = index;
            }

            public bool Equals(Pair other)
            {
                return this.dist.Equals(other.dist) && this.index == other.index;
            }

            public override bool Equals(object obj)
            {
                return obj is Pair other && this.Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    return (this.dist.GetHashCode() * 397) ^ this.index;
                }
            }
        }
    }
}
