// -----------------------------------------------------------------------
// <copyright file="Dynamic.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Collections.Generic;
    using UnityEngine;
    using UnityEngine.Profiling;

#if RVO_FIXEDPOINT
    using fp = Deterministic.FixedPoint.fp;
    using float2 = Deterministic.FixedPoint.fp2;
    using math = Deterministic.FixedPoint.fixmath;
    using Random = Deterministic.FixedPoint.Random;
#else
    using Unity.Mathematics;
    using Random = RVO.RandomValue;
#endif

    internal class Dynamic : MonoBehaviour
    {
        // Random number generator.
        private readonly Random random = new Random(123456);

        private Simulator simulator;

        private CustomSampler sampler;

        private List<RepeatedTask> repeatedTasks;
        private Dictionary<Agent, GoalAndColor> agentData;
        private List<int> obstacles;

        private void SetupScenario()
        {
            // Specify the global time step of the simulation.
            this.simulator.SetTimeStep(FPValue.OneIn100 * 25);

            // Specify the default parameters for agents that are subsequently added.
            this.simulator.SetAgentDefaults(15, 10, 5, 5, 2, FPValue.Half, new float2(0, 0));

            this.repeatedTasks = new List<RepeatedTask>();
            this.agentData = new Dictionary<Agent, GoalAndColor>();

            // Agents spawning.
            for (var i = 0; i < 3; ++i)
            {
                for (var j = 0; j < 3; ++j)
                {
                    var position = new float2(60 + (i * 10), 60 + (j * 10));
                    float2 goal = -position;
                    Color color = Color.red;
                    var task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 0 };
                    this.repeatedTasks.Add(task);

                    position = new float2(-60 - (i * 10), 60 + (j * 10));
                    goal = -position;
                    color = Color.green;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 250 };
                    this.repeatedTasks.Add(task);

                    position = new float2(60 + (i * 10), -60 - (j * 10));
                    goal = -position;
                    color = Color.blue;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 500 };
                    this.repeatedTasks.Add(task);

                    position = new float2(-60 - (i * 10), -60 - (j * 10));
                    goal = -position;
                    color = Color.white;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 750 };
                    this.repeatedTasks.Add(task);
                }
            }

            // Obstacles.
            {
                this.obstacles = new List<int>();

                // Add (polygonal) obstacles, specifying their vertices in counterclockwise order.
                IList<float2> obstacle1 = new List<float2>
                {
                    new float2(-35, 35),
                    new float2(-45, 15),
                    new float2(-35, 15),
                    new float2(-15, 35),
                    new float2(-15, 45),
                };
                var task = new ObstacleSwitchTask(obstacle1, this.simulator, this.obstacles) { interval = 600, frameCounter = -300 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle2 = new List<float2>
                {
                    new float2(35, 35),
                    new float2(15, 45),
                    new float2(15, 35),
                    new float2(35, 15),
                    new float2(45, 15),
                };
                task = new ObstacleSwitchTask(obstacle2, this.simulator, this.obstacles) { interval = 600, frameCounter = 0 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle3 = new List<float2>
                {
                    new float2(35, -35),
                    new float2(45, -15),
                    new float2(35, -15),
                    new float2(15, -35),
                    new float2(15, -45),
                };
                task = new ObstacleSwitchTask(obstacle3, this.simulator, this.obstacles) { interval = 600, frameCounter = 300 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle4 = new List<float2>
                {
                    new float2(-35, -35),
                    new float2(-15, -45),
                    new float2(-15, -35),
                    new float2(-35, -15),
                    new float2(-45, -15),
                };
                task = new ObstacleSwitchTask(obstacle4, this.simulator, this.obstacles) { interval = 600, frameCounter = 600 };
                this.repeatedTasks.Add(task);
            }

            // Agents destroy.
            this.repeatedTasks.Add(new AgentCleanupTask(this.simulator, this.agentData) { interval = 5, frameCounter = 0 });
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            foreach (var obstacle in this.obstacles)
            {
                var first = this.simulator.GetFirstObstacleVertexId(obstacle);

                var current = first;

                while (true)
                {
                    var next = this.simulator.GetNextObstacleVertexId(current);

                    float2 p0 = this.simulator.GetObstacleVertex(current);
                    float2 p1 = this.simulator.GetObstacleVertex(next);

                    Gizmos.DrawLine(p0.AsVector2(), p1.AsVector2());

                    if (next == first)
                    {
                        break;
                    }

                    current = next;
                }
            }

            Color gizmosBackup = Gizmos.color;

            foreach (KeyValuePair<Agent, GoalAndColor> pair in this.agentData)
            {
                Agent agent = pair.Key;
                float2 position = agent.position;
                Gizmos.color = pair.Value.color;
                Gizmos.DrawSphere(position.AsVector2(), 2);
            }

            Gizmos.color = gizmosBackup;
        }

        private void SetPreferredVelocities()
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            foreach (KeyValuePair<Agent, GoalAndColor> pair in this.agentData)
            {
                Agent agent = pair.Key;
                float2 goal = pair.Value.goal;
                float2 goalVector = goal - agent.position;

                if (math.lengthsq(goalVector) > FPValue.OneIn100)
                {
                    goalVector = math.normalize(goalVector);
                    goalVector *= FPValue.Half;
                }

                agent.prefVelocity = goalVector;

                // Perturb a little to avoid deadlocks due to perfect symmetry.
                var angle = random.NextDirection2D();
                var dist = random.NextFp(FPValue.OneIn100);

                agent.prefVelocity += dist * angle;
            }
        }

        private void Start()
        {
            EditorUtils.DrawGizmosSceneView(true);

            this.simulator = new Simulator();

            // Set up the scenario.
            this.SetupScenario();

            this.sampler = CustomSampler.Create("RVO update", false);
        }

        private void Update()
        {
            this.simulator.EnsureCompleted();

            foreach (RepeatedTask task in this.repeatedTasks)
            {
                task.frameCounter++;
                if (task.frameCounter >= task.interval)
                {
                    task.frameCounter -= task.interval;
                    task.Execute();
                }
            }

            this.SetPreferredVelocities();

            this.sampler.Begin();
            this.simulator.DoStep();
            this.sampler.End();
        }

        private void OnDestroy()
        {
            this.simulator.Clear();

            this.simulator.Dispose();
        }

        private readonly struct GoalAndColor
        {
            internal readonly float2 goal;
            internal readonly Color color;

            internal GoalAndColor(float2 goal, Color color)
            {
                this.goal = goal;
                this.color = color;
            }
        }

        private abstract class RepeatedTask
        {
            internal int frameCounter;
            internal int interval;

            internal abstract void Execute();
        }

        private class SpawnTask : RepeatedTask
        {
            private readonly float2 position;
            private readonly float2 goal;
            private readonly Color color;
            private readonly Simulator simulator;
            private readonly Dictionary<Agent, GoalAndColor> agentData;

            internal SpawnTask(float2 position, float2 goal, Color color, Simulator simulator, Dictionary<Agent, GoalAndColor> agentData)
            {
                this.position = position;
                this.goal = goal;
                this.simulator = simulator;
                this.agentData = agentData;
                this.color = color;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                Agent agentId = this.simulator.AddAgent(this.position);
                this.agentData.Add(agentId, new GoalAndColor(this.goal, this.color));
            }
        }

        private class AgentCleanupTask : RepeatedTask
        {
            private readonly Simulator simulator;
            private readonly Dictionary<Agent, GoalAndColor> agentData;
            private readonly List<Agent> buffer = new List<Agent>();

            internal AgentCleanupTask(Simulator simulator, Dictionary<Agent, GoalAndColor> agentData)
            {
                this.simulator = simulator;
                this.agentData = agentData;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                foreach (KeyValuePair<Agent, GoalAndColor> pair in this.agentData)
                {
                    Agent agent = pair.Key;
                    float2 goal = pair.Value.goal;
                    if (math.lengthsq(agent.position - goal) <= agent.radius * agent.radius)
                    {
                        this.buffer.Add(agent);
                        this.simulator.RemoveAgent(agent);
                    }
                }


                this.simulator.EnsureCompleted();

                foreach (Agent agent in this.buffer)
                {
                    this.agentData.Remove(agent);
                }

                this.buffer.Clear();
            }
        }

        private class ObstacleSwitchTask : RepeatedTask
        {
            private readonly IList<float2> points;

            private readonly Simulator simulator;
            private readonly List<int> obstacles;
            private int obstacleId = -1;

            internal ObstacleSwitchTask(IList<float2> points, Simulator simulator, List<int> obstacles)
            {
                this.points = points;
                this.simulator = simulator;
                this.obstacles = obstacles;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                if (this.obstacleId == -1)
                {
                    this.obstacleId = this.simulator.AddObstacle(this.points);
                    this.obstacles.Add(this.obstacleId);
                }
                else
                {
                    this.simulator.RemoveObstacle(this.obstacleId);
                    this.obstacles.Remove(this.obstacleId);
                    this.obstacleId = -1;
                }
            }
        }
    }
}
