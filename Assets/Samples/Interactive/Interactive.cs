// -----------------------------------------------------------------------
// <copyright file="Interactive.cs" company="AillieoTech">
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
    using Deterministic.FixedPoint;
    using float2 = Deterministic.FixedPoint.fp2;
    using math = Deterministic.FixedPoint.fixmath;
    using Random = Deterministic.FixedPoint.Random;
#else
    using Unity.Mathematics;
    using Random = RVO.RandomValue;
#endif

    internal class Interactive : MonoBehaviour
    {
        // Random number generator.
        private readonly Random random = new Random(123456);

        private Simulator simulator;

        private CustomSampler sampler;

        private List<Agent> agents;
        private List<DynamicObstacleData> obstacles;
        private int bounds;

        private enum TouchMode
        {
            Add = 0,
            Move = 1,
            Remove = 2,
            Toggle = 3,
        }

        private void SetupScenario()
        {
            // Specify the global time step of the simulation.
            this.simulator.SetTimeStep(FPValue.OneIn100 * 25);

            // Specify the default parameters for agents that are subsequently added.
            this.simulator.SetAgentDefaults(15, 10, 5, 5, 2, 2, new float2(0, 0));

            // Add agents, specifying their start position.
            this.agents = new List<Agent>();
            for (var i = 0; i < 5; ++i)
            {
                for (var j = 0; j < 5; ++j)
                {
                    Agent agent = this.simulator.AddAgent(new float2(55 + (i * 10), 55 + (j * 10)));
                    this.agents.Add(agent);

                    agent = this.simulator.AddAgent(new float2(-55 - (i * 10), 55 + (j * 10)));
                    this.agents.Add(agent);

                    agent = this.simulator.AddAgent(new float2(55 + (i * 10), -55 - (j * 10)));
                    this.agents.Add(agent);

                    agent = this.simulator.AddAgent(new float2(-55 - (i * 10), -55 - (j * 10)));
                    this.agents.Add(agent);
                }
            }

            this.obstacles = new List<DynamicObstacleData>();

            // Add (polygonal) obstacles, specifying their vertices in counterclockwise order.
            IList<float2> obstacle1 = new List<float2>
            {
                new float2(-10, 40),
                new float2(-40, 40),
                new float2(-40, 10),

                // new float2(-10, 10),
                new float2(-30, 10),
                new float2(-30, 30),
                new float2(-10, 30),
            };
            var obstacle1Id = this.simulator.AddObstacle(obstacle1);

            IList<float2> obstacle2 = new List<float2>
            {
                new float2(10, 40),

                // new float2(10, 10),
                new float2(10, 30),
                new float2(30, 30),
                new float2(30, 10),

                new float2(40, 10),
                new float2(40, 40),
            };
            var obstacle2Id = this.simulator.AddObstacle(obstacle2);

            IList<float2> obstacle3 = new List<float2>
            {
                new float2(10, -40),
                new float2(40, -40),
                new float2(40, -10),

                // new float2(10, -10),
                new float2(30, -10),
                new float2(30, -30),
                new float2(10, -30),
            };
            var obstacle3Id = this.simulator.AddObstacle(obstacle3);

            IList<float2> obstacle4 = new List<float2>
            {
                new float2(-10, -40),

                // new float2(-10, -10),
                new float2(-10, -30),
                new float2(-30, -30),
                new float2(-30, -10),

                new float2(-40, -10),
                new float2(-40, -40),
            };
            var obstacle4Id = this.simulator.AddObstacle(obstacle4);

            this.obstacles.Add(new DynamicObstacleData { points = obstacle1, active = true, obstacleId = obstacle1Id });
            this.obstacles.Add(new DynamicObstacleData { points = obstacle2, active = true, obstacleId = obstacle2Id });
            this.obstacles.Add(new DynamicObstacleData { points = obstacle3, active = true, obstacleId = obstacle3Id });
            this.obstacles.Add(new DynamicObstacleData { points = obstacle4, active = true, obstacleId = obstacle4Id });

            IList<float2> boundVerts = new List<float2>
            {
                new float2(-100, 100),
                new float2(100, 100),
                new float2(100, -100),
                new float2(-100, -100),
            };
            this.bounds = this.simulator.AddObstacle(boundVerts);
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            // Bounds.
            {
                var first = this.simulator.GetFirstObstacleVertexId(this.bounds);

                var current = first;

                Color gizmosBackup = Gizmos.color;
                Gizmos.color = Color.gray;
                while (true)
                {
                    var next = this.simulator.GetNextObstacleVertexId(current);

                    float2 p0 = this.simulator.GetObstacleVertex(current);
                    float2 p1 = this.simulator.GetObstacleVertex(next);

                    Gizmos.DrawLine(p0.AsVector2(), p1.AsVector2());

                    Gizmos.DrawSphere(p0.AsVector2(), 1f);

                    if (next == first)
                    {
                        break;
                    }

                    current = next;
                }

                Gizmos.color = gizmosBackup;
            }

            foreach (DynamicObstacleData obstacle in this.obstacles)
            {
                if (obstacle.active)
                {
                    var first = this.simulator.GetFirstObstacleVertexId(obstacle.obstacleId);

                    var current = first;

                    while (true)
                    {
                        var next = this.simulator.GetNextObstacleVertexId(current);

                        float2 p0 = this.simulator.GetObstacleVertex(current);
                        float2 p1 = this.simulator.GetObstacleVertex(next);

                        Gizmos.DrawLine(p0.AsVector2(), p1.AsVector2());

                        Gizmos.DrawSphere(p0.AsVector2(), 1f);

                        if (next == first)
                        {
                            break;
                        }

                        current = next;
                    }
                }
                else
                {
                    Color gizmosBackup = Gizmos.color;
                    Gizmos.color = new Color(1, 1, 1, 0.2f);

                    var total = obstacle.points.Count;
                    for (var i = 0; i < total; i++)
                    {
                        var next = (i + 1) % total;
                        Gizmos.DrawLine(obstacle.points[i].AsVector2(), obstacle.points[next].AsVector2());
                    }

                    Gizmos.color = gizmosBackup;
                }
            }

            foreach (Agent agent in this.agents)
            {
                float2 position = agent.position;
                Gizmos.DrawSphere(position.AsVector2(), 2);
            }
        }

        private void SetPreferredVelocities(float2 newGoal)
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            foreach (Agent agent in this.agents)
            {
                float2 goalVector = newGoal - agent.position;

                if (math.lengthsq(goalVector) > 1)
                {
                    goalVector = math.normalize(goalVector);
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
            if (!this.GetTouchPosition(out var isTouchBegan, out Vector3 position))
            {
                return;
            }

            Camera mainCam = Camera.main;
            if (mainCam == null)
            {
                return;
            }

            Vector3 worldPos = mainCam.ScreenToWorldPoint(position);
            float2 worldPos2d = ((Vector2)worldPos).AsFloat2();

            switch (this.GetTouchMode())
            {
                case TouchMode.Add:
                    if (isTouchBegan)
                    {
                        Agent agentId = this.simulator.AddAgent(worldPos2d);
                        this.agents.Add(agentId);
                    }

                    break;
                case TouchMode.Move:
                    this.sampler.Begin();
                    this.SetPreferredVelocities(worldPos2d);
                    this.simulator.DoStep();
                    this.simulator.EnsureCompleted();
                    this.sampler.End();
                    break;
                case TouchMode.Remove:
                    if (isTouchBegan)
                    {
                        var selected = new List<int>();
                        this.simulator.QueryAgent(worldPos2d, 2, selected);
                        if (selected.Count > 0)
                        {
                            var toRemove = selected[0];
                            if (this.simulator.TryGetAgent(toRemove, out Agent agent))
                            {
                                this.simulator.RemoveAgent(toRemove);
                                this.agents.Remove(agent);
                            }
                        }
                    }

                    break;
                case TouchMode.Toggle:
                    if (isTouchBegan)
                    {
                        for (var i = 0; i < this.obstacles.Count; ++i)
                        {
                            DynamicObstacleData data = this.obstacles[i];
                            if (!GeomUtils.PointInPolygon(worldPos2d, data.points))
                            {
                                continue;
                            }

                            var newActive = !data.active;

                            if (newActive)
                            {
                                this.simulator.EnsureCompleted();

                                var newId = this.simulator.AddObstacle(data.points);
                                if (newId != -1)
                                {
                                    data.obstacleId = newId;
                                    data.active = newActive;
                                }
                            }
                            else
                            {
                                this.simulator.EnsureCompleted();

                                if (this.simulator.RemoveObstacle(data.obstacleId))
                                {
                                    data.active = newActive;
                                }
                            }

                            this.obstacles[i] = data;

                            break;
                        }
                    }

                    break;
            }
        }

        private void OnDestroy()
        {
            this.simulator.Clear();

            this.simulator.Dispose();
        }

        private void OnGUI()
        {
            GUILayout.Label($"Agents:{this.simulator.GetNumAgents()}");
            GUILayout.Label($"FPS:{1f / Time.deltaTime}");

            GUILayout.Label("- Press mouse and hold to move;");
            GUILayout.Label("- Press A and click to add an agent;");
            GUILayout.Label("- Press R and click to remove an agent;");
            GUILayout.Label("- Press T and click to toggle visibility of an obstacle.");
        }

        private TouchMode GetTouchMode()
        {
            if (Input.GetKey(KeyCode.A))
            {
                return TouchMode.Add;
            }

            if (Input.GetKey(KeyCode.R))
            {
                return TouchMode.Remove;
            }

            if (Input.GetKey(KeyCode.T))
            {
                return TouchMode.Toggle;
            }

            return TouchMode.Move;
        }

        private bool GetTouchPosition(out bool touchBegan, out Vector3 position)
        {
            if (Input.touchSupported)
            {
                if (Input.touchCount > 0)
                {
                    Touch touch = Input.GetTouch(0);
                    touchBegan = touch.phase == TouchPhase.Began;
                    position = Input.GetTouch(0).position;
                    return true;
                }
            }
            else
            {
                if (Input.GetMouseButton(0))
                {
                    touchBegan = Input.GetMouseButtonDown(0);
                    position = Input.mousePosition;
                    return true;
                }
            }

            touchBegan = default;
            position = default;
            return false;
        }

        private struct DynamicObstacleData
        {
            public IList<float2> points;
            public bool active;
            public int obstacleId;
        }
    }
}
