// -----------------------------------------------------------------------
// <copyright file="Circle.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Circle.cs
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

/*
 * Example file showing a demo with 250 agents initially positioned evenly
 * distributed on a circle attempting to move to the antipodal position on the
 * circle.
 */

namespace RVO
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using UnityEngine;

#if RVO_FIXEDPOINT
    using fp = Deterministic.FixedPoint.fp;
    using float2 = Deterministic.FixedPoint.fp2;
    using math = Deterministic.FixedPoint.fixmath;
#else
    using Unity.Mathematics;
#endif

    internal class Circle : MonoBehaviour
    {
        // Store the goals of the agents.
        private Dictionary<Agent, float2> goals;

        private Simulator simulator;

        private void Start()
        {
            EditorUtils.DrawGizmosSceneView(true);

            this.simulator = new Simulator();

            this.StartCoroutine(this.Main());
        }

        private void SetupScenario()
        {
            this.goals = new Dictionary<Agent, float2>();

            // Specify the global time step of the simulation.
            this.simulator.SetTimeStep(FPValue.OneIn100 * 25);

            // Specify the default parameters for agents that are subsequently added.
            this.simulator.SetAgentDefaults(15, 10, 10, 10, FPValue.OneIn100 * 150, 2, new float2(0, 0));

            // Add agents, specifying their start position, and store their
            // goals on the opposite side of the environment.
            for (var i = 0; i < 250; ++i)
            {
                Agent agent = this.simulator.AddAgent(200 *
                    new float2(
                        math.cos(i * 2 * FPValue.PI / 250),
                        math.sin(i * 2 * FPValue.PI / 250)));
                float2 goal = -agent.position;
                this.goals.Add(agent, goal);
            }
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            foreach (KeyValuePair<Agent, float2> pair in this.goals)
            {
                Agent agent = pair.Key;
                float2 position = agent.position;
                Gizmos.DrawSphere(position.AsVector2(), 1.5f);
            }
        }

        private void SetPreferredVelocities()
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            foreach (KeyValuePair<Agent, float2> pair in this.goals)
            {
                Agent agent = pair.Key;
                float2 goal = pair.Value;
                float2 goalVector = goal - agent.position;

                if (math.lengthsq(goalVector) > 1)
                {
                    goalVector = math.normalize(goalVector);
                }

                agent.prefVelocity = goalVector;
            }
        }

        private bool ReachedGoal()
        {
            // Check if all agents have reached their goals.
            foreach (KeyValuePair<Agent, float2> pair in this.goals)
            {
                Agent agent = pair.Key;
                float2 goal = pair.Value;
                if (math.lengthsq(agent.position - goal) > agent.radius * agent.radius)
                {
                    return false;
                }
            }

            return true;
        }

        private IEnumerator Main()
        {
            do
            {
                // Set up the scenario.
                this.SetupScenario();

                // Perform (and manipulate) the simulation.
                do
                {
                    this.SetPreferredVelocities();

                    this.simulator.DoStep();

                    yield return null;

                    this.simulator.EnsureCompleted();
                }
                while (!this.ReachedGoal());

                yield return new WaitForSeconds(1);

                this.simulator.Clear();
            }
            while (true);
        }

        private void OnDestroy()
        {
            this.simulator.Clear();
            this.simulator.Dispose();
        }
    }
}
