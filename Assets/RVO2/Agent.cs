
namespace RVO
{
#if RVO_FIXEDPOINT
    using fp = Deterministic.FixedPoint.fp;
    using float2 = Deterministic.FixedPoint.fp2;
#else
    using Unity.Mathematics;
    using fp = System.Single;
#endif

    /// <summary>
    /// Defines an agent in the simulation.
    /// </summary>
    public class Agent
    {
        public int id;

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

        public Agent()
        {

        }

        public Agent(AgentData agentData)
        {
            id = agentData.id;
            maxNeighbors = agentData.maxNeighbors;
            maxSpeed = agentData.maxSpeed;
            neighborDist = agentData.neighborDist;
            position = agentData.position;
            radius = agentData.radius;
            timeHorizon = agentData.timeHorizon;
            timeHorizonObst = agentData.timeHorizonObst;
            velocity = agentData.velocity;
            weight = agentData.weight;
        }
    }
}
