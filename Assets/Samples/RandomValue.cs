
namespace RVO
{
    using Unity.Mathematics;

    internal class RandomValue
    {
        Random random;

        public RandomValue(uint seed)
        {
            random = new Random(seed);
        }

        public float NextFp()
        {
            return random.NextFloat();
        }

        public float NextFp(float max)
        {
            return random.NextFloat(max);
        }

        public float2 NextDirection2D()
        {
            return random.NextFloat2Direction();
        }
    }
}
