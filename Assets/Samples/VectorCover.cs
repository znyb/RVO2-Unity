using Deterministic.FixedPoint;
using Unity.Mathematics;
using UnityEngine;

namespace RVO
{
    internal static class VectorCover
    {
        public static Vector2 AsVector2(this float2 value)
        {
            return (Vector2)value;
        }

        public static Vector2 AsVector2(this fp2 value)
        {
            return new Vector2(value.x.AsFloat, value.y.AsFloat);
        }

#if RVO_FIXEDPOINT
        public static fp2 AsFloat2(this Vector2 value)
        {
            return new fp2(fp.ParseUnsafe(value.x), fp.ParseUnsafe(value.y));
        }
#else
        public static float2 AsFloat2(this Vector2 value)
        {
            return new float2(value.x, value.y);
        }
#endif

    }
}
