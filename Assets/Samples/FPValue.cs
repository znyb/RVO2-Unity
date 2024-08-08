
using UnityEngine;

namespace RVO
{

    internal class FPValue
    {
#if RVO_FIXEDPOINT
        public static Deterministic.FixedPoint.fp Zero = Deterministic.FixedPoint.fp._0;
        public static Deterministic.FixedPoint.fp One = Deterministic.FixedPoint.fp._1;
        public static Deterministic.FixedPoint.fp Half = Deterministic.FixedPoint.fp._0_50;
        public static Deterministic.FixedPoint.fp OneIn100 = One / 100;
        public static Deterministic.FixedPoint.fp OneIn1000 = One / 1000;
        public static Deterministic.FixedPoint.fp OneIn10000 = One / 10000;
        public static Deterministic.FixedPoint.fp PI = Deterministic.FixedPoint.fp.pi;
#else
        public static float Zero = 0f;
        public static float One = 1f;
        public static float Half = 0.5f;
        public static float OneIn100 = One / 100;
        public static float OneIn1000 = One / 1000;
        public static float OneIn10000 = One / 10000;
        public static float PI = Mathf.PI;
#endif
    }
}
