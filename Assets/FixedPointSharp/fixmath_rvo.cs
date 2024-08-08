
using System.Runtime.CompilerServices;

namespace Deterministic.FixedPoint {
    public partial struct fixmath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp dot(fp2 a, fp2 b)
        {
            a.x.value = ((a.x.value * b.x.value) >> fixlut.PRECISION) + ((a.y.value * b.y.value) >> fixlut.PRECISION);
            return a.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp lengthsq(fp2 v)
        {
            v.x.value =
                ((v.x.value * v.x.value) >> fixlut.PRECISION) +
                ((v.y.value * v.y.value) >> fixlut.PRECISION);

            if((v.x != 0 ||  v.y != 0) && v.x == 0) 
            {
                v.x = fp.epsilon;
            }
            return v.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp2 normalize(fp2 v)
        {
            fp2 v1 = v;
            fp m = default;
            fp r;

            r.value =
                ((v1.x.value * v1.x.value) >> fixlut.PRECISION) +
                ((v1.y.value * v1.y.value) >> fixlut.PRECISION);
            fp r1;

            if (r.value == 0)
            {
                r1.value = 0;
            }
            else
            {
                var b = (r.value >> 1) + 1L;
                var c = (b + (r.value / b)) >> 1;

                while (c < b)
                {
                    b = c;
                    c = (b + (r.value / b)) >> 1;
                }

                r1.value = b << (fixlut.PRECISION >> 1);
            }

            m = r1;

            if (m.value <= fp.epsilon.value)
            {
                v1 = default;
            }
            else
            {
                v1.x.value = ((v1.x.value << fixlut.PRECISION) / m.value);
                v1.y.value = ((v1.y.value << fixlut.PRECISION) / m.value);
            }

            return v1;
        }

        public static fp sqrt(fp num)
        {
            fp r;

            if (num.value == 0)
            {
                r.value = 0;
            }
            else
            {
                var b = (num.value >> 1) + 1L;
                var c = (b + (num.value / b)) >> 1;

                while (c < b)
                {
                    b = c;
                    c = (b + (num.value / b)) >> 1;
                }

                r.value = b << (fixlut.PRECISION >> 1);
            }

            return r;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp abs(fp num)
        {
            return new fp(num.value < 0 ? -num.value : num.value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp min(fp a, fp b)
        {
            return a.value < b.value ? a : b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp max(fp a, fp b)
        {
            return a.value > b.value ? a : b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp distancesq(fp2 a, fp2 b)
        {
            var x = a.x.value - b.x.value;
            var z = a.y.value - b.y.value;

            a.x.value = ((x * x) >> fixlut.PRECISION) + ((z * z) >> fixlut.PRECISION);
            return a.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp length(fp2 v)
        {
            fp r;

            r.value =
                ((v.x.value * v.x.value) >> fixlut.PRECISION) +
                ((v.y.value * v.y.value) >> fixlut.PRECISION);
            fp r1;

            if (r.value == 0)
            {
                r1.value = 0;
            }
            else
            {
                var b = (r.value >> 1) + 1L;
                var c = (b + (r.value / b)) >> 1;

                while (c < b)
                {
                    b = c;
                    c = (b + (r.value / b)) >> 1;
                }

                r1.value = b << (fixlut.PRECISION >> 1);
            }

            return r1;
        }


        /// <param name="num">Angle in radians</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp cos(fp num)
        {
            num.value %= fp.pi2.value;
            num *= fp.one_div_pi2;
            return new fp(fixlut.cos(num.value));
        }

        /// <param name="num">Angle in radians</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp sin(fp num)
        {
            num.value %= fp.pi2.value;
            num *= fp.one_div_pi2;
            var raw = fixlut.sin(num.value);
            fp result;
            result.value = raw;
            return result;
        }

    }
}
