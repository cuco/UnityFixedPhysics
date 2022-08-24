using System;
using Unity.Mathematics.FixedPoint;
using Unity.Mathematics;

namespace Fixed.Physics
{
    public static partial class Math
    {
        /// <summary>
        /// Range of possible values for some constrained parameter.
        /// </summary>
        public struct FloatRange : IEquatable<FloatRange>
        {
            public fp Min;
            public fp Max;

            public FloatRange(fp min, fp max)
            {
                Min = min;
                Max = max;
            }

            public fp Mid => fpmath.lerp(Min, Max, fp.half);

            public bool Equals(FloatRange other) => Min.Equals(other.Min) && Max.Equals(other.Max);

            public override bool Equals(object obj) => obj is FloatRange other && Equals(other);

            public override int GetHashCode() => unchecked((int)fpmath.hash(new fp2(Min, Max)));

            public static implicit operator fp2(FloatRange range) => new fp2(range.Min, range.Max);

            public static implicit operator FloatRange(fp2 f) => new FloatRange { Min = f.x, Max = f.y };

            public override string ToString() => $"FloatRange {{ Min = {Min}, Max = {Max} }}";

            /// <summary>
            /// Returns a sorted copy of this instance.
            /// </summary>
            /// <returns>A copy of this instance, where <see cref="Min"/> is the lesser of <see cref="Min"/> and <see cref="Max"/>, and <see cref="Max"/> is the greater of the two.</returns>
            public FloatRange Sorted() => fpmath.select(this, ((fp2)this).yx, Min > Max);
        }
    }
}
