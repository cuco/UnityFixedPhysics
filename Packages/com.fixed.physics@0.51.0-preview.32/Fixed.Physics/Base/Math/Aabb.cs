using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics.FixedPoint;
using Unity.Mathematics;

namespace Fixed.Physics
{
    // An axis aligned bounding box
    [DebuggerDisplay("{Min} - {Max}")]
    [Serializable]
    public struct Aabb
    {
        public fp3 Min;
        public fp3 Max;

        public fp3 Extents => Max - Min;
        public fp3 Center => (Max + Min) * fp.half;
        public bool IsValid => math.all(Min <= Max);

        // Create an empty, invalid AABB
        public static readonly Aabb Empty = new Aabb { Min = Math.Constants.Max3F, Max = Math.Constants.Min3F };

        public fp SurfaceArea
        {
            get
            {
                fp3 diff = Max - Min;
                return fp.two * fpmath.dot(diff, diff.yzx);
            }
        }

        public static Aabb Union(Aabb a, Aabb b)
        {
            a.Include(b);
            return a;
        }

        [DebuggerStepThrough]
        public void Intersect(Aabb aabb)
        {
            Min = fpmath.max(Min, aabb.Min);
            Max = fpmath.min(Max, aabb.Max);
        }

        [DebuggerStepThrough]
        public void Include(fp3 point)
        {
            Min = fpmath.min(Min, point);
            Max = fpmath.max(Max, point);
        }

        [DebuggerStepThrough]
        public void Include(Aabb aabb)
        {
            Min = fpmath.min(Min, aabb.Min);
            Max = fpmath.max(Max, aabb.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(fp3 point) => math.all(point >= Min & point <= Max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(Aabb aabb) => math.all((Min <= aabb.Min) & (Max >= aabb.Max));

        public void Expand(fp distance)
        {
            Min -= distance;
            Max += distance;
        }

        internal static Aabb CreateFromPoints(fp3x4 points)
        {
            Aabb aabb;
            aabb.Min = points.c0;
            aabb.Max = aabb.Min;

            aabb.Min = fpmath.min(aabb.Min, points.c1);
            aabb.Max = fpmath.max(aabb.Max, points.c1);

            aabb.Min = fpmath.min(aabb.Min, points.c2);
            aabb.Max = fpmath.max(aabb.Max, points.c2);

            aabb.Min = fpmath.min(aabb.Min, points.c3);
            aabb.Max = fpmath.max(aabb.Max, points.c3);

            return aabb;
        }

        public bool Overlaps(Aabb other)
        {
            return math.all(Max >= other.Min & Min <= other.Max);
        }

        /// <summary>
        /// Returns the closest point on the bounds of the AABB to the specified position.
        /// <param name="position">A target point in space.</param>
        /// </summary>
        public fp3 ClosestPoint(fp3 position)
        {
            return fpmath.min(Max, fpmath.max(Min, position));
        }
    }

    // Helper functions
    public static partial class Math
    {
        // Transform an AABB into another space, expanding it as needed.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Aabb TransformAabb(FpRigidTransform transform, Aabb aabb)
        {
            // Transforming an empty AABB results in NaNs!
            if (!aabb.IsValid)
            {
                return aabb;
            }

            fp3 halfExtentsInA = aabb.Extents * fp.half;
            fp3 x = fpmath.rotate(transform.rot, new fp3(halfExtentsInA.x, fp.zero, fp.zero));
            fp3 y = fpmath.rotate(transform.rot, new fp3(fp.zero, halfExtentsInA.y, fp.zero));
            fp3 z = fpmath.rotate(transform.rot, new fp3(fp.zero, fp.zero, halfExtentsInA.z));

            fp3 halfExtentsInB = fpmath.abs(x) + fpmath.abs(y) + fpmath.abs(z);
            fp3 centerInB = fpmath.transform(transform, aabb.Center);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }

        // Transform an AABB into another space, expanding it as needed.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Aabb TransformAabb(MTransform transform, Aabb aabb)
        {
            // Transforming an empty AABB results in NaNs!
            if (!aabb.IsValid)
            {
                return aabb;
            }

            fp3 halfExtentsInA = aabb.Extents * fp.half;
            fp3 transformedX = fpmath.abs(transform.Rotation.c0 * halfExtentsInA.x);
            fp3 transformedY = fpmath.abs(transform.Rotation.c1 * halfExtentsInA.y);
            fp3 transformedZ = fpmath.abs(transform.Rotation.c2 * halfExtentsInA.z);

            fp3 halfExtentsInB = transformedX + transformedY + transformedZ;
            fp3 centerInB = Math.Mul(transform, aabb.Center);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }
    }
}
