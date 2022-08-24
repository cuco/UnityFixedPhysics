using System.Runtime.CompilerServices;
using System.Diagnostics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    // A plane described by a normal and a distance from the origin
    [DebuggerDisplay("{Normal}, {Distance}")]
    public struct Plane
    {
        private fp4 m_NormalAndDistance;

        public fp3 Normal
        {
            get => m_NormalAndDistance.xyz;
            set => m_NormalAndDistance.xyz = value;
        }

        public fp Distance
        {
            get => m_NormalAndDistance.w;
            set => m_NormalAndDistance.w = value;
        }

        // Returns the distance from the point to the plane, positive if the point is on the side of
        // the plane on which the plane normal points, zero if the point is on the plane, negative otherwise.
        public fp SignedDistanceToPoint(fp3 point)
        {
            return Math.Dotxyz1(m_NormalAndDistance, point);
        }

        // Returns the closest point on the plane to the input point.
        public fp3 Projection(fp3 point)
        {
            return point - Normal * SignedDistanceToPoint(point);
        }

        public Plane Flipped => new Plane { m_NormalAndDistance = -m_NormalAndDistance };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(fp3 normal, fp distance)
        {
            m_NormalAndDistance = new fp4(normal, distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fp4(Plane plane) => plane.m_NormalAndDistance;
    }

    // Helper functions
    public static partial class Math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane PlaneFromDirection(fp3 origin, fp3 direction)
        {
            fp3 normal = fpmath.normalize(direction);
            return new Plane(normal, -fpmath.dot(normal, origin));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane PlaneFromTwoEdges(fp3 origin, fp3 edgeA, fp3 edgeB)
        {
            return PlaneFromDirection(origin, fpmath.cross(edgeA, edgeB));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane TransformPlane(FpRigidTransform transform, Plane plane)
        {
            fp3 normal = fpmath.rotate(transform.rot, plane.Normal);
            return new Plane(normal, plane.Distance - fpmath.dot(normal, transform.pos));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane TransformPlane(MTransform transform, Plane plane)
        {
            fp3 normal = fpmath.mul(transform.Rotation, plane.Normal);
            return new Plane(normal, plane.Distance - fpmath.dot(normal, transform.Translation));
        }
    }
}
