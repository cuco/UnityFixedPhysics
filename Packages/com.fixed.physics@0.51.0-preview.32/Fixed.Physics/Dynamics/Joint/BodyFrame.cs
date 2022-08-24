using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    /// <summary>
    /// A target in the space of a rigid body that will align with a corresponding target in the space of the other body to which it is constrained.
    /// </summary>
    public struct BodyFrame : IEquatable<BodyFrame>
    {
        /// <summary>
        /// The bind pose anchor or target position of the joint in the space of its rigid body.
        /// </summary>
        public fp3 Position;
        /// <summary>
        /// The bind pose orientation of the joint's x-axis in the space of its rigid body.
        /// </summary>
        public fp3 Axis;
        /// <summary>
        /// The bind pose orientation of the joint's y-axis in the space of its rigid body.
        /// </summary>
        public fp3 PerpendicularAxis;

        public BodyFrame(FpRigidTransform transform)
        {
            Position = transform.pos;
            var rotation = new fp3x3(transform.rot);
            Axis = rotation.c0;
            PerpendicularAxis = rotation.c1;
        }

        static readonly fp3 k_DefaultAxis = new fp3(fp.one, fp.zero, fp.zero);
        static readonly fp3 k_DefaultPerpendicular = new fp3(fp.zero, fp.one, fp.zero);

        public static readonly BodyFrame Identity =
            new BodyFrame { Axis = k_DefaultAxis, PerpendicularAxis = k_DefaultPerpendicular };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FpRigidTransform AsRigidTransform() => new FpRigidTransform(ValidateAxes(), Position);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal Math.MTransform AsMTransform() => new Math.MTransform(ValidateAxes(), Position);

        // slower than fpmath.orthonormalize(), this method replicates UnityEngine.Vector3.OrthoNormalize()
        // it is more robust if input Axis is not pre-normalized or frame is degenerate
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static fp3x3 OrthoNormalize(fp3 u, fp3 v)
        {
            var mag = fpmath.length(u);
            u = fpmath.select(k_DefaultAxis, u / mag, mag > Math.Constants.UnityEpsilon);

            v -= fpmath.dot(u, v) * u;
            mag = fpmath.length(v);
            v = fpmath.select(OrthoNormalVectorFast(u), v / mag, mag > Math.Constants.UnityEpsilon);

            return new fp3x3(u, v, fpmath.cross(u, v));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static fp3 OrthoNormalVectorFast(fp3 n)
        {
            fp kRcpSqrt2 = fp.FromRaw(0x3f3504f3);
            var usePlaneYZ = fpmath.abs(n.z) > kRcpSqrt2;
            var a = fpmath.select(fpmath.dot(n.xy, n.xy), fpmath.dot(n.yz, n.yz), usePlaneYZ);
            var k = fpmath.rcp(fpmath.sqrt(a));
            return fpmath.select(new fp3(-n.y * k, n.x * k, fp.zero), new fp3(fp.zero, -n.z * k, n.y * k), usePlaneYZ);
        }

        internal fp3x3 ValidateAxes()
        {
            // TODO: fpmath.orthonormalize() does not guarantee an ortho-normalized result when Axis is non-normalized
            var sqrMag = fpmath.lengthsq(Axis);
            fp kEpsilon = Math.Constants.UnityEpsilon;
            return sqrMag >= fp.one - kEpsilon && sqrMag <= fp.one + kEpsilon
                ? fpmath.orthonormalize(new fp3x3(Axis, PerpendicularAxis, default))
                : OrthoNormalize(Axis, PerpendicularAxis);
        }

        public bool Equals(BodyFrame other) =>
            Position.Equals(other.Position)
            && Axis.Equals(other.Axis)
            && PerpendicularAxis.Equals(other.PerpendicularAxis);

        public override bool Equals(object obj) => obj is BodyFrame other && Equals(other);

        public override int GetHashCode() => unchecked((int)fpmath.hash(new fp3x3(Position, Axis, PerpendicularAxis)));

        public override string ToString() =>
            $"BodyFrame {{ Axis = {Axis}, PerpendicularAxis = {PerpendicularAxis}, Position = {Position} }}";

        public static implicit operator BodyFrame(FpRigidTransform transform) => new BodyFrame(transform);
    }
}
