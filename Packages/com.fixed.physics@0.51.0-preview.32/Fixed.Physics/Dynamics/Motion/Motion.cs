using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    // Describes how mass is distributed within an object
    // Represented by a transformed box inertia of unit mass.
    public struct MassDistribution
    {
        // The center of mass and the orientation to principal axis space
        public FpRigidTransform Transform;

        // Diagonalized inertia tensor for a unit mass
        public fp3 InertiaTensor;

        // Get the inertia as a 3x3 matrix
        public fp3x3 InertiaMatrix
        {
            get
            {
                var r = new fp3x3(Transform.rot);
                var r2 = new fp3x3(InertiaTensor.x * r.c0, InertiaTensor.y * r.c1, InertiaTensor.z * r.c2);
                return fpmath.mul(r2, fpmath.inverse(r));
            }
        }
    }

    // The mass properties of an object.
    public struct MassProperties
    {
        // The distribution of a unit mass throughout the object.
        public MassDistribution MassDistribution;

        // The volume of the object.
        public fp Volume;

        // Upper bound on the rate of change of the object's extent in any direction,
        // with respect to angular speed around its center of mass.
        // Used to determine how much to expand a rigid body's AABB to enclose its swept volume.
        public fp AngularExpansionFactor;

        // The mass properties of a unit sphere
        public static readonly MassProperties UnitSphere = new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = FpRigidTransform.identity,
                InertiaTensor = new fp3(new fp(0, 2, 5))
            },
            Volume = new fp(1, 1, 3) * fp.Pi, //(4.0f / 3.0f) * (fp)fpmath.PI,
            AngularExpansionFactor = fp.zero
        };
    }

    // A dynamic rigid body's "cold" motion data, used during Jacobian building and integration.
    public struct MotionData
    {
        // Center of mass and inertia orientation in world space
        public FpRigidTransform WorldFromMotion;

        // Center of mass and inertia orientation in rigid body space
        public FpRigidTransform BodyFromMotion;

        // Damping applied to the motion during each simulation step
        public fp LinearDamping;
        public fp AngularDamping;

        public static readonly MotionData Zero = new MotionData
        {
            WorldFromMotion = FpRigidTransform.identity,
            BodyFromMotion = FpRigidTransform.identity,
            LinearDamping = fp.zero,
            AngularDamping = fp.zero
        };
    }

    // A dynamic rigid body's "hot" motion data, used during solving.
    public struct MotionVelocity
    {
        public fp3 LinearVelocity;   // world space
        public fp3 AngularVelocity;  // motion space
        public fp3 InverseInertia;
        public fp InverseMass;
        public fp AngularExpansionFactor;

        // A multiplier applied to the simulation step's gravity vector
        public fp GravityFactor;

        public bool HasInfiniteMass => InverseMass == fp.zero;
        public bool HasInfiniteInertia => !fpmath.any(InverseInertia);
        public bool IsKinematic => HasInfiniteMass && HasInfiniteInertia;

        public static readonly MotionVelocity Zero = new MotionVelocity
        {
            LinearVelocity = new fp3(0),
            AngularVelocity = new fp3(0),
            InverseInertia = new fp3(0),
            InverseMass = fp.zero,
            AngularExpansionFactor = fp.zero,
            GravityFactor = fp.zero
        };

        // Apply a linear impulse (in world space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(fp3 impulse)
        {
            LinearVelocity += impulse * InverseMass;
        }

        // Apply an angular impulse (in motion space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(fp3 impulse)
        {
            AngularVelocity += impulse * InverseInertia;
        }

        // Calculate the distances by which to expand collision tolerances based on the speed of the object.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal MotionExpansion CalculateExpansion(fp timeStep) => new MotionExpansion
        {
            Linear = LinearVelocity * timeStep,
            // fpmath.length(AngularVelocity) * timeStep is conservative approximation of sin((fpmath.length(AngularVelocity) * timeStep)
            Uniform = fpmath.min(fpmath.length(AngularVelocity) * timeStep * AngularExpansionFactor, AngularExpansionFactor)
        };
    }

    // Provides an upper bound on change in a body's extents in any direction during a step.
    // Used to determine how far away from the body to look for collisions.
    struct MotionExpansion
    {
        public fp3 Linear;   // how far to look ahead of the object
        public fp Uniform;   // how far to look around the object

        public fp MaxDistance => fpmath.length(Linear) + Uniform;

        public static readonly MotionExpansion Zero = new MotionExpansion
        {
            Linear = new fp3(fp.zero),
            Uniform = fp.zero
        };

        // Expand an AABB
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Aabb ExpandAabb(Aabb aabb) => new Aabb
        {
            Max = fpmath.max(aabb.Max, aabb.Max + Linear) + Uniform,
            Min = fpmath.min(aabb.Min, aabb.Min + Linear) - Uniform
        };
    }

    // A linear and angular velocity
    public struct Velocity
    {
        public fp3 Linear;   // world space
        public fp3 Angular;  // motion space

        public static readonly Velocity Zero = new Velocity
        {
            Linear = new fp3(fp.zero),
            Angular = new fp3(fp.zero)
        };
    }
}
