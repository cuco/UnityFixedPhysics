using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics.FixedPoint;
using UnityEngine.Assertions;

namespace Fixed.Physics
{
    public enum JacobianType : byte
    {
        // Contact Jacobians
        Contact,
        Trigger,

        // Joint Jacobians
        LinearLimit,
        AngularLimit1D,
        AngularLimit2D,
        AngularLimit3D
    }

    // Flags which enable optional Jacobian behaviors
    [Flags]
    public enum JacobianFlags : byte
    {
        // These flags apply to all Jacobians
        Disabled = 1 << 0,
        EnableMassFactors = 1 << 1,
        UserFlag0 = 1 << 2,
        UserFlag1 = 1 << 3,
        UserFlag2 = 1 << 4,

        // These flags apply only to contact Jacobians
        IsTrigger = 1 << 5,
        EnableCollisionEvents = 1 << 6,
        EnableSurfaceVelocity = 1 << 7
    }

    // Jacobian header, first part of each Jacobian in the stream
    struct JacobianHeader
    {
        public BodyIndexPair BodyPair { get; internal set; }
        public JacobianType Type { get; internal set; }
        public JacobianFlags Flags { get; internal set; }

        // Whether the Jacobian should be solved or not
        public bool Enabled
        {
            get => ((Flags & JacobianFlags.Disabled) == 0);
            set => Flags = value ? (Flags & ~JacobianFlags.Disabled) : (Flags | JacobianFlags.Disabled);
        }

        // Whether the Jacobian contains manifold data for collision events or not
        public bool HasContactManifold => (Flags & JacobianFlags.EnableCollisionEvents) != 0;

        // Collider keys for the collision events
        public ColliderKeyPair ColliderKeys
        {
            get => HasContactManifold? AccessColliderKeys() : ColliderKeyPair.Empty;
            set
            {
                if (HasContactManifold)
                    AccessColliderKeys() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have collision events enabled");
            }
        }

        // Overrides for the mass properties of the pair of bodies
        public bool HasMassFactors => (Flags & JacobianFlags.EnableMassFactors) != 0;
        public MassFactors MassFactors
        {
            get => HasMassFactors? AccessMassFactors() : MassFactors.Default;
            set
            {
                if (HasMassFactors)
                    AccessMassFactors() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have mass factors enabled");
            }
        }

        // The surface velocity to apply to contact points
        public bool HasSurfaceVelocity => (Flags & JacobianFlags.EnableSurfaceVelocity) != 0;
        public SurfaceVelocity SurfaceVelocity
        {
            get => HasSurfaceVelocity? AccessSurfaceVelocity() : new SurfaceVelocity();
            set
            {
                if (HasSurfaceVelocity)
                    AccessSurfaceVelocity() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have surface velocity enabled");
            }
        }

        // Solve the Jacobian
        public void Solve([NoAlias] ref MotionVelocity velocityA, [NoAlias] ref MotionVelocity velocityB, Solver.StepInput stepInput,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter, [NoAlias] ref NativeStream.Writer triggerEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            if (Enabled)
            {
                switch (Type)
                {
                    case JacobianType.Contact:
                        AccessBaseJacobian<ContactJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                            enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
                        break;
                    case JacobianType.Trigger:
                        AccessBaseJacobian<TriggerJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref triggerEventsWriter);
                        break;
                    case JacobianType.LinearLimit:
                        AccessBaseJacobian<LinearLimitJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit1D:
                        AccessBaseJacobian<AngularLimit1DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit2D:
                        AccessBaseJacobian<AngularLimit2DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit3D:
                        AccessBaseJacobian<AngularLimit3DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        return;
                }
            }
        }

        #region Helpers

        public static int CalculateSize(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return UnsafeUtility.SizeOf<JacobianHeader>() +
                SizeOfBaseJacobian(type) + SizeOfModifierData(type, flags) +
                numContactPoints * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                SizeOfContactPointData(type, flags, numContactPoints);
        }

        private static int SizeOfColliderKeys(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<ColliderKeyPair>() : 0;
        }

        private static int SizeOfEntityPair(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<EntityPair>() : 0;
        }

        private static int SizeOfSurfaceVelocity(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableSurfaceVelocity) != 0) ?
                UnsafeUtility.SizeOf<SurfaceVelocity>() : 0;
        }

        private static int SizeOfMassFactors(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableMassFactors) != 0) ?
                UnsafeUtility.SizeOf<MassFactors>() : 0;
        }

        private static int SizeOfModifierData(JacobianType type, JacobianFlags flags)
        {
            return SizeOfColliderKeys(type, flags) + SizeOfEntityPair(type, flags) + SizeOfSurfaceVelocity(type, flags) +
                SizeOfMassFactors(type, flags);
        }

        private static int SizeOfContactPointData(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                numContactPoints * UnsafeUtility.SizeOf<ContactPoint>() : 0;
        }

        private static int SizeOfBaseJacobian(JacobianType type)
        {
            switch (type)
            {
                case JacobianType.Contact:
                    return UnsafeUtility.SizeOf<ContactJacobian>();
                case JacobianType.Trigger:
                    return UnsafeUtility.SizeOf<TriggerJacobian>();
                case JacobianType.LinearLimit:
                    return UnsafeUtility.SizeOf<LinearLimitJacobian>();
                case JacobianType.AngularLimit1D:
                    return UnsafeUtility.SizeOf<AngularLimit1DJacobian>();
                case JacobianType.AngularLimit2D:
                    return UnsafeUtility.SizeOf<AngularLimit2DJacobian>();
                case JacobianType.AngularLimit3D:
                    return UnsafeUtility.SizeOf<AngularLimit3DJacobian>();
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }
        }

        // Access to "base" jacobian - a jacobian that comes after the header
        public unsafe ref T AccessBaseJacobian<T>() where T : struct
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>();
            return ref UnsafeUtility.AsRef<T>(ptr);
        }

        public unsafe ref ColliderKeyPair AccessColliderKeys()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type);
            return ref UnsafeUtility.AsRef<ColliderKeyPair>(ptr);
        }

        public unsafe ref EntityPair AccessEntities()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfColliderKeys(Type, Flags);
            return ref UnsafeUtility.AsRef<EntityPair>(ptr);
        }

        public unsafe ref SurfaceVelocity AccessSurfaceVelocity()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableSurfaceVelocity) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags);
            return ref UnsafeUtility.AsRef<SurfaceVelocity>(ptr);
        }

        public unsafe ref MassFactors AccessMassFactors()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableMassFactors) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags) + SizeOfSurfaceVelocity(Type, Flags);
            return ref UnsafeUtility.AsRef<MassFactors>(ptr);
        }

        public unsafe ref ContactJacAngAndVelToReachCp AccessAngularJacobian(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact || Type == JacobianType.Trigger);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                pointIndex * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>();
            return ref UnsafeUtility.AsRef<ContactJacAngAndVelToReachCp>(ptr);
        }

        public unsafe ref ContactPoint AccessContactPoint(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact);

            var baseJac = AccessBaseJacobian<ContactJacobian>();
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                baseJac.BaseJacobian.NumContacts * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                pointIndex * UnsafeUtility.SizeOf<ContactPoint>();
            return ref UnsafeUtility.AsRef<ContactPoint>(ptr);
        }

        #endregion
    }

    // Helper functions for working with Jacobians
    static class JacobianUtilities
    {
        // This is the inverse function to CalculateConstraintTauAndDamping
        // Given a final Tau and Damping you can get the original Spring Frequency and Damping for a given solver step
        // See Fixed.Physics.Constraint struct for discussion about default Spring Frequency and Damping values.
        public static void CalculateSpringFrequencyAndDamping(fp constraintTau, fp constraintDamping, fp timestep, int iterations, out fp springFrequency, out fp springDamping)
        {
            int n = iterations;
            fp sn = (fp)iterations;
            fp h = timestep;
            fp hh = h * h;
            fp a = fp.one - constraintDamping;
            fp aSum = fp.one;
            for (int i = 1; i < n; i++)
            {
                aSum += fpmath.pow(a, (fp)i);
            }

            fp w = fpmath.sqrt(constraintTau * aSum / fpmath.pow(a, sn)) / h;
            fp ww = w * w;
            springFrequency = w / ((fp)2.0f * fpmath.PI);
            springDamping = (fpmath.pow(a, -sn) - fp.one - hh * ww) / ((fp)2.0f * h * w);
        }

        // This is the inverse function to CalculateSpringFrequencyAndDamping
        public static void CalculateConstraintTauAndDamping(fp springFrequency, fp springDamping, fp timestep, int iterations, out fp constraintTau, out fp constraintDamping)
        {
            // TODO
            // - it's a significant amount of work to calculate tau and damping.  They depend on step length, so they have to be calculated each step.
            //   probably worth caching tau and damping for the default spring constants on the world and branching.
            // - you always get a higher effective damping ratio than you ask for because of the error from to discrete integration. The error varies
            //   with step length.  Can we estimate or bound that error and compensate for it?

            /*

            How to derive these formulas for tau and damping:

            1) implicit euler integration of a damped spring

               damped spring equation: x'' = -kx - cx'
               h = step length

               x2 = x1 + hv2
               v2 = v1 + h(-kx2 - cv2)/m
                  = v1 + h(-kx1 - hkv2 - cv2)/m
                  = v1 / (1 + h^2k/m + hc/m) - hkx1 / (m + h^2k + hc)

            2) gauss-seidel iterations of a stiff constraint.  Example for four iterations:

               t = tau, d = damping, a = 1 - d
               v2 = av1 - (t / h)x1
               v3 = av2 - (t / h)x1
               v4 = av3 - (t / h)x1
               v5 = av4 - (t / h)x1
                  = a^4v1 - (a^3 + a^2 + a + 1)(t / h)x1

            3) by matching coefficients of v1 and x1 in the formulas for v2 in step (1) and v5 in step (2), we see that if:

               (1 - damping)^4 = 1 / (1 + h^2k / m + hc / m)
               ((1 - damping)^3 + (1 - damping)^2 + (1 - damping) + 1)(tau / h) = hk / (m + h^2k + hc)

               then our constraint is equivalent to the implicit euler integration of a spring.
               solve the first equation for damping, then solve the second equation for tau.
               then substitute in k = mw^2, c = 2mzw.

            */

            fp h = timestep;
            fp w = springFrequency * fp.two * (fp)fpmath.PI; // convert oscillations/sec to radians/sec
            fp z = springDamping;
            fp hw = h * w;
            fp hhww = hw * hw;

            // a = 1-d, aExp = a^iterations, aSum = aExp / sum(i in [0, iterations), a^i)
            fp aExp = fp.one / (fp.one + hhww + fp.two * hw * z);
            fp a, aSum;
            if (iterations == 4)
            {
                // special case expected iterations = 4
                fp invA2 = fpmath.rsqrt(aExp);
                fp a2 = invA2 * aExp;
                a = fpmath.rsqrt(invA2);
                aSum = (fp.one + a2 + a * (fp.one + a2));
            }
            else
            {
                a = fpmath.pow(aExp, fp.one / (fp)iterations);
                aSum = fp.one;
                for (int i = 1; i < iterations; i++)
                {
                    aSum = a * aSum + fp.one;
                }
            }

            constraintDamping = fp.one - a;
            constraintTau = hhww * aExp / aSum;
        }

        // Returns x - clamp(x, min, max)
        public static fp CalculateError(fp x, fp min, fp max)
        {
            fp error = fpmath.max(x - max, fp.zero);
            error = fpmath.min(x - min, error);
            return error;
        }

        // Returns the amount of error for the solver to correct, where initialError is the pre-integration error and predictedError is the expected post-integration error
        public static fp CalculateCorrection(fp predictedError, fp initialError, fp tau, fp damping)
        {
            return fpmath.max(predictedError - initialError, fp.zero) * damping + fpmath.min(predictedError, initialError) * tau;
        }

        // Integrate the relative orientation of a pair of bodies, faster and less memory than storing both bodies' orientations and integrating them separately
        public static fpquaternion IntegrateOrientationBFromA(fpquaternion bFromA, fp3 angularVelocityA, fp3 angularVelocityB, fp timestep)
        {
            fpquaternion dqA = Integrator.IntegrateAngularVelocity(angularVelocityA, timestep);
            fpquaternion dqB = Integrator.IntegrateAngularVelocity(angularVelocityB, timestep);
            return fpmath.normalize(fpmath.mul(fpmath.mul(fpmath.inverse(dqB), bFromA), dqA));
        }

        // Calculate the inverse effective mass of a linear jacobian
        public static fp CalculateInvEffectiveMassDiag(
            fp3 angA, fp3 invInertiaA, fp invMassA,
            fp3 angB, fp3 invInertiaB, fp invMassB)
        {
            fp3 angularPart = angA * angA * invInertiaA + angB * angB * invInertiaB;
            fp linearPart = invMassA + invMassB;
            return (angularPart.x + angularPart.y) + (angularPart.z + linearPart);
        }

        // Calculate the inverse effective mass for a pair of jacobians with perpendicular linear parts
        public static fp CalculateInvEffectiveMassOffDiag(
            fp3 angA0, fp3 angA1, fp3 invInertiaA,
            fp3 angB0, fp3 angB1, fp3 invInertiaB)
        {
            return fpmath.csum(angA0 * angA1 * invInertiaA + angB0 * angB1 * invInertiaB);
        }

        // Inverts a symmetrix 3x3 matrix with diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static bool InvertSymmetricMatrix(fp3 diag, fp3 offDiag, out fp3 invDiag, out fp3 invOffDiag)
        {
            fp3 offDiagSq = offDiag.zyx * offDiag.zyx;
            fp determinant = (Math.HorizontalMul(diag) + fp.two * Math.HorizontalMul(offDiag) - fpmath.csum(offDiagSq * diag));
            bool determinantOk = determinant != fp.zero;
            fp invDeterminant = fpmath.select(fp.zero, fp.one / determinant, determinantOk);
            invDiag = (diag.yxx * diag.zzy - offDiagSq) * invDeterminant;
            invOffDiag = (offDiag.yxx * offDiag.zzy - diag.zyx * offDiag) * invDeterminant;
            return determinantOk;
        }

        // Builds a symmetric 3x3 matrix from diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static fp3x3 BuildSymmetricMatrix(fp3 diag, fp3 offDiag)
        {
            return new fp3x3(
                new fp3(diag.x, offDiag.x, offDiag.y),
                new fp3(offDiag.x, diag.y, offDiag.z),
                new fp3(offDiag.y, offDiag.z, diag.z)
            );
        }
    }

    // Iterator (and modifier) for jacobians
    unsafe struct JacobianIterator
    {
        NativeStream.Reader m_Reader;

        public JacobianIterator(NativeStream.Reader jacobianStreamReader, int workItemIndex)
        {
            m_Reader = jacobianStreamReader;
            m_Reader.BeginForEachIndex(workItemIndex);
        }

        public bool HasJacobiansLeft()
        {
            return m_Reader.RemainingItemCount > 0;
        }

        public ref JacobianHeader ReadJacobianHeader()
        {
            int readSize = Read<int>();
            return ref UnsafeUtility.AsRef<JacobianHeader>(Read(readSize));
        }

        private byte* Read(int size)
        {
            byte* dataPtr = m_Reader.ReadUnsafePtr(size);

            return dataPtr;
        }

        private ref T Read<T>() where T : struct
        {
            int size = UnsafeUtility.SizeOf<T>();
            return ref UnsafeUtility.AsRef<T>(Read(size));
        }
    }
}
