using Unity.Burst;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // Solve data for a constraint that limits three degrees of angular freedom
    [NoAlias]
    struct AngularLimit3DJacobian
    {
        // Relative angle limits
        public fp MinAngle;
        public fp MaxAngle;

        // Relative orientation of motions before solving
        public fpquaternion BFromA;

        // Angle is zero when BFromA = RefBFromA
        public fpquaternion RefBFromA;

        // Error before solving
        public fp InitialError;

        // Fraction of the position error to correct per step
        public fp Tau;

        // Fraction of the velocity error to correct per step
        public fp Damping;

        // Build the Jacobian
        public void Build(
            MTransform aFromConstraint, MTransform bFromConstraint,
            MotionVelocity velocityA, MotionVelocity velocityB,
            MotionData motionA, MotionData motionB,
            Constraint constraint, fp tau, fp damping)
        {
            BFromA = fpmath.mul(fpmath.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            RefBFromA = new fpquaternion(fpmath.mul(bFromConstraint.Rotation, aFromConstraint.InverseRotation));
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;

            fpquaternion jointOrientation = fpmath.mul(fpmath.inverse(RefBFromA), BFromA);
            fp initialAngle = fp.Asin(fpmath.length(jointOrientation.value.xyz)) * fp.two;
            InitialError = JacobianUtilities.CalculateError(initialAngle, MinAngle, MaxAngle);
        }

        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, fp timestep, fp invTimestep)
        {
            // Predict the relative orientation at the end of the step
            fpquaternion futureBFromA = JacobianUtilities.IntegrateOrientationBFromA(BFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Find the future axis and angle of rotation between the free axes
            fp3 jacA0, jacA1, jacA2, jacB0, jacB1, jacB2;
            fp3 effectiveMass; // first column of 3x3 effective mass matrix, don't need the others because only jac0 can have nonzero error
            fp futureAngle;
            {
                // Calculate the relative rotation between joint spaces
                fpquaternion jointOrientation = fpmath.mul(fpmath.inverse(RefBFromA), futureBFromA);

                // Find the axis and angle of rotation
                jacA0 = jointOrientation.value.xyz;
                fp sinHalfAngleSq = fpmath.lengthsq(jacA0);
                fp invSinHalfAngle = Math.RSqrtSafe(sinHalfAngleSq);
                fp sinHalfAngle = sinHalfAngleSq * invSinHalfAngle;
                futureAngle = fp.Asin(sinHalfAngle) * fp.two;

                jacA0 = fpmath.select(jacA0 * invSinHalfAngle, new fp3(fp.one, fp.zero, fp.zero), invSinHalfAngle == fp.zero);
                jacA0 = fpmath.select(jacA0, -jacA0, jointOrientation.value.w < fp.zero);
                Math.CalculatePerpendicularNormalized(jacA0, out jacA1, out jacA2);

                jacB0 = fpmath.mul(futureBFromA, -jacA0);
                jacB1 = fpmath.mul(futureBFromA, -jacA1);
                jacB2 = fpmath.mul(futureBFromA, -jacA2);

                // Calculate the effective mass
                fp3 invEffectiveMassDiag = new fp3(
                    fpmath.csum(jacA0 * jacA0 * velocityA.InverseInertia + jacB0 * jacB0 * velocityB.InverseInertia),
                    fpmath.csum(jacA1 * jacA1 * velocityA.InverseInertia + jacB1 * jacB1 * velocityB.InverseInertia),
                    fpmath.csum(jacA2 * jacA2 * velocityA.InverseInertia + jacB2 * jacB2 * velocityB.InverseInertia));
                fp3 invEffectiveMassOffDiag = new fp3(
                    fpmath.csum(jacA0 * jacA1 * velocityA.InverseInertia + jacB0 * jacB1 * velocityB.InverseInertia),
                    fpmath.csum(jacA0 * jacA2 * velocityA.InverseInertia + jacB0 * jacB2 * velocityB.InverseInertia),
                    fpmath.csum(jacA1 * jacA2 * velocityA.InverseInertia + jacB1 * jacB2 * velocityB.InverseInertia));
                JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out fp3 effectiveMassDiag, out fp3 effectiveMassOffDiag);
                effectiveMass = JacobianUtilities.BuildSymmetricMatrix(effectiveMassDiag, effectiveMassOffDiag).c0;
            }

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            fp futureError = JacobianUtilities.CalculateError(futureAngle, MinAngle, MaxAngle);
            fp solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            fp solveVelocity = -solveError * invTimestep;
            fp3 impulseA = solveVelocity * (jacA0 * effectiveMass.x + jacA1 * effectiveMass.y + jacA2 * effectiveMass.z);
            fp3 impulseB = solveVelocity * (jacB0 * effectiveMass.x + jacB1 * effectiveMass.y + jacB2 * effectiveMass.z);
            velocityA.ApplyAngularImpulse(impulseA);
            velocityB.ApplyAngularImpulse(impulseB);
        }
    }
}
