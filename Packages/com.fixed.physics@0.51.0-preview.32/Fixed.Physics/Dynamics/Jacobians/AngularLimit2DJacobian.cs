using Unity.Burst;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // Solve data for a constraint that limits two degrees of angular freedom
    [NoAlias]
    struct AngularLimit2DJacobian
    {
        // Free axes in motion space
        public fp3 AxisAinA;
        public fp3 AxisBinB;

        // Relative angle limits
        public fp MinAngle;
        public fp MaxAngle;

        // Relative orientation before solving
        public fpquaternion BFromA;

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
            // Copy the constraint data
            int freeIndex = constraint.FreeAxis2D;
            AxisAinA = aFromConstraint.Rotation[freeIndex];
            AxisBinB = bFromConstraint.Rotation[freeIndex];
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            BFromA = fpmath.mul(fpmath.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);

            // Calculate the initial error
            {
                fp3 axisAinB = fpmath.mul(BFromA, AxisAinA);
                fp sinAngle = fpmath.length(fpmath.cross(axisAinB, AxisBinB));
                fp cosAngle = fpmath.dot(axisAinB, AxisBinB);
                fp angle = fpmath.atan2(sinAngle, cosAngle);
                InitialError = JacobianUtilities.CalculateError(angle, MinAngle, MaxAngle);
            }
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, fp timestep, fp invTimestep)
        {
            // Predict the relative orientation at the end of the step
            fpquaternion futureBFromA = JacobianUtilities.IntegrateOrientationBFromA(BFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Calculate the jacobian axis and angle
            fp3 axisAinB = fpmath.mul(futureBFromA, AxisAinA);
            fp3 jacB0 = fpmath.cross(axisAinB, AxisBinB);
            fp3 jacA0 = fpmath.mul(fpmath.inverse(futureBFromA), -jacB0);
            fp jacLengthSq = fpmath.lengthsq(jacB0);
            fp invJacLength = Math.RSqrtSafe(jacLengthSq);
            fp futureAngle;
            {
                fp sinAngle = jacLengthSq * invJacLength;
                fp cosAngle = fpmath.dot(axisAinB, AxisBinB);
                futureAngle = fpmath.atan2(sinAngle, cosAngle);
            }

            // Choose a second jacobian axis perpendicular to A
            fp3 jacB1 = fpmath.cross(jacB0, axisAinB);
            fp3 jacA1 = fpmath.mul(fpmath.inverse(futureBFromA), -jacB1);

            // Calculate effective mass
            fp2 effectiveMass; // First column of the 2x2 matrix, we don't need the second column because the second component of error is zero
            {
                // Calculate the inverse effective mass matrix, then invert it
                fp invEffMassDiag0 = fpmath.csum(jacA0 * jacA0 * velocityA.InverseInertia + jacB0 * jacB0 * velocityB.InverseInertia);
                fp invEffMassDiag1 = fpmath.csum(jacA1 * jacA1 * velocityA.InverseInertia + jacB1 * jacB1 * velocityB.InverseInertia);
                fp invEffMassOffDiag = fpmath.csum(jacA0 * jacA1 * velocityA.InverseInertia + jacB0 * jacB1 * velocityB.InverseInertia);
                fp det = invEffMassDiag0 * invEffMassDiag1 - invEffMassOffDiag * invEffMassOffDiag;
                fp invDet = fpmath.select(jacLengthSq / det, fp.zero, det == fp.zero); // scale by jacLengthSq because the jacs were not normalized
                effectiveMass = invDet * new fp2(invEffMassDiag1, -invEffMassOffDiag);
            }

            // Normalize the jacobians
            jacA0 *= invJacLength;
            jacB0 *= invJacLength;
            jacA1 *= invJacLength;
            jacB1 *= invJacLength;

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            fp futureError = JacobianUtilities.CalculateError(futureAngle, MinAngle, MaxAngle);
            fp solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            fp2 impulse = -effectiveMass * solveError * invTimestep;
            velocityA.ApplyAngularImpulse(impulse.x * jacA0 + impulse.y * jacA1);
            velocityB.ApplyAngularImpulse(impulse.x * jacB0 + impulse.y * jacB1);
        }
    }
}
