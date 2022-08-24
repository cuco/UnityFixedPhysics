using Unity.Burst;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // Solve data for a constraint that limits one degree of angular freedom
    [NoAlias]
    struct AngularLimit1DJacobian
    {
        // Limited axis in motion A space
        // TODO could calculate this from AxisIndex and MotionAFromJoint
        public fp3 AxisInMotionA;

        // Index of the limited axis
        public int AxisIndex;

        // Relative angle limits
        public fp MinAngle;
        public fp MaxAngle;

        // Relative orientation of the motions before solving
        public fpquaternion MotionBFromA;

        // Rotation to joint space from motion space
        public fpquaternion MotionAFromJoint;
        public fpquaternion MotionBFromJoint;

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
            // Copy the constraint into the jacobian
            AxisIndex = constraint.ConstrainedAxis1D;
            AxisInMotionA = aFromConstraint.Rotation[AxisIndex];
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            MotionBFromA = fpmath.mul(fpmath.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            MotionAFromJoint = new fpquaternion(aFromConstraint.Rotation);
            MotionBFromJoint = new fpquaternion(bFromConstraint.Rotation);

            // Calculate the current error
            InitialError = CalculateError(MotionBFromA);
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, fp timestep, fp invTimestep)
        {
            // Predict the relative orientation at the end of the step
            fpquaternion futureMotionBFromA = JacobianUtilities.IntegrateOrientationBFromA(MotionBFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Calculate the effective mass
            fp3 axisInMotionB = fpmath.mul(futureMotionBFromA, -AxisInMotionA);
            fp effectiveMass;
            {
                fp invEffectiveMass = fpmath.csum(AxisInMotionA * AxisInMotionA * velocityA.InverseInertia +
                    axisInMotionB * axisInMotionB * velocityB.InverseInertia);
                effectiveMass = fpmath.select(fp.one / invEffectiveMass, fp.zero, invEffectiveMass == fp.zero);
            }

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            fp futureError = CalculateError(futureMotionBFromA);
            fp solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            fp impulse = fpmath.mul(effectiveMass, -solveError) * invTimestep;
            velocityA.ApplyAngularImpulse(impulse * AxisInMotionA);
            velocityB.ApplyAngularImpulse(impulse * axisInMotionB);
        }

        // Helper function
        private fp CalculateError(fpquaternion motionBFromA)
        {
            // Calculate the relative body rotation
            fpquaternion jointBFromA = fpmath.mul(fpmath.mul(fpmath.inverse(MotionBFromJoint), motionBFromA), MotionAFromJoint);

            // Find the twist angle of the rotation.
            //
            // There is no one correct solution for the twist angle. Suppose the joint models a pair of bodies connected by
            // three gimbals, one of which is limited by this jacobian. There are multiple configurations of the gimbals that
            // give the bodies the same relative orientation, so it is impossible to determine the configuration from the
            // bodies' orientations alone, nor therefore the orientation of the limited gimbal.
            //
            // This code instead makes a reasonable guess, the twist angle of the swing-twist decomposition of the bodies'
            // relative orientation. It always works when the limited axis itself is unable to rotate freely, as in a limited
            // hinge. It works fairly well when the limited axis can only rotate a small amount, preferably less than 90
            // degrees. It works poorly at higher angles, especially near 180 degrees where it is not continuous. For systems
            // that require that kind of flexibility, the gimbals should be modeled as separate bodies.
            fp angle = CalculateTwistAngle(jointBFromA, AxisIndex);

            // Angle is in [-2pi, 2pi].
            // For comparison against the limits, find k so that angle + 2k * pi is as close to [min, max] as possible.
            fp centerAngle = (MinAngle + MaxAngle) / fp.two;
            bool above = angle > (centerAngle + fpmath.PI);
            bool below = angle < (centerAngle - fpmath.PI);
            angle = fpmath.select(angle, angle - fp.two * fpmath.PI, above);
            angle = fpmath.select(angle, angle + fp.two * fpmath.PI, below);

            // Calculate the relative angle about the twist axis
            return JacobianUtilities.CalculateError(angle, MinAngle, MaxAngle);
        }
    }
}
