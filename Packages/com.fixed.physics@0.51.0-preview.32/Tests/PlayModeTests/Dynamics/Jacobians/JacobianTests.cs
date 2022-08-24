using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics.FixedPoint;
using Assert = UnityEngine.Assertions.Assert;

namespace Fixed.Physics.Tests.Dynamics.Jacobians
{
    class JacobiansTests
    {
        [Test]
        public void JacobianUtilitiesCalculateTauAndDampingTest()
        {
            fp springFrequency = (fp)1.0f;
            fp springDampingRatio = (fp)1.0f;
            fp timestep = (fp)1.0f;
            int iterations = 4;

            JacobianUtilities.CalculateConstraintTauAndDamping(springFrequency, springDampingRatio, timestep, iterations, out fp tau, out fp damping);

            Assert.AreApproximatelyEqual(0.4774722f, (float)tau);
            Assert.AreApproximatelyEqual(0.6294564f, (float)damping);
        }

        [Test]
        public void JacobianUtilitiesCalculateTauAndDampingFromConstraintTest()
        {
            var constraint = new Constraint { SpringFrequency = (fp)1.0f, SpringDamping = (fp)1.0f };
            fp timestep = (fp)1.0f;
            int iterations = 4;

            fp tau;
            fp damping;
            JacobianUtilities.CalculateConstraintTauAndDamping(constraint.SpringFrequency, constraint.SpringDamping, timestep, iterations, out tau, out damping);

            Assert.AreApproximatelyEqual(0.4774722f, (float)tau);
            Assert.AreApproximatelyEqual(0.6294564f, (float)damping);
        }

        [Test]
        public void JacobianUtilitiesCalculateErrorTest()
        {
            fp x = (fp)5.0f;
            fp min = (fp)0.0f;
            fp max = (fp)10.0f;

            Assert.AreApproximatelyEqual(0.0f, (float)JacobianUtilities.CalculateError(x, min, max));
        }

        [Test]
        public void JacobianUtilitiesCalculateCorrectionTest()
        {
            fp predictedError = (fp)0.2f;
            fp initialError = (fp)0.1f;
            fp tau = (fp)0.6f;
            fp damping = (fp)1.0f;

            Assert.AreApproximatelyEqual(0.16f, (float)JacobianUtilities.CalculateCorrection(predictedError, initialError, tau, damping));
        }

        [Test]
        public void JacobianUtilitiesIntegrateOrientationBFromATest()
        {
            var bFromA = fpquaternion.identity;
            var angularVelocityA = fp3.zero;
            var angularVelocityB = fp3.zero;
            var timestep = (fp)1.0f;

            Assert.AreEqual(fpquaternion.identity, JacobianUtilities.IntegrateOrientationBFromA(bFromA, angularVelocityA, angularVelocityB, timestep));
        }

        [Test]
        public void JacobianIteratorHasJacobiansLeftTest()
        {
            var jacobianStream = new NativeStream(1, Allocator.Temp);
            NativeStream.Reader jacobianStreamReader = jacobianStream.AsReader();
            int workItemIndex = 0;
            var jacIterator = new JacobianIterator(jacobianStreamReader, workItemIndex);

            Assert.IsFalse(jacIterator.HasJacobiansLeft());

            jacobianStream.Dispose();
        }
    }
}
