using System;
using NUnit.Framework;
using Unity.Collections;
using Fixed.Mathematics;
using Assert = UnityEngine.Assertions.Assert;

namespace Fixed.Physics.Tests.Dynamics.Jacobians
{
    class JacobiansTests
    {
        [Test]
        public void JacobianUtilitiesCalculateTauAndDampingTest()
        {
            sfloat springFrequency = (sfloat)1.0f;
            sfloat springDampingRatio = (sfloat)1.0f;
            sfloat timestep = (sfloat)1.0f;
            int iterations = 4;

            JacobianUtilities.CalculateConstraintTauAndDamping(springFrequency, springDampingRatio, timestep, iterations, out sfloat tau, out sfloat damping);

            Assert.AreApproximatelyEqual(0.4774722f, (float)tau);
            Assert.AreApproximatelyEqual(0.6294564f, (float)damping);
        }

        [Test]
        public void JacobianUtilitiesCalculateTauAndDampingFromConstraintTest()
        {
            var constraint = new Constraint { SpringFrequency = (sfloat)1.0f, SpringDamping = (sfloat)1.0f };
            sfloat timestep = (sfloat)1.0f;
            int iterations = 4;

            sfloat tau;
            sfloat damping;
            JacobianUtilities.CalculateConstraintTauAndDamping(constraint.SpringFrequency, constraint.SpringDamping, timestep, iterations, out tau, out damping);

            Assert.AreApproximatelyEqual(0.4774722f, (float)tau);
            Assert.AreApproximatelyEqual(0.6294564f, (float)damping);
        }

        [Test]
        public void JacobianUtilitiesCalculateErrorTest()
        {
            sfloat x = (sfloat)5.0f;
            sfloat min = (sfloat)0.0f;
            sfloat max = (sfloat)10.0f;

            Assert.AreApproximatelyEqual(0.0f, (float)JacobianUtilities.CalculateError(x, min, max));
        }

        [Test]
        public void JacobianUtilitiesCalculateCorrectionTest()
        {
            sfloat predictedError = (sfloat)0.2f;
            sfloat initialError = (sfloat)0.1f;
            sfloat tau = (sfloat)0.6f;
            sfloat damping = (sfloat)1.0f;

            Assert.AreApproximatelyEqual(0.16f, (float)JacobianUtilities.CalculateCorrection(predictedError, initialError, tau, damping));
        }

        [Test]
        public void JacobianUtilitiesIntegrateOrientationBFromATest()
        {
            var bFromA = quaternion.identity;
            var angularVelocityA = float3.zero;
            var angularVelocityB = float3.zero;
            var timestep = (sfloat)1.0f;

            Assert.AreEqual(quaternion.identity, JacobianUtilities.IntegrateOrientationBFromA(bFromA, angularVelocityA, angularVelocityB, timestep));
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
