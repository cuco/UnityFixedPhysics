using System;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Fixed.Physics.Tests.Dynamics.Integrator
{
    class IntegratorTests
    {
        [Test]
        public void IntegrateOrientationTest()
        {
            var orientation = fpquaternion.identity;
            var angularVelocity = new fp3((fp)0.0f, (fp)0.0f, (fp)0.0f);
            float timestep = 1.0f;

            Physics.Integrator.IntegrateOrientation(ref orientation, angularVelocity, (fp)timestep);

            Assert.AreEqual(new fpquaternion((fp)0.0f, (fp)0.0f, (fp)0.0f, (fp)1.0f), orientation);
        }

        [Test]
        public void IntegrateAngularVelocityTest()
        {
            var angularVelocity = new fp3((fp)1.0f, fp.two, (fp)3.0f);
            fp timestep = (fp)4.0f;

            var orientation = Fixed.Physics.Integrator.IntegrateAngularVelocity(angularVelocity, timestep);

            Assert.AreEqual(new fpquaternion((fp)2.0f, (fp)4.0f, (fp)6.0f, (fp)1.0f), orientation);
        }
    }
}
