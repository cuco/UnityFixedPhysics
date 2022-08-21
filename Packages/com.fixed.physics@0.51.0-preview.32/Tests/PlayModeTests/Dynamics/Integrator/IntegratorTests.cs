using System;
using NUnit.Framework;
using Fixed.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Fixed.Physics.Tests.Dynamics.Integrator
{
    class IntegratorTests
    {
        [Test]
        public void IntegrateOrientationTest()
        {
            var orientation = quaternion.identity;
            var angularVelocity = new float3((sfloat)0.0f, (sfloat)0.0f, (sfloat)0.0f);
            float timestep = 1.0f;

            Physics.Integrator.IntegrateOrientation(ref orientation, angularVelocity, (sfloat)timestep);

            Assert.AreEqual(new quaternion((sfloat)0.0f, (sfloat)0.0f, (sfloat)0.0f, (sfloat)1.0f), orientation);
        }

        [Test]
        public void IntegrateAngularVelocityTest()
        {
            var angularVelocity = new float3((sfloat)1.0f, (sfloat)2.0f, (sfloat)3.0f);
            sfloat timestep = (sfloat)4.0f;

            var orientation = Fixed.Physics.Integrator.IntegrateAngularVelocity(angularVelocity, timestep);

            Assert.AreEqual(new quaternion((sfloat)2.0f, (sfloat)4.0f, (sfloat)6.0f, (sfloat)1.0f), orientation);
        }
    }
}
