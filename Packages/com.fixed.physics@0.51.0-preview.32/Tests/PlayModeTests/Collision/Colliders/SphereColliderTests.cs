using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using TestUtils = Fixed.Physics.Tests.Utils.TestUtils;

namespace Fixed.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Contains all tests for the <see cref="SphereCollider"/>
    /// </summary>
    class SphereColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() => SphereCollider.Create(new SphereGeometry { Radius = (fp)1f }).Dispose();
        }

        [Test]
        public void SphereCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Tests if a created <see cref="SphereCollider"/> has its attributes set correctly
        /// </summary>
        [Test]
        unsafe public void TestSphereColliderCreate()
        {
            var sphere = new SphereGeometry
            {
                Center = new fp3(-(fp)8.45f, (fp)9.65f, -(fp)0.10f),
                Radius = (fp)0.98f
            };

            var collider = SphereCollider.Create(sphere);
            var sphereCollider = UnsafeUtility.AsRef<SphereCollider>(collider.GetUnsafePtr());

            TestUtils.AreEqual(sphere.Center, sphereCollider.Center, (fp)1e-3f);
            TestUtils.AreEqual(sphere.Center, sphereCollider.Geometry.Center, (fp)1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Radius, (fp)1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Geometry.Radius, (fp)1e-3f);
            Assert.AreEqual(ColliderType.Sphere, sphereCollider.Type);
            Assert.AreEqual(CollisionType.Convex, sphereCollider.CollisionType);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void SphereCollider_Create_WhenCenterInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float value
        )
        {
            var geometry = new SphereGeometry { Center = new fp3((fp)value) };

            var ex = Assert.Throws<ArgumentException>(() => SphereCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(SphereGeometry.Center)));
        }

        [Test]
        public void SphereCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float value
        )
        {
            var geometry = new SphereGeometry { Radius = (fp)value };

            var ex = Assert.Throws<ArgumentException>(() => SphereCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(SphereGeometry.Radius)));
        }

#endif

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB of a <see cref="SphereCollider"/> are calculated correctly
        /// </summary>
        [Test]
        public void TestSphereColliderCalculateAabbLocal()
        {
            fp3 center = new fp3(-(fp)8.4f, (fp)5.63f, -(fp)7.2f);
            fp radius = (fp)2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            Aabb expected = new Aabb();
            expected.Min = center - new fp3(radius, radius, radius);
            expected.Max = center + new fp3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expected.Min, actual.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed <see cref="SphereCollider"/> is calculated correctly
        /// </summary>
        [Test]
        public void TestSphereColliderCalculateAabbTransformed()
        {
            fp3 center = new fp3(-(fp)3.4f, (fp)0.63f, -(fp)17.2f);
            fp radius = (fp)5.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            fp3 translation = new fp3((fp)8.3f, -fp.half, (fp)170.0f);
            fpquaternion rotation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)1.1f, (fp)4.5f, (fp)0.0f)), (fp)146.0f);

            Aabb expected = new Aabb();
            expected.Min = fpmath.mul(rotation, center) + translation - new fp3(radius, radius, radius);
            expected.Max = fpmath.mul(rotation, center) + translation + new fp3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb(new FpRigidTransform(rotation, translation));
            TestUtils.AreEqual(expected.Min, actual.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test that the inertia tensor of a <see cref="SphereCollider"/> is calculated correctly
        /// </summary>
        /// <remarks>
        /// Inertia tensor formula taken from here: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        /// </remarks>
        [Test]
        public void TestSphereColliderMassProperties()
        {
            fp3 center = new fp3(-(fp)8.4f, (fp)5.63f, (fp)77.2f);
            fp radius = (fp)2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            fp inertia = fp.two / (fp)5.0f * radius * radius;
            fp3 expectedInertiaTensor = new fp3(inertia, inertia, inertia);
            fp3 inertiaTensor = sphereCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (fp)1e-3f);
        }

        #endregion
    }
}
