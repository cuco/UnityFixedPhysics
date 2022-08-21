using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Fixed.Mathematics;
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
            public void Execute() => SphereCollider.Create(new SphereGeometry { Radius = (sfloat)1f }).Dispose();
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
                Center = new float3(-(sfloat)8.45f, (sfloat)9.65f, -(sfloat)0.10f),
                Radius = (sfloat)0.98f
            };

            var collider = SphereCollider.Create(sphere);
            var sphereCollider = UnsafeUtility.AsRef<SphereCollider>(collider.GetUnsafePtr());

            TestUtils.AreEqual(sphere.Center, sphereCollider.Center, (sfloat)1e-3f);
            TestUtils.AreEqual(sphere.Center, sphereCollider.Geometry.Center, (sfloat)1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Radius, (sfloat)1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Geometry.Radius, (sfloat)1e-3f);
            Assert.AreEqual(ColliderType.Sphere, sphereCollider.Type);
            Assert.AreEqual(CollisionType.Convex, sphereCollider.CollisionType);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void SphereCollider_Create_WhenCenterInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float value
        )
        {
            var geometry = new SphereGeometry { Center = new float3((sfloat)value) };

            var ex = Assert.Throws<ArgumentException>(() => SphereCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(SphereGeometry.Center)));
        }

        [Test]
        public void SphereCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float value
        )
        {
            var geometry = new SphereGeometry { Radius = (sfloat)value };

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
            float3 center = new float3(-(sfloat)8.4f, (sfloat)5.63f, -(sfloat)7.2f);
            sfloat radius = (sfloat)2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            Aabb expected = new Aabb();
            expected.Min = center - new float3(radius, radius, radius);
            expected.Max = center + new float3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expected.Min, actual.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed <see cref="SphereCollider"/> is calculated correctly
        /// </summary>
        [Test]
        public void TestSphereColliderCalculateAabbTransformed()
        {
            float3 center = new float3(-(sfloat)3.4f, (sfloat)0.63f, -(sfloat)17.2f);
            sfloat radius = (sfloat)5.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            float3 translation = new float3((sfloat)8.3f, -(sfloat)0.5f, (sfloat)170.0f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3((sfloat)1.1f, (sfloat)4.5f, (sfloat)0.0f)), (sfloat)146.0f);

            Aabb expected = new Aabb();
            expected.Min = math.mul(rotation, center) + translation - new float3(radius, radius, radius);
            expected.Max = math.mul(rotation, center) + translation + new float3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb(new RigidTransform(rotation, translation));
            TestUtils.AreEqual(expected.Min, actual.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, (sfloat)1e-3f);
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
            float3 center = new float3(-(sfloat)8.4f, (sfloat)5.63f, (sfloat)77.2f);
            sfloat radius = (sfloat)2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            sfloat inertia = (sfloat)2.0f / (sfloat)5.0f * radius * radius;
            float3 expectedInertiaTensor = new float3(inertia, inertia, inertia);
            float3 inertiaTensor = sphereCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (sfloat)1e-3f);
        }

        #endregion
    }
}
