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
    /// Test class containing tests for the <see cref="BoxCollider"/>
    /// </summary>
    class BoxColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                BoxCollider.Create(new BoxGeometry { Orientation = quaternion.identity, Size = new float3((sfloat)1f) }).Dispose();
        }

        [Test]
        public void BoxCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Create a <see cref="BoxCollider"/> and check that all attributes are set as expected
        /// </summary>
        [Test]
        public unsafe void TestBoxColliderCreate()
        {
            var geometry = new BoxGeometry
            {
                Center = new float3(-(sfloat)10.10f, (sfloat)10.12f, (sfloat)0.01f),
                Orientation = quaternion.AxisAngle(math.normalize(new float3((sfloat)1.4f, (sfloat)0.2f, (sfloat)1.1f)), (sfloat)38.50f),
                Size = new float3((sfloat)0.01f, (sfloat)120.40f, (sfloat)5.4f),
                BevelRadius = (sfloat)0.0f
            };

            var collider = BoxCollider.Create(geometry);
            var boxCollider = UnsafeUtility.AsRef<BoxCollider>(collider.GetUnsafePtr());

            Assert.AreEqual(geometry.Center, boxCollider.Center);
            Assert.AreEqual(geometry.Center, boxCollider.Geometry.Center);
            Assert.AreEqual(geometry.Orientation, boxCollider.Orientation);
            Assert.AreEqual(geometry.Orientation, boxCollider.Geometry.Orientation);
            Assert.AreEqual(geometry.Size, boxCollider.Size);
            Assert.AreEqual(geometry.Size, boxCollider.Geometry.Size);
            Assert.AreEqual(geometry.BevelRadius, boxCollider.BevelRadius);
            Assert.AreEqual(geometry.BevelRadius, boxCollider.Geometry.BevelRadius);
            Assert.AreEqual(CollisionType.Convex, boxCollider.CollisionType);
            Assert.AreEqual(ColliderType.Box, boxCollider.Type);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void BoxCollider_Create_WhenCenterInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Center = new float3((sfloat)errantValue), Size = new float3((sfloat)1f), Orientation = quaternion.identity };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Center)));
        }

        [Test]
        public void BoxCollider_Create_WhenOrientationInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, 0f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new float3((sfloat)1f), Orientation = new quaternion((sfloat)0f, (sfloat)0f, (sfloat)0f, (sfloat)errantValue) };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Orientation)));
        }

        [Test]
        public void BoxCollider_Create_WhenSizeInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new float3((sfloat)errantValue), Orientation = quaternion.identity };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Size)));
        }

        [Test]
        public void BoxCollider_Create_WhenBevelRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f, 0.55f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new float3((sfloat)1f), Orientation = quaternion.identity, BevelRadius = (sfloat)errantValue};

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.BevelRadius)));
        }

#endif

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that a translated box collider generates the correct local AABB
        /// </summary>
        /// <remarks>
        /// The following code was used to produce reference data for Aabbs:
        /// <code>
        /// private Aabb CalculateBoxAabbNaive(float3 center, quaternion orientation, float3 size, quaternion bRotation, float3 bTranslation)
        ///{
        ///    float3[] points = {
        ///        0.5f * new float3(-size.x, -size.y, -size.z),
        ///        0.5f * new float3(-size.x, -size.y, size.z),
        ///        0.5f * new float3(-size.x, size.y, -size.z),
        ///        0.5f * new float3(-size.x, size.y, size.z),
        ///        0.5f * new float3(size.x, -size.y, -size.z),
        ///        0.5f * new float3(size.x, -size.y, size.z),
        ///        0.5f * new float3(size.x, size.y, -size.z),
        ///        0.5f * new float3(size.x, size.y, size.z)
        ///    };
        ///
        ///    for (int i = 0; i < 8; ++i)
        ///    {
        ///        points[i] = center + math.mul(orientation, points[i]);
        ///        points[i] = bTranslation + math.mul(bRotation, points[i]);
        ///    }
        ///
        ///    Aabb result = Aabb.CreateFromPoints(new float3x4(points[0], points[1], points[2], points[3]));
        ///    for (int i = 4; i < 8; ++i)
        ///    {
        ///        result.Include(points[i]);
        ///    }
        ///    return result;
        ///}
        /// </code>
        /// </remarks>
        [Test]
        public void TestBoxColliderCalculateAabbLocalTranslation()
        {
            // Expected values in this test were generated using CalculateBoxAabbNaive above
            {
                var geometry = new BoxGeometry
                {
                    Center = new float3(-(sfloat)0.59f, (sfloat)0.36f, (sfloat)0.35f),
                    Orientation = quaternion.identity,
                    Size = new float3((sfloat)2.32f, (sfloat)10.87f, (sfloat)16.49f),
                    BevelRadius = (sfloat)0.25f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new float3(-(sfloat)1.75f, -(sfloat)5.075f, -(sfloat)7.895f),
                    Max = new float3((sfloat)0.57f, (sfloat)5.795f, (sfloat)8.595f)
                };

                var boxCollider = BoxCollider.Create(geometry);
                Aabb aabb = boxCollider.Value.CalculateAabb();
                TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (sfloat)1e-3f);
                TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (sfloat)1e-3f);
            }
        }

        /// <summary>
        /// Test that the created inertia tensor of the <see cref="BoxCollider"/> is correct
        /// </summary>
        /// <remarks>
        /// Formula for inertia tensor from here was used: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        /// </remarks>
        [Test]
        public void TestBoxColliderMassProperties()
        {
            var geometry = new BoxGeometry
            {
                Center = float3.zero,
                Orientation = quaternion.identity,
                Size = new float3((sfloat)1.0f, (sfloat)250.0f, (sfloat)2.0f),
                BevelRadius = (sfloat)0.25f
            };

            var boxCollider = BoxCollider.Create(geometry);

            float3 expectedInertiaTensor = (sfloat)1.0f / (sfloat)12.0f * new float3(
                geometry.Size.y * geometry.Size.y + geometry.Size.z * geometry.Size.z,
                geometry.Size.x * geometry.Size.x + geometry.Size.z * geometry.Size.z,
                geometry.Size.y * geometry.Size.y + geometry.Size.x * geometry.Size.x);

            MassProperties massProperties = boxCollider.Value.MassProperties;
            float3 inertiaTensor = massProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (sfloat)1e-3f);
        }

        #endregion
    }
}
