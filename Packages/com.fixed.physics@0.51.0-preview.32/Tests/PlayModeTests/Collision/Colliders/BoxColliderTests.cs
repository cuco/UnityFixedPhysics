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
    /// Test class containing tests for the <see cref="BoxCollider"/>
    /// </summary>
    class BoxColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                BoxCollider.Create(new BoxGeometry { Orientation = fpquaternion.identity, Size = new fp3((fp)1f) }).Dispose();
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
                Center = new fp3(-(fp)10.10f, (fp)10.12f, (fp)0.01f),
                Orientation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)1.4f, (fp)0.2f, (fp)1.1f)), (fp)38.50f),
                Size = new fp3((fp)0.01f, (fp)120.40f, (fp)5.4f),
                BevelRadius = (fp)0.0f
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
            var geometry = new BoxGeometry { Center = new fp3((fp)errantValue), Size = new fp3((fp)1f), Orientation = fpquaternion.identity };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Center)));
        }

        [Test]
        public void BoxCollider_Create_WhenOrientationInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, 0f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new fp3((fp)1f), Orientation = new fpquaternion((fp)0f, (fp)0f, (fp)0f, (fp)errantValue) };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Orientation)));
        }

        [Test]
        public void BoxCollider_Create_WhenSizeInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new fp3((fp)errantValue), Orientation = fpquaternion.identity };

            var ex = Assert.Throws<ArgumentException>(() => BoxCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(BoxGeometry.Size)));
        }

        [Test]
        public void BoxCollider_Create_WhenBevelRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f, 0.55f)] float errantValue
        )
        {
            var geometry = new BoxGeometry { Size = new fp3((fp)1f), Orientation = fpquaternion.identity, BevelRadius = (fp)errantValue};

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
        /// private Aabb CalculateBoxAabbNaive(fp3 center, fpquaternion orientation, fp3 size, fpquaternion bRotation, fp3 bTranslation)
        ///{
        ///    fp3[] points = {
        ///        0.5f * new fp3(-size.x, -size.y, -size.z),
        ///        0.5f * new fp3(-size.x, -size.y, size.z),
        ///        0.5f * new fp3(-size.x, size.y, -size.z),
        ///        0.5f * new fp3(-size.x, size.y, size.z),
        ///        0.5f * new fp3(size.x, -size.y, -size.z),
        ///        0.5f * new fp3(size.x, -size.y, size.z),
        ///        0.5f * new fp3(size.x, size.y, -size.z),
        ///        0.5f * new fp3(size.x, size.y, size.z)
        ///    };
        ///
        ///    for (int i = 0; i < 8; ++i)
        ///    {
        ///        points[i] = center + fpmath.mul(orientation, points[i]);
        ///        points[i] = bTranslation + fpmath.mul(bRotation, points[i]);
        ///    }
        ///
        ///    Aabb result = Aabb.CreateFromPoints(new fp3x4(points[0], points[1], points[2], points[3]));
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
                    Center = new fp3(-(fp)0.59f, (fp)0.36f, (fp)0.35f),
                    Orientation = fpquaternion.identity,
                    Size = new fp3((fp)2.32f, (fp)10.87f, (fp)16.49f),
                    BevelRadius = (fp)0.25f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new fp3(-(fp)1.75f, -(fp)5.075f, -(fp)7.895f),
                    Max = new fp3((fp)0.57f, (fp)5.795f, (fp)8.595f)
                };

                var boxCollider = BoxCollider.Create(geometry);
                Aabb aabb = boxCollider.Value.CalculateAabb();
                TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (fp)1e-3f);
                TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (fp)1e-3f);
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
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = new fp3((fp)1.0f, (fp)250.0f, fp.two),
                BevelRadius = (fp)0.25f
            };

            var boxCollider = BoxCollider.Create(geometry);

            fp3 expectedInertiaTensor = (fp)1.0f / (fp)12.0f * new fp3(
                geometry.Size.y * geometry.Size.y + geometry.Size.z * geometry.Size.z,
                geometry.Size.x * geometry.Size.x + geometry.Size.z * geometry.Size.z,
                geometry.Size.y * geometry.Size.y + geometry.Size.x * geometry.Size.x);

            MassProperties massProperties = boxCollider.Value.MassProperties;
            fp3 inertiaTensor = massProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (fp)1e-3f);
        }

        #endregion
    }
}
