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
    /// Contains all test for the <see cref=" PolygonCollider"/>
    /// </summary>
    class PolygonColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateQuadFromBurstJob : IJob
        {
            public void Execute() =>
                PolygonCollider.CreateQuad(new float3(-(sfloat)1f, (sfloat)1f, (sfloat)0f), new float3((sfloat)1f, (sfloat)1f, (sfloat)0f), new float3((sfloat)1f, -(sfloat)1f, (sfloat)0f), new float3(-(sfloat)1f, -(sfloat)1f, (sfloat)0f)).Dispose();
        }

        [Test]
        public void PolygonCollider_CreateQuad_WhenCalledFromBurstJob_DoesNotThrow() => new CreateQuadFromBurstJob().Run();

        [BurstCompile(CompileSynchronously = true)]
        struct CreateTriangleFromBurstJob : IJob
        {
            public void Execute() =>
                PolygonCollider.CreateTriangle(new float3(-(sfloat)1f, (sfloat)1f, (sfloat)0f), new float3((sfloat)1f, (sfloat)1f, (sfloat)0f), new float3((sfloat)1f, -(sfloat)1f, (sfloat)0f)).Dispose();
        }

        [Test]
        public void PolygonCollider_CreateTriangle_WhenCalledFromBurstJob_DoesNotThrow() => new CreateTriangleFromBurstJob().Run();

        /// <summary>
        /// Test whether a triangle collider's attributes are set to the expected values after creation.
        /// </summary>
        [Test]
        unsafe public void TestCreateTriangle()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)1.4f, (sfloat)1.4f, (sfloat)5.6f),
                new float3((sfloat)1.4f, (sfloat)1.4f, (sfloat)3.6f),
                new float3((sfloat)0.2f, (sfloat)1.2f, (sfloat)5.6f)
            };
            float3 normal = math.normalize(math.cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            var triangleCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsTrue(triangleCollider.IsTriangle);
            Assert.IsFalse(triangleCollider.IsQuad);

            TestUtils.AreEqual(triangleCollider.Vertices[0], vertices[0], (sfloat)1e-3f);
            TestUtils.AreEqual(triangleCollider.Vertices[1], vertices[1], (sfloat)1e-3f);
            TestUtils.AreEqual(triangleCollider.Vertices[2], vertices[2], (sfloat)1e-3f);
            Assert.AreEqual(2, triangleCollider.Planes.Length);
            TestUtils.AreEqual(normal, triangleCollider.Planes[0].Normal, (sfloat)1e-3f);
            TestUtils.AreEqual(-normal, triangleCollider.Planes[1].Normal, (sfloat)1e-3f);
            Assert.AreEqual(ColliderType.Triangle, triangleCollider.Type);
            Assert.AreEqual(CollisionType.Convex, triangleCollider.CollisionType);
        }

        /// <summary>
        /// Test whether a quad collider's attributes are set correctly after creation.
        /// </summary>
        [Test]
        unsafe public void TestCreateQuad()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)4.5f, (sfloat)0.0f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)0.7f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)2.7f, (sfloat)1.0f),
                new float3(-(sfloat)3.4f, (sfloat)1.2f, (sfloat)1.0f)
            };
            float3 normal = math.normalize(math.cross(vertices[2] - vertices[1], vertices[0] - vertices[1]));

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            var quadCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsFalse(quadCollider.IsTriangle);
            Assert.IsTrue(quadCollider.IsQuad);

            TestUtils.AreEqual(quadCollider.Vertices[0], vertices[0], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[1], vertices[1], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[2], vertices[2], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[3], vertices[3], (sfloat)1e-3f);
            Assert.AreEqual(2, quadCollider.Planes.Length);
            TestUtils.AreEqual(normal, quadCollider.Planes[0].Normal, (sfloat)1e-3f);
            TestUtils.AreEqual(-normal, quadCollider.Planes[1].Normal, (sfloat)1e-3f);
            Assert.AreEqual(ColliderType.Quad, quadCollider.Type);
            Assert.AreEqual(CollisionType.Convex, quadCollider.CollisionType);
        }

        /// <summary>
        /// Test that 'unsorted', i.e. neither clockwise nor counter-clockwise winding, quads are created correctly
        /// </summary>
        [Test]
        unsafe public void TestCreateQuadUnsorted()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)4.5f, (sfloat)0.0f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)2.7f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)0.7f, (sfloat)1.0f),
                new float3(-(sfloat)3.4f, (sfloat)1.2f, (sfloat)1.0f)
            };
            float3 normal = math.normalize(math.cross(vertices[2] - vertices[1], vertices[0] - vertices[1]));

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            var quadCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsFalse(quadCollider.IsTriangle);
            Assert.IsTrue(quadCollider.IsQuad);

            TestUtils.AreEqual(quadCollider.Vertices[0], vertices[0], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[1], vertices[1], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[2], vertices[2], (sfloat)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[3], vertices[3], (sfloat)1e-3f);
            Assert.AreEqual(2, quadCollider.Planes.Length);
            TestUtils.AreEqual(normal, quadCollider.Planes[0].Normal, (sfloat)1e-3f);
            TestUtils.AreEqual(-normal, quadCollider.Planes[1].Normal, (sfloat)1e-3f);
            Assert.AreEqual(ColliderType.Quad, quadCollider.Type);
            Assert.AreEqual(CollisionType.Convex, quadCollider.CollisionType);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void PolygonCollider_CreateTriangle_WhenArgOutOfRange_Throws(
            [Values(0, 1, 2)] int errantArg,
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var vertices = new[]
            {
                math.select(default, new float3((sfloat)errantValue), errantArg == 0),
                math.select(default, new float3((sfloat)errantValue), errantArg == 1),
                math.select(default, new float3((sfloat)errantValue), errantArg == 2)
            };

            var ex = Assert.Throws<ArgumentException>(() => PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]));
            Assert.That(ex.ParamName, Is.EqualTo($"vertex{errantArg}"));
        }

        [Test]
        public void PolygonCollider_CreateQuad_WhenArgOutOfRange_Throws(
            [Values(0, 1, 2, 3)] int errantArg,
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var vertices = new[]
            {
                math.select(default, new float3((sfloat)errantValue), errantArg == 0),
                math.select(default, new float3((sfloat)errantValue), errantArg == 1),
                math.select(default, new float3((sfloat)errantValue), errantArg == 2),
                math.select(default, new float3((sfloat)errantValue), errantArg == 3)
            };

            var ex = Assert.Throws<ArgumentException>(() => PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2],
                vertices[3]));
            Assert.That(ex.ParamName, Is.EqualTo($"vertex{errantArg}"));
        }

        [TestCase(
            -4.5f, 0.0f, 1.0f,
            3.4f, 0.7f, 1.0f,
            3.4f, 2.7f, 1.0f,
            -3.4f, 1.2f, 1.1f,
            TestName = "Sorted quad vertices"
         )]
        [TestCase(
            -4.5f, -0.30f, 1.0f,
            3.4f, 2.7f, 1.0f,
            3.4f, -0.7f, 1.0f,
            -3.4f, 1.2f, 1.1f,
            TestName = "Unsorted quad vertices"
         )]
        public void PolygonCollider_CreateQuad_WhenNotCoplanar_Throws(
            float v00, float v01, float v02,
            float v10, float v11, float v12,
            float v20, float v21, float v22,
            float v30, float v31, float v32
        )
        {
            Assert.Throws<ArgumentException>(() =>
            {
                PolygonCollider.CreateQuad(
                    new float3((sfloat)v00, (sfloat)v01, (sfloat)v02),
                    new float3((sfloat)v10, (sfloat)v11, (sfloat)v12),
                    new float3((sfloat)v20, (sfloat)v21, (sfloat)v22),
                    new float3((sfloat)v30, (sfloat)v31, (sfloat)v32)
                );
            });
        }

#endif

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB of a triangle collider is computed correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbLocalTriangle()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)1.8f, (sfloat)2.4f, (sfloat)4.6f),
                new float3((sfloat)1.4f, (sfloat)1.6f, (sfloat)1.6f),
                new float3((sfloat)0.2f, (sfloat)1.2f, (sfloat)3.6f)
            };

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            Aabb aabb = collider.Value.CalculateAabb();

            Aabb expected = new Aabb()
            {
                Min = math.min(math.min(vertices[0], vertices[1]), vertices[2]),
                Max = math.max(math.max(vertices[0], vertices[1]), vertices[2])
            };

            TestUtils.AreEqual(expected.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test that the local AABB of a quad collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbLocalQuad()
        {
            float3[] quadVertices =
            {
                new float3(-(sfloat)4.5f, (sfloat)0.0f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)0.7f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)2.7f, (sfloat)1.0f),
                new float3(-(sfloat)3.4f, (sfloat)1.2f, (sfloat)1.0f)
            };
            var collider = PolygonCollider.CreateQuad(quadVertices[0], quadVertices[1], quadVertices[2], quadVertices[3]);
            Aabb aabb = collider.Value.CalculateAabb();
            Aabb expected = Aabb.CreateFromPoints(new float3x4(quadVertices[0], quadVertices[1], quadVertices[2], quadVertices[3]));

            TestUtils.AreEqual(expected.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed triangle collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbTransformedTriangle()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)1.8f, (sfloat)2.4f, (sfloat)4.6f),
                new float3((sfloat)1.4f, (sfloat)1.6f, (sfloat)1.6f),
                new float3((sfloat)0.2f, (sfloat)1.2f, (sfloat)3.6f)
            };

            float3 translation = new float3((sfloat)3.4f, (sfloat)2.5f, -(sfloat)1.1f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3((sfloat)1.1f, (sfloat)10.1f, -(sfloat)3.4f)), (sfloat)78.0f);

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            Aabb aabb = collider.Value.CalculateAabb(new RigidTransform(rotation, translation));

            for (int i = 0; i < 3; ++i)
            {
                vertices[i] = translation + math.mul(rotation, vertices[i]);
            }

            Aabb expected = new Aabb()
            {
                Min = math.min(math.min(vertices[0], vertices[1]), vertices[2]),
                Max = math.max(math.max(vertices[0], vertices[1]), vertices[2])
            };

            TestUtils.AreEqual(expected.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed quad collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbTransformedQuad()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)4.5f, (sfloat)0.0f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)0.7f, (sfloat)1.0f),
                new float3((sfloat)3.4f, (sfloat)2.7f, (sfloat)1.0f),
                new float3(-(sfloat)3.4f, (sfloat)1.2f, (sfloat)1.0f)
            };

            float3 translation = new float3(-(sfloat)3.4f, -(sfloat)2.5f, -(sfloat)1.1f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3((sfloat)11.1f, (sfloat)10.1f, -(sfloat)3.4f)), (sfloat)178.0f);

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            Aabb aabb = collider.Value.CalculateAabb(new RigidTransform(rotation, translation));

            for (int i = 0; i < 4; ++i)
            {
                vertices[i] = translation + math.mul(rotation, vertices[i]);
            }

            Aabb expected = Aabb.CreateFromPoints(new float3x4(vertices[0], vertices[1], vertices[2], vertices[3]));

            TestUtils.AreEqual(expected.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (sfloat)1e-3f);
        }

        //[Test] // #TODO: Add test back in once we have implemented this in Physics
        unsafe public void TestMassPropertiesTriangle()
        {
            // constructing the triangle to be parallel to the xy plane so we don't have to figure out the transformations first
            float3[] vertices =
            {
                new float3(-(sfloat)1.1f, -(sfloat)0.4f, (sfloat)0.0f),
                new float3((sfloat)0.8f, -(sfloat)0.1f, (sfloat)0.0f),
                new float3(-(sfloat)0.2f, (sfloat)1.3f, (sfloat)0.0f)
            };

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);

            float3 inertiaTensor = collider.Value.MassProperties.MassDistribution.InertiaTensor;
            float3 expectedInertiaTensor = calcTriangleInertiaTensor(vertices[0], vertices[1], vertices[2]);
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (sfloat)1e-3f);
        }

        //[Test] // #TODO: Add test back in once we have implemented this in Physics
        public void TestMassPropertiesQuad()
        {
            float3[] vertices =
            {
                new float3(-(sfloat)1.1f, -(sfloat)0.4f, (sfloat)0.0f),
                new float3((sfloat)0.8f, -(sfloat)0.1f, (sfloat)0.0f),
                new float3((sfloat)1.2f, (sfloat)1.3f, (sfloat)0.0f),
                new float3(-(sfloat)0.2f, (sfloat)1.3f, (sfloat)0.0f)
            };

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);

            float3 inertiaTensor = collider.Value.MassProperties.MassDistribution.InertiaTensor;
            float3 expectedInertiaTensor = calcQuadInertiaTensor(vertices[0], vertices[1], vertices[2], vertices[3]);
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (sfloat)1e-3f);
        }

        private float3 calcTriangleInertiaTensor(float3 v0, float3 v1, float3 v2)
        {
            // #TODO: Add function once inertia is properly computed in Physics
            return new float3((sfloat)0, (sfloat)0, (sfloat)0);
        }

        private float3 calcQuadInertiaTensor(float3 v0, float3 v1, float3 v2, float3 v3)
        {
            // #TODO: Add function once inertia is properly computed in Physics
            return new float3((sfloat)0, (sfloat)0, (sfloat)0);
        }

        #endregion
    }
}
