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
    /// Contains all test for the <see cref=" PolygonCollider"/>
    /// </summary>
    class PolygonColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateQuadFromBurstJob : IJob
        {
            public void Execute() =>
                PolygonCollider.CreateQuad(new fp3(-(fp)1f, (fp)1f, (fp)0f), new fp3((fp)1f, (fp)1f, (fp)0f), new fp3((fp)1f, -(fp)1f, (fp)0f), new fp3(-(fp)1f, -(fp)1f, (fp)0f)).Dispose();
        }

        [Test]
        public void PolygonCollider_CreateQuad_WhenCalledFromBurstJob_DoesNotThrow() => new CreateQuadFromBurstJob().Run();

        [BurstCompile(CompileSynchronously = true)]
        struct CreateTriangleFromBurstJob : IJob
        {
            public void Execute() =>
                PolygonCollider.CreateTriangle(new fp3(-(fp)1f, (fp)1f, (fp)0f), new fp3((fp)1f, (fp)1f, (fp)0f), new fp3((fp)1f, -(fp)1f, (fp)0f)).Dispose();
        }

        [Test]
        public void PolygonCollider_CreateTriangle_WhenCalledFromBurstJob_DoesNotThrow() => new CreateTriangleFromBurstJob().Run();

        /// <summary>
        /// Test whether a triangle collider's attributes are set to the expected values after creation.
        /// </summary>
        [Test]
        unsafe public void TestCreateTriangle()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)1.4f, (fp)1.4f, (fp)5.6f),
                new fp3((fp)1.4f, (fp)1.4f, (fp)3.6f),
                new fp3((fp)0.2f, (fp)1.2f, (fp)5.6f)
            };
            fp3 normal = fpmath.normalize(fpmath.cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            var triangleCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsTrue(triangleCollider.IsTriangle);
            Assert.IsFalse(triangleCollider.IsQuad);

            TestUtils.AreEqual(triangleCollider.Vertices[0], vertices[0], (fp)1e-3f);
            TestUtils.AreEqual(triangleCollider.Vertices[1], vertices[1], (fp)1e-3f);
            TestUtils.AreEqual(triangleCollider.Vertices[2], vertices[2], (fp)1e-3f);
            Assert.AreEqual(2, triangleCollider.Planes.Length);
            TestUtils.AreEqual(normal, triangleCollider.Planes[0].Normal, (fp)1e-3f);
            TestUtils.AreEqual(-normal, triangleCollider.Planes[1].Normal, (fp)1e-3f);
            Assert.AreEqual(ColliderType.Triangle, triangleCollider.Type);
            Assert.AreEqual(CollisionType.Convex, triangleCollider.CollisionType);
        }

        /// <summary>
        /// Test whether a quad collider's attributes are set correctly after creation.
        /// </summary>
        [Test]
        unsafe public void TestCreateQuad()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)4.5f, (fp)0.0f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)0.7f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)2.7f, (fp)1.0f),
                new fp3(-(fp)3.4f, (fp)1.2f, (fp)1.0f)
            };
            fp3 normal = fpmath.normalize(fpmath.cross(vertices[2] - vertices[1], vertices[0] - vertices[1]));

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            var quadCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsFalse(quadCollider.IsTriangle);
            Assert.IsTrue(quadCollider.IsQuad);

            TestUtils.AreEqual(quadCollider.Vertices[0], vertices[0], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[1], vertices[1], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[2], vertices[2], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[3], vertices[3], (fp)1e-3f);
            Assert.AreEqual(2, quadCollider.Planes.Length);
            TestUtils.AreEqual(normal, quadCollider.Planes[0].Normal, (fp)1e-3f);
            TestUtils.AreEqual(-normal, quadCollider.Planes[1].Normal, (fp)1e-3f);
            Assert.AreEqual(ColliderType.Quad, quadCollider.Type);
            Assert.AreEqual(CollisionType.Convex, quadCollider.CollisionType);
        }

        /// <summary>
        /// Test that 'unsorted', i.e. neither clockwise nor counter-clockwise winding, quads are created correctly
        /// </summary>
        [Test]
        unsafe public void TestCreateQuadUnsorted()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)4.5f, (fp)0.0f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)2.7f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)0.7f, (fp)1.0f),
                new fp3(-(fp)3.4f, (fp)1.2f, (fp)1.0f)
            };
            fp3 normal = fpmath.normalize(fpmath.cross(vertices[2] - vertices[1], vertices[0] - vertices[1]));

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            var quadCollider = UnsafeUtility.AsRef<PolygonCollider>(collider.GetUnsafePtr());
            Assert.IsFalse(quadCollider.IsTriangle);
            Assert.IsTrue(quadCollider.IsQuad);

            TestUtils.AreEqual(quadCollider.Vertices[0], vertices[0], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[1], vertices[1], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[2], vertices[2], (fp)1e-3f);
            TestUtils.AreEqual(quadCollider.Vertices[3], vertices[3], (fp)1e-3f);
            Assert.AreEqual(2, quadCollider.Planes.Length);
            TestUtils.AreEqual(normal, quadCollider.Planes[0].Normal, (fp)1e-3f);
            TestUtils.AreEqual(-normal, quadCollider.Planes[1].Normal, (fp)1e-3f);
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
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 0),
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 1),
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 2)
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
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 0),
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 1),
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 2),
                fpmath.select(default, new fp3((fp)errantValue), errantArg == 3)
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
                    new fp3((fp)v00, (fp)v01, (fp)v02),
                    new fp3((fp)v10, (fp)v11, (fp)v12),
                    new fp3((fp)v20, (fp)v21, (fp)v22),
                    new fp3((fp)v30, (fp)v31, (fp)v32)
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
            fp3[] vertices =
            {
                new fp3(-(fp)1.8f, (fp)2.4f, (fp)4.6f),
                new fp3((fp)1.4f, (fp)1.6f, (fp)1.6f),
                new fp3((fp)0.2f, (fp)1.2f, (fp)3.6f)
            };

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            Aabb aabb = collider.Value.CalculateAabb();

            Aabb expected = new Aabb()
            {
                Min = fpmath.min(fpmath.min(vertices[0], vertices[1]), vertices[2]),
                Max = fpmath.max(fpmath.max(vertices[0], vertices[1]), vertices[2])
            };

            TestUtils.AreEqual(expected.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test that the local AABB of a quad collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbLocalQuad()
        {
            fp3[] quadVertices =
            {
                new fp3(-(fp)4.5f, (fp)0.0f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)0.7f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)2.7f, (fp)1.0f),
                new fp3(-(fp)3.4f, (fp)1.2f, (fp)1.0f)
            };
            var collider = PolygonCollider.CreateQuad(quadVertices[0], quadVertices[1], quadVertices[2], quadVertices[3]);
            Aabb aabb = collider.Value.CalculateAabb();
            Aabb expected = Aabb.CreateFromPoints(new fp3x4(quadVertices[0], quadVertices[1], quadVertices[2], quadVertices[3]));

            TestUtils.AreEqual(expected.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed triangle collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbTransformedTriangle()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)1.8f, (fp)2.4f, (fp)4.6f),
                new fp3((fp)1.4f, (fp)1.6f, (fp)1.6f),
                new fp3((fp)0.2f, (fp)1.2f, (fp)3.6f)
            };

            fp3 translation = new fp3((fp)3.4f, (fp)2.5f, -(fp)1.1f);
            fpquaternion rotation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)1.1f, (fp)10.1f, -(fp)3.4f)), (fp)78.0f);

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);
            Aabb aabb = collider.Value.CalculateAabb(new FpRigidTransform(rotation, translation));

            for (int i = 0; i < 3; ++i)
            {
                vertices[i] = translation + fpmath.mul(rotation, vertices[i]);
            }

            Aabb expected = new Aabb()
            {
                Min = fpmath.min(fpmath.min(vertices[0], vertices[1]), vertices[2]),
                Max = fpmath.max(fpmath.max(vertices[0], vertices[1]), vertices[2])
            };

            TestUtils.AreEqual(expected.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed quad collider is calculated correctly
        /// </summary>
        [Test]
        public void TestCalculateAabbTransformedQuad()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)4.5f, (fp)0.0f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)0.7f, (fp)1.0f),
                new fp3((fp)3.4f, (fp)2.7f, (fp)1.0f),
                new fp3(-(fp)3.4f, (fp)1.2f, (fp)1.0f)
            };

            fp3 translation = new fp3(-(fp)3.4f, -(fp)2.5f, -(fp)1.1f);
            fpquaternion rotation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)11.1f, (fp)10.1f, -(fp)3.4f)), (fp)178.0f);

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
            Aabb aabb = collider.Value.CalculateAabb(new FpRigidTransform(rotation, translation));

            for (int i = 0; i < 4; ++i)
            {
                vertices[i] = translation + fpmath.mul(rotation, vertices[i]);
            }

            Aabb expected = Aabb.CreateFromPoints(new fp3x4(vertices[0], vertices[1], vertices[2], vertices[3]));

            TestUtils.AreEqual(expected.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expected.Max, aabb.Max, (fp)1e-3f);
        }

        //[Test] // #TODO: Add test back in once we have implemented this in Physics
        unsafe public void TestMassPropertiesTriangle()
        {
            // constructing the triangle to be parallel to the xy plane so we don't have to figure out the transformations first
            fp3[] vertices =
            {
                new fp3(-(fp)1.1f, -(fp)0.4f, (fp)0.0f),
                new fp3((fp)0.8f, -(fp)0.1f, (fp)0.0f),
                new fp3(-(fp)0.2f, (fp)1.3f, (fp)0.0f)
            };

            var collider = PolygonCollider.CreateTriangle(vertices[0], vertices[1], vertices[2]);

            fp3 inertiaTensor = collider.Value.MassProperties.MassDistribution.InertiaTensor;
            fp3 expectedInertiaTensor = calcTriangleInertiaTensor(vertices[0], vertices[1], vertices[2]);
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (fp)1e-3f);
        }

        //[Test] // #TODO: Add test back in once we have implemented this in Physics
        public void TestMassPropertiesQuad()
        {
            fp3[] vertices =
            {
                new fp3(-(fp)1.1f, -(fp)0.4f, (fp)0.0f),
                new fp3((fp)0.8f, -(fp)0.1f, (fp)0.0f),
                new fp3((fp)1.2f, (fp)1.3f, (fp)0.0f),
                new fp3(-(fp)0.2f, (fp)1.3f, (fp)0.0f)
            };

            var collider = PolygonCollider.CreateQuad(vertices[0], vertices[1], vertices[2], vertices[3]);

            fp3 inertiaTensor = collider.Value.MassProperties.MassDistribution.InertiaTensor;
            fp3 expectedInertiaTensor = calcQuadInertiaTensor(vertices[0], vertices[1], vertices[2], vertices[3]);
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (fp)1e-3f);
        }

        private fp3 calcTriangleInertiaTensor(fp3 v0, fp3 v1, fp3 v2)
        {
            // #TODO: Add function once inertia is properly computed in Physics
            return new fp3((fp)0, (fp)0, (fp)0);
        }

        private fp3 calcQuadInertiaTensor(fp3 v0, fp3 v1, fp3 v2, fp3 v3)
        {
            // #TODO: Add function once inertia is properly computed in Physics
            return new fp3((fp)0, (fp)0, (fp)0);
        }

        #endregion
    }
}
