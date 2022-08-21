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
    /// Class collecting all tests for the <see cref="CapsuleCollider"/>
    /// </summary>
    class CapsuleColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                CapsuleCollider.Create(new CapsuleGeometry { Vertex0 = math.up(), Vertex1 = -math.up(), Radius = (sfloat)0.5f }).Dispose();
        }

        [Test]
        public void Capsule_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Test if all attributes are set as expected when creating a new <see cref="CapsuleCollider"/>.
        /// </summary>
        [Test]
        unsafe public void TestCapsuleColliderCreate()
        {
            var geometry = new CapsuleGeometry
            {
                Vertex0 = new float3((sfloat)1.45f, (sfloat)0.34f, -(sfloat)8.65f),
                Vertex1 = new float3((sfloat)100.45f, -(sfloat)80.34f, -(sfloat)8.65f),
                Radius = (sfloat)1.45f
            };
            var collider = CapsuleCollider.Create(geometry);
            var capsuleCollider = UnsafeUtility.AsRef<CapsuleCollider>(collider.GetUnsafePtr());
            Assert.AreEqual(ColliderType.Capsule, capsuleCollider.Type);
            Assert.AreEqual(CollisionType.Convex, capsuleCollider.CollisionType);
            TestUtils.AreEqual(geometry.Vertex0, capsuleCollider.Vertex0, sfloat.Zero);
            TestUtils.AreEqual(geometry.Vertex0, capsuleCollider.Geometry.Vertex0, sfloat.Zero);
            TestUtils.AreEqual(geometry.Vertex1, capsuleCollider.Vertex1, sfloat.Zero);
            TestUtils.AreEqual(geometry.Vertex1, capsuleCollider.Geometry.Vertex1, sfloat.Zero);
            TestUtils.AreEqual(geometry.Radius, capsuleCollider.Radius, sfloat.Zero);
            TestUtils.AreEqual(geometry.Radius, capsuleCollider.Geometry.Radius, sfloat.Zero);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CapsuleCollider_Create_WhenVertexInvalid_Throws(
            [Values(0, 1)] int errantArg,
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var v0 = math.select(default, new float3((sfloat)errantValue), errantArg == 0);
            var v1 = math.select(default, new float3((sfloat)errantValue), errantArg == 1);
            var geometry = new CapsuleGeometry { Vertex0 = v0, Vertex1 = v1 };

            var ex = Assert.Throws<ArgumentException>(() => CapsuleCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match($"Vertex{errantArg}"));
        }

        [Test]
        public void CapsuleCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CapsuleGeometry { Radius = (sfloat)errantValue };

            var ex = Assert.Throws<ArgumentException>(() => CapsuleCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CapsuleGeometry.Radius)));
        }

#endif

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test if the local AABB of the <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        [Test]
        public void TestCapsuleColliderCalculateAabbLocal()
        {
            sfloat radius = (sfloat)2.3f;
            sfloat length = (sfloat)5.5f;
            float3 p0 = new float3((sfloat)1.1f, (sfloat)2.2f, (sfloat)3.4f);
            float3 p1 = p0 + length * math.normalize(new float3((sfloat)1, (sfloat)1, (sfloat)1));
            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });

            Aabb expectedAabb = new Aabb();
            expectedAabb.Min = math.min(p0, p1) - new float3(radius);
            expectedAabb.Max = math.max(p0, p1) + new float3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test whether the AABB of the transformed <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        [Test]
        public void TestCapsuleColliderCalculateAabbTransformed()
        {
            sfloat radius = (sfloat)2.3f;
            sfloat length = (sfloat)5.5f;
            float3 p0 = new float3((sfloat)1.1f, (sfloat)2.2f, (sfloat)3.4f);
            float3 p1 = p0 + length * math.normalize(new float3((sfloat)1, (sfloat)1, (sfloat)1));
            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });

            float3 translation = new float3(-(sfloat)3.4f, (sfloat)0.5f, (sfloat)0.0f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3((sfloat)0.4f, (sfloat)0.0f, (sfloat)150.0f)), (sfloat)123.0f);

            Aabb expectedAabb = new Aabb();
            float3 p0Transformed = math.mul(rotation, p0) + translation;
            float3 p1Transformed = math.mul(rotation, p1) + translation;
            expectedAabb.Min = math.min(p0Transformed, p1Transformed) - new float3(radius);
            expectedAabb.Max = math.max(p0Transformed, p1Transformed) + new float3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb(new RigidTransform(rotation, translation));
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (sfloat)1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (sfloat)1e-3f);
        }

        /// <summary>
        /// Test whether the inertia tensor of the <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        /// <remarks>
        /// Used the formula from the following article as reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
        /// NOTE: There is an error in eq. 14 of the article: it should be H^2 / 4 instead of H^2 / 2 in Ixx and Izz.
        /// </remarks>
        [Test]
        public void TestCapsuleColliderMassProperties()
        {
            sfloat radius = (sfloat)2.3f;
            sfloat length = (sfloat)5.5f;
            float3 p0 = new float3((sfloat)1.1f, (sfloat)2.2f, (sfloat)3.4f);
            float3 p1 = p0 + length * math.normalize(new float3((sfloat)1, (sfloat)1, (sfloat)1));

            sfloat hemisphereMass = (sfloat)0.5f * (sfloat)4.0f / (sfloat)3.0f * (sfloat)math.PI * radius * radius * radius;
            sfloat cylinderMass = (sfloat)math.PI * radius * radius * length;
            sfloat totalMass = (sfloat)2.0f * hemisphereMass + cylinderMass;
            hemisphereMass /= totalMass;
            cylinderMass /= totalMass;

            sfloat itX = cylinderMass * (length * length / (sfloat)12.0f + radius * radius / (sfloat)4.0f) + (sfloat)2.0f * hemisphereMass * ((sfloat)2.0f * radius * radius / (sfloat)5.0f + length * length / (sfloat)4.0f + (sfloat)3.0f * length * radius / (sfloat)8.0f);
            sfloat itY = cylinderMass * radius * radius / (sfloat)2.0f + (sfloat)4.0f * hemisphereMass * radius * radius / (sfloat)5.0f;
            sfloat itZ = itX;
            float3 expectedInertiaTensor = new float3(itX, itY, itZ);

            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });
            float3 inertiaTensor = capsuleCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (sfloat)1e-3f);
        }

        #endregion
    }
}
