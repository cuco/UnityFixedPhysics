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
    /// Class collecting all tests for the <see cref="CapsuleCollider"/>
    /// </summary>
    class CapsuleColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                CapsuleCollider.Create(new CapsuleGeometry { Vertex0 = fpmath.up(), Vertex1 = -fpmath.up(), Radius = fp.half }).Dispose();
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
                Vertex0 = new fp3((fp)1.45f, (fp)0.34f, -(fp)8.65f),
                Vertex1 = new fp3((fp)100.45f, -(fp)80.34f, -(fp)8.65f),
                Radius = (fp)1.45f
            };
            var collider = CapsuleCollider.Create(geometry);
            var capsuleCollider = UnsafeUtility.AsRef<CapsuleCollider>(collider.GetUnsafePtr());
            Assert.AreEqual(ColliderType.Capsule, capsuleCollider.Type);
            Assert.AreEqual(CollisionType.Convex, capsuleCollider.CollisionType);
            TestUtils.AreEqual(geometry.Vertex0, capsuleCollider.Vertex0, fp.zero);
            TestUtils.AreEqual(geometry.Vertex0, capsuleCollider.Geometry.Vertex0, fp.zero);
            TestUtils.AreEqual(geometry.Vertex1, capsuleCollider.Vertex1, fp.zero);
            TestUtils.AreEqual(geometry.Vertex1, capsuleCollider.Geometry.Vertex1, fp.zero);
            TestUtils.AreEqual(geometry.Radius, capsuleCollider.Radius, fp.zero);
            TestUtils.AreEqual(geometry.Radius, capsuleCollider.Geometry.Radius, fp.zero);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CapsuleCollider_Create_WhenVertexInvalid_Throws(
            [Values(0, 1)] int errantArg,
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var v0 = fpmath.select(default, new fp3((fp)errantValue), errantArg == 0);
            var v1 = fpmath.select(default, new fp3((fp)errantValue), errantArg == 1);
            var geometry = new CapsuleGeometry { Vertex0 = v0, Vertex1 = v1 };

            var ex = Assert.Throws<ArgumentException>(() => CapsuleCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match($"Vertex{errantArg}"));
        }

        [Test]
        public void CapsuleCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CapsuleGeometry { Radius = (fp)errantValue };

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
            fp radius = (fp)2.3f;
            fp length = (fp)5.5f;
            fp3 p0 = new fp3((fp)1.1f, (fp)2.2f, (fp)3.4f);
            fp3 p1 = p0 + length * fpmath.normalize(new fp3((fp)1, (fp)1, (fp)1));
            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });

            Aabb expectedAabb = new Aabb();
            expectedAabb.Min = fpmath.min(p0, p1) - new fp3(radius);
            expectedAabb.Max = fpmath.max(p0, p1) + new fp3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test whether the AABB of the transformed <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        [Test]
        public void TestCapsuleColliderCalculateAabbTransformed()
        {
            fp radius = (fp)2.3f;
            fp length = (fp)5.5f;
            fp3 p0 = new fp3((fp)1.1f, (fp)2.2f, (fp)3.4f);
            fp3 p1 = p0 + length * fpmath.normalize(new fp3((fp)1, (fp)1, (fp)1));
            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });

            fp3 translation = new fp3(-(fp)3.4f, fp.half, (fp)0.0f);
            fpquaternion rotation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)0.4f, (fp)0.0f, (fp)150.0f)), (fp)123.0f);

            Aabb expectedAabb = new Aabb();
            fp3 p0Transformed = fpmath.mul(rotation, p0) + translation;
            fp3 p1Transformed = fpmath.mul(rotation, p1) + translation;
            expectedAabb.Min = fpmath.min(p0Transformed, p1Transformed) - new fp3(radius);
            expectedAabb.Max = fpmath.max(p0Transformed, p1Transformed) + new fp3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb(new FpRigidTransform(rotation, translation));
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, (fp)1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, (fp)1e-3f);
        }

        /// <summary>
        /// Test whether the inertia tensor of the <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        /// <remarks>
        /// Used the formula from the following article as reference: https://www.gamedev.net/articles/programming/fpmath-and-physics/capsule-inertia-tensor-r3856/
        /// NOTE: There is an error in eq. 14 of the article: it should be H^2 / 4 instead of H^2 / 2 in Ixx and Izz.
        /// </remarks>
        [Test]
        public void TestCapsuleColliderMassProperties()
        {
            fp radius = (fp)2.3f;
            fp length = (fp)5.5f;
            fp3 p0 = new fp3((fp)1.1f, (fp)2.2f, (fp)3.4f);
            fp3 p1 = p0 + length * fpmath.normalize(new fp3((fp)1, (fp)1, (fp)1));

            fp hemisphereMass = fp.half * (fp)4.0f / (fp)3.0f * (fp)fpmath.PI * radius * radius * radius;
            fp cylinderMass = (fp)fpmath.PI * radius * radius * length;
            fp totalMass = fp.two * hemisphereMass + cylinderMass;
            hemisphereMass /= totalMass;
            cylinderMass /= totalMass;

            fp itX = cylinderMass * (length * length / (fp)12.0f + radius * radius / (fp)4.0f) + fp.two * hemisphereMass * ((fp)2.0f * radius * radius / (fp)5.0f + length * length / (fp)4.0f + (fp)3.0f * length * radius / (fp)8.0f);
            fp itY = cylinderMass * radius * radius / fp.two + (fp)4.0f * hemisphereMass * radius * radius / (fp)5.0f;
            fp itZ = itX;
            fp3 expectedInertiaTensor = new fp3(itX, itY, itZ);

            var capsuleCollider = CapsuleCollider.Create(new CapsuleGeometry
            {
                Vertex0 = p0,
                Vertex1 = p1,
                Radius = radius
            });
            fp3 inertiaTensor = capsuleCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, (fp)1e-3f);
        }

        #endregion
    }
}
