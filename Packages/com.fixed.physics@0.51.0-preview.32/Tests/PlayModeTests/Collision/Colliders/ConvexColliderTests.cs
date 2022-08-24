using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Tests.Utils;
using Random = Unity.Mathematics.FixedPoint.Random;

namespace Fixed.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Contains all <see cref="ConvexCollider"/> unit tests
    /// </summary>
    class ConvexColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute()
            {
                var points = new NativeArray<fp3>(1024, Allocator.Temp);
                var random = new Random(1234);
                for (var i = 0; i < points.Length; ++i)
                    points[i] = random.Nextfp3(new fp3(-(fp)1f), new fp3((fp)1f));
                ConvexCollider.Create(points, ConvexHullGenerationParameters.Default).Dispose();
            }
        }

        [Test]
        public void ConvexCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        static readonly fp3[] k_TestPoints =
        {
            new fp3((fp)1.45f, (fp)8.67f, (fp)3.45f),
            new fp3((fp)8.75f, (fp)1.23f, (fp)6.44f),
            new fp3((fp)100.34f, (fp)5.33f, -(fp)2.55f),
            new fp3((fp)8.76f, (fp)4.56f, -(fp)4.54f),
            new fp3((fp)9.75f, -(fp)0.45f, -(fp)8.99f),
            new fp3((fp)7.66f, (fp)3.44f, (fp)0.0f)
        };

        /// <summary>
        /// Test that a <see cref="ConvexCollider"/> created with a point cloud has its attributes filled correctly
        /// </summary>
        [Test]
        public void TestConvexColliderCreate()
        {
            var points = new NativeArray<fp3>(k_TestPoints, Allocator.Temp);
            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = (fp)0.15f }, CollisionFilter.Default
            );

            Assert.AreEqual(ColliderType.Convex, collider.Value.Type);
            Assert.AreEqual(CollisionType.Convex, collider.Value.CollisionType);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void ConvexCollider_Create_WhenPointsInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var points = new NativeArray<fp3>(k_TestPoints, Allocator.Temp) { [2] = new fp3((fp)errantValue) };

            var ex = Assert.Throws<ArgumentException>(() => ConvexCollider.Create(points, ConvexHullGenerationParameters.Default));
            Assert.That(ex.ParamName, Is.EqualTo("points"));
        }

        [Test]
        public void ConvexCollider_Create_WhenGenerationParametersBevelRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var points = new NativeArray<fp3>(k_TestPoints, Allocator.Temp);
            var generationParameters = ConvexHullGenerationParameters.Default;
            generationParameters.BevelRadius = (fp)errantValue;

            var ex = Assert.Throws<ArgumentException>(() => ConvexCollider.Create(points, generationParameters));
            Assert.That(ex.ParamName, Is.EqualTo("generationParameters"));
        }

#endif

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public unsafe void TestConvexColliderCalculateAabbLocal([Values(0, 0.01f, 1.25f)] float maxShrinkMovement)
        {
            var points = new NativeArray<fp3>(6, Allocator.TempJob)
            {
                [0] = new fp3((fp)1.45f, (fp)8.67f, (fp)3.45f),
                [1] = new fp3((fp)8.75f, (fp)1.23f, (fp)6.44f),
                [2] = new fp3((fp)100.34f, (fp)5.33f, -(fp)2.55f),
                [3] = new fp3((fp)8.76f, (fp)4.56f, -(fp)4.54f),
                [4] = new fp3((fp)9.75f, -(fp)0.45f, -(fp)8.99f),
                [5] = new fp3((fp)7.66f, (fp)3.44f, (fp)0.0f)
            };

            Aabb expectedAabb = Aabb.CreateFromPoints(new fp3x4(points[0], points[1], points[2], points[3]));
            expectedAabb.Include(points[4]);
            expectedAabb.Include(points[5]);

            //TODO 计算有问题
            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = (fp)maxShrinkMovement }, CollisionFilter.Default
            );
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb();
            fp convexRadius = ((ConvexCollider*)collider.GetUnsafePtr())->ConvexHull.ConvexRadius;
            fp maxError = (fp)1e-3f + (fp)maxShrinkMovement - convexRadius;
            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, maxError);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, maxError);
        }

        /// <summary>
        /// Test that the transformed AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public unsafe void TestConvexColliderCalculateAabbTransformed([Values(0, 0.01f, 1.25f)] float maxShrinkMovement)
        {
            var points = new NativeArray<fp3>(6, Allocator.TempJob)
            {
                [0] = new fp3((fp)1.45f, (fp)8.67f, (fp)3.45f),
                [1] = new fp3((fp)8.75f, (fp)1.23f, (fp)6.44f),
                [2] = new fp3((fp)100.34f, (fp)5.33f, -(fp)2.55f),
                [3] = new fp3((fp)8.76f, (fp)4.56f, -(fp)4.54f),
                [4] = new fp3((fp)9.75f, -(fp)0.45f, -(fp)8.99f),
                [5] = new fp3((fp)7.66f, (fp)3.44f, (fp)0.0f)
            };

            fp3 translation = new fp3((fp)43.56f, -(fp)87.32f, -(fp)0.02f);
            fpquaternion rotation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)8.45f, -(fp)2.34f, (fp)0.82f)), (fp)43.21f);

            fp3[] transformedPoints = new fp3[points.Length];
            for (int i = 0; i < points.Length; ++i)
            {
                transformedPoints[i] = translation + fpmath.mul(rotation, points[i]);
            }

            Aabb expectedAabb = Aabb.CreateFromPoints(new fp3x4(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]));
            expectedAabb.Include(transformedPoints[4]);
            expectedAabb.Include(transformedPoints[5]);

            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = (fp)maxShrinkMovement }, CollisionFilter.Default
            );
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb(new FpRigidTransform(rotation, translation));
            fp convexRadius = ((ConvexCollider*)collider.GetUnsafePtr())->ConvexHull.ConvexRadius;
            fp maxError = (fp)1e-3f + (fp)maxShrinkMovement - convexRadius;

            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, maxError);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, maxError);
        }

        #endregion
    }
}
