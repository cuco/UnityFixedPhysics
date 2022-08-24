using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.Tests.Collision.Colliders
{
    class CylinderColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() => CylinderCollider.Create(new CylinderGeometry
            {
                Orientation = fpquaternion.identity,
                Height = (fp)1f,
                Radius = (fp)1f,
                SideCount = CylinderGeometry.MaxSideCount
            }).Dispose();
        }

        [Test]
        public void CylinderCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        [Test]
        public unsafe void CylinderCollider_Create_ResultHasExpectedValues()
        {
            var geometry = new CylinderGeometry
            {
                Center = new fp3(-(fp)10.10f, (fp)10.12f, (fp)0.01f),
                Orientation = fpquaternion.AxisAngle(fpmath.normalize(new fp3((fp)1.4f, (fp)0.2f, (fp)1.1f)), (fp)38.50f),
                Height = (fp)2f,
                Radius = (fp)0.25f,
                BevelRadius = (fp)0.05f,
                SideCount = 10
            };

            var collider = CylinderCollider.Create(geometry);
            var cylinderCollider = UnsafeUtility.AsRef<CylinderCollider>(collider.GetUnsafePtr());

            Assert.AreEqual(geometry.Center, cylinderCollider.Center);
            Assert.AreEqual(geometry.Center, cylinderCollider.Geometry.Center);
            Assert.AreEqual(geometry.Orientation, cylinderCollider.Orientation);
            Assert.AreEqual(geometry.Orientation, cylinderCollider.Geometry.Orientation);
            Assert.AreEqual(geometry.Height, cylinderCollider.Height);
            Assert.AreEqual(geometry.Height, cylinderCollider.Geometry.Height);
            Assert.AreEqual(geometry.Radius, cylinderCollider.Radius);
            Assert.AreEqual(geometry.Radius, cylinderCollider.Geometry.Radius);
            Assert.AreEqual(geometry.BevelRadius, cylinderCollider.BevelRadius);
            Assert.AreEqual(geometry.BevelRadius, cylinderCollider.Geometry.BevelRadius);
            Assert.AreEqual(CollisionType.Convex, cylinderCollider.CollisionType);
            Assert.AreEqual(ColliderType.Cylinder, cylinderCollider.Type);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CylinderCollider_Create_WhenCenterInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Center = new fp3((fp)errantValue),
                Orientation = fpquaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Center)));
        }

        [Test]
        public void CylinderCollider_Create_WhenOrientationInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, 0f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Orientation = new fpquaternion((fp)0f, (fp)0f, (fp)0f, (fp)errantValue),
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Orientation)));
        }

        [Test]
        public void CylinderCollider_Create_WhenHeightInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Height = (fp)errantValue,
                Orientation = fpquaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Height)));
        }

        [Test]
        public void CylinderCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Radius = (fp)errantValue,
                Orientation = fpquaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Radius)));
        }

        [Test]
        public void CylinderCollider_Create_WhenBevelRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f, 0.55f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Height = (fp)1f,
                Radius = fp.half,
                Orientation = fpquaternion.identity,
                BevelRadius = (fp)errantValue,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.BevelRadius)));
        }

#endif

        #endregion
    }
}
