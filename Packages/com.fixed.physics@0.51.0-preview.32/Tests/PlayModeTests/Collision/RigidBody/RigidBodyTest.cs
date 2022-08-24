using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics.FixedPoint;
using Unity.Entities;

using Assert = UnityEngine.Assertions.Assert;

namespace Fixed.Physics.Tests.Collision.RigidBody
{
    class RigidBodyTest
    {
        [Test]
        public unsafe void RigidBodyCalculateAabb_BoxColliderTest()
        {
            var geometry = new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = (fp)1.0f,
                BevelRadius = (fp)0.2f
            };

            Physics.RigidBody rigidbodyBox = Fixed.Physics.RigidBody.Zero;
            rigidbodyBox.Collider = BoxCollider.Create(geometry);

            var boxAabb = rigidbodyBox.CalculateAabb();
            var boxCollider = (BoxCollider*)BoxCollider.Create(geometry).GetUnsafePtr();
            Assert.IsTrue(boxAabb.Equals(boxCollider->CalculateAabb()));
        }

        [Test]
        public unsafe void RigidBodyCalculateAabb_SphereColliderTest()
        {
            var geometry = new SphereGeometry
            {
                Center = fp3.zero,
                Radius = (fp)1.0f
            };

            Physics.RigidBody rigidbodySphere = Fixed.Physics.RigidBody.Zero;
            rigidbodySphere.Collider = SphereCollider.Create(geometry);

            var sphereAabb = rigidbodySphere.CalculateAabb();
            var sphere = (Collider*)SphereCollider.Create(geometry).GetUnsafePtr();
            Assert.IsTrue(sphereAabb.Equals(sphere->CalculateAabb()));
        }

        [Test]
        public unsafe void RigidBodyCastRayTest()
        {
            Physics.RigidBody rigidbody = Fixed.Physics.RigidBody.Zero;

            fp size = (fp)1.0f;
            fp convexRadius = (fp)0.0f;

            var rayStartOK = new fp3(-(fp)10, -(fp)10, -(fp)10);
            var rayEndOK = new fp3((fp)10, (fp)10, (fp)10);

            var rayStartFail = new fp3(-(fp)10, (fp)10, -(fp)10);
            var rayEndFail = new fp3((fp)10, (fp)10, (fp)10);

            rigidbody.Collider = BoxCollider.Create(new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = size,
                BevelRadius = convexRadius
            });

            var raycastInput = new RaycastInput();
            var closestHit = new RaycastHit();
            var allHits = new NativeList<RaycastHit>(Allocator.Temp);

            // OK case : Ray hits the box collider
            raycastInput.Start = rayStartOK;
            raycastInput.End = rayEndOK;
            raycastInput.Filter = CollisionFilter.Default;

            Assert.IsTrue(rigidbody.CastRay(raycastInput));
            Assert.IsTrue(rigidbody.CastRay(raycastInput, out closestHit));
            Assert.IsTrue(rigidbody.CastRay(raycastInput, ref allHits));

            // Fail Case : wrong direction
            raycastInput.Start = rayStartFail;
            raycastInput.End = rayEndFail;

            Assert.IsFalse(rigidbody.CastRay(raycastInput));
            Assert.IsFalse(rigidbody.CastRay(raycastInput, out closestHit));
            Assert.IsFalse(rigidbody.CastRay(raycastInput, ref allHits));
        }

        [Test]
        public unsafe void RigidBodyCastColliderTest()
        {
            Physics.RigidBody rigidbody = Fixed.Physics.RigidBody.Zero;

            fp size = (fp)1.0f;
            fp convexRadius = (fp)0.0f;
            fp sphereRadius = (fp)1.0f;

            var rayStartOK = new fp3(-(fp)10, -(fp)10, -(fp)10);
            var rayEndOK = new fp3((fp)10, (fp)10, (fp)10);

            var rayStartFail = new fp3(-(fp)10, (fp)10, -(fp)10);
            var rayEndFail = new fp3((fp)10, (fp)10, (fp)10);

            rigidbody.Collider = BoxCollider.Create(new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = size,
                BevelRadius = convexRadius
            });

            var colliderCastInput = new ColliderCastInput();
            var closestHit = new ColliderCastHit();
            var allHits = new NativeList<ColliderCastHit>(Allocator.Temp);

            // OK case : Sphere hits the box collider
            colliderCastInput.Start = rayStartOK;
            colliderCastInput.End = rayEndOK;
            colliderCastInput.Collider = (Collider*)SphereCollider.Create(
                new SphereGeometry { Center = fp3.zero, Radius = sphereRadius }
                ).GetUnsafePtr();

            Assert.IsTrue(rigidbody.CastCollider(colliderCastInput));
            Assert.IsTrue(rigidbody.CastCollider(colliderCastInput, out closestHit));
            Assert.IsTrue(rigidbody.CastCollider(colliderCastInput, ref allHits));

            // Fail case : wrong direction
            colliderCastInput.Start = rayStartFail;
            colliderCastInput.End = rayEndFail;

            Assert.IsFalse(rigidbody.CastCollider(colliderCastInput));
            Assert.IsFalse(rigidbody.CastCollider(colliderCastInput, out closestHit));
            Assert.IsFalse(rigidbody.CastCollider(colliderCastInput, ref allHits));
        }

        [Test]
        public unsafe void RigidBodyCalculateDistancePointTest()
        {
            Physics.RigidBody rigidbody = Fixed.Physics.RigidBody.Zero;

            fp size = (fp)1.0f;
            fp convexRadius = (fp)0.0f;

            var queryPos = new fp3(-(fp)10, -(fp)10, -(fp)10);

            rigidbody.Collider = BoxCollider.Create(new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = size,
                BevelRadius = convexRadius
            });

            var pointDistanceInput = new PointDistanceInput();

            pointDistanceInput.Position = queryPos;
            pointDistanceInput.Filter = CollisionFilter.Default;

            var closestHit = new DistanceHit();
            var allHits = new NativeList<DistanceHit>(Allocator.Temp);

            // OK case : with enough max distance
            pointDistanceInput.MaxDistance = (fp)10000.0f;
            Assert.IsTrue(rigidbody.CalculateDistance(pointDistanceInput));
            Assert.IsTrue(rigidbody.CalculateDistance(pointDistanceInput, out closestHit));
            Assert.IsTrue(rigidbody.CalculateDistance(pointDistanceInput, ref allHits));

            // Fail case : not enough max distance
            pointDistanceInput.MaxDistance = (fp)1;
            Assert.IsFalse(rigidbody.CalculateDistance(pointDistanceInput));
            Assert.IsFalse(rigidbody.CalculateDistance(pointDistanceInput, out closestHit));
            Assert.IsFalse(rigidbody.CalculateDistance(pointDistanceInput, ref allHits));
        }

        [Test]
        public unsafe void RigidBodyCalculateDistanceTest()
        {
            fp size = (fp)1.0f;
            fp convexRadius = (fp)0.0f;
            fp sphereRadius = (fp)1.0f;

            var queryPos = new fp3(-(fp)10, -(fp)10, -(fp)10);

            BlobAssetReference<Collider> boxCollider = BoxCollider.Create(new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = size,
                BevelRadius = convexRadius
            });
            BlobAssetReference<Collider> sphereCollider = SphereCollider.Create(new SphereGeometry
            {
                Center = fp3.zero,
                Radius = sphereRadius
            });

            var rigidBody = new Physics.RigidBody
            {
                WorldFromBody = FpRigidTransform.identity,
                Collider = boxCollider
            };

            var colliderDistanceInput = new ColliderDistanceInput
            {
                Collider = (Collider*)sphereCollider.GetUnsafePtr(),
                Transform = new FpRigidTransform(fpquaternion.identity, queryPos)
            };

            var closestHit = new DistanceHit();
            var allHits = new NativeList<DistanceHit>(Allocator.Temp);

            // OK case : with enough max distance
            colliderDistanceInput.MaxDistance = (fp)10000.0f;
            Assert.IsTrue(rigidBody.CalculateDistance(colliderDistanceInput));
            Assert.IsTrue(rigidBody.CalculateDistance(colliderDistanceInput, out closestHit));
            Assert.IsTrue(rigidBody.CalculateDistance(colliderDistanceInput, ref allHits));

            // Fail case : not enough max distance
            colliderDistanceInput.MaxDistance = (fp)1;
            Assert.IsFalse(rigidBody.CalculateDistance(colliderDistanceInput));
            Assert.IsFalse(rigidBody.CalculateDistance(colliderDistanceInput, out closestHit));
            Assert.IsFalse(rigidBody.CalculateDistance(colliderDistanceInput, ref allHits));

            boxCollider.Dispose();
            sphereCollider.Dispose();
        }
    }
}
