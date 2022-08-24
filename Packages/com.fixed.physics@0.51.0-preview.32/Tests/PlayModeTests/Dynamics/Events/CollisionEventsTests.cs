using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.Tests.Dynamics.CollisionEvents
{
    class CollisionEventsTests
    {
        // Tests the situation where none of the contact points
        // reaches the contact plane after sub-integration using
        // input velocities. This happens due to other bodies actually
        // causing the collision, not the input velocities.
        // In that case, we want to return the closest point,
        // and we verify here that only 1 point is returned.
        [Test]
        public void CalculateDetailsTest()
        {
            // Simple collision event with straight normal and 3 contact points
            CollisionEventDataRef collisionEventData;
            unsafe
            {
                int length = CollisionEventData.CalculateSize(3);
                collisionEventData = new CollisionEventDataRef((CollisionEventData*)(UnsafeUtility.Malloc(length, 1, Allocator.Temp)));
            }
            collisionEventData.Value.BodyIndices.BodyIndexA = 0;
            collisionEventData.Value.BodyIndices.BodyIndexB = 1;
            collisionEventData.Value.ColliderKeys.ColliderKeyA = ColliderKey.Empty;
            collisionEventData.Value.ColliderKeys.ColliderKeyB = ColliderKey.Empty;
            collisionEventData.Value.Normal = new fp3((fp)0.0f, -(fp)1.00000f, (fp)0.0f);
            collisionEventData.Value.NumNarrowPhaseContactPoints = 3;
            collisionEventData.Value.SolverImpulse = (fp)1.0f;

            // Initialize 3 contact points
            collisionEventData.Value.AccessContactPoint(0) =
                new ContactPoint
            {
                Distance = (fp)0.177905f,
                Position = new fp3(-(fp)22.744950f, (fp)2.585318f, -(fp)50.108990f)
            };
            collisionEventData.Value.AccessContactPoint(1) =
                new ContactPoint
            {
                Distance = (fp)0.276652f,
                Position = new fp3(-(fp)20.731140f, (fp)2.486506f, -(fp)50.322240f)
            };
            collisionEventData.Value.AccessContactPoint(2) =
                new ContactPoint
            {
                Distance = (fp)0.278534f,
                Position = new fp3(-(fp)20.766140f, (fp)2.484623f, -(fp)50.652630f)
            };

            // Wrapping collision event
            CollisionEvent collisionEvent = new CollisionEvent();
            collisionEvent.EventData = collisionEventData;
            collisionEvent.TimeStep = (fp)1.0f / (fp)60.0f;

            // Input velocity is obviously separating, but high angular velocity still caused an event
            collisionEvent.InputVelocityA = new Velocity
            {
                Angular = new fp3(-(fp)0.00064f, (fp)11.17604f, (fp)0.02133f),
                Linear = new fp3(-(fp)3.81205f, -(fp)0.56607f, (fp)9.14945f)
            };
            collisionEvent.InputVelocityB = new Velocity
            {
                Angular = new fp3((fp)0.00000f, (fp)0.00000f, (fp)0.00000f),
                Linear = new fp3((fp)0.00000f, (fp)0.00000f, (fp)0.00000f)
            };

            // Allocate a simple world of 1 dynamic and 1 static body
            var simpleWorld = new Physics.PhysicsWorld(1, 1, 0);
            var motionVelocities = simpleWorld.MotionVelocities;
            var motionDatas = simpleWorld.MotionDatas;
            motionDatas[0] = new MotionData
            {
                LinearDamping = (fp)0.0f,
                AngularDamping = (fp)0.0f,
                BodyFromMotion = new FpRigidTransform(new fpquaternion((fp)0.0f, (fp)0.0f, (fp)0.0f, (fp)1.0f), new fp3((fp)0.0f, (fp)0.0f, (fp)0.0f)),
                WorldFromMotion = new FpRigidTransform(new fpquaternion((fp)0.09212853f, (fp)0.1400256f, -(fp)0.006776567f, -(fp)0.9858292f), new fp3(-(fp)22.17587f, (fp)0.5172966f, -(fp)52.24425f))
            };
            motionVelocities[0] = new MotionVelocity
            {
                LinearVelocity = new fp3(-(fp)3.81221f, -(fp)1.37538f, -(fp)15.41893f),
                AngularVelocity = new fp3(-(fp)7.30913f, -(fp)4.78899f, (fp)1.14168f),
                InverseInertia = new fp3((fp)0.00045f, (fp)0.00045f, (fp)0.00045f),
                InverseMass = (fp)0.00018f,
                AngularExpansionFactor = (fp)2.05061f,
                GravityFactor = (fp)1.0f
            };

            // Calculate the collision event details and make sure 1 contact point is returned
            var details = collisionEvent.CalculateDetails(ref simpleWorld);
            Assert.AreEqual(details.EstimatedContactPointPositions.Length, 1);

            // Dispose the world data
            simpleWorld.Dispose();
        }
    }
}
