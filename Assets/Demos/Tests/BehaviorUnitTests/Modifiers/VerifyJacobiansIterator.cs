using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Physics.Systems;
using UnityEngine;
using UnityEngine.Assertions;

namespace Fixed.Physics.Tests
{
    public struct VerifyJacobiansIteratorData : IComponentData
    {
    }

    [Serializable]
    public class VerifyJacobiansIterator : MonoBehaviour, IConvertGameObjectToEntity
    {
        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity, new VerifyJacobiansIteratorData());

#if HAVOK_PHYSICS_EXISTS
            Havok.Physics.HavokConfiguration config = Havok.Physics.HavokConfiguration.Default;
            config.EnableSleeping = 0;
            dstManager.AddComponentData(entity, config);
#endif
        }
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class VerifyJacobiansIteratorSystem : SystemBase
    {
        EntityQuery m_VerificationGroup;
        StepPhysicsWorld m_StepPhysicsWorld;

        protected override void OnCreate()
        {
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_VerificationGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(VerifyJacobiansIteratorData) }
            });
        }

        struct VerifyJacobiansIteratorJob : IJacobiansJob
        {
            [ReadOnly]
            public NativeArray<RigidBody> Bodies;

            [ReadOnly]
            public ComponentDataFromEntity<VerifyJacobiansIteratorData> VerificationData;

            public void Execute(ref ModifiableJacobianHeader header, ref ModifiableContactJacobian jacobian)
            {
                // Header verification
                Assert.IsFalse(header.AngularChanged);
                Assert.AreNotEqual(header.BodyIndexA, header.BodyIndexB);
                Assert.AreEqual(header.EntityA, Bodies[header.BodyIndexA].Entity);
                Assert.AreEqual(header.EntityB, Bodies[header.BodyIndexB].Entity);
                Assert.AreEqual(header.Flags, (JacobianFlags)0);
                Assert.IsFalse(header.HasMassFactors);
                Assert.IsFalse(header.HasSurfaceVelocity);
                Assert.IsFalse(header.ModifiersChanged);
                Assert.AreEqual(header.Type, JacobianType.Contact);

                // Jacobian verification
                Assert.AreApproximatelyEqual((float)jacobian.CoefficientOfFriction, 0.5f, 0.01f);
                Assert.IsFalse(jacobian.Modified);
                Assert.AreEqual(jacobian.NumContacts, 4);
                for (int i = 0; i < jacobian.NumContacts; i++)
                {
                    ContactJacAngAndVelToReachCp jacAng = header.GetAngularJacobian(i);
                    Assert.AreEqual((float)jacAng.Jac.Impulse, 0.0f);
                }
            }

            public void Execute(ref ModifiableJacobianHeader header, ref ModifiableTriggerJacobian jacobian) {}
        }

        protected override void OnUpdate()
        {
            SimulationCallbacks.Callback verifyJacobiansIteratorJobCallback = (ref ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps) =>
            {
                return new VerifyJacobiansIteratorJob
                {
                    Bodies = world.Bodies,
                    VerificationData = GetComponentDataFromEntity<VerifyJacobiansIteratorData>(true)
                }.Schedule(simulation, ref world, inDeps);
            };

            m_StepPhysicsWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContactJacobians, verifyJacobiansIteratorJobCallback, Dependency);
        }
    }
}
