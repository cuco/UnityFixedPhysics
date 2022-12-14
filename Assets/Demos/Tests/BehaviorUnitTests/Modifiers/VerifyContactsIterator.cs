using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Physics.Systems;
using UnityEngine;
using UnityEngine.Assertions;

namespace Fixed.Physics.Tests
{
    public struct VerifyContactsIteratorData : IComponentData
    {
    }

    [Serializable]
    public class VerifyContactsIterator : MonoBehaviour, IConvertGameObjectToEntity
    {
        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity, new VerifyContactsIteratorData());

#if HAVOK_PHYSICS_EXISTS
            Havok.Physics.HavokConfiguration config = Havok.Physics.HavokConfiguration.Default;
            config.EnableSleeping = 0;
            dstManager.AddComponentData(entity, config);
#endif
        }
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class VerifyContactsIteratorSystem : SystemBase
    {
        EntityQuery m_VerificationGroup;
        StepPhysicsWorld m_StepPhysicsWorld;

        public NativeArray<int> CurrentManifoldNumContacts;

        protected override void OnCreate()
        {
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_VerificationGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(VerifyContactsIteratorData) }
            });
            CurrentManifoldNumContacts = new NativeArray<int>(2, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            CurrentManifoldNumContacts.Dispose();
        }

        struct VerifyContactsIteratorJob : IContactsJob
        {
            private bool m_Initialized;

            public int NumContacts;
            public ModifiableContactHeader CurrentManifold;

            public NativeArray<int> CurrentManifoldNumContacts;

            [ReadOnly]
            public NativeArray<RigidBody> Bodies;

            [ReadOnly]
            public ComponentDataFromEntity<VerifyContactsIteratorData> VerificationData;

            public void Execute(ref ModifiableContactHeader manifold, ref ModifiableContactPoint contact)
            {
                if (!m_Initialized)
                {
                    m_Initialized = true;
                    CurrentManifold = manifold;
                    NumContacts = 0;
                }

                // Header verification
                Assert.AreEqual(manifold.CustomTagsA, (byte)0);
                Assert.AreEqual(manifold.CustomTagsB, (byte)0);
                Assert.AreNotEqual(manifold.BodyIndexA, manifold.BodyIndexB);
                Assert.AreApproximatelyEqual((float)manifold.CoefficientOfFriction, 0.5f, 0.01f);
                Assert.AreApproximatelyEqual((float)manifold.CoefficientOfRestitution, 0.0f, 0.01f);
                Assert.AreEqual(manifold.ColliderKeyA.Value, ColliderKey.Empty.Value);
                Assert.AreEqual(manifold.ColliderKeyB.Value, ColliderKey.Empty.Value);
                Assert.AreEqual(manifold.EntityA, Bodies[manifold.BodyIndexA].Entity);
                Assert.AreEqual(manifold.EntityB, Bodies[manifold.BodyIndexB].Entity);
                Assert.AreEqual(manifold.JacobianFlags, (JacobianFlags)0);
                Assert.IsFalse(manifold.Modified);
                Assert.AreEqual(manifold.NumContacts, 4);

                NumContacts++;

                // Contact point verification
                Assert.IsTrue(contact.Index == NumContacts - 1);
                Assert.IsFalse(contact.Modified);

                // Save for later verification
                CurrentManifoldNumContacts[0] = CurrentManifold.NumContacts;
                CurrentManifoldNumContacts[1] = NumContacts;
            }
        }

        struct VerifyNumContactsJob : IJob
        {
            public NativeArray<int> CurrentManifoldNumContacts;

            [ReadOnly]
            public ComponentDataFromEntity<VerifyContactsIteratorData> VerificationData;

            public void Execute()
            {
                Assert.AreEqual(CurrentManifoldNumContacts[0], CurrentManifoldNumContacts[1]);
                Assert.AreEqual(CurrentManifoldNumContacts[0], 4);
            }
        }

        protected override void OnUpdate()
        {
            SimulationCallbacks.Callback verifyContactsIteratorJobCallback = (ref ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps) =>
            {
                return new VerifyContactsIteratorJob
                {
                    CurrentManifoldNumContacts = CurrentManifoldNumContacts,
                    Bodies = world.Bodies,
                    VerificationData = GetComponentDataFromEntity<VerifyContactsIteratorData>(true)
                }.Schedule(simulation, ref world, inDeps);
            };

            SimulationCallbacks.Callback verifyNumContactsJobCallback = (ref ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps) =>
            {
                return new VerifyNumContactsJob
                {
                    CurrentManifoldNumContacts = CurrentManifoldNumContacts,
                    VerificationData = GetComponentDataFromEntity<VerifyContactsIteratorData>(true)
                }.Schedule(inDeps);
            };

            m_StepPhysicsWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContacts, verifyContactsIteratorJobCallback, Dependency);
            m_StepPhysicsWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContactJacobians, verifyNumContactsJobCallback, Dependency);
        }
    }
}
