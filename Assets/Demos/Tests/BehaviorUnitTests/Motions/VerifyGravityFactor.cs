using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Physics.Systems;
using Fixed.Transforms;
using UnityEngine;
using UnityEngine.Assertions;

namespace Fixed.Physics.Tests
{
    public struct VerifyGravityFactorData : IComponentData
    {
    }

    public class VerifyGravityFactor : MonoBehaviour, IConvertGameObjectToEntity
    {
        void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            dstManager.AddComponentData(entity, new VerifyGravityFactorData());
        }
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class VerifyGravityFactorSystem : SystemBase
    {
        EntityQuery m_VerificationGroup;

        protected override void OnCreate()
        {
            m_VerificationGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(VerifyGravityFactorData) }
            });
        }

        protected override void OnUpdate()
        {
            using (var entities = m_VerificationGroup.ToEntityArray(Allocator.TempJob))
            {
                foreach (var entity in entities)
                {
                    var translation = EntityManager.GetComponentData<Translation>(entity);

                    // Sphere should never move due to gravity factor being 0
                    Assert.AreEqual((float)translation.Value.x, 0.0f);
                    Assert.AreEqual((float)translation.Value.y, 1.0f);
                    Assert.AreEqual((float)translation.Value.z, 0.0f);
                }
            }
        }
    }
}
