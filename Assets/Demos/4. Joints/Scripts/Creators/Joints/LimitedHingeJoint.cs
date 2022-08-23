using Unity.Entities;
using Fixed.Mathematics;
using UnityEngine;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Authoring
{
    public class LimitedHingeJoint : FreeHingeJoint
    {
        // Editor only settings
        [HideInInspector]
        public bool EditLimits;

        public float3 PerpendicularAxisLocal;
        public float3 PerpendicularAxisInConnectedEntity;
        public sfloat MinAngle;
        public sfloat MaxAngle;

        public override void UpdateAuto()
        {
            base.UpdateAuto();
            if (AutoSetConnected)
            {
                RigidTransform bFromA = math.mul(math.inverse(worldFromB), worldFromA);
                HingeAxisInConnectedEntity = math.mul(bFromA.rot, HingeAxisLocal);
                PerpendicularAxisInConnectedEntity = math.mul(bFromA.rot, PerpendicularAxisLocal);
            }
        }

        public override void Create(EntityManager entityManager, GameObjectConversionSystem conversionSystem)
        {
            UpdateAuto();
            conversionSystem.World.GetOrCreateSystem<EndJointConversionSystem>().CreateJointEntity(
                this,
                GetConstrainedBodyPair(conversionSystem),
                PhysicsJoint.CreateLimitedHinge(
                    new BodyFrame
                    {
                        Axis = math.normalize(HingeAxisLocal),
                        PerpendicularAxis = math.normalize(PerpendicularAxisLocal),
                        Position = PositionLocal
                    },
                    new BodyFrame
                    {
                        Axis = math.normalize(HingeAxisInConnectedEntity),
                        PerpendicularAxis = math.normalize(PerpendicularAxisInConnectedEntity),
                        Position = PositionInConnectedEntity
                    },
                    math.radians(new FloatRange(MinAngle, MaxAngle))
                )
            );
        }
    }
}
