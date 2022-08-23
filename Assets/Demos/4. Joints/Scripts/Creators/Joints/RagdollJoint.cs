using Unity.Collections;
using Unity.Entities;
using Fixed.Mathematics;
using UnityEngine;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Authoring
{
    public class RagdollJoint : BallAndSocketJoint
    {
        const int k_LatestVersion = 1;

        // Editor only settings
        [HideInInspector]
        public bool EditAxes;
        [HideInInspector]
        public bool EditLimits;

        [SerializeField]
        int m_Version;

        public float3 TwistAxisLocal;
        public float3 TwistAxisInConnectedEntity;
        public float3 PerpendicularAxisLocal;
        public float3 PerpendicularAxisInConnectedEntity;
        public sfloat MaxConeAngle;
        public sfloat MinPerpendicularAngle;
        public sfloat MaxPerpendicularAngle;
        public sfloat MinTwistAngle;
        public sfloat MaxTwistAngle;

        void UpgradeVersionIfNecessary()
        {
            if (m_Version >= k_LatestVersion)
                return;

            MinPerpendicularAngle -= (sfloat)90f;
            MaxPerpendicularAngle -= (sfloat)90f;
            m_Version = k_LatestVersion;
        }

        void OnValidate()
        {
            UpgradeVersionIfNecessary();

            MaxConeAngle = math.clamp(MaxConeAngle, (sfloat)0f, (sfloat)180f);

            MaxPerpendicularAngle = math.clamp(MaxPerpendicularAngle, -(sfloat)90f, (sfloat)90f);
            MinPerpendicularAngle = math.clamp(MinPerpendicularAngle, -(sfloat)90f, (sfloat)90f);
            if (MaxPerpendicularAngle < MinPerpendicularAngle)
            {
                var swap = new FloatRange(MinPerpendicularAngle, MaxPerpendicularAngle).Sorted();
                MinPerpendicularAngle = swap.Min;
                MaxPerpendicularAngle = swap.Max;
            }

            MinTwistAngle = math.clamp(MinTwistAngle, -(sfloat)180f, (sfloat)180f);
            MaxTwistAngle = math.clamp(MaxTwistAngle, -(sfloat)180f, (sfloat)180f);
            if (MaxTwistAngle < MinTwistAngle)
            {
                var swap = new FloatRange(MinTwistAngle, MaxTwistAngle).Sorted();
                MinTwistAngle = swap.Min;
                MaxTwistAngle = swap.Max;
            }
        }

        public override void UpdateAuto()
        {
            base.UpdateAuto();
            if (AutoSetConnected)
            {
                RigidTransform bFromA = math.mul(math.inverse(worldFromB), worldFromA);
                TwistAxisInConnectedEntity = math.mul(bFromA.rot, TwistAxisLocal);
                PerpendicularAxisInConnectedEntity = math.mul(bFromA.rot, PerpendicularAxisLocal);
            }
        }

        public override void Create(EntityManager entityManager, GameObjectConversionSystem conversionSystem)
        {
            UpdateAuto();
            UpgradeVersionIfNecessary();
            PhysicsJoint.CreateRagdoll(
                new BodyFrame { Axis = TwistAxisLocal, PerpendicularAxis = PerpendicularAxisLocal, Position = PositionLocal },
                new BodyFrame { Axis = TwistAxisInConnectedEntity, PerpendicularAxis = PerpendicularAxisInConnectedEntity, Position = PositionInConnectedEntity },
                math.radians(MaxConeAngle),
                math.radians(new FloatRange(MinPerpendicularAngle, MaxPerpendicularAngle)),
                math.radians(new FloatRange(MinTwistAngle, MaxTwistAngle)),
                out var primaryCone,
                out var perpendicularCone
            );

            conversionSystem.World.GetOrCreateSystem<EndJointConversionSystem>().CreateJointEntities(
                this,
                GetConstrainedBodyPair(conversionSystem),
                new NativeArray<PhysicsJoint>(2, Allocator.Temp) { [0] = primaryCone, [1] = perpendicularCone }
            );
        }
    }
}
