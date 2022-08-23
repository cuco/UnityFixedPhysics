using Unity.Entities;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Authoring
{
    public class LimitedDistanceJoint : BallAndSocketJoint
    {
        public sfloat MinDistance;
        public sfloat MaxDistance;

        public override void Create(EntityManager entityManager, GameObjectConversionSystem conversionSystem)
        {
            UpdateAuto();
            conversionSystem.World.GetOrCreateSystem<EndJointConversionSystem>().CreateJointEntity(
                this,
                GetConstrainedBodyPair(conversionSystem),
                PhysicsJoint.CreateLimitedDistance(
                    PositionLocal,
                    PositionInConnectedEntity,
                    new FloatRange(MinDistance, MaxDistance)
                )
            );
        }
    }
}
