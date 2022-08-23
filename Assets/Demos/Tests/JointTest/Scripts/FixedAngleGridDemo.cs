using Unity.Collections;
using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;

public class FixedAngleGridScene : SceneCreationSettings {};

public class FixedAngleGridDemo : SceneCreationAuthoring<FixedAngleGridScene> {}

public class FixAngleGridDemoSystem : SceneCreationSystem<FixedAngleGridScene>
{
    public override void CreateScene(FixedAngleGridScene sceneSettings)
    {
        BlobAssetReference<Fixed.Physics.Collider> collider = Fixed.Physics.BoxCollider.Create(new BoxGeometry
        {
            Center = float3.zero,
            Orientation = quaternion.identity,
            Size = new float3((sfloat)0.25f),
            BevelRadius = (sfloat)0.0f
        });
        CreatedColliders.Add(collider);

        quaternion orientationA = quaternion.identity;
        bool identityA = true;
        if (!identityA)
        {
            orientationA = quaternion.AxisAngle(new float3(0, 1, 0), math.PI * (sfloat)3.0f / (sfloat)2.0f);
        }

        quaternion orientationB = quaternion.identity;
        bool identityB = true;
        if (!identityB)
        {
            orientationB = quaternion.AxisAngle(math.normalize(new float3(1)), math.PI / (sfloat)4.0f);
        }

        // Make some joints with fixed position, limited 3D angle
        for (int i = 0; i < 10; i++)
        {
            // Create a body
            Entity body = CreateDynamicBody(
                new float3((i - 4.5f) * 1.0f, 0, 0), quaternion.identity, collider, float3.zero, float3.zero, (sfloat)1.0f);

            // Create the ragdoll joint
            float3 pivotLocal = float3.zero;
            float3 pivotInWorld = math.transform(GetBodyTransform(body), pivotLocal);

            var jointData = new PhysicsJoint
            {
                BodyAFromJoint = new RigidTransform(orientationA, pivotLocal),
                BodyBFromJoint = new RigidTransform(orientationB, pivotInWorld)
            };
            jointData.SetConstraints(new FixedList128Bytes<Constraint>
            {
                Length = 2,
                [0] = Constraint.BallAndSocket(),
                [1] = new Constraint
                {
                    ConstrainedAxes = new bool3(true, true, true),
                    Type = ConstraintType.Angular,
                    Min = math.max((sfloat)(i - 5), sfloat.Zero) * (sfloat)0.1f,
                    Max = (sfloat)i * (sfloat)0.1f,
                    SpringDamping = Constraint.DefaultSpringDamping,
                    SpringFrequency = Constraint.DefaultSpringFrequency
                }
            });
            CreateJoint(jointData, body, Entity.Null);
        }
    }
}
