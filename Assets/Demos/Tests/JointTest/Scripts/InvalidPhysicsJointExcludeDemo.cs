using Unity.Collections;
using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Systems;
using Fixed.Transforms;

public class InvalidPhysicsJointExcludeDemoScene : SceneCreationSettings
{
    public sfloat TimeToSwap = (sfloat)0.5f;
}

public class InvalidPhysicsJointExcludeDemo : SceneCreationAuthoring<InvalidPhysicsJointExcludeDemoScene>
{
    public sfloat TimeToSwap = (sfloat)0.5f;

    public override void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new InvalidPhysicsJointExcludeDemoScene
        {
            DynamicMaterial = DynamicMaterial,
            StaticMaterial = StaticMaterial,
            TimeToSwap = TimeToSwap
        });
    }
}

public struct InvalidPhysicsJointExcludeTimerEvent : IComponentData
{
    public sfloat TimeLimit;
    internal sfloat Timer;

    public void Reset() => Timer = TimeLimit;
    public void Tick(sfloat deltaTime) => Timer -= deltaTime;
    public bool Fired(bool resetIfFired)
    {
        bool isFired = Timer < sfloat.Zero;
        if (isFired && resetIfFired) Reset();
        return isFired;
    }
}


public struct InvalidPhysicsJointExcludeBodies : IComponentData {}


[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class InvalidPhysicsJointExcludeDemoSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var deltaTime = (sfloat)Time.DeltaTime;

        Entities
            .WithName("InvalidPhysicsJointExcludeTimerEvent")
            .ForEach((ref InvalidPhysicsJointExcludeTimerEvent timer) =>
            {
                timer.Tick(deltaTime);
            }).Run();

        // add/remove PhysicsExclude
        using (var commandBuffer = new EntityCommandBuffer(Allocator.TempJob))
        {
            Entities
                .WithName("InvalidPhysicsJointExcludeBodies_Exclude")
                .WithoutBurst()
                .WithAll<InvalidPhysicsJointExcludeBodies, PhysicsWorldIndex>()
                .ForEach((Entity entity, ref InvalidPhysicsJointExcludeTimerEvent timer) =>
                {
                    if (timer.Fired(true))
                    {
                        // If we want to support multiple worlds, we need to store PhysicsWorldIndex.Value somewhere
                        commandBuffer.RemoveComponent<PhysicsWorldIndex>(entity);
                    }
                }).Run();

            Entities
                .WithName("InvalidPhysicsJointExcludeBodies_Include")
                .WithoutBurst()
                .WithAll<InvalidPhysicsJointExcludeBodies>()
                .WithNone<PhysicsWorldIndex>()
                .ForEach((Entity entity, ref InvalidPhysicsJointExcludeTimerEvent timer) =>
                {
                    if (timer.Fired(true))
                    {
                        commandBuffer.AddSharedComponent(entity, new PhysicsWorldIndex());
                    }
                }).Run();

            commandBuffer.Playback(EntityManager);
        }
    }
}


public class InvalidPhyiscsJointExcludeDemoSceneCreationSystem : SceneCreationSystem<InvalidPhysicsJointExcludeDemoScene>
{
    public override void CreateScene(InvalidPhysicsJointExcludeDemoScene sceneSettings)
    {
        sfloat colliderSize = (sfloat)0.25f;

        BlobAssetReference<Fixed.Physics.Collider> collider = Fixed.Physics.BoxCollider.Create(new BoxGeometry
        {
            Center = float3.zero,
            Orientation = quaternion.identity,
            Size = new float3(colliderSize),
            BevelRadius = (sfloat)0.0f
        });
        CreatedColliders.Add(collider);

        sfloat timeToSwap = sceneSettings.TimeToSwap;

        var bodyAPos = new float3(2f, 5.0f, 2);
        var bodyBPos = new float3(2f, 6.0f, 2);

        // Add constrained dynamic/dynamic body pair that will have their bodies excluded
        bool buildThisSection = true;
        if (buildThisSection)
        {
            bodyAPos += new float3(1, 0, 0);
            bodyBPos += new float3(1, 0, 0);

            // Create a body
            Entity bodyA = CreateDynamicBody(bodyAPos, quaternion.identity, collider, float3.zero, float3.zero, (sfloat)1.0f);
            Entity bodyB = CreateDynamicBody(bodyBPos, quaternion.identity, collider, float3.zero, float3.zero, (sfloat)1.0f);

            for (int i = 0; i < 2; i++)
            {
                // Create the joint
                var joint = PhysicsJoint.CreateBallAndSocket(new float3(sfloat.Zero, colliderSize, sfloat.Zero), new float3(sfloat.Zero, -colliderSize, sfloat.Zero));
                var jointEntity = CreateJoint(joint, bodyA, bodyB);

                var pair = EntityManager.GetComponentData<PhysicsConstrainedBodyPair>(jointEntity);
                pair.EnableCollision = 1;
                EntityManager.SetComponentData(jointEntity, pair);
            }

            // add exclude components.
            EntityManager.AddComponentData(bodyA, new InvalidPhysicsJointExcludeBodies {});
            EntityManager.AddComponentData(bodyA, new InvalidPhysicsJointExcludeTimerEvent { TimeLimit = timeToSwap, Timer = timeToSwap });
            EntityManager.AddComponentData(bodyB, new InvalidPhysicsJointExcludeBodies {});
            EntityManager.AddComponentData(bodyB, new InvalidPhysicsJointExcludeTimerEvent { TimeLimit = timeToSwap, Timer = timeToSwap });
        }
    }
}
