using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Extensions;
using Fixed.Physics.Systems;
using Fixed.Transforms;

public struct ExplosionCountdown : IComponentData
{
    public Entity Source;
    public int Countdown;
    public float3 Center;
    public sfloat Force;
}


[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class ExplosionCountdownSystem : SystemBase
{
    private BuildPhysicsWorld m_BuildPhysicsWorld;
    private EndFixedStepSimulationEntityCommandBufferSystem m_CommandBufferSystem;

    protected override void OnCreate()
    {
        m_BuildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
        m_CommandBufferSystem = World.GetOrCreateSystem<EndFixedStepSimulationEntityCommandBufferSystem>();
        RequireForUpdate(GetEntityQuery(new ComponentType[] { typeof(ExplosionCountdown) }));
    }

    protected override void OnUpdate()
    {
        var commandBufferParallel = m_CommandBufferSystem.CreateCommandBuffer().AsParallelWriter();

        var timeStep = (sfloat)Time.DeltaTime;
        var up = math.up();

        var positions = GetComponentDataFromEntity<Translation>(true);

        Entities
            .WithName("ExplosionCountdown_Tick")
            .WithReadOnly(positions)
            .WithBurst()
            .ForEach((Entity entity, ref ExplosionCountdown explosion) =>
            {
                explosion.Countdown--;
                bool bang = explosion.Countdown <= 0;
                if (bang && !explosion.Source.Equals(Entity.Null))
                {
                    explosion.Center = positions[explosion.Source].Value;
                }
            }).ScheduleParallel();


        Entities
            .WithName("ExplosionCountdown_Bang")
            .WithBurst()
            .ForEach((int entityInQueryIndex, Entity entity,
                ref ExplosionCountdown explosion, ref PhysicsVelocity pv,
                in PhysicsMass pm, in PhysicsCollider collider,
                in Translation pos, in Rotation rot) =>
                {
                    if (0 < explosion.Countdown) return;

                    pv.ApplyExplosionForce(pm, collider, pos, rot,
                        explosion.Force, explosion.Center, sfloat.Zero, timeStep, up, sfloat.Zero);

                    commandBufferParallel.RemoveComponent<ExplosionCountdown>(entityInQueryIndex, entity);
                }).Schedule();

        m_BuildPhysicsWorld.AddInputDependencyToComplete(Dependency);
    }
}
