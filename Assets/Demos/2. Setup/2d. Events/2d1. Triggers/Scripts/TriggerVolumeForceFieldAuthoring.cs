using Unity.Entities;
using Unity.Jobs;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Stateful;
using Fixed.Physics.Systems;
using Fixed.Transforms;
using UnityEngine;

public struct TriggerVolumeForceField : IComponentData
{
    public float3 Center;
    public sfloat DeadZone;
    public sfloat Strength;
    public sfloat Rotation;
    public int Axis;
    public int Proportional;
    public int MassInvariant;
}

public class TriggerVolumeForceFieldAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public enum Direction { Center, XAxis, YAxis, ZAxis };

    public sfloat Strength = (sfloat)10f;
    public sfloat DeadZone = (sfloat)0.5f;
    public Direction Axis = Direction.Center;
    public sfloat Rotation = sfloat.Zero;
    public bool Proportional = true;
    public bool MassInvariant = false;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new TriggerVolumeForceField
        {
            Center = transform.position,
            Strength = Strength,
            DeadZone = (DeadZone == sfloat.Zero) ? (sfloat)0.001f : math.abs(DeadZone),
            Axis = (int)Axis - 1,
            Rotation = math.radians(Rotation),
            Proportional = Proportional ? 1 : 0,
            MassInvariant = MassInvariant ? 1 : 0
        });
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(ExportPhysicsWorld))]
[UpdateAfter(typeof(StatefulTriggerEventBufferSystem))]
public partial class TriggerVolumeForceFieldSystem : SystemBase
{
    private EntityQueryMask m_NonTriggerDynamicBodyMask;

    protected override void OnCreate()
    {
        m_NonTriggerDynamicBodyMask = EntityManager.GetEntityQueryMask(GetEntityQuery(new EntityQueryDesc
        {
            All = new ComponentType[]
            {
                typeof(PhysicsVelocity),
                typeof(PhysicsMass),
                typeof(Translation)
            },
            None = new ComponentType[]
            {
                typeof(StatefulTriggerEvent)
            }
        }));
    }

    public static void ApplyForceField(
        in sfloat dt,
        ref PhysicsVelocity bodyVelocity,
        in Translation pos, in PhysicsMass bodyMass, in TriggerVolumeForceField forceField
    )
    {
        if (forceField.Strength.IsZero())
            return;

        // Don't do anything if in eye
        float3 dir = float3.zero;
        dir = (forceField.Center - pos.Value);
        if (!math.any(dir))
            return;

        // If force field around axis then project dir onto axis
        float3 axis = float3.zero;
        if (forceField.Axis != -1)
        {
            axis[forceField.Axis] = sfloat.One;
            dir -= axis * math.dot(dir, axis);
        }

        sfloat strength = forceField.Strength;
        sfloat dist2 = math.lengthsq(dir);

        // Kill strength if in deadzone
        sfloat dz2 = forceField.DeadZone * forceField.DeadZone;
        if (dz2 > dist2)
            strength = sfloat.Zero;

        // If out of center and proportional divide by distance squared
        if (forceField.Proportional != 0)
            strength = (dist2 > (sfloat)1e-4f) ? strength / dist2 : sfloat.Zero;

        // Multiple through mass if want all objects moving equally
        dir = math.normalizesafe(dir);
        sfloat mass = math.rcp(bodyMass.InverseMass);
        if (forceField.MassInvariant != 0) mass = sfloat.One;
        strength *= mass * dt;
        bodyVelocity.Linear += strength * dir;

        // If want a rotational force field add extra twist deltas
        if ((forceField.Axis != -1) && (!forceField.Rotation.IsZero()))
        {
            bodyVelocity.Linear += forceField.Rotation * strength * dir;
            dir = math.cross(axis, -dir);
            bodyVelocity.Linear += forceField.Rotation * strength * dir;
        }
    }

    protected override void OnUpdate()
    {
        sfloat dt = (sfloat)Time.DeltaTime;

        // Need extra variables here so that they can be
        // captured by the Entities.Foreach loop below
        var stepComponent = HasSingleton<PhysicsStep>() ? GetSingleton<PhysicsStep>() : PhysicsStep.Default;
        var nonTriggerDynamicBodyMask = m_NonTriggerDynamicBodyMask;

        Entities
            .WithName("ApplyForceFieldJob")
            .WithBurst()
            .ForEach((Entity e, ref DynamicBuffer<StatefulTriggerEvent> triggerEventBuffer, ref TriggerVolumeForceField forceField) =>
            {
                forceField.Center = GetComponent<Translation>(e).Value;

                for (int i = 0; i < triggerEventBuffer.Length; i++)
                {
                    var triggerEvent = triggerEventBuffer[i];

                    var otherEntity = triggerEvent.GetOtherEntity(e);

                    // exclude static bodies, other triggers and enter/exit events
                    if (triggerEvent.State != StatefulEventState.Stay || !nonTriggerDynamicBodyMask.Matches(otherEntity))
                    {
                        continue;
                    }

                    var physicsVelocity = GetComponent<PhysicsVelocity>(otherEntity);
                    var physicsMass = GetComponent<PhysicsMass>(otherEntity);
                    var pos = GetComponent<Translation>(otherEntity);

                    ApplyForceField(dt, ref physicsVelocity, pos, physicsMass, forceField);

                    // counter-act gravity
                    physicsVelocity.Linear += -(sfloat)1.25f * stepComponent.Gravity * dt;

                    // write back
                    SetComponent(otherEntity, physicsVelocity);
                }
            }).Schedule();
    }
}
