using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Systems;
using Fixed.Transforms;
using UnityEngine;
using Random = Fixed.Mathematics.Random;

public struct PlanetGravity : IComponentData
{
    public float3 GravitationalCenter;
    public sfloat GravitationalMass;
    public sfloat GravitationalConstant;
    public sfloat EventHorizonDistance;
    public sfloat RotationMultiplier;
}

public class PlanetGravityAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public sfloat GravitationalMass;
    public sfloat GravitationalConstant;
    public sfloat EventHorizonDistance;
    public sfloat RotationMultiplier;

    void IConvertGameObjectToEntity.Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        var component = new PlanetGravity
        {
            GravitationalCenter = transform.position,
            GravitationalMass = GravitationalMass,
            GravitationalConstant = GravitationalConstant,
            EventHorizonDistance = EventHorizonDistance,
            RotationMultiplier = RotationMultiplier
        };
        dstManager.AddComponentData(entity, component);

        if (dstManager.HasComponent<PhysicsMass>(entity))
        {
            var bodyMass = dstManager.GetComponentData<PhysicsMass>(entity);
            var random = new Random();
            random.InitState(10);
            bodyMass.InverseMass = random.NextFloat(bodyMass.InverseMass, bodyMass.InverseMass * (sfloat)4f);

            dstManager.SetComponentData(entity, bodyMass);
        }
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class PlanetGravitySystem : SystemBase
{
    static readonly quaternion k_GravityOrientation = quaternion.RotateY(math.PI / (sfloat)2f);

    protected override void OnUpdate()
    {
        var dt = (sfloat)Time.DeltaTime;

        Entities
            .WithName("ApplyGravityFromPlanet")
            .WithBurst()
            .ForEach((ref PhysicsMass bodyMass, ref PhysicsVelocity bodyVelocity, in Translation pos, in PlanetGravity gravity) =>
            {
                sfloat mass = math.rcp(bodyMass.InverseMass);

                float3 dir = (gravity.GravitationalCenter - pos.Value);
                sfloat dist = math.length(dir);
                sfloat invDist = sfloat.One / dist;
                dir = math.normalize(dir);
                float3 xtraGravity = (gravity.GravitationalConstant * (gravity.GravitationalMass * mass) * dir) * invDist * invDist;
                bodyVelocity.Linear += xtraGravity * dt;
                if (dist < gravity.EventHorizonDistance)
                {
                    xtraGravity = (gravity.RotationMultiplier * gravity.GravitationalConstant * gravity.GravitationalMass * dir) * invDist;
                    bodyVelocity.Linear += math.rotate(k_GravityOrientation, xtraGravity) * gravity.RotationMultiplier * dt;
                }
            }).Schedule();
    }
}
