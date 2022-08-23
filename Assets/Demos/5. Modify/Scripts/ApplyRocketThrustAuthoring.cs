using System;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Authoring;
using Fixed.Physics.Extensions;
using Fixed.Physics.Systems;
using Fixed.Transforms;
using UnityEngine;
using Math = Fixed.Physics.Math;

[RequireComponent(typeof(PhysicsBodyAuthoring))]
public class ApplyRocketThrustAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public sfloat Magnitude = (sfloat)1.0f;
    public Vector3 LocalDirection = -Vector3.forward;
    public Vector3 LocalOffset = Vector3.zero;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new ApplyRocketThrust
        {
            Magnitude = Magnitude,
            Direction = LocalDirection.normalized,
            Offset = LocalOffset,
        });
    }

    public void OnDrawGizmos()
    {
        if (LocalDirection.Equals(Vector3.zero)) return;

        var originalColor = Gizmos.color;
        var originalMatrix = Gizmos.matrix;

        Gizmos.color = Color.red;

        // Calculate the final Physics Body runtime coordinate system which bakes out skew from non-uniform scaling in parent
        var worldFromLocalRigidTransform = Math.DecomposeRigidBodyTransform(transform.localToWorldMatrix);
        var worldFromLocal = Matrix4x4.TRS(worldFromLocalRigidTransform.pos, worldFromLocalRigidTransform.rot, Vector3.one);

        Vector3 directionWorld = worldFromLocal.MultiplyVector(LocalDirection.normalized);
        Vector3 offsetWorld = worldFromLocal.MultiplyPoint(LocalOffset);

        // Calculate the final world Thrust coordinate system from the world Body transform and local offset and direction
        Math.CalculatePerpendicularNormalized(directionWorld, out _, out var directionPerpendicular);
        var worldFromThrust = Matrix4x4.TRS(offsetWorld, Quaternion.LookRotation(directionWorld, directionPerpendicular), Vector3.one);

        Gizmos.matrix = worldFromThrust;

        float Shift = (float)Magnitude * 0.1f;
        Gizmos.DrawFrustum(new Vector3(0, 0, -Shift), UnityEngine.Random.Range(1.0f, 2.5f), (float)Magnitude, Shift, 1.0f);

        Gizmos.matrix = originalMatrix;
        Gizmos.color = originalColor;
    }
}

public struct ApplyRocketThrust : IComponentData
{
    public sfloat Magnitude;
    public float3 Direction;
    public float3 Offset;
}


[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class ApplyRocketThrustSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var deltaTime = (sfloat)Time.DeltaTime;

        Entities
            .WithName("ApplyRocketThrust")
            .WithBurst()
            .ForEach((ref ApplyRocketThrust rocket, ref Translation t, ref Rotation r, ref PhysicsVelocity pv, ref PhysicsMass pm) =>
            {
                // Newton's 3rd law states that for every action there is an equal and opposite reaction.
                // As this is a rocket thrust the impulse applied with therefore use negative Direction.
                float3 impulse = -rocket.Direction * rocket.Magnitude;
                impulse = math.rotate(r.Value, impulse);
                impulse *= deltaTime;

                float3 offset = math.rotate(r.Value, rocket.Offset) + t.Value;

                pv.ApplyImpulse(pm, t, r, impulse, offset);
            }).Schedule();
    }
}
