using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Systems;
using Fixed.Transforms;
using UnityEngine;
using SphereCollider = Fixed.Physics.SphereCollider;

public struct ChangeSphereColliderRadius : IComponentData
{
    public sfloat Min;
    public sfloat Max;
    public sfloat Target;
}

// In general, you should treat colliders as immutable data at run-time, as several bodies might share the same collider.
// If you plan to modify mesh or convex colliders at run-time, remember to tick the Force Unique box on the PhysicsShapeAuthoring component.
// This guarantees that the PhysicsCollider component will have a unique instance in all cases.

// Converted in PhysicsSamplesConversionSystem so Physics and Graphics conversion is over
public class ChangeSphereColliderRadiusAuthoring : MonoBehaviour//, IConvertGameObjectToEntity
{
    [Range(0, 10)] public float Min = 0;
    [Range(0, 10)] public float Max = 10;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new ChangeSphereColliderRadius
        {
            Min = (sfloat)Min,
            Max = (sfloat)Max,
            Target = math.lerp((sfloat)Min, (sfloat)Max, (sfloat)0.5f),
        });
        // Physics and graphics representations of bodies can be largely independent.
        // Positions and Rotations of each representation are associated through the BuildPhysicsWorld & ExportPhysicsWorld systems.
        // As scale is generally baked for runtime performance, we specifically need to add a scale component here
        // and will update both the graphical and physical scales in our own demo update system.
        dstManager.AddComponentData(entity, new Scale
        {
            Value = (sfloat)1.0f,
        });
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(BuildPhysicsWorld))]
public partial class ChangeSphereColliderRadiusSystem : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithName("ChangeSphereColliderRadius")
            .WithBurst()
            .ForEach((ref PhysicsCollider collider, ref ChangeSphereColliderRadius radius, ref Scale scale) =>
            {
                // make sure we are dealing with spheres
                if (collider.Value.Value.Type != ColliderType.Sphere) return;

                // tweak the physical representation of the sphere

                // NOTE: this approach affects all instances using the same BlobAsset
                // so you cannot simply use this approach for instantiated prefabs
                // if you want to modify prefab instances independently, you need to create
                // unique BlobAssets at run-time and dispose them when you are done

                sfloat oldRadius = (sfloat)1.0f;
                sfloat newRadius = (sfloat)1.0f;
                unsafe
                {
                    // grab the sphere pointer
                    SphereCollider* scPtr = (SphereCollider*)collider.ColliderPtr;
                    oldRadius = scPtr->Radius;
                    newRadius = math.lerp(oldRadius, radius.Target, (sfloat)0.05f);
                    // if we have reached the target radius get a new target
                    if (math.abs(newRadius - radius.Target) < (sfloat)0.01f)
                    {
                        radius.Target = radius.Target == radius.Min ? radius.Max : radius.Min;
                    }

                    // update the collider geometry
                    var sphereGeometry = scPtr->Geometry;
                    sphereGeometry.Radius = newRadius;
                    scPtr->Geometry = sphereGeometry;
                }

                // now tweak the graphical representation of the sphere
                sfloat oldScale = scale.Value;
                sfloat newScale = oldScale;
                if (oldRadius == (sfloat)0.0f)
                {
                    // avoid the divide by zero errors.
                    newScale = newRadius;
                }
                else
                {
                    newScale *= newRadius / oldRadius;
                }
                scale.Value = newScale;
            }).Schedule();
    }
}
