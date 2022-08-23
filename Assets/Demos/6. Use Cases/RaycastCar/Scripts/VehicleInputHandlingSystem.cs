using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Transforms;

struct VehicleInput : IComponentData
{
    public float2 Looking;
    public float2 Steering;
    public sfloat Throttle;
    public int Change; // positive to change to a subsequent vehicle, negative to change to a previous one
}

[UpdateInGroup(typeof(InitializationSystemGroup))]
partial class VehicleInputHandlingSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var input = GetSingleton<VehicleInput>();

        Entities
            .WithName("ActiveVehicleInputHandlingJob")
            .WithoutBurst()
            .WithStructuralChanges()
            .WithAll<ActiveVehicle>()
            .ForEach((ref VehicleSpeed speed, ref VehicleSteering steering, in VehicleCameraSettings cameraSettings, in VehicleCameraReferences references) =>
            {
                sfloat x = input.Steering.x;
                sfloat a = input.Throttle;
                sfloat z = input.Looking.x;

                var newSpeed = a * speed.TopSpeed;
                speed.DriveEngaged = (byte)(newSpeed == sfloat.Zero ? 0 : 1);
                speed.DesiredSpeed = math.lerp(speed.DesiredSpeed, newSpeed, speed.Damping);

                var newSteeringAngle = x * steering.MaxSteeringAngle;
                steering.DesiredSteeringAngle = math.lerp(steering.DesiredSteeringAngle, newSteeringAngle, steering.Damping);

                if (HasComponent<Rotation>(references.CameraOrbit))
                {
                    var orientation = GetComponent<Rotation>(references.CameraOrbit);
                    switch (cameraSettings.OrientationType)
                    {
                        case VehicleCameraOrientation.Relative:
                            orientation.Value = math.mul(orientation.Value, quaternion.Euler(sfloat.Zero, z * (sfloat)Time.DeltaTime * cameraSettings.OrbitAngularSpeed, sfloat.Zero));
                            break;
                        case VehicleCameraOrientation.Absolute:
                            float4x4 worldFromLocal = HasComponent<LocalToWorld>(references.CameraOrbit)
                                ? GetComponent<LocalToWorld>(references.CameraOrbit).Value
                                : float4x4.identity;
                            float4x4 worldFromParent = HasComponent<LocalToParent>(references.CameraOrbit)
                                ? math.mul(worldFromLocal, math.inverse(GetComponent<LocalToParent>(references.CameraOrbit).Value))
                                : worldFromLocal;
                            var worldOrientation = quaternion.Euler(sfloat.Zero, z * math.PI, sfloat.Zero);
                            orientation.Value = new quaternion(math.mul(worldFromParent, new float4x4(worldOrientation, float3.zero)));
                            break;
                    }
                    SetComponent(references.CameraOrbit, orientation);
                }
            }).Run();
    }
}
