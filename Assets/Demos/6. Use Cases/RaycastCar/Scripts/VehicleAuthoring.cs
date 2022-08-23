using Unity.Entities;
using Fixed.Mathematics;
using UnityEngine;

struct Vehicle : IComponentData {}

struct VehicleSpeed : IComponentData
{
    public sfloat TopSpeed;
    public sfloat DesiredSpeed;
    public sfloat Damping;
    public byte DriveEngaged;
}

struct VehicleSteering : IComponentData
{
    public sfloat MaxSteeringAngle;
    public sfloat DesiredSteeringAngle;
    public sfloat Damping;
}

enum VehicleCameraOrientation
{
    Absolute,
    Relative
}

struct VehicleCameraSettings : IComponentData
{
    public VehicleCameraOrientation OrientationType;
    public sfloat OrbitAngularSpeed;
}

struct VehicleCameraReferences : IComponentData
{
    public Entity CameraOrbit;
    public Entity CameraTarget;
    public Entity CameraTo;
    public Entity CameraFrom;
}

class VehicleAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    #pragma warning disable 649
    public bool ActiveAtStart;

    [Header("Handling")]
    public sfloat TopSpeed = (sfloat)10.0f;
    public sfloat MaxSteeringAngle = (sfloat)30.0f;
    //0-1
    public sfloat SteeringDamping = (sfloat)0.1f;
    //0-1
    public sfloat SpeedDamping = (sfloat)0.01f;

    [Header("Camera Settings")]
    public Transform CameraOrbit;
    public VehicleCameraOrientation CameraOrientation = VehicleCameraOrientation.Relative;
    public sfloat CameraOrbitAngularSpeed = (sfloat)180f;
    public Transform CameraTarget;
    public Transform CameraTo;
    public Transform CameraFrom;
    #pragma warning restore 649

    void OnValidate()
    {
        TopSpeed = math.max(sfloat.Zero, TopSpeed);
        MaxSteeringAngle = math.max(sfloat.Zero, MaxSteeringAngle);
        SteeringDamping = math.clamp(SteeringDamping, sfloat.Zero, sfloat.One);
        SpeedDamping = math.clamp(SpeedDamping, sfloat.Zero, sfloat.One);
    }

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        if (ActiveAtStart)
            dstManager.AddComponent<ActiveVehicle>(entity);

        dstManager.AddComponent<Vehicle>(entity);

        dstManager.AddComponentData(entity, new VehicleCameraSettings
        {
            OrientationType = CameraOrientation,
            OrbitAngularSpeed = math.radians(CameraOrbitAngularSpeed)
        });

        dstManager.AddComponentData(entity, new VehicleSpeed
        {
            TopSpeed = TopSpeed,
            Damping = SpeedDamping
        });

        dstManager.AddComponentData(entity, new VehicleSteering
        {
            MaxSteeringAngle = math.radians(MaxSteeringAngle),
            Damping = SteeringDamping
        });

        dstManager.AddComponentData(entity, new VehicleCameraReferences
        {
            CameraOrbit = conversionSystem.GetPrimaryEntity(CameraOrbit),
            CameraTarget = conversionSystem.GetPrimaryEntity(CameraTarget),
            CameraTo = conversionSystem.GetPrimaryEntity(CameraTo),
            CameraFrom = conversionSystem.GetPrimaryEntity(CameraFrom)
        });
    }
}
