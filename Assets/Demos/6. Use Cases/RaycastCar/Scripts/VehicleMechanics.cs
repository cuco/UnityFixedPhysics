using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Fixed.Physics;
using Fixed.Physics.Extensions;
using Fixed.Physics.Systems;
using Fixed.Mathematics;
using Fixed.Physics.Authoring;
using Fixed.Transforms;
using UnityEngine;

namespace Demos
{
    [RequireComponent(typeof(PhysicsBodyAuthoring))]
    public class VehicleMechanics : MonoBehaviour
    {
        [Header("Wheel Parameters...")]
        public List<GameObject> wheels = new List<GameObject>();
        public sfloat wheelBase = (sfloat)0.5f;
        public sfloat wheelFrictionRight = (sfloat)0.5f;
        public sfloat wheelFrictionForward = (sfloat)0.5f;
        public sfloat wheelMaxImpulseRight = (sfloat)10.0f;
        public sfloat wheelMaxImpulseForward = (sfloat)10.0f;
        [Header("Suspension Parameters...")]
        public sfloat suspensionLength = (sfloat)0.5f;
        public sfloat suspensionStrength = (sfloat)1.0f;
        public sfloat suspensionDamping = (sfloat)0.1f;
        [Header("Steering Parameters...")]
        public List<GameObject> steeringWheels = new List<GameObject>();
        [Header("Drive Parameters...")]
        public List<GameObject> driveWheels = new List<GameObject>();
        [Header("Miscellaneous Parameters...")]
        public bool drawDebugInformation = false;
    }

    // ensure built-in physics conversion systems have run
    [UpdateAfter(typeof(EndColliderConversionSystem))]
    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
    class VehicleMechanicsConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach((VehicleMechanics m) =>
            {
                var entity = GetPrimaryEntity(m);

                foreach (var wheel in m.wheels)
                {
                    var wheelEntity = GetPrimaryEntity(wheel);

                    // Assumed hierarchy:
                    // - chassis
                    //  - mechanics
                    //   - suspension
                    //    - wheel (rotates about yaw axis and translates along suspension up)
                    //     - graphic (rotates about pitch axis)

                    RigidTransform worldFromSuspension = new RigidTransform
                    {
                        pos = wheel.transform.parent.position,
                        rot = wheel.transform.parent.rotation
                    };

                    RigidTransform worldFromChassis = new RigidTransform
                    {
                        pos = wheel.transform.parent.parent.parent.position,
                        rot = wheel.transform.parent.parent.parent.rotation
                    };

                    var chassisFromSuspension = math.mul(math.inverse(worldFromChassis), worldFromSuspension);

                    DstEntityManager.AddComponentData(wheelEntity, new Wheel
                    {
                        Vehicle = entity,
                        GraphicalRepresentation = GetPrimaryEntity(wheel.transform.GetChild(0)), // assume wheel has a single child with rotating graphic
                        // TODO assume for now that driving/steering wheels also appear in this list
                        UsedForSteering = (byte)(m.steeringWheels.Contains(wheel) ? 1 : 0),
                        UsedForDriving = (byte)(m.driveWheels.Contains(wheel) ? 1 : 0),
                        ChassisFromSuspension = chassisFromSuspension
                    });
                }

                DstEntityManager.AddComponent<VehicleBody>(entity);
                DstEntityManager.AddComponentData(entity, new VehicleConfiguration
                {
                    wheelBase = m.wheelBase,
                    wheelFrictionRight = m.wheelFrictionRight,
                    wheelFrictionForward = m.wheelFrictionForward,
                    wheelMaxImpulseRight = m.wheelMaxImpulseRight,
                    wheelMaxImpulseForward = m.wheelMaxImpulseForward,
                    suspensionLength = m.suspensionLength,
                    suspensionStrength = m.suspensionStrength,
                    suspensionDamping = m.suspensionDamping,
                    invWheelCount = sfloat.One / (sfloat)m.wheels.Count,
                    drawDebugInformation = (byte)(m.drawDebugInformation ? 1 : 0)
                });
            });
        }
    }

    // configuration properties of the vehicle mechanics, which change with low frequency at run-time
    struct VehicleConfiguration : IComponentData
    {
        public sfloat wheelBase;
        public sfloat wheelFrictionRight;
        public sfloat wheelFrictionForward;
        public sfloat wheelMaxImpulseRight;
        public sfloat wheelMaxImpulseForward;
        public sfloat suspensionLength;
        public sfloat suspensionStrength;
        public sfloat suspensionDamping;
        public sfloat invWheelCount;
        public byte drawDebugInformation;
    }

    // physics properties of the vehicle rigid body, which change with high frequency at run-time
    struct VehicleBody : IComponentData
    {
        public sfloat SlopeSlipFactor;
        public float3 WorldCenterOfMass;
    }

    struct Wheel : IComponentData
    {
        public Entity Vehicle;
        public Entity GraphicalRepresentation;
        public byte UsedForSteering;
        public byte UsedForDriving;
        public RigidTransform ChassisFromSuspension;
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class VehicleMechanicsSystem : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            RequireForUpdate(GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(VehicleConfiguration) }
            }));
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadWrite();
        }

        protected override void OnUpdate()
        {
            // update vehicle properties first
            Dependency = Entities
                .WithName("PrepareVehiclesJob")
                .WithBurst()
                .ForEach((
                    Entity entity, ref VehicleBody vehicleBody,
                    in VehicleConfiguration mechanics, in PhysicsMass mass, in Translation translation, in Rotation rotation
                    ) =>
                    {
                        vehicleBody.WorldCenterOfMass = mass.GetCenterOfMassWorldSpace(in translation, in rotation);

                        // calculate a simple slip factor based on chassis tilt
                        float3 worldUp = math.mul(rotation.Value, math.up());
                        vehicleBody.SlopeSlipFactor = math.pow(math.abs(math.dot(worldUp, math.up())), (sfloat)4f);
                    })
                .Schedule(Dependency);

            Dependency.Complete();

            // this sample makes direct modifications to impulses between BuildPhysicsWorld and StepPhysicsWorld
            // we thus use PhysicsWorldExtensions rather than modifying component data, since they have already been consumed by BuildPhysicsWorld
            PhysicsWorld world = m_BuildPhysicsWorldSystem.PhysicsWorld;

            // update each wheel
            var commandBuffer = new EntityCommandBuffer(Allocator.TempJob);
            Entities
                .WithName("VehicleWheelsJob")
                .WithBurst()
                .ForEach((
                    Entity entity, in Translation localPosition, in Rotation localRotation, in Wheel wheel
                    ) =>
                    {
                        Entity ce = wheel.Vehicle;
                        if (ce == Entity.Null) return;
                        int ceIdx = world.GetRigidBodyIndex(ce);
                        if (-1 == ceIdx || ceIdx >= world.NumDynamicBodies) return;

                        var mechanics = GetComponent<VehicleConfiguration>(ce);
                        var vehicleBody = GetComponent<VehicleBody>(ce);

                        float3 cePosition = GetComponent<Translation>(ce).Value;
                        quaternion ceRotation = GetComponent<Rotation>(ce).Value;
                        float3 ceCenterOfMass = vehicleBody.WorldCenterOfMass;
                        float3 ceUp = math.mul(ceRotation, new float3(0, 1, 0));
                        float3 ceForward = math.mul(ceRotation, new float3(0, 0, 1));
                        float3 ceRight = math.mul(ceRotation, new float3(1, 0, 0));

                        CollisionFilter filter = world.GetCollisionFilter(ceIdx);

                        sfloat driveDesiredSpeed = sfloat.Zero;
                        bool driveEngaged = false;
                        if (HasComponent<VehicleSpeed>(ce))
                        {
                            var vehicleSpeed = GetComponent<VehicleSpeed>(ce);
                            driveDesiredSpeed = vehicleSpeed.DesiredSpeed;
                            driveEngaged = vehicleSpeed.DriveEngaged != 0;
                        }

                        sfloat desiredSteeringAngle = HasComponent<VehicleSteering>(ce)
                            ? GetComponent<VehicleSteering>(ce).DesiredSteeringAngle
                            : sfloat.Zero;

                        RigidTransform worldFromChassis = new RigidTransform
                        {
                            pos = cePosition,
                            rot = ceRotation
                        };

                        RigidTransform suspensionFromWheel = new RigidTransform
                        {
                            pos = localPosition.Value,
                            rot = localRotation.Value
                        };

                        RigidTransform chassisFromWheel = math.mul(wheel.ChassisFromSuspension, suspensionFromWheel);
                        RigidTransform worldFromLocal = math.mul(worldFromChassis, chassisFromWheel);

                        // create a raycast from the suspension point on the chassis
                        var worldFromSuspension = math.mul(worldFromChassis, wheel.ChassisFromSuspension);
                        float3 rayStart = worldFromSuspension.pos;
                        float3 rayEnd = (-ceUp * (mechanics.suspensionLength + mechanics.wheelBase)) + rayStart;

                        if (mechanics.drawDebugInformation != 0)
                            Debug.DrawRay(rayStart, rayEnd - rayStart);

                        var raycastInput = new RaycastInput
                        {
                            Start = rayStart,
                            End = rayEnd,
                            Filter = filter
                        };

                        var hit = world.CastRay(raycastInput, out var rayResult);

                        var invWheelCount = mechanics.invWheelCount;

                        // Calculate a simple slip factor based on chassis tilt.
                        sfloat slopeSlipFactor = vehicleBody.SlopeSlipFactor;

                        float3 wheelPos = math.select(raycastInput.End, rayResult.Position, hit);
                        wheelPos -= (cePosition - ceCenterOfMass);

                        float3 velocityAtWheel = world.GetLinearVelocity(ceIdx, wheelPos);

                        float3 weUp = ceUp;
                        float3 weRight = ceRight;
                        float3 weForward = ceForward;

                        // Assumed hierarchy:
                        // - chassis
                        //  - mechanics
                        //   - suspension
                        //    - wheel (rotates about yaw axis and translates along suspension up)
                        //     - graphic (rotates about pitch axis)

                        #region handle wheel steering
                        {
                            // update yaw angle if wheel is used for steering
                            if (wheel.UsedForSteering != 0)
                            {
                                quaternion wRotation = quaternion.AxisAngle(ceUp, desiredSteeringAngle);
                                weRight = math.rotate(wRotation, weRight);
                                weForward = math.rotate(wRotation, weForward);

                                commandBuffer.SetComponent(entity, new Rotation { Value = quaternion.AxisAngle(math.up(), desiredSteeringAngle) });
                            }
                        }
                        #endregion

                        sfloat currentSpeedUp = math.dot(velocityAtWheel, weUp);
                        sfloat currentSpeedForward = math.dot(velocityAtWheel, weForward);
                        sfloat currentSpeedRight = math.dot(velocityAtWheel, weRight);

                        #region handle wheel rotation
                        {
                            // update rotation of graphical representation about axle
                            bool isDriven = driveEngaged && wheel.UsedForDriving != 0;
                            sfloat weRotation = isDriven
                                ? (driveDesiredSpeed / mechanics.wheelBase)
                                : (currentSpeedForward / mechanics.wheelBase);

                            weRotation = math.radians(weRotation);
                            var currentRotation = GetComponent<Rotation>(wheel.GraphicalRepresentation).Value;
                            commandBuffer.SetComponent(wheel.GraphicalRepresentation, new Rotation
                            {
                                // assumes wheels are aligned with chassis in "bind pose"
                                Value = math.mul(currentRotation, quaternion.AxisAngle(new float3(1, 0, 0), weRotation))
                            });
                        }
                        #endregion

                        var parentFromWorld = math.inverse(worldFromSuspension);
                        if (!hit)
                        {
                            float3 wheelDesiredPos = (-ceUp * mechanics.suspensionLength) + rayStart;
                            var worldPosition = math.lerp(worldFromLocal.pos, wheelDesiredPos, mechanics.suspensionDamping / mechanics.suspensionStrength);
                            // update translation of wheels along suspension column
                            commandBuffer.SetComponent(entity, new Translation
                            {
                                Value = math.mul(parentFromWorld, new float4(worldPosition, sfloat.One)).xyz
                            });
                        }
                        else
                        {
                            // remove the wheelbase to get wheel position.
                            sfloat fraction = rayResult.Fraction - (mechanics.wheelBase) / (mechanics.suspensionLength + mechanics.wheelBase);

                            float3 wheelDesiredPos = math.lerp(rayStart, rayEnd, fraction);
                            // update translation of wheels along suspension column
                            var worldPosition = math.lerp(worldFromLocal.pos, wheelDesiredPos, mechanics.suspensionDamping / mechanics.suspensionStrength);
                            commandBuffer.SetComponent(entity, new Translation
                            {
                                Value = math.mul(parentFromWorld, new float4(worldPosition, sfloat.One)).xyz
                            });

                            #region Suspension
                            {
                                // Calculate and apply the impulses
                                var posA = rayEnd;
                                var posB = rayResult.Position;
                                var lvA = currentSpeedUp * weUp;
                                var lvB = world.GetLinearVelocity(rayResult.RigidBodyIndex, posB);

                                var impulse = mechanics.suspensionStrength * (posB - posA) + mechanics.suspensionDamping * (lvB - lvA);
                                impulse = impulse * invWheelCount;
                                sfloat impulseUp = math.dot(impulse, weUp);

                                // Suspension shouldn't necessarily pull the vehicle down!
                                sfloat downForceLimit = -(sfloat)0.25f;
                                if (downForceLimit < impulseUp)
                                {
                                    impulse = impulseUp * weUp;

                                    UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                                    world.ApplyImpulse(ceIdx, impulse, posA);

                                    if (mechanics.drawDebugInformation != 0)
                                        Debug.DrawRay(wheelDesiredPos, impulse, Color.green);
                                }
                            }
                            #endregion

                            #region Sideways friction
                            {
                                sfloat deltaSpeedRight = (sfloat.Zero - currentSpeedRight);
                                deltaSpeedRight = math.clamp(deltaSpeedRight, -mechanics.wheelMaxImpulseRight, mechanics.wheelMaxImpulseRight);
                                deltaSpeedRight *= mechanics.wheelFrictionRight;
                                deltaSpeedRight *= slopeSlipFactor;

                                float3 impulse = deltaSpeedRight * weRight;
                                sfloat effectiveMass = world.GetEffectiveMass(ceIdx, impulse, wheelPos);
                                impulse = impulse * effectiveMass * invWheelCount;

                                UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                                world.ApplyImpulse(ceIdx, impulse, wheelPos);
                                world.ApplyImpulse(rayResult.RigidBodyIndex, -impulse, wheelPos);

                                if (mechanics.drawDebugInformation != 0)
                                    Debug.DrawRay(wheelDesiredPos, impulse, Color.red);
                            }
                            #endregion

                            #region Drive
                            {
                                if (driveEngaged && wheel.UsedForDriving != 0)
                                {
                                    sfloat deltaSpeedForward = (driveDesiredSpeed - currentSpeedForward);
                                    deltaSpeedForward = math.clamp(deltaSpeedForward, -mechanics.wheelMaxImpulseForward, mechanics.wheelMaxImpulseForward);
                                    deltaSpeedForward *= mechanics.wheelFrictionForward;
                                    deltaSpeedForward *= slopeSlipFactor;

                                    float3 impulse = deltaSpeedForward * weForward;

                                    sfloat effectiveMass = world.GetEffectiveMass(ceIdx, impulse, wheelPos);
                                    impulse = impulse * effectiveMass * invWheelCount;

                                    UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                                    world.ApplyImpulse(ceIdx, impulse, wheelPos);
                                    world.ApplyImpulse(rayResult.RigidBodyIndex, -impulse, wheelPos);

                                    if (mechanics.drawDebugInformation != 0)
                                        Debug.DrawRay(wheelDesiredPos, impulse, Color.blue);
                                }
                            }
                            #endregion
                        }
                    })
                .Run();

            commandBuffer.Playback(EntityManager);
            commandBuffer.Dispose();
        }
    }
}
