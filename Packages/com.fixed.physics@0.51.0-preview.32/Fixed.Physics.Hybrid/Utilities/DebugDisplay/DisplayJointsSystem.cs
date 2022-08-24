using System;
using Unity.Burst;
using Fixed.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Authoring
{
    // Creates DisplayJointsJobs
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayJointsSystem : SystemBase
    {
        /// Job which draws every joint
        [BurstCompile]
        protected struct DisplayJointsJob : IJob
        {
            static readonly fp k_Scale = fp.half;

            public DebugStream.Context OutputStream;
            [ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public NativeArray<Joint> Joints;

            public unsafe void Execute()
            {
                // Color palette
                var colorA = Fixed.DebugDisplay.ColorIndex.Cyan;
                var colorB = Fixed.DebugDisplay.ColorIndex.Magenta;
                var colorError = Fixed.DebugDisplay.ColorIndex.Red;
                var colorRange = Fixed.DebugDisplay.ColorIndex.Yellow;

                OutputStream.Begin(0);

                for (int iJoint = 0; iJoint < Joints.Length; iJoint++)
                {
                    Joint joint = Joints[iJoint];

                    if (!joint.BodyPair.IsValid) continue;

                    RigidBody bodyA = Bodies[joint.BodyPair.BodyIndexA];
                    RigidBody bodyB = Bodies[joint.BodyPair.BodyIndexB];

                    MTransform worldFromA, worldFromB;
                    MTransform worldFromJointA, worldFromJointB;
                    {
                        worldFromA = new MTransform(bodyA.WorldFromBody);
                        worldFromB = new MTransform(bodyB.WorldFromBody);

                        worldFromJointA = Mul(worldFromA, joint.AFromJoint);
                        worldFromJointB = Mul(worldFromB, joint.BFromJoint);
                    }

                    fp3 pivotA = worldFromJointA.Translation;
                    fp3 pivotB = worldFromJointB.Translation;

                    //TODO
                    fp ep = (fp)1e-5;

                    for (var i = 0; i < joint.Constraints.Length; i++)
                    {
                        Constraint constraint = joint.Constraints[i];
                        switch (constraint.Type)
                        {
                            case ConstraintType.Linear:

                                fp3 diff = pivotA - pivotB;

                                // Draw the feature on B and find the range for A
                                fp3 rangeOrigin;
                                fp3 rangeDirection;
                                fp rangeDistance;
                                switch (constraint.Dimension)
                                {
                                    case 0:
                                        continue;
                                    case 1:
                                        fp3 normal = worldFromJointB.Rotation[constraint.ConstrainedAxis1D];
                                        OutputStream.Plane(pivotB, normal * k_Scale, colorB);
                                        rangeDistance = fpmath.dot(normal, diff);
                                        rangeOrigin = pivotA - normal * rangeDistance;
                                        rangeDirection = normal;
                                        break;
                                    case 2:
                                        fp3 direction = worldFromJointB.Rotation[constraint.FreeAxis2D];
                                        OutputStream.Line(pivotB - direction * k_Scale, pivotB + direction * k_Scale, colorB);
                                        fp dot = fpmath.dot(direction, diff);
                                        rangeOrigin = pivotB + direction * dot;
                                        rangeDirection = diff - direction * dot;
                                        rangeDistance = fpmath.length(rangeDirection);
                                        //TODO
                                        rangeDirection = fpmath.select(rangeDirection / rangeDistance, fp3.zero, rangeDistance < ep);
                                        break;
                                    case 3:
                                        OutputStream.Point(pivotB, k_Scale, colorB);
                                        rangeOrigin = pivotB;
                                        rangeDistance = fpmath.length(diff);
                                        rangeDirection = fpmath.select(diff / rangeDistance, fp3.zero, rangeDistance < ep);
                                        break;
                                    default:
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
                                }

                                // Draw the pivot on A
                                OutputStream.Point(pivotA, k_Scale, colorA);

                                // Draw error
                                fp3 rangeA = rangeOrigin + rangeDistance * rangeDirection;
                                fp3 rangeMin = rangeOrigin + constraint.Min * rangeDirection;
                                fp3 rangeMax = rangeOrigin + constraint.Max * rangeDirection;
                                if (rangeDistance < constraint.Min)
                                {
                                    OutputStream.Line(rangeA, rangeMin, colorError);
                                }
                                else if (rangeDistance > constraint.Max)
                                {
                                    OutputStream.Line(rangeA, rangeMax, colorError);
                                }
                                if (fpmath.length(rangeA - pivotA) > ep)
                                {
                                    OutputStream.Line(rangeA, pivotA, colorError);
                                }

                                // Draw the range
                                if (constraint.Min != constraint.Max)
                                {
                                    OutputStream.Line(rangeMin, rangeMax, colorRange);
                                }

                                break;
                            case ConstraintType.Angular:
                                switch (constraint.Dimension)
                                {
                                    case 0:
                                        continue;
                                    case 1:
                                        // Get the limited axis and perpendicular in joint space
                                        int constrainedAxis = constraint.ConstrainedAxis1D;
                                        fp3 axisInWorld = worldFromJointA.Rotation[constrainedAxis];
                                        fp3 perpendicularInWorld = worldFromJointA.Rotation[(constrainedAxis + 1) % 3] * k_Scale;

                                        // Draw the angle of A
                                        OutputStream.Line(pivotA, pivotA + perpendicularInWorld, colorA);

                                        // Calculate the relative angle
                                        fp angle;
                                        {
                                            fp3x3 jointBFromA = fpmath.mul(fpmath.inverse(worldFromJointB.Rotation), worldFromJointA.Rotation);
                                            angle = CalculateTwistAngle(new fpquaternion(jointBFromA), constrainedAxis);
                                        }

                                        // Draw the range in B
                                        fp3 axis = worldFromJointA.Rotation[constraint.ConstrainedAxis1D];
                                        OutputStream.Arc(pivotB, axis, fpmath.mul(fpquaternion.AxisAngle(axis, constraint.Min - angle), perpendicularInWorld), constraint.Max - constraint.Min, colorB);

                                        break;
                                    case 2:
                                        // Get axes in world space
                                        int axisIndex = constraint.FreeAxis2D;
                                        fp3 axisA = worldFromJointA.Rotation[axisIndex];
                                        fp3 axisB = worldFromJointB.Rotation[axisIndex];

                                        // Draw the cones in B
                                        if (constraint.Min == fp.zero)
                                        {
                                            OutputStream.Line(pivotB, pivotB + axisB * k_Scale, colorB);
                                        }
                                        else
                                        {
                                            OutputStream.Cone(pivotB, axisB * k_Scale, constraint.Min, colorB);
                                        }
                                        if (constraint.Max != constraint.Min)
                                        {
                                            OutputStream.Cone(pivotB, axisB * k_Scale, constraint.Max, colorB);
                                        }

                                        // Draw the axis in A
                                        OutputStream.Arrow(pivotA, axisA * k_Scale, colorA);

                                        break;
                                    case 3:
                                        // TODO - no idea how to visualize this if the limits are nonzero :)
                                        break;
                                    default:
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
                                }
                                break;
                            default:
                                SafetyChecks.ThrowNotImplementedException();
                                return;
                        }
                    }
                }

                OutputStream.End();
            }
        }

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadOnly();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawJoints != 0))
            {
                return;
            }

#pragma warning disable 618
            Dependency = new DisplayJointsJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies,
                Joints = m_BuildPhysicsWorldSystem.PhysicsWorld.Joints
            }.Schedule(Dependency);
#pragma warning restore 618
        }
    }
}
