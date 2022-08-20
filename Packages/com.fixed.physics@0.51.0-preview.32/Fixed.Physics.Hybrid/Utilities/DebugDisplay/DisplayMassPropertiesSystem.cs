using Fixed.Physics;
using Fixed.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Mathematics;
using UnityEngine;
using Unity.Burst;

namespace Fixed.Physics.Authoring
{
    /// Create and dispatch a DisplayMassPropertiesJob
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public partial class DisplayMassPropertiesSystem : SystemBase
    {
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
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawMassProperties != 0))
            {
                return;
            }

            Dependency = new DisplayMassPropertiesJob
            {
                OutputStream = m_DebugStreamSystem.GetContext(1),
                MotionDatas = m_BuildPhysicsWorldSystem.PhysicsWorld.MotionDatas,
                MotionVelocities = m_BuildPhysicsWorldSystem.PhysicsWorld.MotionVelocities
            }.Schedule(Dependency);
        }

        // Job to write mass properties info to a DebugStream for any moving bodies
        // Attempts to build a box which has the same inertia tensor as the body.
        [BurstCompile]
        struct DisplayMassPropertiesJob : IJob //<todo.eoin.udebug This can be a parallelfor job
        {
            public DebugStream.Context OutputStream;

            [ReadOnly] public NativeArray<MotionData> MotionDatas;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;

            public void Execute()
            {
                OutputStream.Begin(0);
                for (int m = 0; m < MotionDatas.Length; m++)
                {
                    float3 com = MotionDatas[m].WorldFromMotion.pos;
                    quaternion o = MotionDatas[m].WorldFromMotion.rot;

                    float3 invInertiaLocal = MotionVelocities[m].InverseInertia;
                    float3 il = new float3(sfloat.One / invInertiaLocal.x, sfloat.One / invInertiaLocal.y, sfloat.One / invInertiaLocal.z);
                    sfloat invMass = MotionVelocities[m].InverseMass;

                    // Reverse the inertia tensor computation to build a box which has the inerta tensor 'il'
                    // The diagonal inertia of a box with dimensions h,w,d and mass m is:
                    // Ix = 1/12 m (ww + dd)
                    // Iy = 1/12 m (dd + hh)
                    // Iz = 1/12 m (ww + hh)
                    //
                    // For simplicity, set K = I * 12 / m
                    // Then K = (ww + dd, dd + hh, ww + hh)
                    // => ww = Kx - dd, dd = Ky - hh, hh = Kz - ww
                    // By manipulation:
                    // 2ww = Kx - Ky + Kz
                    // => w = ((0.5)(Kx - Ky + Kz))^-1
                    // Then, substitution gives h and d.
                    var temp = (sfloat)12;

                    float3 k = new float3(il.x * temp * invMass, il.y * temp * invMass, il.z * temp * invMass);
                    sfloat w = math.sqrt((k.x - k.y + k.z) * (sfloat)0.5f);
                    sfloat h = math.sqrt(k.z - w * w);
                    sfloat d = math.sqrt(k.y - h * h);

                    float3 boxSize = new float3(h, w, d);
                    OutputStream.Box(boxSize, com, o, DebugDisplay.ColorIndex.Magenta);
                }
                OutputStream.End();
            }
        }
    }
}
