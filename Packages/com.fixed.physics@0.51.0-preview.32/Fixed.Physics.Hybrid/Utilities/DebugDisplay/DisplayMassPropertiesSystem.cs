using Fixed.Physics;
using Fixed.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
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
                    fp3 com = MotionDatas[m].WorldFromMotion.pos;
                    fpquaternion o = MotionDatas[m].WorldFromMotion.rot;

                    fp3 invInertiaLocal = MotionVelocities[m].InverseInertia;
                    fp3 il = new fp3(fp.one / invInertiaLocal.x, fp.one / invInertiaLocal.y, fp.one / invInertiaLocal.z);
                    fp invMass = MotionVelocities[m].InverseMass;

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
                    var temp = (fp)12;

                    fp3 k = new fp3(il.x * temp * invMass, il.y * temp * invMass, il.z * temp * invMass);
                    fp w = fpmath.sqrt((k.x - k.y + k.z) * fp.half);
                    fp h = fpmath.sqrt(k.z - w * w);
                    fp d = fpmath.sqrt(k.y - h * h);

                    fp3 boxSize = new fp3(h, w, d);
                    OutputStream.Box(boxSize, com, o, DebugDisplay.ColorIndex.Magenta);
                }
                OutputStream.End();
            }
        }
    }
}
