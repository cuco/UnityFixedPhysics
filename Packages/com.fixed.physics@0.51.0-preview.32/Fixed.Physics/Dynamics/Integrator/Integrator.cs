using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public static class Integrator
    {
        // Integrate the world's motions forward by the given time step.
        public static void Integrate(NativeArray<MotionData> motionDatas, NativeArray<MotionVelocity> motionVelocities, fp timeStep)
        {
            for (int i = 0; i < motionDatas.Length; i++)
            {
                ParallelIntegrateMotionsJob.ExecuteImpl(i, motionDatas, motionVelocities, timeStep);
            }
        }

        // Integrate a single transform for the provided velocity and time
        public static void Integrate(ref FpRigidTransform transform, in MotionVelocity motionVelocity, in fp timeStep)
        {
            // center of mass
            IntegratePosition(ref transform.pos, motionVelocity.LinearVelocity, timeStep);

            // orientation
            IntegrateOrientation(ref transform.rot, motionVelocity.AngularVelocity, timeStep);
        }

        // Schedule a job to integrate the world's motions forward by the given time step.
        internal static JobHandle ScheduleIntegrateJobs(ref DynamicsWorld world, fp timeStep, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                var job = new IntegrateMotionsJob
                {
                    MotionDatas = world.MotionDatas,
                    MotionVelocities = world.MotionVelocities,
                    TimeStep = timeStep
                };
                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelIntegrateMotionsJob
                {
                    MotionDatas = world.MotionDatas,
                    MotionVelocities = world.MotionVelocities,
                    TimeStep = timeStep
                };
                return job.Schedule(world.NumMotions, 64, inputDeps);
            }
        }

        [BurstCompile]
        private struct ParallelIntegrateMotionsJob : IJobParallelFor
        {
            public NativeArray<MotionData> MotionDatas;
            public NativeArray<MotionVelocity> MotionVelocities;
            public fp TimeStep;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionDatas, MotionVelocities, TimeStep);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionData> motionDatas, NativeArray<MotionVelocity> motionVelocities, fp timeStep)
            {
                MotionData motionData = motionDatas[i];
                MotionVelocity motionVelocity = motionVelocities[i];

                // Update motion space
                Integrate(ref motionData.WorldFromMotion, motionVelocity, timeStep);

                // Update velocities
                {
                    // damping
                    motionVelocity.LinearVelocity *= fpmath.clamp(fp.one - motionData.LinearDamping * timeStep, fp.zero, fp.one);
                    motionVelocity.AngularVelocity *= fpmath.clamp(fp.one - motionData.AngularDamping * timeStep, fp.zero, fp.one);
                }

                // Write back
                motionDatas[i] = motionData;
                motionVelocities[i] = motionVelocity;
            }
        }

        [BurstCompile]
        private struct IntegrateMotionsJob : IJob
        {
            public NativeArray<MotionData> MotionDatas;
            public NativeArray<MotionVelocity> MotionVelocities;
            public fp TimeStep;

            public void Execute()
            {
                Integrate(MotionDatas, MotionVelocities, TimeStep);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void IntegratePosition(ref fp3 position, fp3 linearVelocity, fp timestep)
        {
            position += linearVelocity * timestep;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void IntegrateOrientation(ref fpquaternion orientation, fp3 angularVelocity, fp timestep)
        {
            fpquaternion dq = IntegrateAngularVelocity(angularVelocity, timestep);
            fpquaternion r = fpmath.mul(orientation, dq);
            orientation = fpmath.normalize(r);
        }

        // Returns a non-normalized fpquaternion that approximates the change in angle angularVelocity * timestep.
        internal static fpquaternion IntegrateAngularVelocity(fp3 angularVelocity, fp timestep)
        {
            fp3 halfDeltaTime = new fp3(timestep * fp.half);
            fp3 halfDeltaAngle = angularVelocity * halfDeltaTime;
            return new fpquaternion(new fp4(halfDeltaAngle, fp.one));
        }
    }
}
