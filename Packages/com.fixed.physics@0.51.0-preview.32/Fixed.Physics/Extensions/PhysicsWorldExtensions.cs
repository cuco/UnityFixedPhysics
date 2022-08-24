using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.Extensions
{
    // Utility functions acting on a physics world
    public static class PhysicsWorldExtensions
    {
        public static CollisionFilter GetCollisionFilter(this in PhysicsWorld world, int rigidBodyIndex)
        {
            CollisionFilter filter = CollisionFilter.Default;
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumBodies)) return filter;

            unsafe { filter = world.Bodies[rigidBodyIndex].Collider.Value.Filter; }

            return filter;
        }

        public static fp GetMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            return mv.InverseMass == fp.zero ? fp.zero : fp.one / mv.InverseMass;
        }

        // Get the effective mass of a Rigid Body in a given direction and from a particular point (in World Space)
        public static fp GetEffectiveMass(this in PhysicsWorld world, int rigidBodyIndex, fp3 impulse, fp3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            return GetEffectiveMassImpl(GetCenterOfMass(world, rigidBodyIndex), mv.InverseInertia, impulse, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fp GetEffectiveMassImpl(fp3 centerOfMass, fp3 inverseInertia, fp3 impulse, fp3 point)
        {
            fp3 pointDir = fpmath.normalizesafe(point - centerOfMass);
            fp3 impulseDir = fpmath.normalizesafe(impulse);

            fp3 jacobian = fpmath.cross(pointDir, impulseDir);
            fp invEffMass = fpmath.csum(fpmath.dot(jacobian, jacobian) * inverseInertia);
            return fpmath.select(fp.one / invEffMass, fp.zero, fpmath.abs(invEffMass) < fp.FromRaw(0x3727c5ac));
        }

        // Get the Rigid Bodies Center of Mass (in World Space)
        public static fp3 GetCenterOfMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp3.zero;

            return world.MotionDatas[rigidBodyIndex].WorldFromMotion.pos;
        }

        public static fp3 GetPosition(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp3.zero;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            FpRigidTransform worldFromBody = fpmath.mul(md.WorldFromMotion, fpmath.inverse(md.BodyFromMotion));
            return worldFromBody.pos;
        }

        public static fpquaternion GetRotation(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fpquaternion.identity;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            FpRigidTransform worldFromBody = fpmath.mul(md.WorldFromMotion, fpmath.inverse(md.BodyFromMotion));
            return worldFromBody.rot;
        }

        // Get the linear velocity of a rigid body (in world space)
        public static fp3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp3.zero;

            return world.MotionVelocities[rigidBodyIndex].LinearVelocity;
        }

        // Set the linear velocity of a rigid body (in world space)
        public static void SetLinearVelocity(this PhysicsWorld world, int rigidBodyIndex, fp3 linearVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.LinearVelocity = linearVelocity;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Get the linear velocity of a rigid body at a given point (in world space)
        public static fp3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex, fp3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            return GetLinearVelocityImpl(md.WorldFromMotion, mv.AngularVelocity, mv.LinearVelocity, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fp3 GetLinearVelocityImpl(FpRigidTransform worldFromMotion, fp3 angularVelocity, fp3 linearVelocity, fp3 point)
        {
            angularVelocity = fpmath.rotate(worldFromMotion, angularVelocity);
            return linearVelocity + fpmath.cross(angularVelocity, point - worldFromMotion.pos);
        }

        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static fp3 GetAngularVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return fp3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            return fpmath.rotate(md.WorldFromMotion, mv.AngularVelocity);
        }

        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(this PhysicsWorld world, int rigidBodyIndex, fp3 angularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            fp3 angularVelocityMotionSpace = fpmath.rotate(fpmath.inverse(md.WorldFromMotion.rot), angularVelocity);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.AngularVelocity = angularVelocityMotionSpace;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an impulse to a rigid body at a point (in world space)
        public static void ApplyImpulse(this PhysicsWorld world, int rigidBodyIndex, fp3 linearImpulse, fp3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            fp3 angularImpulseWorldSpace = fpmath.cross(point - md.WorldFromMotion.pos, linearImpulse);
            fp3 angularImpulseMotionSpace = fpmath.rotate(fpmath.inverse(md.WorldFromMotion.rot), angularImpulseWorldSpace);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse);
            mv.ApplyAngularImpulse(angularImpulseMotionSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply a linear impulse to a rigid body (in world space)
        public static void ApplyLinearImpulse(this PhysicsWorld world, int rigidBodyIndex, fp3 linearImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an angular impulse to a rigidBodyIndex (in world space)
        public static void ApplyAngularImpulse(this PhysicsWorld world, int rigidBodyIndex, fp3 angularImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            fp3 angularImpulseInertiaSpace = fpmath.rotate(fpmath.inverse(md.WorldFromMotion.rot), angularImpulse);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyAngularImpulse(angularImpulseInertiaSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Calculate a linear and angular velocity required to move the given rigid body to the given target transform
        // in the given time step.
        public static void CalculateVelocityToTarget(
            this PhysicsWorld world, int rigidBodyIndex, FpRigidTransform targetTransform, fp timestep,
            out fp3 requiredLinearVelocity, out fp3 requiredAngularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies))
            {
                requiredLinearVelocity = default;
                requiredAngularVelocity = default;
                return;
            }

            MotionData md = world.MotionDatas[rigidBodyIndex];
            FpRigidTransform worldFromBody = fpmath.mul(md.WorldFromMotion, fpmath.inverse(md.BodyFromMotion));
            CalculateVelocityToTargetImpl(
                worldFromBody, fpmath.inverse(md.WorldFromMotion.rot), md.BodyFromMotion.pos, targetTransform, timestep,
                out requiredLinearVelocity, out requiredAngularVelocity
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CalculateVelocityToTargetImpl(
            FpRigidTransform worldFromBody, fpquaternion motionFromWorld, fp3 centerOfMass,
            FpRigidTransform targetTransform, in fp stepFrequency,
            out fp3 requiredLinearVelocity, out fp3 requiredAngularVelocity
        )
        {
            var com = new fp4(centerOfMass, fp.one);
            requiredLinearVelocity = (fpmath.mul(targetTransform, com) - fpmath.mul(worldFromBody, com)).xyz * stepFrequency;
            var angularVelocity = fpmath.mul(targetTransform.rot, fpmath.inverse(worldFromBody.rot)).ToEulerAngles() * stepFrequency;
            requiredAngularVelocity = fpmath.rotate(motionFromWorld, angularVelocity);
        }
    }
}
