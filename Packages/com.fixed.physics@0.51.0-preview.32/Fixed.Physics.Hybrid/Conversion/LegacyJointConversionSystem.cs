#if LEGACY_PHYSICS
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine;
using FloatRange = Fixed.Physics.Math.FloatRange;
using LegacyCharacter = UnityEngine.CharacterJoint;
using LegacyConfigurable = UnityEngine.ConfigurableJoint;
using LegacyFixed = UnityEngine.FixedJoint;
using LegacyHinge = UnityEngine.HingeJoint;
using LegacyJoint = UnityEngine.Joint;
using LegacySpring = UnityEngine.SpringJoint;

namespace Fixed.Physics.Authoring
{
    [AlwaysUpdateSystem]
    [UpdateAfter(typeof(BeginJointConversionSystem))]
    [UpdateBefore(typeof(EndJointConversionSystem))]
    sealed class LegacyJointConversionSystem : GameObjectConversionSystem
    {
        PhysicsJoint CreateConfigurableJoint(
            fpquaternion jointFrameOrientation,
            LegacyJoint joint, bool3 linearLocks, bool3 linearLimited, SoftJointLimit linearLimit, SoftJointLimitSpring linearSpring, bool3 angularFree, bool3 angularLocks,
            bool3 angularLimited, SoftJointLimit lowAngularXLimit, SoftJointLimit highAngularXLimit, SoftJointLimitSpring angularXLimitSpring, SoftJointLimit angularYLimit,
            SoftJointLimit angularZLimit, SoftJointLimitSpring angularYZLimitSpring)
        {
            var constraints = new FixedList128Bytes<Constraint>();

            if (angularLimited[0])
            {
                constraints.Add(Constraint.Twist(
                    0,
                    fpmath.radians(new FloatRange(-(fp)highAngularXLimit.limit, -(fp)lowAngularXLimit.limit).Sorted()),
                    CalculateSpringFrequencyFromSpringConstant((fp)angularXLimitSpring.spring),
                    (fp)angularXLimitSpring.damper)
                );
            }

            if (angularLimited[1])
            {
                constraints.Add(Constraint.Twist(
                    1,
                    fpmath.radians(new FloatRange(-(fp)angularYLimit.limit, (fp)angularYLimit.limit).Sorted()),
                    CalculateSpringFrequencyFromSpringConstant((fp)angularYZLimitSpring.spring),
                    (fp)angularYZLimitSpring.damper));
            }

            if (angularLimited[2])
            {
                constraints.Add(Constraint.Twist(
                    2,
                    fpmath.radians(new FloatRange(-(fp)angularZLimit.limit, (fp)angularZLimit.limit).Sorted()),
                    CalculateSpringFrequencyFromSpringConstant((fp)angularYZLimitSpring.spring),
                    (fp)angularYZLimitSpring.damper));
            }

            if (math.any(linearLimited))
            {
                //If spring=0, then need to treat it and damper as locked. Okay for damper=0 if spring>0
                var spring = Constraint.DefaultSpringFrequency; //stiff spring
                var damping = 1.0f; //critically damped
                if (linearSpring.spring > 0.0f)
                {
                    spring = (fp)linearSpring.spring;
                    damping = linearSpring.damper;
                }

                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLimited,
                    Type = ConstraintType.Linear,
                    Min = fp.zero,
                    Max = (fp)linearLimit.limit,  //allow movement up to limit from anchor
                    SpringFrequency = CalculateSpringFrequencyFromSpringConstant(spring),
                    SpringDamping = (fp)damping
                });
            }

            if (math.any(linearLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = linearLocks,
                    Type = ConstraintType.Linear,
                    Min = (fp)linearLimit.limit,    //lock at distance from anchor
                    Max = (fp)linearLimit.limit,
                    SpringFrequency =  Constraint.DefaultSpringFrequency, //stiff spring
                    SpringDamping = fp.one //critically damped
                });
            }

            if (math.any(angularLocks))
            {
                constraints.Add(new Constraint
                {
                    ConstrainedAxes = angularLocks,
                    Type = ConstraintType.Angular,
                    Min = fp.zero,
                    Max = fp.zero,
                    SpringFrequency = Constraint.DefaultSpringFrequency, //stiff spring
                    SpringDamping = fp.one //critically damped
                });
            }

            FpRigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            FpRigidTransform worldFromBodyB = joint.connectedBody == null
                ? FpRigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var legacyWorldFromJointA = fpmath.mul(
                new FpRigidTransform(joint.transform.rotation, joint.transform.position),
                new FpRigidTransform(jointFrameOrientation, joint.anchor)
            );
            var bodyAFromJoint = new BodyFrame(fpmath.mul(fpmath.inverse(worldFromBodyA), legacyWorldFromJointA));

            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            FpRigidTransform bFromA = isConnectedBodyConverted ? fpmath.mul(fpmath.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            FpRigidTransform bFromBSource =
                isConnectedBodyConverted ? FpRigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new BodyFrame
            {
                Axis = fpmath.mul(bFromA.rot, bodyAFromJoint.Axis),
                PerpendicularAxis = fpmath.mul(bFromA.rot, bodyAFromJoint.PerpendicularAxis),
                Position = fpmath.mul(bFromBSource, new fp4(joint.connectedAnchor, fp.one)).xyz
            };

            var jointData = new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint
            };
            jointData.SetConstraints(constraints);
            return jointData;
        }

        fp CalculateSpringFrequencyFromSpringConstant(fp springConstant)
        {
            if (springConstant < Math.Constants.Eps) return fp.zero;

            return fpmath.sqrt(springConstant) * Math.Constants.OneOverTau;
        }

        bool3 GetAxesWithMotionType(
            ConfigurableJointMotion motionType,
            ConfigurableJointMotion x, ConfigurableJointMotion y, ConfigurableJointMotion z
        ) =>
            new bool3(x == motionType, y == motionType, z == motionType);

        PhysicsConstrainedBodyPair GetConstrainedBodyPair(LegacyJoint joint) =>
            new PhysicsConstrainedBodyPair(
                GetPrimaryEntity(joint.gameObject),
                joint.connectedBody == null ? Entity.Null : GetPrimaryEntity(joint.connectedBody),
                joint.enableCollision
            );

        void ConvertConfigurableJoint(LegacyConfigurable joint)
        {
            var linearLocks =
                GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.xMotion, joint.yMotion, joint.zMotion);
            var linearLimited =
                GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.xMotion, joint.yMotion, joint.zMotion);
            var angularFree =
                GetAxesWithMotionType(ConfigurableJointMotion.Free, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);
            var angularLocks =
                GetAxesWithMotionType(ConfigurableJointMotion.Locked, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);
            var angularLimited =
                GetAxesWithMotionType(ConfigurableJointMotion.Limited, joint.angularXMotion, joint.angularYMotion, joint.angularZMotion);

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.secondaryAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited,
                joint.linearLimit, joint.linearLimitSpring, angularFree, angularLocks, angularLimited,
                joint.lowAngularXLimit, joint.highAngularXLimit, joint.angularXLimitSpring, joint.angularYLimit,
                joint.angularZLimit, joint.angularYZLimitSpring);

            m_EndJointConversionSystem.CreateJointEntity(joint, GetConstrainedBodyPair(joint), jointData);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static fpquaternion GetJointFrameOrientation(fp3 axis, fp3 secondaryAxis)
        {
            // classic Unity uses a different approach than BodyFrame.ValidateAxes() for ortho-normalizing degenerate inputs
            // ortho-normalizing here ensures behavior is consistent with classic Unity
            var a = (Vector3)axis;
            var p = (Vector3)secondaryAxis;
            Vector3.OrthoNormalize(ref a, ref p);
            return new BodyFrame { Axis = a, PerpendicularAxis = p }.AsRigidTransform().rot;
        }

        void ConvertCharacterJoint(LegacyCharacter joint)
        {
            var linearLocks = new bool3(true);
            var linearLimited = new bool3(false);

            var angularFree = new bool3(false);
            var angularLocks = new bool3(false);
            var angularLimited = new bool3(true);

            var jointFrameOrientation = GetJointFrameOrientation(joint.axis, joint.swingAxis);
            var jointData = CreateConfigurableJoint(jointFrameOrientation, joint, linearLocks, linearLimited,
                new SoftJointLimit { limit = 0f, bounciness = 0f },
                new SoftJointLimitSpring { spring = 0f, damper = 0f },
                angularFree, angularLocks, angularLimited,
                joint.lowTwistLimit, joint.highTwistLimit, joint.twistLimitSpring,
                joint.swing1Limit, joint.swing2Limit, joint.swingLimitSpring);

            m_EndJointConversionSystem.CreateJointEntity(joint, GetConstrainedBodyPair(joint), jointData);
        }

        void ConvertSpringJoint(LegacySpring joint)
        {
            var distanceRange = new FloatRange((fp)joint.minDistance, (fp)joint.maxDistance).Sorted();
            var constraint = new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = CalculateSpringFrequencyFromSpringConstant((fp)joint.spring),
                SpringDamping = (fp)joint.damper
            };

            var jointFrameA = BodyFrame.Identity;
            jointFrameA.Position = joint.anchor;

            var connectedEntity = GetPrimaryEntity(joint.connectedBody);

            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            FpRigidTransform bFromBSource =
                isConnectedBodyConverted ? FpRigidTransform.identity : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var jointFrameB = BodyFrame.Identity;
            jointFrameB.Position = fpmath.mul(bFromBSource, new fp4(joint.connectedAnchor, fp.one)).xyz;

            var jointData = new PhysicsJoint
            {
                BodyAFromJoint = jointFrameA,
                BodyBFromJoint = jointFrameB
            };
            jointData.SetConstraints(new FixedList128Bytes<Constraint>
            {
                Length = 1,
                [0] = constraint
            });

            m_EndJointConversionSystem.CreateJointEntity(joint, GetConstrainedBodyPair(joint), jointData);
        }

        void ConvertFixedJoint(LegacyFixed joint)
        {
            var legacyWorldFromJointA = fpmath.mul(
                new FpRigidTransform(joint.transform.rotation, joint.transform.position),
                new FpRigidTransform(fpquaternion.identity, joint.anchor)
            );

            FpRigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            FpRigidTransform worldFromBodyB = connectedEntity == Entity.Null
                ? FpRigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            var bodyAFromJoint = new BodyFrame(fpmath.mul(fpmath.inverse(worldFromBodyA), legacyWorldFromJointA));
            var bodyBFromJoint = new BodyFrame(fpmath.mul(fpmath.inverse(worldFromBodyB), legacyWorldFromJointA));

            var jointData = PhysicsJoint.CreateFixed(bodyAFromJoint, bodyBFromJoint);

            m_EndJointConversionSystem.CreateJointEntity(joint, GetConstrainedBodyPair(joint), jointData);
        }

        void ConvertHingeJoint(LegacyHinge joint)
        {
            FpRigidTransform worldFromBodyA = Math.DecomposeRigidBodyTransform(joint.transform.localToWorldMatrix);
            FpRigidTransform worldFromBodyB = joint.connectedBody == null
                ? FpRigidTransform.identity
                : Math.DecomposeRigidBodyTransform(joint.connectedBody.transform.localToWorldMatrix);

            Math.CalculatePerpendicularNormalized(joint.axis, out fp3 perpendicularA, out _);
            var bodyAFromJoint = new BodyFrame
            {
                Axis = joint.axis,
                PerpendicularAxis = perpendicularA,
                Position = joint.anchor
            };

            var connectedEntity = GetPrimaryEntity(joint.connectedBody);
            var isConnectedBodyConverted =
                joint.connectedBody == null || connectedEntity != Entity.Null;

            FpRigidTransform bFromA = isConnectedBodyConverted ? fpmath.mul(fpmath.inverse(worldFromBodyB), worldFromBodyA) : worldFromBodyA;
            FpRigidTransform bFromBSource =
                isConnectedBodyConverted ? FpRigidTransform.identity : worldFromBodyB;

            var bodyBFromJoint = new BodyFrame
            {
                Axis = fpmath.mul(bFromA.rot, joint.axis),
                PerpendicularAxis = fpmath.mul(bFromA.rot, perpendicularA),
                Position = fpmath.mul(bFromBSource, new fp4(joint.connectedAnchor, fp.one)).xyz
            };

            var limits = fpmath.radians(new FloatRange((fp)joint.limits.min, (fp)joint.limits.max).Sorted());
            var jointData = joint.useLimits
                ? PhysicsJoint.CreateLimitedHinge(bodyAFromJoint, bodyBFromJoint, limits)
                : PhysicsJoint.CreateHinge(bodyAFromJoint, bodyBFromJoint);

            m_EndJointConversionSystem.CreateJointEntity(joint, GetConstrainedBodyPair(joint), jointData);
        }

        EndJointConversionSystem m_EndJointConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();

            m_EndJointConversionSystem = World.GetOrCreateSystem<EndJointConversionSystem>();
        }

        static readonly List<LegacyCharacter> s_CharacterJointInstances = new List<CharacterJoint>(8);
        static readonly List<LegacyConfigurable> s_ConfigurableJointInstances = new List<ConfigurableJoint>(8);
        static readonly List<LegacyFixed> s_FixedJointInstances = new List<FixedJoint>(8);
        static readonly List<LegacyHinge> s_HingeJointInstances = new List<HingeJoint>(8);
        static readonly List<LegacySpring> s_SpringJointInstances = new List<SpringJoint>(8);

        protected override void OnUpdate()
        {
            Entities.ForEach((LegacyCharacter joint) =>
            {
                joint.gameObject.GetComponents(s_CharacterJointInstances);
                foreach (var instance in s_CharacterJointInstances)
                    ConvertCharacterJoint(instance);
            });
            Entities.ForEach((LegacyConfigurable joint) =>
            {
                joint.gameObject.GetComponents(s_ConfigurableJointInstances);
                foreach (var instance in s_ConfigurableJointInstances)
                    ConvertConfigurableJoint(instance);
            });
            Entities.ForEach((LegacyFixed joint) =>
            {
                joint.gameObject.GetComponents(s_FixedJointInstances);
                foreach (var instance in s_FixedJointInstances)
                    ConvertFixedJoint(instance);
            });
            Entities.ForEach((LegacyHinge joint) =>
            {
                joint.gameObject.GetComponents(s_HingeJointInstances);
                foreach (var instance in s_HingeJointInstances)
                    ConvertHingeJoint(instance);
            });
            Entities.ForEach((LegacySpring joint) =>
            {
                joint.gameObject.GetComponents(s_SpringJointInstances);
                foreach (var instance in s_SpringJointInstances)
                    ConvertSpringJoint(instance);
            });

            s_CharacterJointInstances.Clear();
            s_ConfigurableJointInstances.Clear();
            s_FixedJointInstances.Clear();
            s_HingeJointInstances.Clear();
            s_SpringJointInstances.Clear();
        }
    }
}
#endif
