using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public struct ContactJacobianAngular
    {
        public fp3 AngularA;
        public fp3 AngularB;
        public fp EffectiveMass;
        public fp Impulse; // Accumulated impulse
    }

    public struct ContactJacAngAndVelToReachCp  // TODO: better name
    {
        public ContactJacobianAngular Jac;

        // Velocity needed to reach the contact plane in one frame,
        // both if approaching (negative) and depenetrating (positive)
        public fp VelToReachCp;
    }

    public struct SurfaceVelocity
    {
        // Velocities between the two contacting objects
        public fp3 LinearVelocity;
        public fp3 AngularVelocity;
    }

    public struct MassFactors
    {
        public fp3 InverseInertiaFactorA;
        public fp InverseMassFactorA;
        public fp3 InverseInertiaFactorB;
        public fp InverseMassFactorB;

        public static MassFactors Default => new MassFactors
        {
            InverseInertiaFactorA = new fp3(fp.one),
            InverseMassFactorA = fp.one,
            InverseInertiaFactorB = new fp3(fp.one),
            InverseMassFactorB = fp.one
        };
    }

    struct BaseContactJacobian
    {
        public int NumContacts;
        public fp3 Normal;

        internal static fp GetJacVelocity(fp3 linear, ContactJacobianAngular jacAngular,
            fp3 linVelA, fp3 angVelA, fp3 linVelB, fp3 angVelB)
        {
            fp3 temp = (linVelA - linVelB) * linear;
            temp += angVelA * jacAngular.AngularA;
            temp += angVelB * jacAngular.AngularB;
            return fpmath.csum(temp);
        }
    }

    // A Jacobian representing a set of contact points that apply impulses
    [NoAlias]
    struct ContactJacobian
    {
        public BaseContactJacobian BaseJacobian;

        // Linear friction jacobians.  Only store the angular part, linear part can be recalculated from BaseJacobian.Normal
        public ContactJacobianAngular Friction0; // EffectiveMass stores friction effective mass matrix element (0, 0)
        public ContactJacobianAngular Friction1; // EffectiveMass stores friction effective mass matrix element (1, 1)

        // Angular friction about the contact normal, no linear part
        public ContactJacobianAngular AngularFriction; // EffectiveMass stores friction effective mass matrix element (2, 2)
        public fp3 FrictionEffectiveMassOffDiag; // Effective mass matrix (0, 1), (0, 2), (1, 2) == (1, 0), (2, 0), (2, 1)

        public fp CoefficientOfFriction;

        // Generic solve method that dispatches to specific ones
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            bool bothMotionsAreKinematic = velocityA.IsKinematic && velocityB.IsKinematic;
            if (bothMotionsAreKinematic)
            {
                // Note that static bodies are assigned with MotionVelocity.Zero.
                // So, at this point the bodies could be a kinematic vs kinematic pair, or a kinematic vs static pair.
                // Either way, both bodies have an infinite mass and applying contact impulses would have no effect.
                SolveInfMassPair(ref jacHeader, velocityA, velocityB, stepInput, ref collisionEventsWriter);
            }
            else
            {
                SolveContact(ref jacHeader, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                    enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
            }
        }

        // Solve the Jacobian
        public void SolveContact(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            // Copy velocity data
            MotionVelocity tempVelocityA = velocityA;
            MotionVelocity tempVelocityB = velocityB;
            if (jacHeader.HasMassFactors)
            {
                MassFactors jacMod = jacHeader.AccessMassFactors();
                tempVelocityA.InverseInertia *= jacMod.InverseInertiaFactorA;
                tempVelocityA.InverseMass *= jacMod.InverseMassFactorA;
                tempVelocityB.InverseInertia *= jacMod.InverseInertiaFactorB;
                tempVelocityB.InverseMass *= jacMod.InverseMassFactorB;
            }

            // Solve normal impulses
            fp sumImpulses = fp.zero;
            fp totalAccumulatedImpulse = fp.zero;
            bool forceCollisionEvent = false;
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                fp relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    tempVelocityA.LinearVelocity, tempVelocityA.AngularVelocity, tempVelocityB.LinearVelocity, tempVelocityB.AngularVelocity);
                fp dv = jacAngular.VelToReachCp - relativeVelocity;

                fp impulse = dv * jacAngular.Jac.EffectiveMass;
                fp accumulatedImpulse = fpmath.max(jacAngular.Jac.Impulse + impulse, fp.zero);
                if (accumulatedImpulse != jacAngular.Jac.Impulse)
                {
                    fp deltaImpulse = accumulatedImpulse - jacAngular.Jac.Impulse;
                    ApplyImpulse(deltaImpulse, BaseJacobian.Normal, jacAngular.Jac, ref tempVelocityA, ref tempVelocityB,
                        motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);
                }

                jacAngular.Jac.Impulse = accumulatedImpulse;
                sumImpulses += accumulatedImpulse;
                totalAccumulatedImpulse += jacAngular.Jac.Impulse;

                // Force contact event even when no impulse is applied, but there is penetration.
                forceCollisionEvent |= jacAngular.VelToReachCp > fp.zero;
            }

            // Export collision event
            if (stepInput.IsLastIteration && (totalAccumulatedImpulse > fp.zero || forceCollisionEvent) && jacHeader.HasContactManifold)
            {
                ExportCollisionEvent(totalAccumulatedImpulse, ref jacHeader, ref collisionEventsWriter);
            }

            // Solve friction
            if (sumImpulses > fp.zero)
            {
                // Choose friction axes
                Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out fp3 frictionDir0, out fp3 frictionDir1);

                // Calculate impulses for full stop
                fp3 imp;
                {
                    // Take velocities that produce minimum energy (between input and solver velocity) as friction input
                    fp3 frictionLinVelA = tempVelocityA.LinearVelocity;
                    fp3 frictionAngVelA = tempVelocityA.AngularVelocity;
                    fp3 frictionLinVelB = tempVelocityB.LinearVelocity;
                    fp3 frictionAngVelB = tempVelocityB.AngularVelocity;
                    if (enableFrictionVelocitiesHeuristic)
                    {
                        GetFrictionVelocities(motionStabilizationSolverInputA.InputVelocity.Linear, motionStabilizationSolverInputA.InputVelocity.Angular,
                            tempVelocityA.LinearVelocity, tempVelocityA.AngularVelocity,
                            fpmath.rcp(tempVelocityA.InverseInertia), fpmath.rcp(tempVelocityA.InverseMass),
                            out frictionLinVelA, out frictionAngVelA);
                        GetFrictionVelocities(motionStabilizationSolverInputB.InputVelocity.Linear, motionStabilizationSolverInputB.InputVelocity.Angular,
                            tempVelocityB.LinearVelocity, tempVelocityB.AngularVelocity,
                            fpmath.rcp(tempVelocityB.InverseInertia), fpmath.rcp(tempVelocityB.InverseMass),
                            out frictionLinVelB, out frictionAngVelB);
                    }

                    fp3 extraFrictionDv = fp3.zero;
                    if (jacHeader.HasSurfaceVelocity)
                    {
                        var surfVel = jacHeader.AccessSurfaceVelocity();

                        Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out fp3 dir0, out fp3 dir1);
                        fp linVel0 = fpmath.dot(surfVel.LinearVelocity, dir0);
                        fp linVel1 = fpmath.dot(surfVel.LinearVelocity, dir1);

                        fp angVelProj = fpmath.dot(surfVel.AngularVelocity, BaseJacobian.Normal);
                        extraFrictionDv = new fp3(linVel0, linVel1, angVelProj);
                    }

                    // Calculate the jacobian dot velocity for each of the friction jacobians
                    fp dv0 = extraFrictionDv.x - BaseContactJacobian.GetJacVelocity(frictionDir0, Friction0, frictionLinVelA, frictionAngVelA, frictionLinVelB, frictionAngVelB);
                    fp dv1 = extraFrictionDv.y - BaseContactJacobian.GetJacVelocity(frictionDir1, Friction1, frictionLinVelA, frictionAngVelA, frictionLinVelB, frictionAngVelB);
                    fp dva = extraFrictionDv.z - fpmath.csum(AngularFriction.AngularA * frictionAngVelA + AngularFriction.AngularB * frictionAngVelB);

                    // Reassemble the effective mass matrix
                    fp3 effectiveMassDiag = new fp3(Friction0.EffectiveMass, Friction1.EffectiveMass, AngularFriction.EffectiveMass);
                    fp3x3 effectiveMass = JacobianUtilities.BuildSymmetricMatrix(effectiveMassDiag, FrictionEffectiveMassOffDiag);

                    // Calculate the impulse
                    imp = fpmath.mul(effectiveMass, new fp3(dv0, dv1, dva));
                }

                // Clip TODO.ma calculate some contact radius and use it to influence balance between linear and angular friction
                fp maxImpulse = sumImpulses * CoefficientOfFriction * stepInput.InvNumSolverIterations;
                fp frictionImpulseSquared = fpmath.lengthsq(imp);
                imp *= fpmath.min(fp.one, maxImpulse * fpmath.rsqrt(frictionImpulseSquared));

                // Apply impulses
                ApplyImpulse(imp.x, frictionDir0, Friction0, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);
                ApplyImpulse(imp.y, frictionDir1, Friction1, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);

                tempVelocityA.ApplyAngularImpulse(imp.z * AngularFriction.AngularA * motionStabilizationSolverInputA.InverseInertiaScale);
                tempVelocityB.ApplyAngularImpulse(imp.z * AngularFriction.AngularB * motionStabilizationSolverInputB.InverseInertiaScale);

                // Accumulate them
                Friction0.Impulse += imp.x;
                Friction1.Impulse += imp.y;
                AngularFriction.Impulse += imp.z;
            }

            // Write back linear and angular velocities. Changes to other properties, like InverseMass, should not be persisted.
            velocityA.LinearVelocity = tempVelocityA.LinearVelocity;
            velocityA.AngularVelocity = tempVelocityA.AngularVelocity;
            velocityB.LinearVelocity = tempVelocityB.LinearVelocity;
            velocityB.AngularVelocity = tempVelocityB.AngularVelocity;
        }

        // Solve the infinite mass pair Jacobian
        public void SolveInfMassPair(
            ref JacobianHeader jacHeader, MotionVelocity velocityA, MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            // Infinite mass pairs are only interested in collision events,
            // so only last iteration is performed in that case
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            // Calculate normal impulses and fire collision event
            // if at least one contact point would have an impulse applied
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                fp relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                fp dv = jacAngular.VelToReachCp - relativeVelocity;
                if (jacAngular.VelToReachCp > fp.zero || dv > fp.zero)
                {
                    // Export collision event only if impulse would be applied, or objects are penetrating
                    ExportCollisionEvent(fp.zero, ref jacHeader, ref collisionEventsWriter);

                    return;
                }
            }
        }

        // Helper functions
        void GetFrictionVelocities(
            fp3 inputLinearVelocity, fp3 inputAngularVelocity,
            fp3 intermediateLinearVelocity, fp3 intermediateAngularVelocity,
            fp3 inertia, fp mass,
            out fp3 frictionLinearVelocityOut, out fp3 frictionAngularVelocityOut)
        {
            fp inputEnergy;
            {
                fp linearEnergySq = mass * fpmath.lengthsq(inputLinearVelocity);
                fp angularEnergySq = fpmath.dot(inertia * inputAngularVelocity, inputAngularVelocity);
                inputEnergy = linearEnergySq + angularEnergySq;
            }

            fp intermediateEnergy;
            {
                fp linearEnergySq = mass * fpmath.lengthsq(intermediateLinearVelocity);
                fp angularEnergySq = fpmath.dot(inertia * intermediateAngularVelocity, intermediateAngularVelocity);
                intermediateEnergy = linearEnergySq + angularEnergySq;
            }

            if (inputEnergy < intermediateEnergy)
            {
                // Make sure we don't change the sign of intermediate velocity when using the input one.
                // If sign was to be changed, zero it out since it produces less energy.
                bool3 changedSignLin = inputLinearVelocity * intermediateLinearVelocity < fp3.zero;
                bool3 changedSignAng = inputAngularVelocity * intermediateAngularVelocity < fp3.zero;
                frictionLinearVelocityOut = fpmath.select(inputLinearVelocity, fp3.zero, changedSignLin);
                frictionAngularVelocityOut = fpmath.select(inputAngularVelocity, fp3.zero, changedSignAng);
            }
            else
            {
                frictionLinearVelocityOut = intermediateLinearVelocity;
                frictionAngularVelocityOut = intermediateAngularVelocity;
            }
        }

        private static void ApplyImpulse(
            fp impulse, fp3 linear, ContactJacobianAngular jacAngular,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            fp inverseInertiaScaleA /* = 1.0f */, fp inverseInertiaScaleB /* = 1.0f */)
        {
            velocityA.ApplyLinearImpulse(impulse * linear);
            velocityB.ApplyLinearImpulse(-impulse * linear);

            // Scale the impulse with inverseInertiaScale
            velocityA.ApplyAngularImpulse(impulse * jacAngular.AngularA * inverseInertiaScaleA);
            velocityB.ApplyAngularImpulse(impulse * jacAngular.AngularB * inverseInertiaScaleB);
        }

        private unsafe void ExportCollisionEvent(fp totalAccumulatedImpulse, [NoAlias] ref JacobianHeader jacHeader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter)
        {
            // Write size before every event
            int collisionEventSize = CollisionEventData.CalculateSize(BaseJacobian.NumContacts);
            collisionEventsWriter.Write(collisionEventSize);

            // Allocate all necessary data for this event
            byte* eventPtr = collisionEventsWriter.Allocate(collisionEventSize);

            // Fill up event data
            ref CollisionEventData collisionEvent = ref UnsafeUtility.AsRef<CollisionEventData>(eventPtr);
            collisionEvent.BodyIndices = jacHeader.BodyPair;
            collisionEvent.ColliderKeys = jacHeader.AccessColliderKeys();
            collisionEvent.Entities = jacHeader.AccessEntities();
            collisionEvent.Normal = BaseJacobian.Normal;
            collisionEvent.SolverImpulse = totalAccumulatedImpulse;
            collisionEvent.NumNarrowPhaseContactPoints = BaseJacobian.NumContacts;
            for (int i = 0; i < BaseJacobian.NumContacts; i++)
            {
                collisionEvent.AccessContactPoint(i) = jacHeader.AccessContactPoint(i);
            }
        }
    }

    // A Jacobian representing a set of contact points that export trigger events
    [NoAlias]
    struct TriggerJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public ColliderKeyPair ColliderKeys;
        public EntityPair Entities;

        // Solve the Jacobian
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput,
            ref NativeStream.Writer triggerEventsWriter)
        {
            // Export trigger events only in last iteration
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                fp relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                fp dv = jacAngular.VelToReachCp - relativeVelocity;
                if (jacAngular.VelToReachCp > fp.zero || dv > fp.zero)
                {
                    // Export trigger event only if impulse would be applied, or objects are penetrating
                    triggerEventsWriter.Write(new TriggerEventData
                    {
                        BodyIndices = jacHeader.BodyPair,
                        ColliderKeys = ColliderKeys,
                        Entities = Entities
                    });

                    return;
                }
            }
        }
    }
}
