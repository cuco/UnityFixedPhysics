using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using Fixed.Physics.Extensions;
using Fixed.Physics.Stateful;
using UnityEngine.Assertions;

// Stores the impulse to be applied by the character controller body
public struct DeferredCharacterControllerImpulse
{
    public Entity Entity;
    public float3 Impulse;
    public float3 Point;
}

public static class CharacterControllerUtilities
{
    static readonly sfloat k_SimplexSolverEpsilon = (sfloat)0.0001f;
    static readonly sfloat k_SimplexSolverEpsilonSq = k_SimplexSolverEpsilon * k_SimplexSolverEpsilon;

    const int k_DefaultQueryHitsCapacity = 8;
    const int k_DefaultConstraintsCapacity = 2 * k_DefaultQueryHitsCapacity;

    public enum CharacterSupportState : byte
    {
        Unsupported = 0,
        Sliding,
        Supported
    }

    public struct CharacterControllerStepInput
    {
        public PhysicsWorld World;
        public sfloat DeltaTime;
        public float3 Gravity;
        public float3 Up;
        public int MaxIterations;
        public sfloat Tau;
        public sfloat Damping;
        public sfloat SkinWidth;
        public sfloat ContactTolerance;
        public sfloat MaxSlope;
        public int RigidBodyIndex;
        public float3 CurrentVelocity;
        public sfloat MaxMovementSpeed;
    }

    public struct CharacterControllerAllHitsCollector<T> : ICollector<T> where T : unmanaged, IQueryResult
    {
        private int m_selfRBIndex;

        public bool EarlyOutOnFirstHit => false;
        public sfloat MaxFraction { get; }
        public int NumHits => AllHits.Length;

        public sfloat MinHitFraction;
        public NativeList<T> AllHits;
        public NativeList<T> TriggerHits;

        private PhysicsWorld m_world;

        public CharacterControllerAllHitsCollector(int rbIndex, sfloat maxFraction, ref NativeList<T> allHits, PhysicsWorld world,
                                                   NativeList<T> triggerHits = default)
        {
            MaxFraction = maxFraction;
            AllHits = allHits;
            m_selfRBIndex = rbIndex;
            m_world = world;
            TriggerHits = triggerHits;
            MinHitFraction = sfloat.MaxValue;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);

            if (hit.RigidBodyIndex == m_selfRBIndex)
            {
                return false;
            }

            if (hit.Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents)
            {
                if (TriggerHits.IsCreated)
                {
                    TriggerHits.Add(hit);
                }
                return false;
            }

            MinHitFraction = math.min(MinHitFraction, hit.Fraction);
            AllHits.Add(hit);
            return true;
        }

        #endregion
    }

    // A collector which stores only the closest hit different from itself, the triggers, and predefined list of values it hit.
    public struct CharacterControllerClosestHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public sfloat MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        private T m_ClosestHit;
        public T ClosestHit => m_ClosestHit;

        private int m_selfRBIndex;
        private PhysicsWorld m_world;

        private NativeList<SurfaceConstraintInfo> m_PredefinedConstraints;

        public CharacterControllerClosestHitCollector(NativeList<SurfaceConstraintInfo> predefinedConstraints, PhysicsWorld world, int rbIndex, sfloat maxFraction)
        {
            MaxFraction = maxFraction;
            m_ClosestHit = default;
            NumHits = 0;
            m_selfRBIndex = rbIndex;
            m_world = world;
            m_PredefinedConstraints = predefinedConstraints;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);

            // Check self hits and trigger hits
            if ((hit.RigidBodyIndex == m_selfRBIndex) || (hit.Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents))
            {
                return false;
            }

            // Check predefined hits
            for (int i = 0; i < m_PredefinedConstraints.Length; i++)
            {
                SurfaceConstraintInfo constraint = m_PredefinedConstraints[i];
                if (constraint.RigidBodyIndex == hit.RigidBodyIndex &&
                    constraint.ColliderKey.Equals(hit.ColliderKey))
                {
                    // Hit was already defined, skip it
                    return false;
                }
            }

            // Finally, accept the hit
            MaxFraction = hit.Fraction;
            m_ClosestHit = hit;
            NumHits = 1;
            return true;
        }

        #endregion
    }

    public static unsafe void CheckSupport(
        ref PhysicsWorld world, ref PhysicsCollider collider, CharacterControllerStepInput stepInput, RigidTransform transform,
        out CharacterSupportState characterState, out float3 surfaceNormal, out float3 surfaceVelocity,
        NativeList<StatefulCollisionEvent> collisionEvents = default)
    {
        surfaceNormal = float3.zero;
        surfaceVelocity = float3.zero;

        // Up direction must be normalized
        Assert.IsTrue(Fixed.Physics.Math.IsNormalized(stepInput.Up));

        // Query the world
        NativeList<ColliderCastHit> castHits = new NativeList<ColliderCastHit>(k_DefaultQueryHitsCapacity, Allocator.Temp);
        CharacterControllerAllHitsCollector<ColliderCastHit> castHitsCollector = new CharacterControllerAllHitsCollector<ColliderCastHit>(
            stepInput.RigidBodyIndex, sfloat.One, ref castHits, world);
        var maxDisplacement = -stepInput.ContactTolerance * stepInput.Up;
        {
            ColliderCastInput input = new ColliderCastInput()
            {
                Collider = collider.ColliderPtr,
                Orientation = transform.rot,
                Start = transform.pos,
                End = transform.pos + maxDisplacement
            };

            world.CastCollider(input, ref castHitsCollector);
        }

        // If no hits, proclaim unsupported state
        if (castHitsCollector.NumHits == 0)
        {
            characterState = CharacterSupportState.Unsupported;
            return;
        }

        sfloat maxSlopeCos = math.cos(stepInput.MaxSlope);

        // Iterate over distance hits and create constraints from them
        NativeList<SurfaceConstraintInfo> constraints = new NativeList<SurfaceConstraintInfo>(k_DefaultConstraintsCapacity, Allocator.Temp);
        sfloat maxDisplacementLength = math.length(maxDisplacement);
        for (int i = 0; i < castHitsCollector.NumHits; i++)
        {
            ColliderCastHit hit = castHitsCollector.AllHits[i];
            CreateConstraint(stepInput.World, stepInput.Up,
                hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, hit.Fraction * maxDisplacementLength,
                stepInput.SkinWidth, maxSlopeCos, ref constraints);
        }

        // Velocity for support checking
        float3 initialVelocity = maxDisplacement / stepInput.DeltaTime;
        Math.ClampToMaxLength(stepInput.MaxMovementSpeed, ref initialVelocity);

        // Solve downwards (don't use min delta time, try to solve full step)
        float3 outVelocity = initialVelocity;
        float3 outPosition = transform.pos;
        SimplexSolver.Solve(stepInput.DeltaTime, stepInput.DeltaTime, stepInput.Up, stepInput.MaxMovementSpeed,
            constraints, ref outPosition, ref outVelocity, out sfloat integratedTime, false);

        // Get info on surface
        int numSupportingPlanes = 0;
        {
            for (int j = 0; j < constraints.Length; j++)
            {
                var constraint = constraints[j];
                if (constraint.Touched && !constraint.IsTooSteep && !constraint.IsMaxSlope)
                {
                    numSupportingPlanes++;
                    surfaceNormal += constraint.Plane.Normal;
                    surfaceVelocity += constraint.Velocity;

                    // Add supporting planes to collision events
                    if (collisionEvents.IsCreated)
                    {
                        var collisionEvent = new StatefulCollisionEvent()
                        {
                            EntityA = stepInput.World.Bodies[stepInput.RigidBodyIndex].Entity,
                            EntityB = stepInput.World.Bodies[constraint.RigidBodyIndex].Entity,
                            BodyIndexA = stepInput.RigidBodyIndex,
                            BodyIndexB = constraint.RigidBodyIndex,
                            ColliderKeyA = ColliderKey.Empty,
                            ColliderKeyB = constraint.ColliderKey,
                            Normal = constraint.Plane.Normal
                        };
                        collisionEvent.CollisionDetails = new StatefulCollisionEvent.Details(1, sfloat.Zero, constraint.HitPosition);
                        collisionEvents.Add(collisionEvent);
                    }
                }
            }

            if (numSupportingPlanes > 0)
            {
                sfloat invNumSupportingPlanes = sfloat.One / (sfloat)numSupportingPlanes;
                surfaceNormal *= invNumSupportingPlanes;
                surfaceVelocity *= invNumSupportingPlanes;

                surfaceNormal = math.normalize(surfaceNormal);
            }
        }

        // Check support state
        {
            if (math.lengthsq(initialVelocity - outVelocity) < k_SimplexSolverEpsilonSq)
            {
                // If velocity hasn't changed significantly, declare unsupported state
                characterState = CharacterSupportState.Unsupported;
            }
            else if (math.lengthsq(outVelocity) < k_SimplexSolverEpsilonSq && numSupportingPlanes > 0)
            {
                // If velocity is very small, declare supported state
                characterState = CharacterSupportState.Supported;
            }
            else
            {
                // Check if sliding
                outVelocity = math.normalize(outVelocity);
                sfloat slopeAngleSin = math.max(sfloat.Zero, math.dot(outVelocity, -stepInput.Up));
                sfloat slopeAngleCosSq = sfloat.One - slopeAngleSin * slopeAngleSin;
                if (slopeAngleCosSq <= maxSlopeCos * maxSlopeCos)
                {
                    characterState = CharacterSupportState.Sliding;
                }
                else if (numSupportingPlanes > 0)
                {
                    characterState = CharacterSupportState.Supported;
                }
                else
                {
                    // If numSupportingPlanes is 0, surface normal is invalid, so state is unsupported
                    characterState = CharacterSupportState.Unsupported;
                }
            }
        }
    }

    public static unsafe void CollideAndIntegrate(
        CharacterControllerStepInput stepInput, sfloat characterMass, bool affectBodies, Fixed.Physics.Collider* collider,
        ref RigidTransform transform, ref float3 linearVelocity, ref NativeStream.Writer deferredImpulseWriter,
        NativeList<StatefulCollisionEvent> collisionEvents = default, NativeList<StatefulTriggerEvent> triggerEvents = default)
    {
        // Copy parameters
        sfloat deltaTime = stepInput.DeltaTime;
        float3 up = stepInput.Up;
        PhysicsWorld world = stepInput.World;

        sfloat remainingTime = deltaTime;

        float3 newPosition = transform.pos;
        quaternion orientation = transform.rot;
        float3 newVelocity = linearVelocity;

        sfloat maxSlopeCos = math.cos(stepInput.MaxSlope);

        sfloat timeEpsilon = (sfloat)0.000001f;
        for (int i = 0; i < stepInput.MaxIterations && remainingTime > timeEpsilon; i++)
        {
            NativeList<SurfaceConstraintInfo> constraints = new NativeList<SurfaceConstraintInfo>(k_DefaultConstraintsCapacity, Allocator.Temp);

            // Do a collider cast
            {
                float3 displacement = newVelocity * remainingTime;
                NativeList<ColliderCastHit> triggerHits = default;
                if (triggerEvents.IsCreated)
                {
                    triggerHits = new NativeList<ColliderCastHit>(k_DefaultQueryHitsCapacity / 4, Allocator.Temp);
                }
                NativeList<ColliderCastHit> castHits = new NativeList<ColliderCastHit>(k_DefaultQueryHitsCapacity, Allocator.Temp);
                CharacterControllerAllHitsCollector<ColliderCastHit> collector = new CharacterControllerAllHitsCollector<ColliderCastHit>(
                    stepInput.RigidBodyIndex, sfloat.One, ref castHits, world, triggerHits);
                ColliderCastInput input = new ColliderCastInput()
                {
                    Collider = collider,
                    Orientation = orientation,
                    Start = newPosition,
                    End = newPosition + displacement
                };
                world.CastCollider(input, ref collector);

                // Iterate over hits and create constraints from them
                for (int hitIndex = 0; hitIndex < collector.NumHits; hitIndex++)
                {
                    ColliderCastHit hit = collector.AllHits[hitIndex];
                    CreateConstraint(stepInput.World, stepInput.Up,
                        hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, math.dot(-hit.SurfaceNormal, hit.Fraction * displacement),
                        stepInput.SkinWidth, maxSlopeCos, ref constraints);
                }

                // Update trigger events
                if (triggerEvents.IsCreated)
                {
                    UpdateTriggersSeen(stepInput, triggerHits, triggerEvents, collector.MinHitFraction);
                }
            }

            // Then do a collider distance for penetration recovery,
            // but only fix up penetrating hits
            {
                // Collider distance query
                NativeList<DistanceHit> distanceHits = new NativeList<DistanceHit>(k_DefaultQueryHitsCapacity, Allocator.Temp);
                CharacterControllerAllHitsCollector<DistanceHit> distanceHitsCollector = new CharacterControllerAllHitsCollector<DistanceHit>(
                    stepInput.RigidBodyIndex, stepInput.ContactTolerance, ref distanceHits, world);
                {
                    ColliderDistanceInput input = new ColliderDistanceInput()
                    {
                        MaxDistance = stepInput.ContactTolerance,
                        Transform = transform,
                        Collider = collider
                    };
                    world.CalculateDistance(input, ref distanceHitsCollector);
                }

                // Iterate over penetrating hits and fix up distance and normal
                int numConstraints = constraints.Length;
                for (int hitIndex = 0; hitIndex < distanceHitsCollector.NumHits; hitIndex++)
                {
                    DistanceHit hit = distanceHitsCollector.AllHits[hitIndex];
                    if (hit.Distance < stepInput.SkinWidth)
                    {
                        bool found = false;

                        // Iterate backwards to locate the original constraint before the max slope constraint
                        for (int constraintIndex = numConstraints - 1; constraintIndex >= 0; constraintIndex--)
                        {
                            SurfaceConstraintInfo constraint = constraints[constraintIndex];
                            if (constraint.RigidBodyIndex == hit.RigidBodyIndex &&
                                constraint.ColliderKey.Equals(hit.ColliderKey))
                            {
                                // Fix up the constraint (normal, distance)
                                {
                                    // Create new constraint
                                    CreateConstraintFromHit(world, hit.RigidBodyIndex, hit.ColliderKey,
                                        hit.Position, hit.SurfaceNormal, hit.Distance,
                                        stepInput.SkinWidth, out SurfaceConstraintInfo newConstraint);

                                    // Resolve its penetration
                                    ResolveConstraintPenetration(ref newConstraint);

                                    // Write back
                                    constraints[constraintIndex] = newConstraint;
                                }

                                found = true;
                                break;
                            }
                        }

                        // Add penetrating hit not caught by collider cast
                        if (!found)
                        {
                            CreateConstraint(stepInput.World, stepInput.Up,
                                hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, hit.Distance,
                                stepInput.SkinWidth, maxSlopeCos, ref constraints);
                        }
                    }
                }
            }

            // Min delta time for solver to break
            sfloat minDeltaTime = sfloat.Zero;
            if (math.lengthsq(newVelocity) > k_SimplexSolverEpsilonSq)
            {
                // Min delta time to travel at least 1cm
                minDeltaTime = (sfloat)0.01f / math.length(newVelocity);
            }

            // Solve
            float3 prevVelocity = newVelocity;
            float3 prevPosition = newPosition;
            SimplexSolver.Solve(remainingTime, minDeltaTime, up, stepInput.MaxMovementSpeed, constraints, ref newPosition, ref newVelocity, out sfloat integratedTime);

            // Apply impulses to hit bodies and store collision events
            if (affectBodies || collisionEvents.IsCreated)
            {
                CalculateAndStoreDeferredImpulsesAndCollisionEvents(stepInput, affectBodies, characterMass,
                    prevVelocity, constraints, ref deferredImpulseWriter, collisionEvents);
            }

            // Calculate new displacement
            float3 newDisplacement = newPosition - prevPosition;

            // If simplex solver moved the character we need to re-cast to make sure it can move to new position
            if (math.lengthsq(newDisplacement) > k_SimplexSolverEpsilon)
            {
                // Check if we can walk to the position simplex solver has suggested
                var newCollector = new CharacterControllerClosestHitCollector<ColliderCastHit>(constraints, world, stepInput.RigidBodyIndex, sfloat.One);

                ColliderCastInput input = new ColliderCastInput()
                {
                    Collider = collider,
                    Orientation = orientation,
                    Start = prevPosition,
                    End = prevPosition + newDisplacement
                };

                world.CastCollider(input, ref newCollector);

                if (newCollector.NumHits > 0)
                {
                    ColliderCastHit hit = newCollector.ClosestHit;

                    // Move character along the newDisplacement direction until it reaches this new contact
                    {
                        Assert.IsTrue(hit.Fraction >= sfloat.Zero && hit.Fraction <= sfloat.One);

                        integratedTime *= hit.Fraction;
                        newPosition = prevPosition + newDisplacement * hit.Fraction;
                    }
                }
            }

            // Reduce remaining time
            remainingTime -= integratedTime;

            // Write back position so that the distance query will update results
            transform.pos = newPosition;
        }

        // Write back final velocity
        linearVelocity = newVelocity;
    }

    private static void CreateConstraintFromHit(PhysicsWorld world, int rigidBodyIndex, ColliderKey colliderKey,
        float3 hitPosition, float3 normal, sfloat distance, sfloat skinWidth, out SurfaceConstraintInfo constraint)
    {
        bool bodyIsDynamic = 0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies;
        constraint = new SurfaceConstraintInfo()
        {
            Plane = new Fixed.Physics.Plane
            {
                Normal = normal,
                Distance = distance - skinWidth,
            },
            RigidBodyIndex = rigidBodyIndex,
            ColliderKey = colliderKey,
            HitPosition = hitPosition,
            Velocity = bodyIsDynamic ?
                world.GetLinearVelocity(rigidBodyIndex, hitPosition) :
                float3.zero,
            Priority = bodyIsDynamic ? 1 : 0
        };
    }

    private static void CreateMaxSlopeConstraint(float3 up, ref SurfaceConstraintInfo constraint, out SurfaceConstraintInfo maxSlopeConstraint)
    {
        sfloat verticalComponent = math.dot(constraint.Plane.Normal, up);

        SurfaceConstraintInfo newConstraint = constraint;
        newConstraint.Plane.Normal = math.normalize(newConstraint.Plane.Normal - verticalComponent * up);
        newConstraint.IsMaxSlope = true;

        sfloat distance = newConstraint.Plane.Distance;

        // Calculate distance to the original plane along the new normal.
        // Clamp the new distance to 2x the old distance to avoid penetration recovery explosions.
        newConstraint.Plane.Distance = distance / math.max(math.dot(newConstraint.Plane.Normal, constraint.Plane.Normal), (sfloat)0.5f);

        if (newConstraint.Plane.Distance < sfloat.Zero)
        {
            // Disable penetration recovery for the original plane
            constraint.Plane.Distance = sfloat.Zero;

            // Prepare velocity to resolve penetration
            ResolveConstraintPenetration(ref newConstraint);
        }

        // Output max slope constraint
        maxSlopeConstraint = newConstraint;
    }

    private static void ResolveConstraintPenetration(ref SurfaceConstraintInfo constraint)
    {
        // Fix up the velocity to enable penetration recovery
        if (constraint.Plane.Distance < sfloat.Zero)
        {
            float3 newVel = constraint.Velocity - constraint.Plane.Normal * constraint.Plane.Distance;
            constraint.Velocity = newVel;
            constraint.Plane.Distance = sfloat.Zero;
        }
    }

    private static void CreateConstraint(PhysicsWorld world, float3 up,
        int hitRigidBodyIndex, ColliderKey hitColliderKey, float3 hitPosition, float3 hitSurfaceNormal, sfloat hitDistance,
        sfloat skinWidth, sfloat maxSlopeCos, ref NativeList<SurfaceConstraintInfo> constraints)
    {
        CreateConstraintFromHit(world, hitRigidBodyIndex, hitColliderKey, hitPosition,
            hitSurfaceNormal, hitDistance, skinWidth, out SurfaceConstraintInfo constraint);

        // Check if max slope plane is required
        sfloat verticalComponent = math.dot(constraint.Plane.Normal, up);
        bool shouldAddPlane = verticalComponent > k_SimplexSolverEpsilon && verticalComponent < maxSlopeCos;
        if (shouldAddPlane)
        {
            constraint.IsTooSteep = true;
            CreateMaxSlopeConstraint(up, ref constraint, out SurfaceConstraintInfo maxSlopeConstraint);
            constraints.Add(maxSlopeConstraint);
        }

        // Prepare velocity to resolve penetration
        ResolveConstraintPenetration(ref constraint);

        // Add original constraint to the list
        constraints.Add(constraint);
    }

    private static unsafe void CalculateAndStoreDeferredImpulsesAndCollisionEvents(
        CharacterControllerStepInput stepInput, bool affectBodies, sfloat characterMass,
        float3 linearVelocity, NativeList<SurfaceConstraintInfo> constraints, ref NativeStream.Writer deferredImpulseWriter,
        NativeList<StatefulCollisionEvent> collisionEvents)
    {
        PhysicsWorld world = stepInput.World;
        for (int i = 0; i < constraints.Length; i++)
        {
            SurfaceConstraintInfo constraint = constraints[i];
            int rigidBodyIndex = constraint.RigidBodyIndex;

            float3 impulse = float3.zero;

            if (rigidBodyIndex < 0)
            {
                continue;
            }

            // Skip static bodies if needed to calculate impulse
            if (affectBodies && (rigidBodyIndex < world.NumDynamicBodies))
            {
                RigidBody body = world.Bodies[rigidBodyIndex];

                float3 pointRelVel = world.GetLinearVelocity(rigidBodyIndex, constraint.HitPosition);
                pointRelVel -= linearVelocity;

                sfloat projectedVelocity = math.dot(pointRelVel, constraint.Plane.Normal);

                // Required velocity change
                sfloat deltaVelocity = -projectedVelocity * stepInput.Damping;

                sfloat distance = constraint.Plane.Distance;
                if (distance < sfloat.Zero)
                {
                    deltaVelocity += (distance / stepInput.DeltaTime) * stepInput.Tau;
                }

                // Calculate impulse
                MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
                if (deltaVelocity < sfloat.Zero)
                {
                    // Impulse magnitude
                    sfloat impulseMagnitude = sfloat.Zero;
                    {
                        sfloat objectMassInv = GetInvMassAtPoint(constraint.HitPosition, constraint.Plane.Normal, body, mv);
                        impulseMagnitude = deltaVelocity / objectMassInv;
                    }

                    impulse = impulseMagnitude * constraint.Plane.Normal;
                }

                // Add gravity
                {
                    // Effect of gravity on character velocity in the normal direction
                    float3 charVelDown = stepInput.Gravity * stepInput.DeltaTime;
                    sfloat relVelN = math.dot(charVelDown, constraint.Plane.Normal);

                    // Subtract separation velocity if separating contact
                    {
                        bool isSeparatingContact = projectedVelocity < sfloat.Zero;
                        sfloat newRelVelN = relVelN - projectedVelocity;
                        relVelN = math.select(relVelN, newRelVelN, isSeparatingContact);
                    }

                    // If resulting velocity is negative, an impulse is applied to stop the character
                    // from falling into the body
                    {
                        float3 newImpulse = impulse;
                        newImpulse += relVelN * characterMass * constraint.Plane.Normal;
                        impulse = math.select(impulse, newImpulse, relVelN < sfloat.Zero);
                    }
                }

                // Store impulse
                deferredImpulseWriter.Write(
                    new DeferredCharacterControllerImpulse()
                    {
                        Entity = body.Entity,
                        Impulse = impulse,
                        Point = constraint.HitPosition
                    });
            }

            if (collisionEvents.IsCreated && constraint.Touched && !constraint.IsMaxSlope)
            {
                var collisionEvent = new StatefulCollisionEvent()
                {
                    EntityA = world.Bodies[stepInput.RigidBodyIndex].Entity,
                    EntityB = world.Bodies[rigidBodyIndex].Entity,
                    BodyIndexA = stepInput.RigidBodyIndex,
                    BodyIndexB = rigidBodyIndex,
                    ColliderKeyA = ColliderKey.Empty,
                    ColliderKeyB = constraint.ColliderKey,
                    Normal = constraint.Plane.Normal
                };
                collisionEvent.CollisionDetails = new StatefulCollisionEvent.Details(
                    1, math.dot(impulse, collisionEvent.Normal), constraint.HitPosition);

                // check if collision event exists for the same bodyID and colliderKey
                // although this is a nested for, number of solved constraints shouldn't be high
                // if the same constraint (same entities, rigidbody indices and collider keys)
                // is solved in multiple solver iterations, pick the one from latest iteration
                bool newEvent = true;
                for (int j = 0; j < collisionEvents.Length; j++)
                {
                    if (collisionEvents[j].CompareTo(collisionEvent) == 0)
                    {
                        collisionEvents[j] = collisionEvent;
                        newEvent = false;
                        break;
                    }
                }
                if (newEvent)
                {
                    collisionEvents.Add(collisionEvent);
                }
            }
        }
    }

    private static void UpdateTriggersSeen<T>(CharacterControllerStepInput stepInput, NativeList<T> triggerHits,
        NativeList<StatefulTriggerEvent> currentFrameTriggerEvents, sfloat maxFraction) where T : unmanaged, IQueryResult
    {
        var world = stepInput.World;
        for (int i = 0; i < triggerHits.Length; i++)
        {
            var hit = triggerHits[i];

            if (hit.Fraction > maxFraction)
            {
                continue;
            }

            var found = false;
            for (int j = 0; j < currentFrameTriggerEvents.Length; j++)
            {
                var triggerEvent = currentFrameTriggerEvents[j];
                if ((triggerEvent.EntityB == hit.Entity) &&
                    (triggerEvent.ColliderKeyB.Value == hit.ColliderKey.Value))
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                currentFrameTriggerEvents.Add(new StatefulTriggerEvent()
                {
                    EntityA = world.Bodies[stepInput.RigidBodyIndex].Entity,
                    EntityB = hit.Entity,
                    BodyIndexA = stepInput.RigidBodyIndex,
                    BodyIndexB = hit.RigidBodyIndex,
                    ColliderKeyA = ColliderKey.Empty,
                    ColliderKeyB = hit.ColliderKey
                });
            }
        }
    }

    static sfloat GetInvMassAtPoint(float3 point, float3 normal, RigidBody body, MotionVelocity mv)
    {
        var massCenter =
            math.transform(body.WorldFromBody, body.Collider.Value.MassProperties.MassDistribution.Transform.pos);
        float3 arm = point - massCenter;
        float3 jacAng = math.cross(arm, normal);
        float3 armC = jacAng * mv.InverseInertia;

        sfloat objectMassInv = math.dot(armC, jacAng);
        objectMassInv += mv.InverseMass;

        return objectMassInv;
    }
}
