using System;
using System.Runtime.CompilerServices;
using Unity.Assertions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // An instance of a collider in a physics world.
    public struct RigidBody : ICollidable
    {
        // The rigid body's collider (allowed to be null)
        public BlobAssetReference<Collider> Collider;

        // The rigid body's transform in world space
        public FpRigidTransform WorldFromBody;

        // The entity that rigid body represents
        public Entity Entity;

        // Arbitrary custom tags.
        // These get copied into contact manifolds and can be used to inform contact modifiers.
        public byte CustomTags;

        public static readonly RigidBody Zero = new RigidBody
        {
            WorldFromBody = FpRigidTransform.identity,
            Collider = default,
            Entity = Entity.Null,
            CustomTags = 0
        };

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(WorldFromBody);
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public Aabb CalculateAabb(FpRigidTransform transform)
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(fpmath.mul(transform, WorldFromBody));
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            // Transform the ray into body space
            var worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
            input.Ray.Displacement = fpmath.mul(bodyFromWorld.Rotation, input.Ray.Displacement);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CastRay(input, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Orientation = fpmath.mul(fpmath.inverse(WorldFromBody.rot), input.Orientation);
            input.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
            input.Ray.Displacement = fpmath.mul(bodyFromWorld.Rotation, input.Ray.Displacement);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CastCollider(input, ref collector);
        }

        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Position = Mul(bodyFromWorld, input.Position);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Transform = new FpRigidTransform(
                fpmath.mul(fpmath.inverse(WorldFromBody.rot), input.Transform.rot),
                Mul(bodyFromWorld, input.Transform.pos));

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        #region GO API Queries

        // Interfaces that represent queries that exist in the GameObjects world.

        public bool CheckSphere(fp3 position, fp radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckSphere(ref this, position, radius, filter, queryInteraction);
        public bool OverlapSphere(fp3 position, fp radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapSphere(ref this, position, radius, ref outHits, filter, queryInteraction);
        public bool OverlapSphereCustom<T>(fp3 position, fp radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapSphereCustom(ref this, position, radius, ref collector, filter, queryInteraction);

        public bool CheckCapsule(fp3 point1, fp3 point2, fp radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckCapsule(ref this, point1, point2, radius, filter, queryInteraction);
        public bool OverlapCapsule(fp3 point1, fp3 point2, fp radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapCapsule(ref this, point1, point2, radius, ref outHits, filter, queryInteraction);
        public bool OverlapCapsuleCustom<T>(fp3 point1, fp3 point2, fp radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapCapsuleCustom(ref this, point1, point2, radius, ref collector, filter, queryInteraction);

        public bool CheckBox(fp3 center, fpquaternion orientation, fp3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckBox(ref this, center, orientation, halfExtents, filter, queryInteraction);
        public bool OverlapBox(fp3 center, fpquaternion orientation, fp3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapBox(ref this, center, orientation, halfExtents, ref outHits, filter, queryInteraction);
        public bool OverlapBoxCustom<T>(fp3 center, fpquaternion orientation, fp3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapBoxCustom(ref this, center, orientation, halfExtents, ref collector, filter, queryInteraction);

        public bool SphereCast(fp3 origin, fp radius, fp3 direction, fp maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, filter, queryInteraction);
        public bool SphereCast(fp3 origin, fp radius, fp3 direction, fp maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool SphereCastAll(fp3 origin, fp radius, fp3 direction, fp maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCastAll(ref this, origin, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool SphereCastCustom<T>(fp3 origin, fp radius, fp3 direction, fp maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.SphereCastCustom(ref this, origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool BoxCast(fp3 center, fpquaternion orientation, fp3 halfExtents, fp3 direction, fp maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, filter, queryInteraction);
        public bool BoxCast(fp3 center, fpquaternion orientation, fp3 halfExtents, fp3 direction, fp maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool BoxCastAll(fp3 center, fpquaternion orientation, fp3 halfExtents, fp3 direction, fp maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCastAll(ref this, center, orientation, halfExtents, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool BoxCastCustom<T>(fp3 center, fpquaternion orientation, fp3 halfExtents, fp3 direction, fp maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.BoxCastCustom(ref this, center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool CapsuleCast(fp3 point1, fp3 point2, fp radius, fp3 direction, fp maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, filter, queryInteraction);
        public bool CapsuleCast(fp3 point1, fp3 point2, fp radius, fp3 direction, fp maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool CapsuleCastAll(fp3 point1, fp3 point2, fp radius, fp3 direction, fp maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCastAll(ref this, point1, point2, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool CapsuleCastCustom<T>(fp3 point1, fp3 point2, fp radius, fp3 direction, fp maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.CapsuleCastCustom(ref this, point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        #endregion

        #endregion

        #region private

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SetQueryContextParameters(ref QueryContext context, ref MTransform worldFromBody)
        {
            // QueryContext.WorldFromLocalTransform is not expected to be initialized at this point
            // and should have default value (zeros in all fields)
            Assert.IsTrue(context.WorldFromLocalTransform.Translation.Equals(fp3.zero));
            Assert.IsTrue(context.WorldFromLocalTransform.Rotation.Equals(fp3x3.zero));

            context.ColliderKey = ColliderKey.Empty;
            context.Entity = Entity;
            context.WorldFromLocalTransform = worldFromBody;
            if (!context.IsInitialized)
            {
                context.RigidBodyIndex = -1;
                context.IsInitialized = true;
            }
        }

        #endregion
    }

    // A pair of rigid body indices
    public struct BodyIndexPair
    {
        // B before A to match Havok
        public int BodyIndexB;
        public int BodyIndexA;

        public bool IsValid => BodyIndexB != -1 && BodyIndexA != -1;

        public static BodyIndexPair Invalid => new BodyIndexPair { BodyIndexB = -1, BodyIndexA = -1 };
    }

    // A pair of entities
    public struct EntityPair
    {
        // B before A for consistency with other pairs
        public Entity EntityB;
        public Entity EntityA;
    }

    // A pair of custom rigid body tags
    public struct CustomTagsPair
    {
        // B before A for consistency with other pairs
        public byte CustomTagsB;
        public byte CustomTagsA;
    }
}
