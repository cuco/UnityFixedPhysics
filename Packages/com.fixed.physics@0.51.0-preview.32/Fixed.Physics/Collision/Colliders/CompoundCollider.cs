using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // A collider containing instances of other colliders
    public struct CompoundCollider : ICompositeCollider
    {
        private ColliderHeader m_Header;

        // A child collider, within the same blob as the compound collider.
        // Warning: This references the collider via a relative offset, so must always be passed by reference.
        public struct Child
        {
            public FpRigidTransform CompoundFromChild;
            internal int m_ColliderOffset;
            public Entity Entity;

            public unsafe Collider* Collider
            {
                get
                {
                    fixed(int* offsetPtr = &m_ColliderOffset)
                    {
                        return (Collider*)((byte*)offsetPtr + *offsetPtr);
                    }
                }
            }
        }

        internal fp m_BoundingRadius;

        // The array of child colliders
        private BlobArray m_ChildrenBlob;

        public int NumChildren => m_ChildrenBlob.Length;
        public BlobArray.Accessor<Child> Children => new BlobArray.Accessor<Child>(ref m_ChildrenBlob);

        // The bounding volume hierarchy
        // TODO: Store node filters array too, for filtering queries within the BVH
        private BlobArray m_BvhNodesBlob;

        internal BlobArray.Accessor<BoundingVolumeHierarchy.Node> BvhNodes => new BlobArray.Accessor<BoundingVolumeHierarchy.Node>(ref m_BvhNodesBlob);
        internal unsafe BoundingVolumeHierarchy BoundingVolumeHierarchy
        {
            get
            {
                fixed(BlobArray* blob = &m_BvhNodesBlob)
                {
                    var firstNode = (BoundingVolumeHierarchy.Node*)((byte*)&(blob->Offset) + blob->Offset);
                    return new BoundingVolumeHierarchy(firstNode, nodeFilters: null);
                }
            }
        }

        #region Construction

        // Input to Create()
        public struct ColliderBlobInstance  // TODO: better name?
        {
            public FpRigidTransform CompoundFromChild;
            public BlobAssetReference<Collider> Collider;
            public Entity Entity;
        }

        // Create a compound collider containing an array of other colliders.
        // The source colliders are copied into the compound, so that it becomes one blob.
        public static unsafe BlobAssetReference<Collider> Create(NativeArray<ColliderBlobInstance> children)
        {
            SafetyChecks.CheckNotEmptyAndThrow(children, nameof(children));

            // Get the total required memory size for the compound plus all its children,
            // and the combined filter of all children
            // TODO: Verify that the size is enough
            int totalSize = Math.NextMultipleOf16(UnsafeUtility.SizeOf<CompoundCollider>());
            CollisionFilter filter = children[0].Collider.Value.Filter;
            var srcToDestInstanceAddrs = new NativeParallelHashMap<long, long>(children.Length, Allocator.Temp);
            for (var childIndex = 0; childIndex < children.Length; childIndex++)
            {
                var child = children[childIndex];
                var instanceKey = (long)child.Collider.GetUnsafePtr();
                if (srcToDestInstanceAddrs.ContainsKey(instanceKey))
                    continue;
                totalSize += Math.NextMultipleOf16(child.Collider.Value.MemorySize);
                filter = CollisionFilter.CreateUnion(filter, child.Collider.Value.Filter);
                srcToDestInstanceAddrs.Add(instanceKey, 0L);
            }
            totalSize += (children.Length + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches) * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();

            // Allocate the collider
            var compoundCollider = (CompoundCollider*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);
            UnsafeUtility.MemClear(compoundCollider, totalSize);
            compoundCollider->m_Header.Type = ColliderType.Compound;
            compoundCollider->m_Header.CollisionType = CollisionType.Composite;
            compoundCollider->m_Header.Version = 1;
            compoundCollider->m_Header.Magic = 0xff;
            compoundCollider->m_Header.Filter = filter;

            // Initialize children array
            Child* childrenPtr = (Child*)((byte*)compoundCollider + UnsafeUtility.SizeOf<CompoundCollider>());
            compoundCollider->m_ChildrenBlob.Offset = (int)((byte*)childrenPtr - (byte*)(&compoundCollider->m_ChildrenBlob.Offset));
            compoundCollider->m_ChildrenBlob.Length = children.Length;
            byte* end = (byte*)childrenPtr + UnsafeUtility.SizeOf<Child>() * children.Length;
            end = (byte*)Math.NextMultipleOf16((ulong)end);

            uint maxTotalNumColliderKeyBits = 0;

            // Copy children
            for (int i = 0; i < children.Length; i++)
            {
                Collider* collider = (Collider*)children[i].Collider.GetUnsafePtr();
                var srcInstanceKey = (long)collider;
                var dstAddr = srcToDestInstanceAddrs[srcInstanceKey];
                if (dstAddr == 0L)
                {
                    dstAddr = (long)end;
                    srcToDestInstanceAddrs[srcInstanceKey] = dstAddr;
                    UnsafeUtility.MemCpy(end, collider, collider->MemorySize);
                    end += Math.NextMultipleOf16(collider->MemorySize);
                }
                childrenPtr[i].m_ColliderOffset = (int)((byte*)dstAddr - (byte*)(&childrenPtr[i].m_ColliderOffset));
                childrenPtr[i].CompoundFromChild = children[i].CompoundFromChild;
                childrenPtr[i].Entity = children[i].Entity;

                maxTotalNumColliderKeyBits = math.max(maxTotalNumColliderKeyBits, collider->TotalNumColliderKeyBits);
            }

            // Build mass properties
            compoundCollider->MassProperties = compoundCollider->BuildMassProperties();

            // Build bounding volume
            int numNodes = compoundCollider->BuildBoundingVolume(out NativeArray<BoundingVolumeHierarchy.Node> nodes);
            int bvhSize = numNodes * UnsafeUtility.SizeOf<BoundingVolumeHierarchy.Node>();
            compoundCollider->m_BvhNodesBlob.Offset = (int)(end - (byte*)(&compoundCollider->m_BvhNodesBlob.Offset));
            compoundCollider->m_BvhNodesBlob.Length = numNodes;
            UnsafeUtility.MemCpy(end, nodes.GetUnsafeReadOnlyPtr(), bvhSize);
            compoundCollider->UpdateCachedBoundingRadius();
            end += bvhSize;

            // Validate nesting level of composite colliders.
            compoundCollider->TotalNumColliderKeyBits = maxTotalNumColliderKeyBits + compoundCollider->NumColliderKeyBits;

            // If TotalNumColliderKeyBits is greater than 32, it means maximum nesting level of composite colliders has been breached.
            // ColliderKey has 32 bits so it can't handle infinite nesting of composite colliders.
            if (compoundCollider->TotalNumColliderKeyBits > 32)
            {
                SafetyChecks.ThrowArgumentException(nameof(children), "Composite collider exceeded maximum level of nesting!");
            }

            // Copy to blob asset
            int usedSize = (int)(end - (byte*)compoundCollider);
            UnityEngine.Assertions.Assert.IsTrue(usedSize < totalSize);
            compoundCollider->MemorySize = usedSize;
            var blob = BlobAssetReference<Collider>.Create(compoundCollider, usedSize);
            UnsafeUtility.Free(compoundCollider, Allocator.Temp);

            return blob;
        }

        private unsafe int BuildBoundingVolume(out NativeArray<BoundingVolumeHierarchy.Node> nodes)
        {
            // Create inputs
            var points = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var aabbs = new NativeArray<Aabb>(NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < NumChildren; ++i)
            {
                points[i] = new BoundingVolumeHierarchy.PointAndIndex { Position = Children[i].CompoundFromChild.pos, Index = i };
                aabbs[i] = Children[i].Collider->CalculateAabb(Children[i].CompoundFromChild);
            }

            // Build BVH
            // Todo: cleanup, better size of nodes array
            nodes = new NativeArray<BoundingVolumeHierarchy.Node>(2 + NumChildren, Allocator.Temp, NativeArrayOptions.UninitializedMemory)
            {
                [0] = BoundingVolumeHierarchy.Node.Empty,
                [1] = BoundingVolumeHierarchy.Node.Empty
            };

            var bvh = new BoundingVolumeHierarchy(nodes);
            bvh.Build(points, aabbs, out int numNodes);

            return numNodes;
        }

        private unsafe void UpdateCachedBoundingRadius()
        {
            m_BoundingRadius = fp.zero;
            fp3 center = BoundingVolumeHierarchy.Domain.Center;

            for (int i = 0; i < NumChildren; i++)
            {
                ref Child child = ref Children[i];

                fp3 childFromCenter = fpmath.transform(fpmath.inverse(child.CompoundFromChild), center);
                fp radius = fp.zero;

                switch (child.Collider->Type)
                {
                    case ColliderType.Sphere:
                    case ColliderType.Box:
                    case ColliderType.Capsule:
                    case ColliderType.Quad:
                    case ColliderType.Triangle:
                    case ColliderType.Cylinder:
                    case ColliderType.Convex:
                        radius = ((ConvexCollider*)child.Collider)->CalculateBoundingRadius(childFromCenter);
                        break;
                    case ColliderType.Compound:
                        radius = ((CompoundCollider*)child.Collider)->CalculateBoundingRadius(childFromCenter);
                        break;
                    case ColliderType.Mesh:
                        radius = ((MeshCollider*)child.Collider)->CalculateBoundingRadius(childFromCenter);
                        break;
                    case ColliderType.Terrain:
                        Aabb terrainAabb = ((TerrainCollider*)child.Collider)->CalculateAabb();
                        radius = fpmath.length(fpmath.max(fpmath.abs(terrainAabb.Max - childFromCenter), fpmath.abs(terrainAabb.Min - childFromCenter)));
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        break;
                }
                m_BoundingRadius = fpmath.max(m_BoundingRadius, radius);
            }
        }

        // Build mass properties representing a union of all the child collider mass properties.
        // This assumes a uniform density for all children, and returns a mass properties for a compound of unit mass.
        private unsafe MassProperties BuildMassProperties()
        {
            BlobArray.Accessor<Child> children = Children;

            // Check if all children are triggers or have collisions disabled.
            // If so, we'll include them in mass properties of the compound collider.
            // This is mostly targeted for single collider compounds, as they should behave
            // the same as when there is no compound.
            bool skipTriggersAndDisabledColliders = false;
            {
                for (int i = 0; i < NumChildren; ++i)
                {
                    ref Child child = ref children[i];
                    var convexChildCollider = (ConvexCollider*)child.Collider;
                    if (child.Collider->CollisionType == CollisionType.Convex &&
                        (convexChildCollider->Material.CollisionResponse != CollisionResponsePolicy.RaiseTriggerEvents &&
                         convexChildCollider->Material.CollisionResponse != CollisionResponsePolicy.None))
                    {
                        // If there are children with regular collisions, all triggers and disabled colliders should be skipped
                        skipTriggersAndDisabledColliders = true;
                        break;
                    }
                }
            }

            // Calculate combined center of mass
            fp3 combinedCenterOfMass = fp3.zero;
            fp combinedVolume = fp.zero;
            for (int i = 0; i < NumChildren; ++i)
            {
                ref Child child = ref children[i];
                var convexChildCollider = (ConvexCollider*)child.Collider;
                if (skipTriggersAndDisabledColliders && child.Collider->CollisionType == CollisionType.Convex &&
                    (convexChildCollider->Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents ||
                     convexChildCollider->Material.CollisionResponse == CollisionResponsePolicy.None))
                    continue;

                var mp = child.Collider->MassProperties;

                // weight this contribution by its volume (=mass)
                combinedCenterOfMass += fpmath.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) * mp.Volume;
                combinedVolume += mp.Volume;
            }
            if (combinedVolume > fp.zero)
            {
                combinedCenterOfMass /= combinedVolume;
            }

            // Calculate combined inertia, relative to new center of mass
            fp3x3 combinedOrientation;
            fp3 combinedInertiaTensor;
            {
                fp3x3 combinedInertiaMatrix = fp3x3.zero;
                for (int i = 0; i < NumChildren; ++i)
                {
                    ref Child child = ref children[i];
                    var convexChildCollider = (ConvexCollider*)child.Collider;
                    if (skipTriggersAndDisabledColliders && child.Collider->CollisionType == CollisionType.Convex &&
                        (convexChildCollider->Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents ||
                         convexChildCollider->Material.CollisionResponse == CollisionResponsePolicy.None))
                        continue;

                    var mp = child.Collider->MassProperties;

                    // rotate inertia into compound space
                    fp3x3 temp = fpmath.mul(mp.MassDistribution.InertiaMatrix, new fp3x3(fpmath.inverse(child.CompoundFromChild.rot)));
                    fp3x3 inertiaMatrix = fpmath.mul(new fp3x3(child.CompoundFromChild.rot), temp);

                    // shift it to be relative to the new center of mass
                    fp3 shift = fpmath.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) - combinedCenterOfMass;
                    fp3 shiftSq = shift * shift;
                    var diag = new fp3(shiftSq.y + shiftSq.z, shiftSq.x + shiftSq.z, shiftSq.x + shiftSq.y);
                    var offDiag = new fp3(shift.x * shift.y, shift.y * shift.z, shift.z * shift.x) * fp.minusOne;
                    inertiaMatrix.c0 += new fp3(diag.x, offDiag.x, offDiag.z);
                    inertiaMatrix.c1 += new fp3(offDiag.x, diag.y, offDiag.y);
                    inertiaMatrix.c2 += new fp3(offDiag.z, offDiag.y, diag.z);

                    // weight by its proportional volume (=mass)
                    inertiaMatrix *= mp.Volume / (combinedVolume + fp.epsilon);

                    combinedInertiaMatrix += inertiaMatrix;
                }

                // convert to box inertia
                Math.DiagonalizeSymmetricApproximation(
                    combinedInertiaMatrix, out combinedOrientation, out combinedInertiaTensor);
            }

            // Calculate combined angular expansion factor, relative to new center of mass
            fp combinedAngularExpansionFactor = fp.zero;
            for (int i = 0; i < NumChildren; ++i)
            {
                ref Child child = ref children[i];
                var mp = child.Collider->MassProperties;

                fp3 shift = fpmath.transform(child.CompoundFromChild, mp.MassDistribution.Transform.pos) - combinedCenterOfMass;
                fp expansionFactor = mp.AngularExpansionFactor + fpmath.length(shift);
                combinedAngularExpansionFactor = fpmath.max(combinedAngularExpansionFactor, expansionFactor);
            }

            return new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = new FpRigidTransform(combinedOrientation, combinedCenterOfMass),
                    InertiaTensor = combinedInertiaTensor
                },
                Volume = combinedVolume,
                AngularExpansionFactor = combinedAngularExpansionFactor
            };
        }

        #endregion

        // Refreshes combined collision filter of all children.
        // Should be called when child collision filter changes.
        public unsafe void RefreshCollisionFilter()
        {
            CollisionFilter filter = CollisionFilter.Zero;
            for (int childIndex = 0; childIndex < Children.Length; childIndex++)
            {
                ref Child c = ref Children[childIndex];

                // If a child is also a compound, refresh its collision filter first
                if (c.Collider->Type == ColliderType.Compound)
                {
                    ((CompoundCollider*)c.Collider)->RefreshCollisionFilter();
                }

                filter = CollisionFilter.CreateUnion(filter, c.Collider->Filter);
            }

            m_Header.Version += 1;
            m_Header.Filter = filter;
        }

        #region ICompositeCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        // Root level filter that can be used for a quick dismiss of the collision/query.
        // Individual children can have their own collision filters for further filtering.
        // On creation, this filter is a union of all child filters.
        // Child filter changes will update this root level filter.
        // Changing this root level filter sets all child collider filters.
        public CollisionFilter Filter
        {
            get => m_Header.Filter;
            set
            {
                m_Header.Version += 1;
                m_Header.Filter = value;

                for (int childIndex = 0; childIndex < Children.Length; childIndex++)
                {
                    ref Child c = ref Children[childIndex];
                    unsafe
                    {
                        c.Collider->Filter = value;
                    }
                }
            }
        }

        internal unsafe bool RespondsToCollision
        {
            get
            {
                for (int childIndex = 0; childIndex < Children.Length; childIndex++)
                {
                    ref Child c = ref Children[childIndex];
                    if (c.Collider->RespondsToCollision)
                    {
                        return true;
                    }
                }
                return false;
            }
        }

        public MassProperties MassProperties { get; private set; }

        internal fp CalculateBoundingRadius(fp3 pivot)
        {
            return fpmath.distance(pivot, BoundingVolumeHierarchy.Domain.Center) + m_BoundingRadius;
        }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(FpRigidTransform.identity);
        }

        public unsafe Aabb CalculateAabb(FpRigidTransform transform)
        {
            Aabb outAabb = Math.TransformAabb(transform, BoundingVolumeHierarchy.Domain);
            fp3 center = outAabb.Center;
            Aabb sphereAabb = new Aabb
            {
                Min = new fp3(center - m_BoundingRadius),
                Max = new fp3(center + m_BoundingRadius)
            };
            outAabb.Intersect(sphereAabb);

            return outAabb;
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed(CompoundCollider* target = &this)
            {
                return RaycastQueries.RayCollider(input, (Collider*)target, ref collector);
            }
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            fixed(CompoundCollider* target = &this)
            {
                return ColliderCastQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed(CompoundCollider* target = &this)
            {
                return DistanceQueries.PointCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed(CompoundCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
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

        public uint NumColliderKeyBits => (uint)(32 - math.lzcnt(NumChildren));

        internal uint TotalNumColliderKeyBits { get; private set; }

        public unsafe bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            if (key.PopSubKey(NumColliderKeyBits, out uint childIndex))
            {
                ref Child c = ref Children[(int)childIndex];
                child = new ChildCollider(c.Collider, c.CompoundFromChild, c.Entity);
                return true;
            }

            child = new ChildCollider();
            return false;
        }

        public unsafe bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            fixed(CompoundCollider* root = &this)
            {
                return Collider.GetLeafCollider((Collider*)root, FpRigidTransform.identity, key, out leaf);
            }
        }

        public unsafe void GetLeaves<T>([NoAlias] ref T collector) where T : struct, ILeafColliderCollector
        {
            for (uint i = 0; i < NumChildren; i++)
            {
                ref Child c = ref Children[(int)i];
                ColliderKey childKey = new ColliderKey(NumColliderKeyBits, i);
                if (c.Collider->CollisionType == CollisionType.Composite)
                {
                    collector.PushCompositeCollider(new ColliderKeyPath(childKey, NumColliderKeyBits), new MTransform(c.CompoundFromChild), out MTransform worldFromCompound);
                    c.Collider->GetLeaves(ref collector);
                    collector.PopCompositeCollider(NumColliderKeyBits, worldFromCompound);
                }
                else
                {
                    var child = new ChildCollider(c.Collider, c.CompoundFromChild, c.Entity);
                    collector.AddLeaf(childKey, ref child);
                }
            }
        }

        #endregion
    }
}
