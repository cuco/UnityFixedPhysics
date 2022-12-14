using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public struct BoxGeometry : IEquatable<BoxGeometry>
    {
        // The center of the box
        public fp3 Center { get => m_Center; set => m_Center = value; }
        fp3 m_Center;

        // The orientation of the box
        public fpquaternion Orientation { get => m_Orientation; set => m_Orientation = value; }
        private fpquaternion m_Orientation;

        // The length of each side of the box
        public fp3 Size { get => m_Size; set => m_Size = value; }
        private fp3 m_Size;

        // The radius by which to round off the edges of the box.
        // This helps to optimize collision detection performance, by reducing the likelihood
        // of the inner hull being penetrated and incurring expensive collision algorithms.
        public fp BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        private fp m_BevelRadius;

        public bool Equals(BoxGeometry other)
        {
            return m_Center.Equals(other.m_Center)
                && m_Orientation.Equals(other.m_Orientation)
                && m_Size.Equals(other.m_Size)
                && m_BevelRadius.Equals(other.m_BevelRadius);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint3(
                fpmath.hash(m_Center),
                fpmath.hash(m_Orientation),
                fpmath.hash(new fp4(m_Size, m_BevelRadius))
            )));
        }
    }

    // A collider in the shape of a box
    public struct BoxCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types..
        private unsafe fixed byte m_Vertices[sizeof(long) * 3 * 8];         // fp3[8]
        private unsafe fixed byte m_FacePlanes[sizeof(long) * 4 * 6];       // Plane[6]
        private unsafe fixed byte m_Faces[4 * 6];                            // ConvexHull.Face[6]
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 24];    // byte[24]
        private unsafe fixed byte m_VertexEdges[4 * 8];                      // ConvexHull.Edge[8]
        private unsafe fixed byte m_FaceLinks[4 * 24];                       // ConvexHull.Edge[24]

        // Box parameters
        private fp3 m_Center;
        private fpquaternion m_Orientation;
        private fp3 m_Size;

        public fp3 Center => m_Center;
        public fpquaternion Orientation => m_Orientation;
        public fp3 Size => m_Size;
        public fp BevelRadius => ConvexHull.ConvexRadius;

        public BoxGeometry Geometry
        {
            get => new BoxGeometry
            {
                Center = m_Center,
                Orientation = m_Orientation,
                Size = m_Size,
                BevelRadius = ConvexHull.ConvexRadius
            };
            set
            {
                if (!value.Equals(Geometry))
                {
                    SetGeometry(value);
                }
            }
        }

        #region Construction

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> Create(BoxGeometry geometry, CollisionFilter filter, Material material)
        {
            var collider = default(BoxCollider);
            collider.Initialize(geometry, filter, material);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(BoxCollider));
        }

        // Initializes the box collider, enables it to be created on stack.
        public unsafe void Initialize(BoxGeometry geometry, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Box;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            // Build immutable convex data
            fixed(BoxCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 8;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 6;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob.Offset);
                ConvexHull.FacesBlob.Length = 6;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 24;

                ConvexHull.VertexEdgesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_VertexEdges[0], ref ConvexHull.VertexEdgesBlob);
                ConvexHull.VertexEdgesBlob.Length = 8;

                ConvexHull.FaceLinksBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceLinks[0], ref ConvexHull.FaceLinksBlob);
                ConvexHull.FaceLinksBlob.Length = 24;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[1] = new ConvexHull.Face { FirstIndex = 4, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[2] = new ConvexHull.Face { FirstIndex = 8, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[3] = new ConvexHull.Face { FirstIndex = 12, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[4] = new ConvexHull.Face { FirstIndex = 16, NumVertices = 4, MinHalfAngleCompressed = 0x80 };
                faces[5] = new ConvexHull.Face { FirstIndex = 20, NumVertices = 4, MinHalfAngleCompressed = 0x80 };

                byte* index = &collider->m_FaceVertexIndices[0];
                // stackalloc short* instead of byte* because packing size 1 not supported by Burst
                short* faceVertexIndices = stackalloc short[24] { 2, 6, 4, 0, 1, 5, 7, 3, 1, 0, 4, 5, 7, 6, 2, 3, 3, 2, 0, 1, 7, 5, 4, 6 };
                for (int i = 0; i < 24; i++)
                {
                    *index++ = (byte)faceVertexIndices[i];
                }

                ConvexHull.Edge* vertexEdge = (ConvexHull.Edge*)(&collider->m_VertexEdges[0]);
                short* vertexEdgeValuePairs = stackalloc short[16] { 4, 2, 2, 0, 4, 1, 4, 0, 5, 2, 5, 1, 0, 1, 5, 0 };
                for (int i = 0; i < 8; i++)
                {
                    *vertexEdge++ = new ConvexHull.Edge
                    {
                        FaceIndex = vertexEdgeValuePairs[2 * i],
                        EdgeIndex = (byte)vertexEdgeValuePairs[2 * i + 1]
                    };
                }

                ConvexHull.Edge* faceLink = (ConvexHull.Edge*)(&collider->m_FaceLinks[0]);
                short* faceLinkValuePairs = stackalloc short[48]
                {
                    3, 1, 5, 2, 2, 1, 4, 1, 2, 3, 5, 0, 3, 3, 4, 3, 4, 2, 0, 2, 5, 1, 1, 0,
                    5, 3, 0, 0, 4, 0, 1, 2, 3, 2, 0, 3, 2, 0, 1, 3, 1, 1, 2, 2, 0, 1, 3, 0
                };
                for (int i = 0; i < 24; i++)
                {
                    *faceLink++ = new ConvexHull.Edge
                    {
                        FaceIndex = faceLinkValuePairs[2 * i],
                        EdgeIndex = (byte)faceLinkValuePairs[2 * i + 1]
                    };
                }
            }

            // Build mutable convex data
            SetGeometry(geometry);
        }

        unsafe void SetGeometry(BoxGeometry geometry)
        {
            SafetyChecks.CheckValidAndThrow(geometry, nameof(geometry));

            m_Header.Version += 1;
            ConvexHull.ConvexRadius = geometry.BevelRadius;
            m_Center = geometry.Center;
            m_Orientation = geometry.Orientation;
            m_Size = geometry.Size;

            fixed(BoxCollider* collider = &this)
            {
                var transform = new FpRigidTransform(m_Orientation, m_Center);

                // Clamp to avoid extents < 0
                fp3 he = fpmath.max(fp.zero, m_Size * fp.half - ConvexHull.ConvexRadius); // half extents

                fp3* vertices = (fp3*)(&collider->m_Vertices[0]);
                vertices[0] = fpmath.transform(transform, new fp3(he.x, he.y, he.z));
                vertices[1] = fpmath.transform(transform, new fp3(-he.x, he.y, he.z));
                vertices[2] = fpmath.transform(transform, new fp3(he.x, -he.y, he.z));
                vertices[3] = fpmath.transform(transform, new fp3(-he.x, -he.y, he.z));
                vertices[4] = fpmath.transform(transform, new fp3(he.x, he.y, -he.z));
                vertices[5] = fpmath.transform(transform, new fp3(-he.x, he.y, -he.z));
                vertices[6] = fpmath.transform(transform, new fp3(he.x, -he.y, -he.z));
                vertices[7] = fpmath.transform(transform, new fp3(-he.x, -he.y, -he.z));

                Plane* planes = (Plane*)(&collider->m_FacePlanes[0]);
                planes[0] = Math.TransformPlane(transform, new Plane(new fp3(fp.one, fp.zero, fp.zero), -he.x));
                planes[1] = Math.TransformPlane(transform, new Plane(new fp3(fp.minusOne, fp.zero, fp.zero), -he.x));
                planes[2] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.one, fp.zero), -he.y));
                planes[3] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.minusOne, fp.zero), -he.y));
                planes[4] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.zero, fp.one), -he.z));
                planes[5] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.zero, fp.minusOne), -he.z));
            }
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<BoxCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) { m_Header.Version += 1; m_Header.Filter = value; } } }
        internal bool RespondsToCollision => m_Header.Material.CollisionResponse != CollisionResponsePolicy.None;
        public Material Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) { m_Header.Version += 1; m_Header.Material = value; } } }

        public MassProperties MassProperties => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = new FpRigidTransform(m_Orientation, m_Center),
                InertiaTensor = new fp3(
                    (m_Size.y * m_Size.y + m_Size.z * m_Size.z) / 12,
                    (m_Size.x * m_Size.x + m_Size.z * m_Size.z) / 12,
                    (m_Size.x * m_Size.x + m_Size.y * m_Size.y) / 12)
            },
            Volume = m_Size.x * m_Size.y * m_Size.z,
            AngularExpansionFactor = fpmath.length(m_Size * fp.half - ConvexHull.ConvexRadius)
        };

        public Aabb CalculateAabb()
        {
            return CalculateAabb(FpRigidTransform.identity);
        }

        public Aabb CalculateAabb(FpRigidTransform transform)
        {
            fp3 centerInB = fpmath.transform(transform, m_Center);

            fpquaternion worldFromBox = fpmath.mul(transform.rot, m_Orientation);
            fp3 x = fpmath.mul(worldFromBox, new fp3(m_Size.x * fp.half, fp.zero, fp.zero));
            fp3 y = fpmath.mul(worldFromBox, new fp3(fp.zero, m_Size.y * fp.half, fp.zero));
            fp3 z = fpmath.mul(worldFromBox, new fp3(fp.zero, fp.zero, m_Size.z * fp.half));
            fp3 halfExtentsInB = fpmath.abs(x) + fpmath.abs(y) + fpmath.abs(z);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed(BoxCollider* target = &this)
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
            fixed(BoxCollider* target = &this)
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
            fixed(BoxCollider* target = &this)
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
            fixed(BoxCollider* target = &this)
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

        #endregion
    }
}
