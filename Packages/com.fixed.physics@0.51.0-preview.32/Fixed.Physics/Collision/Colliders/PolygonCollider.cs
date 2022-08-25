using System;
using System.ComponentModel;
using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics.FixedPoint;
using Unity.Entities;

namespace Fixed.Physics
{
    // A flat convex collider with either 3 or 4 coplanar vertices (ie, a triangle or a quad)
    public struct PolygonCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types
        private unsafe fixed byte m_Vertices[sizeof(long) * 3 * 4];     // fp3[4]
        private unsafe fixed byte m_FacePlanes[sizeof(long) * 4 * 2];   // Plane[2]
        private unsafe fixed byte m_Faces[4 * 2];                        // ConvexHull.Face[2]
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 8]; // byte[8]

        public bool IsTriangle => Vertices.Length == 3;
        public bool IsQuad => Vertices.Length == 4;

        public BlobArray.Accessor<fp3> Vertices => ConvexHull.Vertices;
        public BlobArray.Accessor<Plane> Planes => ConvexHull.Planes;

        #region Construction

        public static BlobAssetReference<Collider> CreateTriangle(fp3 vertex0, fp3 vertex1, fp3 vertex2) =>
            CreateTriangle(vertex0, vertex1, vertex2, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> CreateTriangle(fp3 vertex0, fp3 vertex1, fp3 vertex2,
            CollisionFilter filter) => CreateTriangle(vertex0, vertex1, vertex2, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> CreateTriangle(fp3 vertex0, fp3 vertex1, fp3 vertex2, CollisionFilter filter, Material material)
        {
            SafetyChecks.CheckFiniteAndThrow(vertex0, nameof(vertex0));
            SafetyChecks.CheckFiniteAndThrow(vertex1, nameof(vertex1));
            SafetyChecks.CheckFiniteAndThrow(vertex2, nameof(vertex2));

            var collider = new PolygonCollider();
            collider.InitAsTriangle(vertex0, vertex1, vertex2, filter, material);

            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<PolygonCollider>());
        }

        // quad vertices must be coplanar and wound consistently
        public static BlobAssetReference<Collider> CreateQuad(fp3 vertex0, fp3 vertex1, fp3 vertex2, fp3 vertex3) =>
            CreateQuad(vertex0, vertex1, vertex2, vertex3, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> CreateQuad(fp3 vertex0, fp3 vertex1, fp3 vertex2, fp3 vertex3, CollisionFilter filter) =>
            CreateQuad(vertex0, vertex1, vertex2, vertex3, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> CreateQuad(fp3 vertex0, fp3 vertex1, fp3 vertex2, fp3 vertex3, CollisionFilter filter, Material material)
        {
            SafetyChecks.CheckFiniteAndThrow(vertex0, nameof(vertex0));
            SafetyChecks.CheckFiniteAndThrow(vertex1, nameof(vertex1));
            SafetyChecks.CheckFiniteAndThrow(vertex2, nameof(vertex2));
            SafetyChecks.CheckFiniteAndThrow(vertex3, nameof(vertex3));
            SafetyChecks.CheckCoplanarAndThrow(vertex0, vertex1, vertex2, vertex3, nameof(vertex3));

            PolygonCollider collider = default;
            collider.InitAsQuad(vertex0, vertex1, vertex2, vertex3, filter, material);
            return BlobAssetReference<Collider>.Create(&collider, UnsafeUtility.SizeOf<PolygonCollider>());
        }

        internal void InitNoVertices(CollisionFilter filter, Material material)
        {
            Init(filter, material);
            ConvexHull.VerticesBlob.Length = 0;
        }

        internal void InitAsTriangle(fp3 vertex0, fp3 vertex1, fp3 vertex2, CollisionFilter filter, Material material)
        {
            Init(filter, material);
            SetAsTriangle(vertex0, vertex1, vertex2);
        }

        internal void InitAsQuad(fp3 vertex0, fp3 vertex1, fp3 vertex2, fp3 vertex3, CollisionFilter filter, Material material)
        {
            Init(filter, material);
            SetAsQuad(vertex0, vertex1, vertex2, vertex3);
        }

        internal unsafe void SetAsTriangle(fp3 v0, fp3 v1, fp3 v2)
        {
            m_Header.Type = ColliderType.Triangle;
            m_Header.Version += 1;

            ConvexHull.VerticesBlob.Length = 3;
            ConvexHull.FaceVertexIndicesBlob.Length = 6;

            fixed(PolygonCollider* collider = &this)
            {
                fp3* vertices = (fp3*)(&collider->m_Vertices[0]);
                vertices[0] = v0;
                vertices[1] = v1;
                vertices[2] = v2;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 3, MinHalfAngleCompressed = 0xff };
                faces[1] = new ConvexHull.Face { FirstIndex = 3, NumVertices = 3, MinHalfAngleCompressed = 0xff };

                byte* index = &collider->m_FaceVertexIndices[0];
                *index++ = 0; *index++ = 1; *index++ = 2;
                *index++ = 2; *index++ = 1; *index++ = 0;
            }

            SetPlanes();
        }

        internal unsafe void SetAsQuad(fp3 v0, fp3 v1, fp3 v2, fp3 v3)
        {
            m_Header.Type = ColliderType.Quad;
            m_Header.Version += 1;

            ConvexHull.VerticesBlob.Length = 4;
            ConvexHull.FaceVertexIndicesBlob.Length = 8;

            fixed(PolygonCollider* collider = &this)
            {
                fp3* vertices = (fp3*)(&collider->m_Vertices[0]);
                vertices[0] = v0;
                vertices[1] = v1;
                vertices[2] = v2;
                vertices[3] = v3;

                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                faces[0] = new ConvexHull.Face { FirstIndex = 0, NumVertices = 4, MinHalfAngleCompressed = 0xff };
                faces[1] = new ConvexHull.Face { FirstIndex = 4, NumVertices = 4, MinHalfAngleCompressed = 0xff };

                byte* index = &collider->m_FaceVertexIndices[0];
                *index++ = 0; *index++ = 1; *index++ = 2; *index++ = 3;
                *index++ = 3; *index++ = 2; *index++ = 1; *index++ = 0;
            }

            SetPlanes();
        }

        unsafe void Init(CollisionFilter filter, Material material)
        {
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            ConvexHull.ConvexRadius = fp.zero;

            fixed(PolygonCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 4;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 2;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob);
                ConvexHull.FacesBlob.Length = 2;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 8;

                // No connectivity needed
                ConvexHull.VertexEdgesBlob.Offset = 0;
                ConvexHull.VertexEdgesBlob.Length = 0;
                ConvexHull.FaceLinksBlob.Offset = 0;
                ConvexHull.FaceLinksBlob.Length = 0;
            }
        }

        private void SetPlanes()
        {
            BlobArray.Accessor<fp3> hullVertices = ConvexHull.Vertices;
            fp3 cross = fpmath.cross(
                hullVertices[1] - hullVertices[0],
                hullVertices[2] - hullVertices[0]);
            fp dot = fpmath.dot(cross, hullVertices[0]);
            fp invLengthCross = fpmath.rsqrt(fpmath.lengthsq(cross));
            Plane plane = new Plane(cross * invLengthCross, -dot * invLengthCross);

            ConvexHull.Planes[0] = plane;
            ConvexHull.Planes[1] = plane.Flipped;
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<PolygonCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) { m_Header.Version += 1; m_Header.Filter = value; } } }
        internal bool RespondsToCollision => m_Header.Material.CollisionResponse != CollisionResponsePolicy.None;
        public Material Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) { m_Header.Version += 1; m_Header.Material = value; } } }

        public MassProperties MassProperties
        {
            get
            {
                // TODO - the inertia computed here is incorrect. Computing the correct inertia is expensive, so it probably ought to be cached.
                // Note this is only called for top level polygon colliders, not for polygons within a mesh.
                fp3 center = (ConvexHull.Vertices[0] + ConvexHull.Vertices[1] + ConvexHull.Vertices[2]) / (fp)3.0f;
                fp radiusSq = fpmath.max(fpmath.max(
                    fpmath.lengthsq(ConvexHull.Vertices[0] - center),
                    fpmath.lengthsq(ConvexHull.Vertices[1] - center)),
                    fpmath.lengthsq(ConvexHull.Vertices[2] - center));
                return new MassProperties
                {
                    MassDistribution = new MassDistribution
                    {
                        Transform = new FpRigidTransform(fpquaternion.identity, center),
                        InertiaTensor = new fp3(new fp(0,2,5) * radiusSq)
                    },
                    Volume = fp.zero,
                    AngularExpansionFactor = fpmath.sqrt(radiusSq)
                };
            }
        }

        public Aabb CalculateAabb()
        {
            fp3 min = fpmath.min(fpmath.min(ConvexHull.Vertices[0], ConvexHull.Vertices[1]), fpmath.min(ConvexHull.Vertices[2], ConvexHull.Vertices[3]));
            fp3 max = fpmath.max(fpmath.max(ConvexHull.Vertices[0], ConvexHull.Vertices[1]), fpmath.max(ConvexHull.Vertices[2], ConvexHull.Vertices[3]));
            return new Aabb
            {
                Min = min - new fp3(ConvexHull.ConvexRadius),
                Max = max + new fp3(ConvexHull.ConvexRadius)
            };
        }

        public Aabb CalculateAabb(FpRigidTransform transform)
        {
            fp3 v0 = fpmath.rotate(transform, ConvexHull.Vertices[0]);
            fp3 v1 = fpmath.rotate(transform, ConvexHull.Vertices[1]);
            fp3 v2 = fpmath.rotate(transform, ConvexHull.Vertices[2]);
            fp3 v3 = IsQuad ? fpmath.rotate(transform, ConvexHull.Vertices[3]) : v2;

            fp3 min = fpmath.min(fpmath.min(v0, v1), fpmath.min(v2, v3));
            fp3 max = fpmath.max(fpmath.max(v0, v1), fpmath.max(v2, v3));
            return new Aabb
            {
                Min = min + transform.pos - new fp3(ConvexHull.ConvexRadius),
                Max = max + transform.pos + new fp3(ConvexHull.ConvexRadius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed(PolygonCollider* target = &this)
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
            fixed(PolygonCollider* target = &this)
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
            fixed(PolygonCollider* target = &this)
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
            fixed(PolygonCollider* target = &this)
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
