using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public struct CylinderGeometry : IEquatable<CylinderGeometry>
    {
        public const int MinSideCount = 3;
        public const int MaxSideCount = 32;

        // The center of the cylinder
        public fp3 Center { get => m_Center; set => m_Center = value; }
        fp3 m_Center;

        // The orientation of the cylinder
        public fpquaternion Orientation { get => m_Orientation; set => m_Orientation = value; }
        private fpquaternion m_Orientation;

        // The height of the cylinder, centered along the local Y axis
        public fp Height { get => m_Height; set => m_Height = value; }
        private fp m_Height;

        // The radius of the cylinder
        public fp Radius { get => m_Radius; set => m_Radius = value; }
        private fp m_Radius;

        // The radius by which to round off the edges of the cylinder.
        // This helps to optimize collision detection performance, by reducing the likelihood
        // of the inner hull being penetrated and incurring expensive collision algorithms.
        public fp BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        private fp m_BevelRadius;

        // The number of faces used to represent the rounded part of the cylinder
        public int SideCount { get => m_SideCount; set => m_SideCount = value; }
        private int m_SideCount;

        public bool Equals(CylinderGeometry other)
        {
            return m_Center.Equals(other.m_Center)
                && m_Orientation.Equals(other.m_Orientation)
                && m_Height.Equals(other.m_Height)
                && m_Radius.Equals(other.m_Radius)
                && m_BevelRadius.Equals(other.m_BevelRadius)
                && m_SideCount.Equals(other.m_SideCount);
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint4(
                fpmath.hash(m_Center),
                fpmath.hash(m_Orientation),
                fpmath.hash(new fp3(m_Height, m_Radius, m_BevelRadius)),
                unchecked((uint)m_SideCount)
            )));
        }
    }

    // A collider in the shape of a cylinder
    public struct CylinderCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        // Convex hull data, sized for the maximum allowed number of cylinder faces
        // Todo: would be nice to use the actual types here but C# only likes fixed arrays of builtin types..
        private unsafe fixed byte m_Vertices[sizeof(long) * 3 * 2 * CylinderGeometry.MaxSideCount];
        private unsafe fixed byte m_FacePlanes[sizeof(long) * 4 * (2 + CylinderGeometry.MaxSideCount)];
        private unsafe fixed byte m_Faces[4 * (2 + CylinderGeometry.MaxSideCount)];
        private unsafe fixed byte m_FaceVertexIndices[sizeof(byte) * 6 * CylinderGeometry.MaxSideCount];

        // Cylinder parameters
        private fp3 m_Center;
        private fpquaternion m_Orientation;
        private fp m_Height;
        private fp m_Radius;
        private int m_SideCount;

        public fp3 Center => m_Center;
        public fpquaternion Orientation => m_Orientation;
        public fp Height => m_Height;
        public fp Radius => m_Radius;
        public fp BevelRadius => ConvexHull.ConvexRadius;
        public int SideCount => m_SideCount;

        public CylinderGeometry Geometry
        {
            get => new CylinderGeometry
            {
                Center = m_Center,
                Orientation = m_Orientation,
                Height = m_Height,
                Radius = m_Radius,
                BevelRadius = ConvexHull.ConvexRadius,
                SideCount = m_SideCount
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

        public static BlobAssetReference<Collider> Create(CylinderGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> Create(CylinderGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> Create(CylinderGeometry geometry, CollisionFilter filter, Material material)
        {
            var collider = default(CylinderCollider);
            collider.Initialize(geometry, filter, material);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(CylinderCollider));
        }

        // Initializes the cylinder collider, enables it to be created on stack.
        public unsafe void Initialize(CylinderGeometry geometry, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Cylinder;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;
            MemorySize = UnsafeUtility.SizeOf<CylinderCollider>();

            // Initialize immutable convex data
            fixed(CylinderCollider* collider = &this)
            {
                ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Vertices[0], ref ConvexHull.VerticesBlob);
                ConvexHull.VerticesBlob.Length = 0;

                ConvexHull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FacePlanes[0], ref ConvexHull.FacePlanesBlob);
                ConvexHull.FacePlanesBlob.Length = 0;

                ConvexHull.FacesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_Faces[0], ref ConvexHull.FacesBlob);
                ConvexHull.FacesBlob.Length = 0;

                ConvexHull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(ref collider->m_FaceVertexIndices[0], ref ConvexHull.FaceVertexIndicesBlob);
                ConvexHull.FaceVertexIndicesBlob.Length = 0;

                // No connectivity
                ConvexHull.VertexEdgesBlob.Offset = 0;
                ConvexHull.VertexEdgesBlob.Length = 0;
                ConvexHull.FaceLinksBlob.Offset = 0;
                ConvexHull.FaceLinksBlob.Length = 0;
            }

            // Set mutable data
            SetGeometry(geometry);
        }

        unsafe void SetGeometry(CylinderGeometry geometry)
        {
            SafetyChecks.CheckValidAndThrow(geometry, nameof(geometry));

            m_Header.Version += 1;
            m_Center = geometry.Center;
            m_Orientation = geometry.Orientation;
            m_Height = geometry.Height;
            m_Radius = geometry.Radius;
            m_SideCount = geometry.SideCount;

            ConvexHull.ConvexRadius = geometry.BevelRadius;
            ConvexHull.VerticesBlob.Length = m_SideCount * 2;
            ConvexHull.FacePlanesBlob.Length = m_SideCount + 2;
            ConvexHull.FacesBlob.Length = m_SideCount + 2;
            ConvexHull.FaceVertexIndicesBlob.Length = m_SideCount * 6;

            var transform = new FpRigidTransform(m_Orientation, m_Center);
            var radius = fpmath.max(m_Radius - ConvexHull.ConvexRadius, fp.zero);
            var halfHeight = fpmath.max(m_Height * fp.half - ConvexHull.ConvexRadius, fp.zero);

            fp sideCount = (fp)m_SideCount;
            fixed(CylinderCollider* collider = &this)
            {
                // vertices
                fp3* vertices = (fp3*)(&collider->m_Vertices[0]);
                var arcStep = fp.TwoPi / sideCount;
                for (var i = 0; i < m_SideCount; i++)
                {
                    fp _i = (fp)i;
                    var x = fpmath.cos(arcStep * _i) * radius;
                    var y = fpmath.sin(arcStep * _i) * radius;
                    vertices[i] = fpmath.transform(transform, new fp3(x, y, -halfHeight));
                    vertices[i + m_SideCount] = fpmath.transform(transform, new fp3(x, y, halfHeight));
                }

                // planes
                Plane* planes = (Plane*)(&collider->m_FacePlanes[0]);
                planes[0] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.zero, fp.minusOne), -halfHeight));
                planes[1] = Math.TransformPlane(transform, new Plane(new fp3(fp.zero, fp.zero, fp.one), -halfHeight));
                fp d = radius * fpmath.cos((fp)fpmath.PI / sideCount);
                for (int i = 0; i < m_SideCount; ++i)
                {
                    fp angle = fp.TwoPi * (i + fp.half) / sideCount;
                    planes[2 + i] = Math.TransformPlane(transform, new Plane(new fp3(fpmath.cos(angle), fpmath.sin(angle), fp.zero), -d));
                }

                // faces
                ConvexHull.Face* faces = (ConvexHull.Face*)(&collider->m_Faces[0]);
                byte* indices = (byte*)(&collider->m_FaceVertexIndices[0]);
                fp halfAngle = fp.Pi * fp.quarter;
                {
                    faces[0].FirstIndex = 0;
                    faces[0].NumVertices = (byte)m_SideCount;
                    faces[0].MinHalfAngle = halfAngle;
                    for (int i = 0; i < m_SideCount; ++i)
                    {
                        indices[i] = (byte)(m_SideCount - 1 - i);
                    }

                    faces[1].FirstIndex = (short)m_SideCount;
                    faces[1].NumVertices = (byte)m_SideCount;
                    faces[1].MinHalfAngle = halfAngle;
                    for (int i = m_SideCount; i < 2 * m_SideCount; ++i)
                    {
                        indices[i] = (byte)(i);
                    }
                }
                halfAngle = (fp)fpmath.PI / sideCount;
                for (int i = 0; i < m_SideCount; ++i)
                {
                    int firstIndex = (2 * m_SideCount) + (4 * i);

                    faces[i + 2].FirstIndex = (short)firstIndex;
                    faces[i + 2].NumVertices = 4;
                    faces[i + 2].MinHalfAngle = halfAngle;

                    indices[firstIndex + 0] = (byte)i;
                    indices[firstIndex + 1] = (byte)((i + 1) % m_SideCount);
                    indices[firstIndex + 2] = (byte)((i + 1) % m_SideCount + m_SideCount);
                    indices[firstIndex + 3] = (byte)(i + m_SideCount);
                }
            }

            fp radiusSq = m_Radius * m_Radius;
            MassProperties = new MassProperties
            {
                MassDistribution = new MassDistribution
                {
                    Transform = transform,
                    InertiaTensor = new fp3(
                        (radiusSq + m_Height * m_Height) * fp.FromRaw(0x3daaaaab),
                        (radiusSq + m_Height * m_Height) * fp.FromRaw(0x3daaaaab),
                        (radiusSq) * fp.half)
                },
                Volume = (fp)fpmath.PI * radiusSq * m_Height,
                AngularExpansionFactor = fpmath.sqrt(radius * radius + halfHeight * halfHeight)
            };
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) { m_Header.Version += 1; m_Header.Filter = value; } } }
        internal bool RespondsToCollision => m_Header.Material.CollisionResponse != CollisionResponsePolicy.None;
        public Material Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) { m_Header.Version += 1; m_Header.Material = value; } } }
        public MassProperties MassProperties { get; private set; }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(FpRigidTransform.identity);
        }

        public unsafe Aabb CalculateAabb(FpRigidTransform transform)
        {
            transform = fpmath.mul(transform, new FpRigidTransform(m_Orientation, m_Center));
            var halfAxis = fpmath.rotate(transform, new fp3(fp.zero, fp.zero, m_Height * fp.half));
            fp3 v0 = transform.pos + halfAxis;
            fp3 v1 = transform.pos - halfAxis;
            var axis = v1 - v0;
            fp axisLen2 = m_Height * m_Height;
            fp invAxisLen2 = fpmath.rcp(axisLen2);
            fp3 axisYZX = new fp3(axis.y, axis.z, axis.x);
            fp3 axisZXY = new fp3(axis.z, axis.x, axis.y);
            fp3 root = axisYZX * axisYZX;
            root += axisZXY * axisZXY;
            root *= invAxisLen2;
            fp3 expansion = fpmath.sqrt(root);
            expansion *= m_Radius;
            return new Aabb
            {
                Min = fpmath.min(v0, v1) - expansion,
                Max = fpmath.max(v0, v1) + expansion
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed(CylinderCollider* target = &this)
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
            fixed(CylinderCollider* target = &this)
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
            fixed(CylinderCollider* target = &this)
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
            fixed(CylinderCollider* target = &this)
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
