using Unity.Assertions;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    /// <summary>
    /// This struct captures the information needed for ray casting.
    /// It is technically not a Ray as it includes a length.
    /// This is to avoid performance issues with infinite length Rays.
    /// </summary>
    public struct Ray
    {
        /// <summary>
        /// The Origin point of the Ray in query space.
        /// </summary>
        /// <value> Point vector coordinate. </value>
        public fp3 Origin;

        /// <summary>
        /// This represents the line from the Ray's Origin to a second point on the Ray. The second point will be the Ray End if nothing is hit.
        /// </summary>
        /// <value> Line vector. </value>
        public fp3 Displacement
        {
            get => m_Displacement;
            set
            {
                m_Displacement = value;
                ReciprocalDisplacement = fpmath.select(fpmath.rcp(m_Displacement), fpmath.sqrt(fp.max_value), m_Displacement == fp3.zero);
            }
        }
        fp3 m_Displacement;

        // Performance optimization used in the BoundingVolumeHierarchy casting functions
        internal fp3 ReciprocalDisplacement { get; private set; }
    }

    /// <summary>
    /// The input to RayCastQueries consists of the Start and End positions of a line segment as well as a CollisionFilter to cull potential hits.
    /// </summary>
    public struct RaycastInput
    {
        /// <summary>
        /// The Start position of a Ray.
        /// </summary>
        public fp3 Start
        {
            get => Ray.Origin;
            set
            {
                fp3 end = Ray.Origin + Ray.Displacement;
                Ray.Origin = value;
                Ray.Displacement = end - value;
                Assert.IsTrue(math.all(fpmath.abs(Ray.Displacement) < Math.Constants.MaxDisplacement3F), "RayCast length is very long. This would lead to floating point inaccuracies and invalid results.");
            }
        }
        /// <summary>
        /// The End position of a Ray.
        /// </summary>
        public fp3 End
        {
            get => Ray.Origin + Ray.Displacement;
            set
            {
                Ray.Displacement = value - Ray.Origin;
                Assert.IsTrue(math.all(fpmath.abs(Ray.Displacement) < Math.Constants.MaxDisplacement3F), "RayCast length is very long. This would lead to floating point inaccuracies and invalid results.");
            }
        }
        /// <summary>
        /// The CollisionFilter is used to determine what objects the Ray is and isn't going to hit.
        /// </summary>
        public CollisionFilter Filter;

        internal Ray Ray;
        internal QueryContext QueryContext;

        public override string ToString() =>
            $"RaycastInput {{ Start = {Start}, End = {End}, Filter = {Filter} }}";
    }

    // A hit from a ray cast query
    /// <summary>
    /// A struct representing the hit from a RaycastQuery.
    /// </summary>
    public struct RaycastHit : IQueryResult
    {
        /// <summary>
        /// Fraction of the distance along the Ray where the hit occurred.
        /// </summary>
        /// <value> Returns a value between 0 and 1. </value>
        public fp Fraction { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns RigidBodyIndex of queried body.</value>
        public int RigidBodyIndex { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns ColliderKey of queried leaf collider</value>
        public ColliderKey ColliderKey { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns Material of queried leaf collider</value>
        public Material Material { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns Entity of queried body</value>
        public Entity Entity { get; set; }

        /// <summary>
        /// The point in query space where the hit occurred.
        /// </summary>
        /// <value> Returns the position of the point where the hit occurred. </value>
        public fp3 Position { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns the normal of the point where the hit occurred. </value>
        public fp3 SurfaceNormal { get; set; }

        public override string ToString() =>
            $"RaycastHit {{ Fraction = {Fraction}, RigidBodyIndex = {RigidBodyIndex}, ColliderKey = {ColliderKey}, Entity = {Entity}, Position = {Position}, SurfaceNormal = {SurfaceNormal} }}";
    }

    // Raycast query implementations
    static class RaycastQueries
    {
        #region Ray vs primitives

        // Note that the primitives are considered solid.
        // Any ray originating from within the primitive will return a hit,
        // however the hit fraction will be zero, and the hit normal
        // will be the negation of the ray displacement vector.

        public static bool RaySphere(
            fp3 rayOrigin, fp3 rayDisplacement,
            fp3 sphereCenter, fp sphereRadius,
            ref fp fraction, out fp3 normal)
        {
            normal = fp3.zero;

            // TODO.ma lots of fp inaccuracy problems with this
            fp3 diff = rayOrigin - sphereCenter;
            fp a = fpmath.dot(rayDisplacement, rayDisplacement);
            fp b = fp.two * fpmath.dot(rayDisplacement, diff);
            fp c = fpmath.dot(diff, diff) - sphereRadius * sphereRadius;
            fp discriminant = b * b - (fp)4.0f * a * c;

            if (c < fp.zero)
            {
                // Inside hit.
                fraction = fp.zero;
                normal = fpmath.normalize(-rayDisplacement);
                return true;
            }

            if (discriminant < fp.zero)
            {
                return false;
            }

            fp sqrtDiscriminant = fpmath.sqrt(discriminant);
            fp invDenom = fp.half / a;

            fp t0 = (sqrtDiscriminant - b) * invDenom;
            fp t1 = (-sqrtDiscriminant - b) * invDenom;
            fp tMin = fpmath.min(t0, t1);

            if (tMin >= fp.zero && tMin < fraction)
            {
                fraction = tMin;
                normal = (rayOrigin + rayDisplacement * fraction - sphereCenter) / sphereRadius;

                return true;
            }

            return false;
        }

        public static bool RayCapsule(
            fp3 rayOrigin, fp3 rayDisplacement,
            fp3 vertex0, fp3 vertex1, fp radius,
            ref fp fraction, out fp3 normal)
        {
            fp axisLength = NormalizeWithLength(vertex1 - vertex0, out fp3 axis);

            // Ray vs infinite cylinder
            {
                fp directionDotAxis = fpmath.dot(rayDisplacement, axis);
                fp originDotAxis = fpmath.dot(rayOrigin - vertex0, axis);
                fp3 rayDisplacement2D = rayDisplacement - axis * directionDotAxis;
                fp3 rayOrigin2D = rayOrigin - axis * originDotAxis;
                fp cylinderFraction = fraction;

                if (RaySphere(rayOrigin2D, rayDisplacement2D, vertex0, radius, ref cylinderFraction, out normal))
                {
                    fp t = originDotAxis + cylinderFraction * directionDotAxis; // distance of the hit from Vertex0 along axis
                    if (t >= fp.zero && t <= axisLength)
                    {
                        if (cylinderFraction == fp.zero)
                        {
                            // Inside hit
                            normal = fpmath.normalize(-rayDisplacement);
                        }

                        fraction = cylinderFraction;
                        return true;
                    }
                }
            }

            // Ray vs caps
            {
                bool hadHit = false;
                fp3 capNormal;
                if (RaySphere(rayOrigin, rayDisplacement, vertex0, radius, ref fraction, out capNormal))
                {
                    hadHit = true;
                    normal = capNormal;
                }
                if (RaySphere(rayOrigin, rayDisplacement, vertex1, radius, ref fraction, out capNormal))
                {
                    hadHit = true;
                    normal = capNormal;
                }
                return hadHit;
            }
        }

        public static bool RayTriangle(
            fp3 rayOrigin, fp3 rayDisplacement,
            fp3 a, fp3 b, fp3 c, // TODO: fp3x3?
            ref fp fraction, out fp3 unnormalizedNormal)
        {
            fp3 vAb = b - a;
            fp3 vCa = a - c;

            fp3 vN = fpmath.cross(vAb, vCa);
            fp3 vAp = rayOrigin - a;
            fp3 end0 = vAp + rayDisplacement * fraction;

            fp d = fpmath.dot(vN, vAp);
            fp e = fpmath.dot(vN, end0);

            if (d * e >= fp.zero)
            {
                unnormalizedNormal = fp3.zero;
                return false;
            }

            fp3 vBc = c - b;
            fraction *= d / (d - e);
            unnormalizedNormal = vN * fpmath.sign(d);

            // edge normals
            fp3 c0 = fpmath.cross(vAb, rayDisplacement);
            fp3 c1 = fpmath.cross(vBc, rayDisplacement);
            fp3 c2 = fpmath.cross(vCa, rayDisplacement);

            fp3 dots;
            {
                fp3 o2 = rayOrigin + rayOrigin;
                fp3 r0 = o2 - (a + b);
                fp3 r1 = o2 - (b + c);
                fp3 r2 = o2 - (c + a);

                dots.x = fpmath.dot(r0, c0);
                dots.y = fpmath.dot(r1, c1);
                dots.z = fpmath.dot(r2, c2);
            }

            // hit if all dots have the same sign
            return math.all(dots <= 0) || math.all(dots >= 0);
        }

        public static bool RayQuad(
            fp3 rayOrigin, fp3 rayDisplacement,
            fp3 a, fp3 b, fp3 c, fp3 d, // TODO: fp3x4?
            ref fp fraction, out fp3 unnormalizedNormal)
        {
            fp3 vAb = b - a;
            fp3 vCa = a - c;

            fp3 vN = fpmath.cross(vAb, vCa);
            fp3 vAp = rayOrigin - a;
            fp3 end0 = vAp + rayDisplacement * fraction;

            fp nDotAp = fpmath.dot(vN, vAp);
            fp e = fpmath.dot(vN, end0);

            if (nDotAp * e >= fp.zero)
            {
                unnormalizedNormal = fp3.zero;
                return false;
            }

            fp3 vBc = c - b;
            fp3 vDa = a - d;
            fp3 vCd = d - c;
            fraction *= nDotAp / (nDotAp - e);
            unnormalizedNormal = vN * fpmath.sign(nDotAp);

            // edge normals
            fp3 c0 = fpmath.cross(vAb, rayDisplacement);
            fp3 c1 = fpmath.cross(vBc, rayDisplacement);
            fp3 c2 = fpmath.cross(vCd, rayDisplacement);
            fp3 c3 = fpmath.cross(vDa, rayDisplacement);

            fp4 dots;
            {
                fp3 o2 = rayOrigin + rayOrigin;
                fp3 r0 = o2 - (a + b);
                fp3 r1 = o2 - (b + c);
                fp3 r2 = o2 - (c + d);
                fp3 r3 = o2 - (d + a);

                dots.x = fpmath.dot(r0, c0);
                dots.y = fpmath.dot(r1, c1);
                dots.z = fpmath.dot(r2, c2);
                dots.w = fpmath.dot(r3, c3);
            }

            bool4 notOutSide = dots < 0;
            // hit if all dots have the same sign
            return math.all(dots <= 0) || math.all(dots >= 0);
        }

        public static bool RayConvex(
            fp3 rayOrigin, fp3 rayDisplacement,
            ref ConvexHull hull, ref fp fraction, out fp3 normal)
        {
            // TODO: Call RaySphere/Capsule/Triangle() if num vertices <= 3 ?

            fp convexRadius = hull.ConvexRadius;
            fp fracEnter = fp.minusOne;
            fp fracExit = fp.two;
            fp3 start = rayOrigin;
            fp3 end = start + rayDisplacement * fraction;
            normal = new fp3(fp.one, fp.zero, fp.zero);
            for (int i = 0; i < hull.NumFaces; i++) // TODO.ma vectorize
            {
                // Calculate the plane's hit fraction
                Plane plane = hull.Planes[i];
                fp startDistance = fpmath.dot(start, plane.Normal) + plane.Distance - convexRadius;
                fp endDistance = fpmath.dot(end, plane.Normal) + plane.Distance - convexRadius;
                fp newFraction = startDistance / (startDistance - endDistance);
                bool startInside = (startDistance < fp.zero);
                bool endInside = (endDistance < fp.zero);

                // If the ray is entirely outside of any plane, then it misses
                if (!(startInside || endInside))
                {
                    return false;
                }

                // If the ray crosses the plane, update the enter or exit fraction
                bool enter = !startInside && newFraction > fracEnter;
                bool exit = !endInside && newFraction < fracExit;
                fracEnter = fpmath.select(fracEnter, newFraction, enter);
                normal = fpmath.select(normal, plane.Normal, enter);
                fracExit = fpmath.select(fracExit, newFraction, exit);
            }

            if (fracEnter < fp.zero)
            {
                // Inside hit.
                fraction = fp.zero;
                normal = fpmath.normalize(-rayDisplacement);
                return true;
            }

            if (fracEnter < fracExit)
            {
                fraction *= fracEnter;
                return true;
            }

            // miss
            return false;
        }

        #endregion

        #region Ray vs colliders

        public static unsafe bool RayCollider<T>(RaycastInput input, Collider* collider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider->Filter))
            {
                return false;
            }

            if (!input.QueryContext.IsInitialized)
            {
                input.QueryContext = QueryContext.DefaultContext;
            }

            Material material = Material.Default;
            fp fraction = collector.MaxFraction;
            fp3 normal;
            bool hadHit;
            switch (collider->Type)
            {
                case ColliderType.Sphere:
                    var sphere = (SphereCollider*)collider;
                    hadHit = RaySphere(input.Ray.Origin, input.Ray.Displacement, sphere->Center, sphere->Radius, ref fraction, out normal);
                    material = sphere->Material;
                    break;
                case ColliderType.Capsule:
                    var capsule = (CapsuleCollider*)collider;
                    hadHit = RayCapsule(input.Ray.Origin, input.Ray.Displacement, capsule->Vertex0, capsule->Vertex1, capsule->Radius, ref fraction, out normal);
                    material = capsule->Material;
                    break;
                case ColliderType.Triangle:
                {
                    var triangle = (PolygonCollider*)collider;
                    hadHit = RayTriangle(input.Ray.Origin, input.Ray.Displacement, triangle->Vertices[0], triangle->Vertices[1], triangle->Vertices[2], ref fraction, out fp3 unnormalizedNormal);
                    normal = hadHit ? fpmath.normalize(unnormalizedNormal) : fp3.zero;
                    material = triangle->Material;
                    break;
                }
                case ColliderType.Quad:
                {
                    var quad = (PolygonCollider*)collider;
                    hadHit = RayQuad(input.Ray.Origin, input.Ray.Displacement, quad->Vertices[0], quad->Vertices[1], quad->Vertices[2], quad->Vertices[3], ref fraction, out fp3 unnormalizedNormal);
                    normal = hadHit ? fpmath.normalize(unnormalizedNormal) : fp3.zero;
                    material = quad->Material;
                    break;
                }
                case ColliderType.Box:
                case ColliderType.Cylinder:
                case ColliderType.Convex:
                    hadHit = RayConvex(input.Ray.Origin, input.Ray.Displacement, ref ((ConvexCollider*)collider)->ConvexHull, ref fraction, out normal);
                    material = ((ConvexCollider*)collider)->Material;
                    break;
                case ColliderType.Mesh:
                    return RayMesh(input, (MeshCollider*)collider, ref collector);
                case ColliderType.Compound:
                    return RayCompound(input, (CompoundCollider*)collider, ref collector);
                case ColliderType.Terrain:
                    return RayTerrain(input, (TerrainCollider*)collider, ref collector);
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            if (hadHit)
            {
                var hit = new RaycastHit
                {
                    Fraction = fraction,
                    Position = Mul(input.QueryContext.WorldFromLocalTransform, input.Ray.Origin + (input.Ray.Displacement * fraction)),
                    SurfaceNormal = fpmath.mul(input.QueryContext.WorldFromLocalTransform.Rotation, normal),
                    RigidBodyIndex = input.QueryContext.RigidBodyIndex,
                    ColliderKey = input.QueryContext.ColliderKey,
                    Material = material,
                    Entity = input.QueryContext.Entity
                };

                return collector.AddHit(hit);
            }
            return false;
        }

        // Mesh
        private unsafe struct RayMeshLeafProcessor : BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            private readonly Mesh* m_Mesh;
            private readonly uint m_NumColliderKeyBits;

            public RayMeshLeafProcessor(MeshCollider* meshCollider)
            {
                m_Mesh = &meshCollider->Mesh;
                m_NumColliderKeyBits = meshCollider->NumColliderKeyBits;
            }

            public bool RayLeaf<T>(RaycastInput input, int primitiveKey, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out fp3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter, out Material material);

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitiveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                bool acceptHit = false;
                fp3 unnormalizedNormal;

                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    fp fraction = collector.MaxFraction;
                    bool hadHit;
                    if (isQuad)
                    {
                        hadHit = RayQuad(input.Ray.Origin, input.Ray.Displacement, vertices[0], vertices[1], vertices[2], vertices[3], ref fraction, out unnormalizedNormal);
                    }
                    else
                    {
                        hadHit = RayTriangle(input.Ray.Origin, input.Ray.Displacement, vertices[0], vertices[polygonIndex + 1], vertices[polygonIndex + 2], ref fraction, out unnormalizedNormal);
                    }

                    if (hadHit && fraction < collector.MaxFraction)
                    {
                        var normalizedNormal = fpmath.normalize(unnormalizedNormal);

                        var hit = new RaycastHit
                        {
                            Fraction = fraction,
                            Position = Mul(input.QueryContext.WorldFromLocalTransform, input.Ray.Origin + input.Ray.Displacement * fraction),
                            SurfaceNormal = fpmath.mul(input.QueryContext.WorldFromLocalTransform.Rotation, normalizedNormal),
                            RigidBodyIndex = input.QueryContext.RigidBodyIndex,
                            ColliderKey = input.QueryContext.SetSubKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex)),
                            Material = material,
                            Entity = input.QueryContext.Entity
                        };

                        acceptHit |= collector.AddHit(hit);
                    }
                }
                return acceptHit;
            }
        }

        private static unsafe bool RayMesh<T>(RaycastInput input, MeshCollider* meshCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            var leafProcessor = new RayMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
        }

        // Compound

        private unsafe struct RayCompoundLeafProcessor : BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            private readonly CompoundCollider* m_CompoundCollider;

            public RayCompoundLeafProcessor(CompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
                {
                    return false;
                }

                MTransform compoundFromChild = new MTransform(child.CompoundFromChild);

                // Transform the ray into child space
                RaycastInput inputLs = input;
                {
                    MTransform childFromCompound = Inverse(compoundFromChild);
                    inputLs.Ray.Origin = Mul(childFromCompound, input.Ray.Origin);
                    inputLs.Ray.Displacement = fpmath.mul(childFromCompound.Rotation, input.Ray.Displacement);
                    inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
                    inputLs.QueryContext.WorldFromLocalTransform = Mul(input.QueryContext.WorldFromLocalTransform, compoundFromChild);
                }

                return child.Collider->CastRay(inputLs, ref collector);
            }
        }

        private static unsafe bool RayCompound<T>(RaycastInput input, CompoundCollider* compoundCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, compoundCollider->Filter))
            {
                return false;
            }

            var leafProcessor = new RayCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
        }

        private static unsafe bool RayTerrain<T>(RaycastInput input, TerrainCollider* terrainCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            ref var terrain = ref terrainCollider->Terrain;
            Material material = terrainCollider->Material;

            bool hadHit = false;

            // Transform the ray into heightfield space
            var ray = new Ray
            {
                Origin = input.Ray.Origin * terrain.InverseScale,
                Displacement = input.Ray.Displacement * terrain.InverseScale
            };
            Terrain.QuadTreeWalker walker;
            {
                fp3 maxDisplacement = ray.Displacement * collector.MaxFraction;
                var rayAabb = new Aabb
                {
                    Min = ray.Origin + fpmath.min(maxDisplacement, fp3.zero),
                    Max = ray.Origin + fpmath.max(maxDisplacement, fp3.zero),
                };
                walker = new Terrain.QuadTreeWalker(&terrainCollider->Terrain, rayAabb);
            }

            // Traverse the tree
            while (walker.Pop())
            {
                bool4 hitMask = walker.Bounds.Raycast(ray, collector.MaxFraction, out fp4 hitFractions);
                hitMask &= (walker.Bounds.Ly <= walker.Bounds.Hy); // Mask off empty children
                if (walker.IsLeaf)
                {
                    // Leaf node, raycast against hit child quads
                    int4 hitIndex;
                    int hitCount = math.compress((int*)(&hitIndex), 0, new int4(0, 1, 2, 3), hitMask);
                    for (int iHit = 0; iHit < hitCount; iHit++)
                    {
                        // Get the quad vertices
                        walker.GetQuad(hitIndex[iHit], out int2 quadIndex, out fp3 a, out fp3 b, out fp3 c, out fp3 d);

                        // Test each triangle in the quad
                        for (int iTriangle = 0; iTriangle < 2; iTriangle++)
                        {
                            // Cast
                            fp fraction = collector.MaxFraction;
                            bool triangleHit = RayTriangle(input.Ray.Origin, input.Ray.Displacement, a, b, c, ref fraction, out fp3 unnormalizedNormal);

                            if (triangleHit && fraction < collector.MaxFraction)
                            {
                                var normalizedNormal = fpmath.normalize(unnormalizedNormal);

                                var hit = new RaycastHit
                                {
                                    Fraction = fraction,
                                    Position = Mul(input.QueryContext.WorldFromLocalTransform, input.Ray.Origin + input.Ray.Displacement * fraction),
                                    SurfaceNormal = fpmath.mul(input.QueryContext.WorldFromLocalTransform.Rotation, normalizedNormal),
                                    RigidBodyIndex = input.QueryContext.RigidBodyIndex,
                                    ColliderKey = input.QueryContext.SetSubKey(terrain.NumColliderKeyBits, terrain.GetSubKey(quadIndex, iTriangle)),
                                    Material = material,
                                    Entity = input.QueryContext.Entity
                                };

                                hadHit |= collector.AddHit(hit);

                                if (collector.EarlyOutOnFirstHit && hadHit)
                                {
                                    return true;
                                }
                            }

                            // Next triangle
                            a = c;
                            c = d;
                        }
                    }
                }
                else
                {
                    // Interior node, add hit child nodes to the stack
                    walker.Push(hitMask);
                }
            }

            return hadHit;
        }

        #endregion
    }
}
