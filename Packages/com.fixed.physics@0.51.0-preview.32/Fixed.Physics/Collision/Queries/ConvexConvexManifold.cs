using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine.Assertions;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // Low level convex-convex contact manifold query implementations
    internal static class ConvexConvexManifoldQueries
    {
        // The output of convex-convex manifold queries
        public unsafe struct Manifold
        {
            public int NumContacts;
            public fp3 Normal;

            public const int k_MaxNumContacts = 32;
            private fixed long /* float */ m_ContactPositions[k_MaxNumContacts * 3];
            private fixed long /* float */ m_Distances[k_MaxNumContacts];

            // Create a single point manifold from a distance query result
            public Manifold(DistanceQueries.Result convexDistance, MTransform worldFromA)
            {
                NumContacts = 1;
                Normal = fpmath.mul(worldFromA.Rotation, convexDistance.NormalInA);
                this[0] = new ContactPoint
                {
                    Distance = convexDistance.Distance,
                    Position = Mul(worldFromA, convexDistance.PositionOnBinA)
                };
            }

            public ContactPoint this[int contactIndex]
            {
                get
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 3;
                    var contact = new ContactPoint();

                    fixed(long* positions = m_ContactPositions)
                    {
                        contact.Position = *(fp3*)(positions + offset);
                    }

                    fixed(long* distances = m_Distances)
                    {
                        contact.Distance = fp.FromRaw(distances[contactIndex]);
                    }

                    return contact;
                }
                set
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 3;
                    fixed(long* positions = m_ContactPositions)
                    {
                        *(fp3*)(positions + offset) = value.Position;
                    }

                    fixed(long* distances = m_Distances)
                    {
                        distances[contactIndex] = value.Distance.RawValue;
                    }
                }
            }

            public void Flip()
            {
                for (int i = 0; i < NumContacts; i++)
                {
                    ContactPoint contact = this[i];
                    contact.Position += Normal * contact.Distance;
                    this[i] = contact;
                }
                Normal = -Normal;
            }
        }

        #region Convex vs convex

        // Create a contact point for a pair of spheres in world space.
        public static unsafe void SphereSphere(
            SphereCollider* sphereA, SphereCollider* sphereB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.SphereSphere(sphereA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a box and a sphere in world space.
        public static unsafe void BoxSphere(
            [NoAlias] BoxCollider* boxA, [NoAlias] SphereCollider* sphereB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.BoxSphere(boxA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of boxes in world space.
        public static unsafe void BoxBox(
            BoxCollider* boxA, BoxCollider* boxB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            manifold = new Manifold();

            // Get transforms with box center at origin
            MTransform bFromBoxB = new MTransform(boxB->Orientation, boxB->Center);
            MTransform aFromBoxA = new MTransform(boxA->Orientation, boxA->Center);
            MTransform boxAFromBoxB = Mul(Inverse(aFromBoxA), Mul(aFromB, bFromBoxB));
            MTransform boxBFromBoxA = Inverse(boxAFromBoxB);

            fp3 halfExtentsA = boxA->Size * fp.half;
            fp3 halfExtentsB = boxB->Size * fp.half;

            // Test planes of each box against the other's vertices
            fp3 normal; // in BoxA-space
            fp distance;
            {
                fp3 normalA = new fp3(fp.one, fp.zero, fp.zero);
                fp3 normalB = new fp3(fp.one, fp.zero, fp.zero);
                fp distA = fp.zero;
                fp distB = fp.zero;
                if (!PointPlanes(boxAFromBoxB, halfExtentsA, halfExtentsB, maxDistance, ref normalA, ref distA) ||
                    !PointPlanes(boxBFromBoxA, halfExtentsB, halfExtentsA, maxDistance, ref normalB, ref distB))
                {
                    return;
                }

                normalB = fpmath.mul(boxAFromBoxB.Rotation, normalB);
                bool aGreater = distA > distB;
                normal = fpmath.select(-normalB, normalA, (bool3)aGreater);
                distance = fpmath.select(distB, distA, aGreater);
            }

            // Test edge pairs
            {
                fp3 edgeA = new fp3(fp.one, fp.zero, fp.zero);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        fp3 edgeB;
                        switch (j)
                        {
                            case 0: edgeB = boxAFromBoxB.Rotation.c0; break;
                            case 1: edgeB = boxAFromBoxB.Rotation.c1; break;
                            case 2: edgeB = boxAFromBoxB.Rotation.c2; break;
                            default: edgeB = new fp3(fp.zero); break;
                        }
                        fp3 dir = fpmath.cross(edgeA, edgeB);

                        // hack around parallel edges
                        if (math.all(fpmath.abs(dir) < fp.fp_1e_5f))
                        {
                            continue;
                        }

                        fp3 edgeNormal = fpmath.normalize(dir);
                        fp3 supportA = fpmath.select(halfExtentsA, -halfExtentsA, dir < new fp3(fp.zero));
                        fp maxA = fpmath.abs(fpmath.dot(supportA, edgeNormal));
                        fp minA = -maxA;
                        fp3 dirInB = fpmath.mul(boxBFromBoxA.Rotation, dir);
                        fp3 supportBinB = fpmath.select(halfExtentsB, -halfExtentsB, dirInB < new fp3(fp.zero));
                        fp3 supportB = fpmath.mul(boxAFromBoxB.Rotation, supportBinB);
                        fp offsetB = fpmath.abs(fpmath.dot(supportB, edgeNormal));
                        fp centerB = fpmath.dot(boxAFromBoxB.Translation, edgeNormal);
                        fp maxB = centerB + offsetB;
                        fp minB = centerB - offsetB;

                        fp2 diffs = new fp2(minB - maxA, minA - maxB); // positive normal, negative normal
                        if (math.all(diffs > new fp2(maxDistance)))
                        {
                            return;
                        }

                        if (diffs.x > distance)
                        {
                            distance = diffs.x;
                            normal = -edgeNormal;
                        }

                        if (diffs.y > distance)
                        {
                            distance = diffs.y;
                            normal = edgeNormal;
                        }
                    }

                    edgeA = edgeA.zxy;
                }
            }

            if (distance < maxDistance)
            {
                // Get the normal and supporting faces
                fp3 normalInA = fpmath.mul(boxA->Orientation, normal);
                manifold.Normal = fpmath.mul(worldFromA.Rotation, normalInA);
                int faceIndexA = boxA->ConvexHull.GetSupportingFace(-normalInA);
                int faceIndexB = boxB->ConvexHull.GetSupportingFace(fpmath.mul(fpmath.transpose(aFromB.Rotation), normalInA));

                // Build manifold
                if (!FaceFace(ref boxA->ConvexHull, ref boxB->ConvexHull, faceIndexA, faceIndexB, worldFromA, aFromB, normalInA, distance, ref manifold))
                {
                    // The closest points are vertices, we need GJK to find them
                    ConvexConvex(
                        ref ((ConvexCollider*)boxA)->ConvexHull, ref ((ConvexCollider*)boxB)->ConvexHull,
                        worldFromA, aFromB, maxDistance, out manifold);
                }
            }
        }

        // Create a single point manifold between a capsule and a sphere in world space.
        public static unsafe void CapsuleSphere(
            [NoAlias] CapsuleCollider* capsuleA, [NoAlias] SphereCollider* sphereB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleSphere(
                capsuleA->Vertex0, capsuleA->Vertex1, capsuleA->Radius, sphereB->Center, sphereB->Radius, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a pair of capsules in world space.
        public static unsafe void CapsuleCapsule(
            CapsuleCollider* capsuleA, CapsuleCollider* capsuleB,
            [NoAlias] MTransform worldFromA, [NoAlias] MTransform aFromB, fp maxDistance,
            out Manifold manifold)
        {
            // TODO: Should produce a multi-point manifold
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleCapsule(capsuleA, capsuleB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a box and triangle in world space.
        public static unsafe void BoxTriangle(
            [NoAlias] BoxCollider* boxA, [NoAlias] PolygonCollider* triangleB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            Assert.IsTrue(triangleB->Vertices.Length == 3);

            // Get triangle in box space
            MTransform aFromBoxA = new MTransform(boxA->Orientation, boxA->Center);
            MTransform boxAFromB = Mul(Inverse(aFromBoxA), aFromB);
            fp3 t0 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[0]);
            fp3 t1 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[1]);
            fp3 t2 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[2]);

            Plane triPlane = triangleB->ConvexHull.Planes[0];
            fp3 triangleNormal = fpmath.mul(boxAFromB.Rotation, triPlane.Normal);
            FourTransposedPoints vertsB;
            FourTransposedPoints edgesB;
            FourTransposedPoints perpsB;
            CalcTrianglePlanes(t0, t1, t2, triangleNormal, out vertsB, out edgesB, out perpsB);

            fp3 halfExtents = boxA->Size * fp.half + maxDistance;

            // find the closest minkowski plane
            fp4 plane;
            {
                // Box face vs triangle vertex
                fp4 planeFaceVertex;
                {
                    // get aabb of minkowski diff
                    fp3 tMin = fpmath.min(fpmath.min(t0, t1), t2) - halfExtents;
                    fp3 tMax = fpmath.max(fpmath.max(t0, t1), t2) + halfExtents;

                    // find the aabb face closest to the origin
                    fp3 axis0 = new fp3(fp.one, fp.zero, fp.zero);
                    fp3 axis1 = axis0.zxy; // 010
                    fp3 axis2 = axis0.yzx; // 001

                    fp4 planeX = SelectMaxW(new fp4(axis0, -tMax.x), new fp4(-axis0, tMin.x));
                    fp4 planeY = SelectMaxW(new fp4(axis1, -tMax.y), new fp4(-axis1, tMin.y));
                    fp4 planeZ = SelectMaxW(new fp4(axis2, -tMax.z), new fp4(-axis2, tMin.z));

                    planeFaceVertex = SelectMaxW(planeX, planeY);
                    planeFaceVertex = SelectMaxW(planeFaceVertex, planeZ);
                }

                // Box vertex vs triangle face
                fp4 planeVertexFace;
                {
                    // Calculate the triangle normal
                    fp triangleOffset = fpmath.dot(triangleNormal, t0);
                    fp expansionOffset = fpmath.dot(fpmath.abs(triangleNormal), halfExtents);
                    planeVertexFace = SelectMaxW(
                        new fp4(triangleNormal, -triangleOffset - expansionOffset),
                        new fp4(-triangleNormal, triangleOffset - expansionOffset));
                }

                // Edge planes
                fp4 planeEdgeEdge = new fp4(fp.zero, fp.zero, fp.zero, -fp.max_value);
                {
                    // Test the planes from crossing axis i with each edge of the triangle, for example if i = 1 then n0 is from (0, 1, 0) x (t1 - t0).
                    for (int i = 0, j = 1, k = 2; i < 3; j = k, k = i, i++)
                    {
                        // Normalize the cross product and flip it to point outward from the edge
                        fp4 lengthsSq = edgesB.GetComponent(j) * edgesB.GetComponent(j) + edgesB.GetComponent(k) * edgesB.GetComponent(k);
                        fp4 invLengths = fpmath.rsqrt(lengthsSq);
                        fp4 dots = edgesB.GetComponent(j) * perpsB.GetComponent(k) - edgesB.GetComponent(k) * perpsB.GetComponent(j);
                        fp4 factors = invLengths * fpmath.sign(dots);

                        fp4 nj = -edgesB.GetComponent(k) * factors;
                        fp4 nk = edgesB.GetComponent(j) * factors;
                        fp4 distances = -nj * vertsB.GetComponent(j) - nk * vertsB.GetComponent(k) - fpmath.abs(nj) * halfExtents[j] - fpmath.abs(nk) * halfExtents[k];

                        // If the box edge is parallel to the triangle face then skip it, the plane is redundant with a vertex-face plane
                        bool4 valid = dots != fp4.zero;
                        distances = fpmath.select(Constants.Min4F, distances, valid);

                        fp3 n0 = new fp3(); n0[i] = fp.zero; n0[j] = nj[0]; n0[k] = nk[0];
                        fp3 n1 = new fp3(); n1[i] = fp.zero; n1[j] = nj[1]; n1[k] = nk[1];
                        fp3 n2 = new fp3(); n2[i] = fp.zero; n2[j] = nj[2]; n2[k] = nk[2];
                        fp4 temp = SelectMaxW(SelectMaxW(new fp4(n0, distances.x), new fp4(n1, distances.y)), new fp4(n2, distances.z));
                        planeEdgeEdge = SelectMaxW(planeEdgeEdge, temp);
                    }
                }

                plane = SelectMaxW(SelectMaxW(planeFaceVertex, planeVertexFace), planeEdgeEdge);
            }

            manifold = new Manifold();

            // Check for a separating plane TODO.ma could early out as soon as any plane with w>0 is found
            if (plane.w <= fp.zero)
            {
                // Get the normal and supporting faces
                fp3 normalInA = fpmath.mul(boxA->Orientation, plane.xyz);
                manifold.Normal = fpmath.mul(worldFromA.Rotation, normalInA);
                int faceIndexA = boxA->ConvexHull.GetSupportingFace(-normalInA);
                int faceIndexB = triangleB->ConvexHull.GetSupportingFace(fpmath.mul(fpmath.transpose(aFromB.Rotation), normalInA));

                // Build manifold
                if (!FaceFace(ref boxA->ConvexHull, ref triangleB->ConvexHull, faceIndexA, faceIndexB, worldFromA, aFromB, normalInA, fp.max_value, ref manifold))
                {
                    // The closest points are vertices, we need GJK to find them
                    ConvexConvex(
                        ref ((ConvexCollider*)boxA)->ConvexHull, ref ((ConvexCollider*)triangleB)->ConvexHull,
                        worldFromA, aFromB, maxDistance, out manifold);
                }
            }
        }

        // Create a single point manifold between a triangle and sphere in world space.
        public static unsafe void TriangleSphere(
            [NoAlias] PolygonCollider* triangleA, [NoAlias] SphereCollider* sphereB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            Assert.IsTrue(triangleA->Vertices.Length == 3);

            DistanceQueries.Result convexDistance = DistanceQueries.TriangleSphere(
                triangleA->Vertices[0], triangleA->Vertices[1], triangleA->Vertices[2], triangleA->Planes[0].Normal,
                sphereB->Center, sphereB->Radius, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a capsule and triangle in world space.
        public static unsafe void CapsuleTriangle(
            [NoAlias] CapsuleCollider* capsuleA, [NoAlias] PolygonCollider* triangleB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            Assert.IsTrue(triangleB->Vertices.Length == 3);

            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleTriangle(capsuleA, triangleB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                // Build manifold
                manifold = new Manifold
                {
                    Normal = fpmath.mul(worldFromA.Rotation, -convexDistance.NormalInA) // negate the normal because we are temporarily flipping to triangle A capsule B
                };
                MTransform worldFromB = Mul(worldFromA, aFromB);
                MTransform bFromA = Inverse(aFromB);
                fp3 normalInB = fpmath.mul(bFromA.Rotation, convexDistance.NormalInA);
                int faceIndexB = triangleB->ConvexHull.GetSupportingFace(normalInB);
                if (FaceEdge(ref triangleB->ConvexHull, ref capsuleA->ConvexHull, faceIndexB, worldFromB, bFromA, -normalInB, convexDistance.Distance + capsuleA->Radius, ref manifold))
                {
                    manifold.Flip();
                }
                else
                {
                    manifold = new Manifold(convexDistance, worldFromA);
                }
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of generic convex hulls in world space.
        public static unsafe void ConvexConvex(
            ref ConvexHull hullA, ref ConvexHull hullB,
            [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB, fp maxDistance,
            [NoAlias] out Manifold manifold)
        {
            // Get closest points on the hulls
            ConvexConvexDistanceQueries.Result result = ConvexConvexDistanceQueries.ConvexConvex(
                hullA.VerticesPtr, hullA.NumVertices, hullB.VerticesPtr, hullB.NumVertices, aFromB, ConvexConvexDistanceQueries.PenetrationHandling.Exact3D);

            fp sumRadii = hullB.ConvexRadius + hullA.ConvexRadius;
            if (result.ClosestPoints.Distance < maxDistance + sumRadii)
            {
                fp3 normal = result.ClosestPoints.NormalInA;

                manifold = new Manifold
                {
                    Normal = fpmath.mul(worldFromA.Rotation, normal)
                };

                if (hullA.NumFaces > 0)
                {
                    int faceIndexA = hullA.GetSupportingFace(-normal, result.SimplexVertexA(0));
                    if (hullB.NumFaces > 0)
                    {
                        // Convex vs convex
                        int faceIndexB = hullB.GetSupportingFace(fpmath.mul(fpmath.transpose(aFromB.Rotation), normal), result.SimplexVertexB(0));
                        if (FaceFace(ref hullA, ref hullB, faceIndexA, faceIndexB, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    }
                    else if (hullB.NumVertices == 2)
                    {
                        // Convex vs capsule
                        if (FaceEdge(ref hullA, ref hullB, faceIndexA, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    } // Else convex vs sphere
                }
                else if (hullA.NumVertices == 2)
                {
                    if (hullB.NumFaces > 0)
                    {
                        // Capsule vs convex
                        manifold.Normal = fpmath.mul(worldFromA.Rotation, -normal); // negate the normal because we are temporarily flipping to triangle A capsule B
                        MTransform worldFromB = Mul(worldFromA, aFromB);
                        MTransform bFromA = Inverse(aFromB);
                        fp3 normalInB = fpmath.mul(bFromA.Rotation, normal);
                        int faceIndexB = hullB.GetSupportingFace(normalInB, result.SimplexVertexB(0));
                        bool foundClosestPoint = FaceEdge(ref hullB, ref hullA, faceIndexB, worldFromB, bFromA, -normalInB, result.ClosestPoints.Distance, ref manifold);
                        manifold.Flip();
                        if (foundClosestPoint)
                        {
                            return;
                        }
                    } // Else capsule vs capsule or sphere
                } // Else sphere vs something

                // Either one of the shapes is a sphere, or both of the shapes are capsules, or both of the closest features are nearly perpendicular to the contact normal,
                // or FaceFace()/FaceEdge() missed the closest point due to numerical error.  In these cases, add the closest point directly to the manifold.
                if (manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    DistanceQueries.Result convexDistance = result.ClosestPoints;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, convexDistance.PositionOnAinA) - manifold.Normal * (convexDistance.Distance - hullB.ConvexRadius),
                        Distance = convexDistance.Distance - sumRadii
                    };
                }
            }
            else
            {
                manifold = new Manifold();
            }
        }

        #endregion

        #region Helpers

        // BoxBox() helper
        private static bool PointPlanes(MTransform aFromB, fp3 halfExtA, fp3 halfExtB, fp maxDistance, ref fp3 normalOut, ref fp distanceOut)
        {
            // Calculate the AABB of box B in A-space
            Aabb aabbBinA;
            {
                Aabb aabbBinB = new Aabb { Min = -halfExtB, Max = halfExtB };
                aabbBinA = Math.TransformAabb(aFromB, aabbBinB);
            }

            // Check for a miss
            fp3 toleranceHalfExt = halfExtA + maxDistance;
            bool3 miss = (aabbBinA.Min > toleranceHalfExt) | (-toleranceHalfExt > aabbBinA.Max);
            if (math.any(miss))
            {
                return false;
            }

            // Return the normal with minimum separating distance
            fp3 diff0 = aabbBinA.Min - halfExtA; // positive normal
            fp3 diff1 = -aabbBinA.Max - halfExtA; // negative normal
            bool3 greater01 = diff0 > diff1;
            fp3 max01 = fpmath.select(diff1, diff0, greater01);
            distanceOut = fpmath.cmax(max01);

            int axis = IndexOfMaxComponent(max01);
            if (axis == 0)
            {
                normalOut = new fp3(fp.one, fp.zero, fp.zero);
            }
            else if (axis == 1)
            {
                normalOut = new fp3(fp.zero, fp.one, fp.zero);
            }
            else
            {
                normalOut = new fp3(fp.zero, fp.zero, fp.one);
            }
            normalOut = fpmath.select(normalOut, -normalOut, greater01);

            return true;
        }

        // returns the argument with greater w component
        private static fp4 SelectMaxW(fp4 a, fp4 b)
        {
            return fpmath.select(b, a, a.w > b.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CalcTrianglePlanes(fp3 v0, fp3 v1, fp3 v2, fp3 normalDirection,
            [NoAlias] out FourTransposedPoints verts, [NoAlias] out FourTransposedPoints edges, [NoAlias] out FourTransposedPoints perps)
        {
            verts = new FourTransposedPoints(v0, v1, v2, v0);
            edges = verts.V1230 - verts;
            perps = edges.Cross(new FourTransposedPoints(normalDirection));
        }

        #endregion

        #region Multiple contact generation

        // Iterates over the edges of a face
        private unsafe struct EdgeIterator
        {
            // Current edge
            public fp3 Vertex0 { get; private set; }
            public fp3 Vertex1 { get; private set; }
            public fp3 Edge { get; private set; }
            public fp3 Perp { get; private set; }
            public fp Offset { get; private set; }
            public int Index { get; private set; }

            // Face description
            private fp3* vertices;
            private byte* indices;
            private fp3 normal;
            private int count;

            public static unsafe EdgeIterator Begin(fp3* vertices, byte* indices, fp3 normal, int count)
            {
                EdgeIterator iterator = new EdgeIterator();
                iterator.vertices = vertices;
                iterator.indices = indices;
                iterator.normal = normal;
                iterator.count = count;

                iterator.Vertex1 = (indices == null) ? vertices[count - 1] : vertices[indices[count - 1]];
                iterator.update();
                return iterator;
            }

            public bool Valid()
            {
                return Index < count;
            }

            public void Advance()
            {
                Index++;
                if (Valid())
                {
                    update();
                }
            }

            private void update()
            {
                Vertex0 = Vertex1;
                Vertex1 = (indices == null) ? vertices[Index] : vertices[indices[Index]];

                Edge = Vertex1 - Vertex0;
                Perp = fpmath.cross(Edge, normal); // points outwards from face
                Offset = fpmath.dot(Perp, Vertex1);
            }
        }

        // Cast ray originA, directionA against plane normalB, offsetB and update the ray hit fractions
        private static void castRayPlane(fp3 originA, fp3 directionA, fp3 normalB, fp offsetB, ref fp fracEnter, ref fp fracExit)
        {
            // Cast edge A against plane B
            fp start = fpmath.dot(originA, normalB) - offsetB;
            fp diff = fpmath.dot(directionA, normalB);
            fp end = start + diff;
            fp frac = fpmath.select(-start / diff, fp.zero, diff == fp.zero);

            bool startInside = (start <= fp.zero);
            bool endInside = (end <= fp.zero);

            bool enter = !startInside & (frac > fracEnter);
            fracEnter = fpmath.select(fracEnter, frac, enter);

            bool exit = !endInside & (frac < fracExit);
            fracExit = fpmath.select(fracExit, frac, exit);

            bool hit = startInside | endInside;
            fracEnter = fpmath.select(fracExit, fracEnter, hit); // mark invalid with enter <= exit in case of a miss
        }

        // If the rejections of the faces from the contact normal are just barely touching, then FaceFace() might miss the closest points because of numerical error.
        // FaceFace() and FaceEdge() check if they found a point as close as the closest, and if not they return false so that the caller can add it.
        private static fp closestDistanceTolerance => fp.fp_1e_4f;

        // Tries to generate a manifold between a pair of faces.  It can fail in some cases due to numerical accuracy:
        // 1) both faces are nearly perpendicular to the normal
        // 2) the closest features on the shapes are vertices, so that the intersection of the projection of the faces to the plane perpendicular to the normal contains only one point
        // In those cases, FaceFace() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceFace(
            ref ConvexHull convexA, ref ConvexHull convexB, int faceIndexA, int faceIndexB, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB,
            fp3 normal, fp distance, [NoAlias] ref Manifold manifold)
        {
            // Get the plane of each face
            Plane planeA = convexA.Planes[faceIndexA];
            Plane planeB = TransformPlane(aFromB, convexB.Planes[faceIndexB]);

            // Handle cases where one of the faces is nearly perpendicular to the contact normal
            // This gets around divide by zero / numerical problems from dividing collider planes which often contain some error by a very small number, amplifying that error
            fp cosMaxAngle = fp.FromRaw(0x3d4ccccd);
            fp dotA = fpmath.dot(planeA.Normal, normal);
            fp dotB = fpmath.dot(planeB.Normal, normal);
            bool acceptB = true; // true if vertices of B projected onto the face of A are accepted
            if (dotA > -cosMaxAngle)
            {
                // Handle cases where both faces are nearly perpendicular to the contact normal.
                if (dotB < cosMaxAngle)
                {
                    // Both faces are nearly perpendicular to the contact normal, let the caller generate a single contact
                    return false;
                }

                // Face of A is nearly perpendicular to the contact normal, don't try to project vertices onto it
                acceptB = false;
            }
            else if (dotB < cosMaxAngle)
            {
                // Face of B is nearly perpendicular to the normal, so we need to clip the edges of B against face A instead
                MTransform bFromA = Inverse(aFromB);
                fp3 normalInB = fpmath.mul(bFromA.Rotation, -normal);
                MTransform worldFromB = Mul(worldFromA, aFromB);
                bool result = FaceFace(ref convexB, ref convexA, faceIndexB, faceIndexA, worldFromB, bFromA, normalInB, distance, ref manifold);
                manifold.Normal = -manifold.Normal;
                manifold.Flip();
                return result;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Transform vertices of B into A-space
            // Initialize validB, which is true for each vertex of B that is inside face A
            ConvexHull.Face faceA = convexA.Faces[faceIndexA];
            ConvexHull.Face faceB = convexB.Faces[faceIndexB];
            bool* validB = stackalloc bool[faceB.NumVertices];
            fp3* verticesBinA = stackalloc fp3[faceB.NumVertices];
            {
                byte* indicesB = convexB.FaceVertexIndicesPtr + faceB.FirstIndex;
                fp3* verticesB = convexB.VerticesPtr;
                for (int i = 0; i < faceB.NumVertices; i++)
                {
                    validB[i] = acceptB;
                    verticesBinA[i] = Mul(aFromB, verticesB[indicesB[i]]);
                }
            }

            // For each edge of A
            fp invDotB = fpmath.rcp(dotB);
            fp sumRadii = convexA.ConvexRadius + convexB.ConvexRadius;
            byte* indicesA = convexA.FaceVertexIndicesPtr + faceA.FirstIndex;
            fp3* verticesA = convexA.VerticesPtr;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                fp fracEnterA = fp.zero;
                fp fracExitA = fp.one;

                // For each edge of B
                for (EdgeIterator edgeB = EdgeIterator.Begin(verticesBinA, null, normal, faceB.NumVertices); edgeB.Valid(); edgeB.Advance())
                {
                    // Cast edge A against plane B and test if vertex B is inside plane A
                    castRayPlane(edgeA.Vertex0, edgeA.Edge, edgeB.Perp, edgeB.Offset, ref fracEnterA, ref fracExitA);
                    validB[edgeB.Index] &= (fpmath.dot(edgeB.Vertex1, edgeA.Perp) < edgeA.Offset);
                }

                // If edge A hits B, add a contact points
                if (fracEnterA < fracExitA)
                {
                    fp distance0 = (fpmath.dot(edgeA.Vertex0, planeB.Normal) + planeB.Distance) * invDotB;
                    fp deltaDistance = fpmath.dot(edgeA.Edge, planeB.Normal) * invDotB;
                    fp3 vertexAOnB = edgeA.Vertex0 - normal * distance0;
                    fp3 edgeAOnB = edgeA.Edge - normal * deltaDistance;
                    foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracEnterA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    if (fracExitA < fp.one) // If the exit fraction is 1, then the next edge has the same contact point with enter fraction 0
                    {
                        foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracExitA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    }
                }
            }

            // For each vertex of B
            fp invDotA = fpmath.rcp(dotA);
            for (int i = 0; i < faceB.NumVertices; i++)
            {
                if (validB[i] && manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    fp3 vertexB = verticesBinA[i];
                    fp distanceB = (fpmath.dot(vertexB, planeA.Normal) + planeA.Distance) * -invDotA;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, vertexB) + manifold.Normal * convexB.ConvexRadius,
                        Distance = distanceB - sumRadii
                    };
                    foundClosestPoint |= distanceB <= distance;
                }
            }

            return foundClosestPoint;
        }

        // Tries to generate a manifold between a face and an edge.  It can fail for the same reasons as FaceFace().
        // In those cases, FaceEdge() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceEdge(
            ref ConvexHull faceConvexA, ref ConvexHull edgeConvexB, int faceIndexA, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform aFromB,
            fp3 normal, fp distance, [NoAlias] ref Manifold manifold)
        {
            // Check if the face is nearly perpendicular to the normal
            fp cosMaxAngle = fp.FromRaw(0x3d4ccccd);
            Plane planeA = faceConvexA.Planes[faceIndexA];
            fp dotA = fpmath.dot(planeA.Normal, normal);
            if (fpmath.abs(dotA) < cosMaxAngle)
            {
                return false;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Get the supporting face on A
            ConvexHull.Face faceA = faceConvexA.Faces[faceIndexA];
            byte* indicesA = faceConvexA.FaceVertexIndicesPtr + faceA.FirstIndex;

            // Get edge in B
            fp3 vertexB0 = Math.Mul(aFromB, edgeConvexB.Vertices[0]);
            fp3 edgeB = fpmath.mul(aFromB.Rotation, edgeConvexB.Vertices[1] - edgeConvexB.Vertices[0]);

            // For each edge of A
            fp3* verticesA = faceConvexA.VerticesPtr;
            fp fracEnterB = fp.zero;
            fp fracExitB = fp.one;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                // Cast edge B against plane A
                castRayPlane(vertexB0, edgeB, edgeA.Perp, edgeA.Offset, ref fracEnterB, ref fracExitB);
            }

            // If edge B hits A, add a contact points
            if (fracEnterB < fracExitB)
            {
                fp invDotA = fpmath.rcp(dotA);
                fp sumRadii = faceConvexA.ConvexRadius + edgeConvexB.ConvexRadius;
                fp distance0 = (fpmath.dot(vertexB0, planeA.Normal) + planeA.Distance) * -invDotA;
                fp deltaDistance = fpmath.dot(edgeB, planeA.Normal) * -invDotA;
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracEnterB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracExitB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
            }

            return foundClosestPoint;
        }

        // Adds a contact to the manifold from an edge and fraction
        private static bool AddEdgeContact(fp3 vertex0, fp3 edge, fp distance0, fp deltaDistance, fp fraction, fp3 normalInA, fp radiusB, fp sumRadii,
            [NoAlias] in MTransform worldFromA, fp distanceThreshold, [NoAlias] ref Manifold manifold)
        {
            if (manifold.NumContacts < Manifold.k_MaxNumContacts)
            {
                fp3 position = vertex0 + fraction * edge;
                fp distance = distance0 + fraction * deltaDistance;

                manifold[manifold.NumContacts++] = new ContactPoint
                {
                    Position = Mul(worldFromA, position + normalInA * radiusB),
                    Distance = distance - sumRadii
                };

                return distance <= distanceThreshold;
            }
            return false;
        }

        #endregion
    }
}
