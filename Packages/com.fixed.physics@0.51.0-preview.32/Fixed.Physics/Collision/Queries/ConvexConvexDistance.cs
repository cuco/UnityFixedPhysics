using System.Diagnostics;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics
{
    // Low level convex-convex distance query implementations
    static class ConvexConvexDistanceQueries
    {
        // Convex distance result
        public struct Result
        {
            public DistanceQueries.Result ClosestPoints;
            public uint3 Simplex;
            public int Iterations;

            public const uint InvalidSimplexVertex = 0xffffffff;

            public bool Valid => math.all(ClosestPoints.NormalInA == new fp3(0));
            public int SimplexDimension => Simplex.z == InvalidSimplexVertex ? (Simplex.y == InvalidSimplexVertex ? 1 : 2) : 3;

            public int SimplexVertexA(int index) => (int)(Simplex[index] >> 16);
            public int SimplexVertexB(int index) => (int)(Simplex[index] & 0xffff);
        }

        public enum PenetrationHandling
        {
            DotNotCompute,
            Exact3D
        }

        // Supporting vertex
        [DebuggerDisplay("{Xyz}:{Id}")]
        private struct SupportVertex
        {
            public fp3 Xyz;
            public uint Id;

            public int IdA => (int)(Id >> 16);
            public int IdB => (int)(Id & 0xffff);
        }

        // Simplex
        private struct Simplex
        {
            public SupportVertex A, B, C, D;
            public fp3 Direction; // Points from the origin towards the closest point on the simplex
            public fp ScaledDistance; // ClosestPoint = Direction * ScaledDistance / lengthSq(Direction)
            public int NumVertices;

            /// <summary>
            /// Compute the closest point on the simplex, returns true if the simplex contains a duplicate vertex
            /// </summary>
            public void SolveDistance()
            {
                int inputVertices = NumVertices;

                switch (NumVertices)
                {
                    // Point.
                    case 1:
                        Direction = A.Xyz;
                        ScaledDistance = fpmath.lengthsq(Direction);
                        break;

                    // Line.
                    case 2:
                    {
                        fp3 delta = B.Xyz - A.Xyz;
                        fp den = fpmath.dot(delta, delta);
                        fp num = fpmath.dot(-A.Xyz, delta);

                        // Reduce if closest point do not project on the line segment.
                        if (num >= den) { NumVertices = 1; A = B; goto case 1; }

                        // Compute support direction
                        Direction = fpmath.cross(fpmath.cross(delta, A.Xyz), delta);
                        ScaledDistance = fpmath.dot(Direction, A.Xyz);
                    }
                    break;

                    // Triangle.
                    case 3:
                    {
                        fp3 ca = A.Xyz - C.Xyz;
                        fp3 cb = B.Xyz - C.Xyz;
                        fp3 n = fpmath.cross(cb, ca);

                        // Reduce if closest point do not project in the triangle.
                        fp3 crossA = fpmath.cross(cb, n);
                        fp3 crossB = fpmath.cross(n, ca);
                        fp detA = fpmath.dot(crossA, B.Xyz);
                        fp detB = fpmath.dot(crossB, C.Xyz);
                        if (detA < fp.zero)
                        {
                            if (detB >= fp.zero || Det(n, crossA, C.Xyz) < fp.zero)
                            {
                                A = B;
                            }
                        }
                        else if (detB >= fp.zero)
                        {
                            fp dot = fpmath.dot(C.Xyz, n);
                            if (dot < fp.zero)
                            {
                                // Reorder vertices so that n points away from the origin
                                SupportVertex temp = A;
                                A = B;
                                B = temp;
                                n = -n;
                                dot = -dot;
                            }
                            Direction = n;
                            ScaledDistance = dot;
                            break;
                        }

                        B = C;
                        NumVertices = 2;
                        goto case 2;
                    }

                    // Tetrahedra.
                    case 4:
                    {
                        FourTransposedPoints tetra = new FourTransposedPoints(A.Xyz, B.Xyz, C.Xyz, D.Xyz);
                        FourTransposedPoints d = new FourTransposedPoints(D.Xyz);

                        // This routine finds the closest feature to the origin on the tetra by testing the origin against the planes of the
                        // voronoi diagram. If the origin is near the border of two regions in the diagram, then the plane tests might exclude
                        // it from both because of fp rounding.  To avoid this problem we use some tolerance testing the face planes and let
                        // EPA handle those border cases.  1e-5 is a somewhat arbitrary value and the actual distance scales with the tetra, so
                        // this might need to be tuned later!
                        fp3 faceTest = tetra.Cross(tetra.V1203).Dot(d).xyz;
                        if (math.all(faceTest >= -fp.fp_1e_5f))
                        {
                            // Origin is inside the tetra
                            Direction = fp3.zero;
                            break;
                        }

                        // Check if the closest point is on a face
                        bool3 insideFace = (faceTest >= fp.zero).xyz;
                        FourTransposedPoints edges = d - tetra;
                        FourTransposedPoints normals = edges.Cross(edges.V1203);
                        bool3 insideEdge0 = (normals.Cross(edges).Dot(d) >= fp.zero).xyz;
                        bool3 insideEdge1 = (edges.V1203.Cross(normals).Dot(d) >= fp.zero).xyz;
                        bool3 onFace = (insideEdge0 & insideEdge1 & !insideFace);
                        if (math.any(onFace))
                        {
                            if (onFace.y) { A = B; B = C; }
                            else if (onFace.z) { B = C; }
                        }
                        else
                        {
                            // Check if the closest point is on an edge
                            // TODO maybe we can safely drop two vertices in this case
                            bool3 insideVertex = (edges.Dot(d) >= 0).xyz;
                            bool3 onEdge = (!insideEdge0 & !insideEdge1.zxy & insideVertex);
                            if (math.any(onEdge.yz)) { A = B; B = C; }
                        }

                        C = D;
                        NumVertices = 3;
                        goto case 3;
                    }
                }
            }

            // Compute the barycentric coordinates of the closest point.
            public fp4 ComputeBarycentricCoordinates(fp3 closestPoint)
            {
                fp4 coordinates = new fp4(0);
                switch (NumVertices)
                {
                    case 1:
                        coordinates.x = fp.one;
                        break;
                    case 2:
                        fp distance = fpmath.distance(A.Xyz, B.Xyz);
                        UnityEngine.Assertions.Assert.AreNotEqual(distance, fp.zero); // TODO just checking if this happens in my tests
                        if (distance == fp.zero) // Very rare case, simplex is really 1D.
                        {
                            goto case 1;
                        }
                        coordinates.x = fpmath.distance(B.Xyz, closestPoint) / distance;
                        coordinates.y = fp.one - coordinates.x;
                        break;
                    case 3:
                    {
                        coordinates.x = fpmath.length(fpmath.cross(B.Xyz - closestPoint, C.Xyz - closestPoint));
                        coordinates.y = fpmath.length(fpmath.cross(C.Xyz - closestPoint, A.Xyz - closestPoint));
                        coordinates.z = fpmath.length(fpmath.cross(A.Xyz - closestPoint, B.Xyz - closestPoint));
                        fp sum = fpmath.csum(coordinates.xyz);
                        if (sum == fp.zero) // Very rare case, simplex is really 2D.  Happens because of int->float conversion from the hull builder.
                        {
                            // Choose the two farthest apart vertices to keep
                            fp3 lengthsSq = new fp3(fpmath.lengthsq(A.Xyz - B.Xyz), fpmath.lengthsq(B.Xyz - C.Xyz), fpmath.lengthsq(C.Xyz - A.Xyz));
                            bool3 longest = fpmath.cmin(lengthsSq) == lengthsSq;
                            if (longest.y)
                            {
                                A.Xyz = C.Xyz;
                            }
                            else if (longest.z)
                            {
                                A.Xyz = B.Xyz;
                                B.Xyz = C.Xyz;
                            }
                            coordinates.z = fp.zero;
                            NumVertices = 2;
                            goto case 2;
                        }
                        coordinates /= sum;
                        break;
                    }
                    case 4:
                    {
                        coordinates.x = Det(D.Xyz, C.Xyz, B.Xyz);
                        coordinates.y = Det(D.Xyz, A.Xyz, C.Xyz);
                        coordinates.z = Det(D.Xyz, B.Xyz, A.Xyz);
                        coordinates.w = Det(A.Xyz, B.Xyz, C.Xyz);
                        fp sum = fpmath.csum(coordinates.xyzw);
                        UnityEngine.Assertions.Assert.AreNotEqual(sum, fp.zero); // TODO just checking that this doesn't happen in my tests
                        if (sum == fp.zero) // Unexpected case, may introduce significant error by dropping a vertex but it's better than nan
                        {
                            coordinates.zw = fp.zero;
                            NumVertices = 3;
                            goto case 3;
                        }
                        coordinates /= sum;
                        break;
                    }
                }

                return coordinates;
            }
        }

        /// <summary>
        /// Generalized convex-convex distance.
        /// </summary>
        /// <param name="verticesA">Vertices of the first collider in local space</param>
        /// <param name="verticesB">Vertices of the second collider in local space</param>
        /// <param name="aFromB">Transform from the local space of B to the local space of A</param>
        /// <param name="penetrationHandling">How to compute penetration.</param>
        /// <returns></returns>
        public static unsafe Result ConvexConvex(
            fp3* verticesA, int numVerticesA, fp3* verticesB, int numVerticesB,
            [NoAlias] in MTransform aFromB, PenetrationHandling penetrationHandling)
        {
            fp epsTerminationSq = 1e-8M; // Main loop quits when it cannot find a point that improves the simplex by at least this much
            fp epsPenetrationSq = 1e-9M; // Epsilon used to check for penetration.  Should be smaller than shape cast ConvexConvex keepDistance^2.

            // Initialize simplex.
            Simplex simplex = new Simplex();
            simplex.NumVertices = 1;
            simplex.A = GetSupportingVertex(new fp3(fp.one, fp.zero, fp.zero), verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
            simplex.Direction = simplex.A.Xyz;
            simplex.ScaledDistance = fpmath.lengthsq(simplex.A.Xyz);
            fp scaleSq = simplex.ScaledDistance;

            // Iterate.
            int iteration = 0;
            bool penetration = false;
            const int maxIterations = 64;
            for (; iteration < maxIterations; ++iteration)
            {
                // Find a new support vertex
                SupportVertex newSv = GetSupportingVertex(-simplex.Direction, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);

                // If the new vertex is not significantly closer to the origin, quit
                fp scaledImprovement = fpmath.dot(simplex.A.Xyz - newSv.Xyz, simplex.Direction);
                if (scaledImprovement * fpmath.abs(scaledImprovement) < epsTerminationSq * scaleSq)
                {
                    break;
                }

                // Add the new vertex and reduce the simplex
                switch (simplex.NumVertices++)
                {
                    case 1: simplex.B = newSv; break;
                    case 2: simplex.C = newSv; break;
                    default: simplex.D = newSv; break;
                }
                simplex.SolveDistance();

                // Check for penetration
                scaleSq = fpmath.lengthsq(simplex.Direction);
                fp scaledDistanceSq = simplex.ScaledDistance * simplex.ScaledDistance;
                if (simplex.NumVertices == 4 || scaledDistanceSq <= epsPenetrationSq * scaleSq)
                {
                    penetration = true;
                    break;
                }
            }

            // Finalize result.
            var ret = new Result { Iterations = iteration + 1 };

            // Handle penetration.
            if (penetration && penetrationHandling != PenetrationHandling.DotNotCompute)
            {
                // Allocate a hull for EPA
                int verticesCapacity = 64;
                int triangleCapacity = 2 * verticesCapacity;
                ConvexHullBuilder.Vertex* vertices = stackalloc ConvexHullBuilder.Vertex[verticesCapacity];
                ConvexHullBuilder.Triangle* triangles = stackalloc ConvexHullBuilder.Triangle[triangleCapacity];
                Aabb domain = GetSupportingAabb(verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                fp simplificationTolerance = fp.zero;
                var hull = new ConvexHullBuilder(verticesCapacity, vertices, triangles, null,
                    domain, simplificationTolerance, ConvexHullBuilder.IntResolution.Low);

                // Add simplex vertices to the hull, remove any vertices from the simplex that do not increase the hull dimension
                hull.AddPoint(simplex.A.Xyz, simplex.A.Id);
                if (simplex.NumVertices > 1)
                {
                    hull.AddPoint(simplex.B.Xyz, simplex.B.Id);
                    if (simplex.NumVertices > 2)
                    {
                        int dimension = hull.Dimension;
                        hull.AddPoint(simplex.C.Xyz, simplex.C.Id);
                        if (dimension == 0 && hull.Dimension == 1)
                        {
                            simplex.B = simplex.C;
                        }
                        if (simplex.NumVertices > 3)
                        {
                            dimension = hull.Dimension;
                            hull.AddPoint(simplex.D.Xyz, simplex.D.Id);
                            if (dimension > hull.Dimension)
                            {
                                if (dimension == 0)
                                {
                                    simplex.B = simplex.D;
                                }
                                else if (dimension == 1)
                                {
                                    simplex.C = simplex.D;
                                }
                            }
                        }
                    }
                }
                simplex.NumVertices = (hull.Dimension + 1);

                // If the simplex is not 3D, try expanding the hull in all directions
                while (hull.Dimension < 3)
                {
                    // Choose expansion directions
                    fp3 support0, support1, support2;
                    switch (simplex.NumVertices)
                    {
                        case 1:
                            support0 = new fp3(fp.one, fp.zero, fp.zero);
                            support1 = new fp3(fp.zero, fp.one, fp.zero);
                            support2 = new fp3(fp.zero, fp.zero, fp.one);
                            break;
                        case 2:
                            Math.CalculatePerpendicularNormalized(fpmath.normalize(simplex.B.Xyz - simplex.A.Xyz), out support0, out support1);
                            support2 = fp3.zero;
                            break;
                        default:
                            UnityEngine.Assertions.Assert.IsTrue(simplex.NumVertices == 3);
                            support0 = fpmath.cross(simplex.B.Xyz - simplex.A.Xyz, simplex.C.Xyz - simplex.A.Xyz);
                            support1 = fp3.zero;
                            support2 = fp3.zero;
                            break;
                    }

                    // Try each one
                    int numSupports = 4 - simplex.NumVertices;
                    bool success = false;
                    for (int i = 0; i < numSupports; i++)
                    {
                        for (int j = 0; j < 2; j++) // +/- each direction
                        {
                            SupportVertex vertex = GetSupportingVertex(support0, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                            hull.AddPoint(vertex.Xyz, vertex.Id);
                            if (hull.Dimension == simplex.NumVertices)
                            {
                                switch (simplex.NumVertices)
                                {
                                    case 1: simplex.B = vertex; break;
                                    case 2: simplex.C = vertex; break;
                                    default: simplex.D = vertex; break;
                                }

                                // Next dimension
                                success = true;
                                simplex.NumVertices++;
                                i = numSupports;
                                break;
                            }
                            support0 = -support0;
                        }
                        support0 = support1;
                        support1 = support2;
                    }

                    if (!success)
                    {
                        break;
                    }
                }

                // We can still fail to build a tetrahedron if the minkowski difference is really flat.
                // In those cases just find the closest point to the origin on the infinite extension of the simplex (point / line / plane)
                if (hull.Dimension != 3)
                {
                    switch (simplex.NumVertices)
                    {
                        case 1:
                        {
                            ret.ClosestPoints.Distance = fpmath.length(simplex.A.Xyz);
                            ret.ClosestPoints.NormalInA = -fpmath.normalizesafe(simplex.A.Xyz, new fp3(fp.one, fp.zero, fp.zero));
                            break;
                        }
                        case 2:
                        {
                            fp3 edge = fpmath.normalize(simplex.B.Xyz - simplex.A.Xyz);
                            fp3 direction = fpmath.cross(fpmath.cross(edge, simplex.A.Xyz), edge);
                            Math.CalculatePerpendicularNormalized(edge, out fp3 safeNormal, out fp3 unused); // backup, take any direction perpendicular to the edge
                            fp3 normal = fpmath.normalizesafe(direction, safeNormal);
                            ret.ClosestPoints.Distance = fpmath.dot(normal, simplex.A.Xyz);
                            ret.ClosestPoints.NormalInA = -normal;
                            break;
                        }
                        default:
                        {
                            UnityEngine.Assertions.Assert.IsTrue(simplex.NumVertices == 3);
                            fp3 cross = fpmath.cross(simplex.B.Xyz - simplex.A.Xyz, simplex.C.Xyz - simplex.A.Xyz);
                            fp crossLengthSq = fpmath.lengthsq(cross);
                            if (crossLengthSq < 1e-8M) // hull builder can accept extremely thin triangles for which we cannot compute an accurate normal
                            {
                                simplex.NumVertices = 2;
                                goto case 2;
                            }
                            fp3 normal = cross * fpmath.rsqrt(crossLengthSq);
                            fp dot = fpmath.dot(normal, simplex.A.Xyz);
                            ret.ClosestPoints.Distance = fpmath.abs(dot);
                            ret.ClosestPoints.NormalInA = fpmath.select(-normal, normal, dot < fp.zero);
                            break;
                        }
                    }
                }
                else
                {
                    int closestTriangleIndex;
                    Plane closestPlane = new Plane();
                    fp stopThreshold = fp.fp_1e_4f;
                    uint* uidsCache = stackalloc uint[triangleCapacity];
                    for (int i = 0; i < triangleCapacity; i++)
                    {
                        uidsCache[i] = 0;
                    }
                    fp* distancesCache = stackalloc fp[triangleCapacity];
                    do
                    {
                        // Select closest triangle.
                        closestTriangleIndex = -1;
                        foreach (int triangleIndex in hull.Triangles.Indices)
                        {
                            if (hull.Triangles[triangleIndex].Uid != uidsCache[triangleIndex])
                            {
                                uidsCache[triangleIndex] = hull.Triangles[triangleIndex].Uid;
                                distancesCache[triangleIndex] = hull.ComputePlane(triangleIndex).Distance;
                            }
                            if (closestTriangleIndex == -1 || distancesCache[closestTriangleIndex] < distancesCache[triangleIndex])
                            {
                                closestTriangleIndex = triangleIndex;
                            }
                        }
                        closestPlane = hull.ComputePlane(closestTriangleIndex);

                        // Add supporting vertex or exit.
                        SupportVertex sv = GetSupportingVertex(closestPlane.Normal, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                        fp d2P = fpmath.dot(closestPlane.Normal, sv.Xyz) + closestPlane.Distance;
                        if (fpmath.abs(d2P) > stopThreshold && hull.AddPoint(sv.Xyz, sv.Id))
                            stopThreshold *= new fp(1, 3, 10);
                        else
                            break;
                    }
                    while (++iteration < maxIterations);

                    // There could be multiple triangles in the closest plane, pick the one that has the closest point to the origin on its face
                    foreach (int triangleIndex in hull.Triangles.Indices)
                    {
                        if (distancesCache[triangleIndex] >= closestPlane.Distance - fp.fp_1e_4f)
                        {
                            ConvexHullBuilder.Triangle triangle = hull.Triangles[triangleIndex];
                            fp3 a = hull.Vertices[triangle.Vertex0].Position;
                            fp3 b = hull.Vertices[triangle.Vertex1].Position;
                            fp3 c = hull.Vertices[triangle.Vertex2].Position;
                            fp3 cross = fpmath.cross(b - a, c - a);
                            fp3 dets = new fp3(
                                fpmath.dot(fpmath.cross(a - c, cross), a),
                                fpmath.dot(fpmath.cross(b - a, cross), b),
                                fpmath.dot(fpmath.cross(c - b, cross), c));
                            if (math.all(dets >= 0))
                            {
                                Plane plane = hull.ComputePlane(triangleIndex);
                                if (fpmath.dot(plane.Normal, closestPlane.Normal) > (1 - fp.fp_1e_4f))
                                {
                                    closestTriangleIndex = triangleIndex;
                                    closestPlane = hull.ComputePlane(triangleIndex);
                                }
                                break;
                            }
                        }
                    }

                    // Generate simplex.
                    {
                        ConvexHullBuilder.Triangle triangle = hull.Triangles[closestTriangleIndex];
                        simplex.NumVertices = 3;
                        simplex.A.Xyz = hull.Vertices[triangle.Vertex0].Position; simplex.A.Id = hull.Vertices[triangle.Vertex0].UserData;
                        simplex.B.Xyz = hull.Vertices[triangle.Vertex1].Position; simplex.B.Id = hull.Vertices[triangle.Vertex1].UserData;
                        simplex.C.Xyz = hull.Vertices[triangle.Vertex2].Position; simplex.C.Id = hull.Vertices[triangle.Vertex2].UserData;
                        simplex.Direction = -closestPlane.Normal;
                        simplex.ScaledDistance = closestPlane.Distance;

                        // Set normal and distance.
                        ret.ClosestPoints.NormalInA = -closestPlane.Normal;
                        ret.ClosestPoints.Distance = closestPlane.Distance;
                    }
                }
            }
            else
            {
                // Compute distance and normal.
                fp lengthSq = fpmath.lengthsq(simplex.Direction);
                fp invLength = fpmath.rsqrt(lengthSq);
                bool smallLength = lengthSq == fp.zero;
                ret.ClosestPoints.Distance = fpmath.select(simplex.ScaledDistance * invLength, fp.zero, smallLength);
                ret.ClosestPoints.NormalInA = fpmath.select(simplex.Direction * invLength, new fp3(fp.one, fp.zero, fp.zero), smallLength);

                // Make sure the normal is always valid.
                if (!math.all(fpmath.isfinite(ret.ClosestPoints.NormalInA)))
                {
                    ret.ClosestPoints.NormalInA = new fp3(fp.one, fp.zero, fp.zero);
                }
            }

            // Compute position.
            fp3 closestPoint = ret.ClosestPoints.NormalInA * ret.ClosestPoints.Distance;
            fp4 coordinates = simplex.ComputeBarycentricCoordinates(closestPoint);
            ret.ClosestPoints.PositionOnAinA =
                verticesA[simplex.A.IdA] * coordinates.x +
                verticesA[simplex.B.IdA] * coordinates.y +
                verticesA[simplex.C.IdA] * coordinates.z +
                verticesA[simplex.D.IdA] * coordinates.w;

            // Encode simplex.
            ret.Simplex.x = simplex.A.Id;
            ret.Simplex.y = simplex.NumVertices >= 2 ? simplex.B.Id : Result.InvalidSimplexVertex;
            ret.Simplex.z = simplex.NumVertices >= 3 ? simplex.C.Id : Result.InvalidSimplexVertex;

            // Done.
            UnityEngine.Assertions.Assert.IsTrue(fpmath.isfinite(ret.ClosestPoints.Distance));
            UnityEngine.Assertions.Assert.IsTrue(fpmath.abs(fpmath.lengthsq(ret.ClosestPoints.NormalInA) - fp.one) < fp.fp_1e_5f);
            return ret;
        }

        // Returns the supporting vertex index given a direction in local space.
        private static unsafe int GetSupportingVertexIndex(fp3 direction, fp3* vertices, int numVertices)
        {
            int maxI = -1;
            fp maxD = fp.zero;
            for (int i = 0; i < numVertices; ++i)
            {
                fp d = fpmath.dot(direction, vertices[i]);
                if (maxI == -1 || d > maxD)
                {
                    maxI = i;
                    maxD = d;
                }
            }
            return maxI;
        }

        // Returns the supporting vertex of the CSO given a direction in 'A' space.
        private static unsafe SupportVertex GetSupportingVertex(
            fp3 direction, fp3* verticesA, int numVerticesA, fp3* verticesB, int numVerticesB, [NoAlias] in MTransform aFromB)
        {
            int ia = GetSupportingVertexIndex(direction, verticesA, numVerticesA);
            int ib = GetSupportingVertexIndex(fpmath.mul(aFromB.InverseRotation, -direction), verticesB, numVerticesB);
            return new SupportVertex { Xyz = verticesA[ia] - Mul(aFromB, verticesB[ib]), Id = ((uint)ia) << 16 | (uint)ib };
        }

        // Returns an AABB containing the CSO in A-space
        private static unsafe Aabb GetSupportingAabb(
            fp3* verticesA, int numVerticesA, fp3* verticesB, int numVerticesB, MTransform aFromB)
        {
            Aabb aabbA = new Aabb { Min = verticesA[0], Max = verticesA[0] };
            for (int i = 1; i < numVerticesA; i++)
            {
                aabbA.Min = fpmath.min(aabbA.Min, verticesA[i]);
                aabbA.Max = fpmath.max(aabbA.Max, verticesA[i]);
            }

            Aabb aabbB = new Aabb { Min = verticesB[0], Max = verticesB[0] };
            for (int i = 1; i < numVerticesB; i++)
            {
                aabbB.Min = fpmath.min(aabbB.Min, verticesB[i]);
                aabbB.Max = fpmath.max(aabbB.Max, verticesB[i]);
            }

            Aabb aabbBinA = Math.TransformAabb(aFromB, aabbB);
            return new Aabb { Min = aabbA.Min - aabbBinA.Max, Max = aabbA.Max - aabbBinA.Min };
        }
    }
}
