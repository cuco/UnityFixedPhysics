using System;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    // A convex hull.
    // Warning: This is just the header, the hull's variable sized data follows it in memory.
    // Therefore this struct must always be passed by reference, never by value.
    struct ConvexHull
    {
        public struct Face : IEquatable<Face>
        {
            public short FirstIndex;             // index into FaceVertexIndices array
            public byte NumVertices;             // number of vertex indices in the FaceVertexIndices array
            public byte MinHalfAngleCompressed;  // 0-255 = 0-90 degrees

            static readonly fp k_CompressionFactor = 255 / (fp.Pi * fp.half);
            public fp MinHalfAngle { set => MinHalfAngleCompressed = (byte)fpmath.min(value * k_CompressionFactor, (fp)255); }
            public bool Equals(Face other) => FirstIndex.Equals(other.FirstIndex) && NumVertices.Equals(other.NumVertices) && MinHalfAngleCompressed.Equals(other.MinHalfAngleCompressed);
        }

        [StructLayout(LayoutKind.Sequential, Size = 4)] // extra 1 byte padding to match Havok
        public struct Edge : IEquatable<Edge>
        {
            public short FaceIndex;             // index into Faces array
            public byte EdgeIndex;              // edge index within the face

            public bool Equals(Edge other) => FaceIndex.Equals(other.FaceIndex) && EdgeIndex.Equals(other.EdgeIndex);

            public override int GetHashCode() => unchecked((ushort)FaceIndex | (EdgeIndex << 16));
        }

        // A distance by which to inflate the surface of the hull for collision detection.
        // This helps to keep the actual hulls from overlapping during simulation, which avoids more costly algorithms.
        // For spheres and capsules, this is the radius of the primitive.
        // For other convex hulls, this is typically a small value.
        // For polygons in a static mesh, this is typically zero.
        public fp ConvexRadius;

        // Relative arrays of convex hull data
        internal BlobArray VerticesBlob;
        internal BlobArray FacePlanesBlob;
        internal BlobArray FacesBlob;
        internal BlobArray FaceVertexIndicesBlob;
        internal BlobArray FaceLinksBlob;
        internal BlobArray VertexEdgesBlob;

        public int NumVertices => VerticesBlob.Length;
        public int NumFaces => FacesBlob.Length;

        // Indexers for the data
        public BlobArray.Accessor<fp3> Vertices => new BlobArray.Accessor<fp3>(ref VerticesBlob);
        public BlobArray.Accessor<Edge> VertexEdges => new BlobArray.Accessor<Edge>(ref VertexEdgesBlob);
        public BlobArray.Accessor<Face> Faces => new BlobArray.Accessor<Face>(ref FacesBlob);
        public BlobArray.Accessor<Plane> Planes => new BlobArray.Accessor<Plane>(ref FacePlanesBlob);
        public BlobArray.Accessor<byte> FaceVertexIndices => new BlobArray.Accessor<byte>(ref FaceVertexIndicesBlob);
        public BlobArray.Accessor<Edge> FaceLinks => new BlobArray.Accessor<Edge>(ref FaceLinksBlob);

        public unsafe fp3* VerticesPtr => (fp3*)((byte*)UnsafeUtility.AddressOf(ref VerticesBlob.Offset) + VerticesBlob.Offset);
        public unsafe byte* FaceVertexIndicesPtr => (byte*)UnsafeUtility.AddressOf(ref FaceVertexIndicesBlob.Offset) + FaceVertexIndicesBlob.Offset;

        // Returns the index of the face with maximum normal dot direction
        public int GetSupportingFace(fp3 direction)
        {
            int bestIndex = 0;
            fp bestDot = fpmath.dot(direction, Planes[0].Normal);
            for (int i = 1; i < NumFaces; i++)
            {
                fp dot = fpmath.dot(direction, Planes[i].Normal);
                if (dot > bestDot)
                {
                    bestDot = dot;
                    bestIndex = i;
                }
            }
            return bestIndex;
        }

        // Returns the index of the best supporting face that contains supportingVertex
        public int GetSupportingFace(fp3 direction, int supportingVertexIndex)
        {
            // Special case for for polygons or colliders without connectivity.
            // Polygons don't need to search edges because both faces contain all vertices.
            if (Faces.Length == 2 || VertexEdges.Length == 0 || FaceLinks.Length == 0)
            {
                return GetSupportingFace(direction);
            }

            // Search the edges that contain supportingVertexIndex for the one that is most perpendicular to direction
            int bestEdgeIndex = -1;
            {
                fp bestEdgeDot = fp.max_value;
                fp3 supportingVertex = Vertices[supportingVertexIndex];
                Edge edge = VertexEdges[supportingVertexIndex];
                int firstFaceIndex = edge.FaceIndex;
                Face face = Faces[firstFaceIndex];
                while (true)
                {
                    // Get the linked edge and test it against the support direction
                    int linkedEdgeIndex = face.FirstIndex + edge.EdgeIndex;
                    edge = FaceLinks[linkedEdgeIndex];
                    face = Faces[edge.FaceIndex];
                    fp3 linkedVertex = Vertices[FaceVertexIndices[face.FirstIndex + edge.EdgeIndex]];
                    fp3 edgeDirection = linkedVertex - supportingVertex;
                    fp dot = fpmath.abs(fpmath.dot(direction, edgeDirection)) * fpmath.rsqrt(fpmath.lengthsq(edgeDirection));
                    bestEdgeIndex = math.select(bestEdgeIndex, linkedEdgeIndex, dot < bestEdgeDot);
                    bestEdgeDot = fpmath.min(bestEdgeDot, dot);

                    // Quit after looping back to the first face
                    if (edge.FaceIndex == firstFaceIndex)
                    {
                        break;
                    }

                    // Get the next edge
                    edge.EdgeIndex = (byte)((edge.EdgeIndex + 1) % face.NumVertices);
                }
                // If no suitable face is found then return the first face containing the supportingVertex
                if (-1 == bestEdgeIndex) return firstFaceIndex;
            }

            // Choose the face containing the best edge that is most parallel to the support direction
            Edge bestEdge = FaceLinks[bestEdgeIndex];
            int faceIndex0 = bestEdge.FaceIndex;
            int faceIndex1 = FaceLinks[Faces[faceIndex0].FirstIndex + bestEdge.EdgeIndex].FaceIndex;
            fp3 normal0 = Planes[faceIndex0].Normal;
            fp3 normal1 = Planes[faceIndex1].Normal;
            return math.select(faceIndex0, faceIndex1, fpmath.dot(direction, normal1) > fpmath.dot(direction, normal0));
        }

        internal fp CalculateBoundingRadius(fp3 pivot)
        {
            // Find the furthest point from the pivot
            fp maxDistanceSq = fp.zero;
            for (int i = 0; i < NumVertices; i++)
            {
                fp3 vertex = Vertices[i].xyz;
                fp distanceSq = fpmath.lengthsq(vertex - pivot);
                maxDistanceSq = fpmath.max(maxDistanceSq, distanceSq);
            }
            return fpmath.sqrt(maxDistanceSq) + ConvexRadius;
        }
    }
}
