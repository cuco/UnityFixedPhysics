using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using Random = Unity.Mathematics.Random;

namespace Fixed.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="MeshCollider"/>
    /// </summary>
    class MeshColliderTests
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        /// <summary>
        /// Create <see cref="MeshCollider"/> with invalid triangle indices
        /// and ensure that the invalid index is detected
        /// </summary>
        [Test]
        public void MeshCollider_Create_WhenTriangleIndexOutOfRange_Throws()
        {
            int numTriangles = 10;
            var vertices = new NativeArray<fp3>(numTriangles * 3, Allocator.Persistent);
            var triangles = new NativeArray<int3>(numTriangles, Allocator.Persistent);

            try
            {
                for (int i = 0; i < numTriangles; i++)
                {
                    var firstVertexIndex = i * 3;

                    vertices[firstVertexIndex]     = new fp3((fp)firstVertexIndex    , (fp)1f * (fp)(firstVertexIndex % 2)      , (fp)
                        (firstVertexIndex + 1));
                    vertices[firstVertexIndex + 1] = new fp3((fp)(firstVertexIndex + 1), (fp)1f * (fp)((firstVertexIndex + 1) % 2), (fp)
                        (firstVertexIndex + 2));
                    vertices[firstVertexIndex + 2] = new fp3((fp)(firstVertexIndex + 2), (fp)1f * (fp)((firstVertexIndex + 2) % 2), (fp)
                        (firstVertexIndex + 3));
                    triangles[i] = new int3(firstVertexIndex, firstVertexIndex + 1, firstVertexIndex + 2);
                }

                Random rnd = new Random(0x12345678);

                for (int i = 0; i < 100; i++)
                {
                    int indexToChange = rnd.NextInt(0, triangles.Length * 3 - 1);

                    int triangleIndex = indexToChange / 3;
                    int vertexInTriangle = indexToChange % 3;
                    int invalidValue = rnd.NextInt() * (rnd.NextBool() ? -1 : 1);

                    var triangle = triangles[triangleIndex];
                    triangle[vertexInTriangle] = invalidValue;
                    triangles[triangleIndex] = triangle;

                    Assert.Throws<ArgumentException>(() => MeshCollider.Create(vertices, triangles));

                    triangle[vertexInTriangle] = indexToChange;
                    triangles[triangleIndex] = triangle;
                }
            }
            finally
            {
                triangles.Dispose();
                vertices.Dispose();
            }
        }

#endif
    }
}
