using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;
using Fixed.Physics.Extensions;
using Unity.Mathematics;
using UnityEngine;

namespace Fixed.Physics.Tests.Authoring
{
    [TestFixture(typeof(BoxCollider), typeof(SphereCollider))]
    [TestFixture(typeof(CylinderCollider), typeof(BoxCollider))]
    [TestFixture(typeof(CapsuleCollider), typeof(CylinderCollider))]
    [TestFixture(typeof(SphereCollider), typeof(MeshCollider))]
    [TestFixture(typeof(MeshCollider), typeof(CompoundCollider))]
    [TestFixture(typeof(ConvexCollider), typeof(TerrainCollider))]
    [TestFixture(typeof(TerrainCollider), typeof(MeshCollider))]
    [TestFixture(typeof(CompoundCollider), typeof(CapsuleCollider))]

    unsafe class BlobAssetReferenceColliderExtentions_UnitTests<T, InvalidCast> where T : unmanaged, ICollider where InvalidCast : unmanaged, ICollider
    {
        public static BlobAssetReference<Collider> MakeBox()
        {
            return BoxCollider.Create(new BoxGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.identity,
                Size = new fp3(fp.one, fp.one, fp.one),
                BevelRadius = fp.zero
            });
        }

        public static BlobAssetReference<Collider> MakeCapsule()
        {
            return CapsuleCollider.Create(new CapsuleGeometry { Vertex0 = fpmath.up(), Vertex1 = -fpmath.up(), Radius = fp.one });
        }

        public static BlobAssetReference<Collider> MakeCylinder()
        {
            return CylinderCollider.Create(new CylinderGeometry
            {
                Center = fp3.zero,
                Orientation = fpquaternion.AxisAngle(new fp3(fp.one, fp.zero, fp.zero), (fp)45.0f),
                Height = (fp)2f,
                Radius = (fp)0.25f,
                BevelRadius = (fp)0.05f,
                SideCount = 8
            });
        }

        public static BlobAssetReference<Collider> MakeSphere()
        {
            return SphereCollider.Create(new SphereGeometry { Center = fp3.zero, Radius = fp.one });
        }

        public static BlobAssetReference<Collider> MakeMesh()
        {
            unsafe
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Mesh mesh = go.GetComponent<MeshFilter>().sharedMesh;

                var vertexBuff = mesh.vertices;
                var indexBuff = mesh.triangles;
                var verts = new NativeArray<fp3>(vertexBuff.Length, Allocator.Temp);
                var tris = new NativeArray<int3>(indexBuff.Length / 3, Allocator.Temp);

                fixed(int* indexPtr = indexBuff)
                UnsafeUtility.MemCpy(tris.GetUnsafePtr(), indexPtr, UnsafeUtility.SizeOf<int>() * indexBuff.Length);

                UnityEngine.Object.DestroyImmediate(go);
                return MeshCollider.Create(verts, tris);
            }
        }

        public static BlobAssetReference<Collider> MakeConvex()
        {
            fp3[] testPoints =
            {
                new fp3((fp)1.45f, (fp)8.67f, (fp)3.45f),
                new fp3((fp)8.75f, (fp)1.23f, (fp)6.44f),
                new fp3((fp)100.34f, (fp)5.33f, -(fp)2.55f),
                new fp3((fp)8.76f, (fp)4.56f, -(fp)4.54f),
                new fp3((fp)9.75f, -(fp)0.45f, -(fp)8.99f),
                new fp3((fp)7.66f, (fp)3.44f, fp.zero)
            };

            return ConvexCollider.Create(new NativeArray<fp3>(testPoints, Allocator.Temp), new ConvexHullGenerationParameters { BevelRadius = (fp)0.125f });
        }

        public static BlobAssetReference<Collider> MakeTerrain()
        {
            return TerrainCollider.Create(new NativeArray<fp>(16, Allocator.Temp), new int2(4, 4), new fp3(fp.one, fp.one, fp.one), TerrainCollider.CollisionMethod.VertexSamples);
        }

        public static BlobAssetReference<Collider> MakeCompound()
        {
            var box = MakeBox();
            var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(4, Allocator.Temp)
            {
                [0] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = fpmath.mul(FpRigidTransform.identity, new FpRigidTransform(fpquaternion.identity, new fp3(fp.half, fp.half, fp.half))) },
                [1] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = fpmath.mul(FpRigidTransform.identity, new FpRigidTransform(fpquaternion.identity, new fp3(-fp.half, fp.half, fp.half))) },
                [2] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = fpmath.mul(FpRigidTransform.identity, new FpRigidTransform(fpquaternion.identity, new fp3(fp.half, -fp.half, fp.half))) },
                [3] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = fpmath.mul(FpRigidTransform.identity, new FpRigidTransform(fpquaternion.identity, new fp3(fp.half, fp.half, -fp.half))) },
            };

            return CompoundCollider.Create(children);
        }

        //not burstable
        public static BlobAssetReference<Collider> MakeCollider<ColliderT>() where ColliderT : ICollider
        {
            switch (default(ColliderT))
            {
                case BoxCollider col:
                    return MakeBox();
                case CapsuleCollider col:
                    return MakeCapsule();
                case CylinderCollider col:
                    return MakeCylinder();
                case SphereCollider col:
                    return MakeSphere();
                case MeshCollider col:
                    return MakeMesh();
                case ConvexCollider col:
                    return MakeConvex();
                case TerrainCollider col:
                    return MakeTerrain();
                case CompoundCollider col:
                    return MakeCompound();
            }

            throw new Exception("Unhandled collider type.");
        }

        [Test]
        public void As_WithCorrectTypeDoesNotThrow()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.DoesNotThrow(() => { col.As<T>(); });
            }
        }

        [Test]
        public void As_WithCorrectTypeCollierPtrMatches()
        {
            using (var col = MakeCollider<T>())
            {
                ref var castedCol = ref col.As<T>();
                fixed(void* ptr = &castedCol)
                Assert.AreEqual((ulong)ptr, (ulong)col.GetUnsafePtr());
            }
        }

        [Test]
        public void As_WithInvalidCastTypeThrows()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.Throws(typeof(Exception), () => { col.As<InvalidCast>(); });
            }
        }

        [Test]
        public void AsPtr_WithCorrectTypeDoesNotThrow()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.DoesNotThrow(() => { col.AsPtr<T>(); });
            }
        }

        [Test]
        public void AsPtr_WithCorrectTypeCollierPtrMatches()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.AreEqual((ulong)col.AsPtr<T>(), (ulong)col.GetUnsafePtr());
            }
        }

        [Test]
        public void AsPtr_UsingSpecializationDoesNotThrow()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.DoesNotThrow(() => { col.AsPtr(); });
            }
        }

        [Test]
        public void AsPtr_WithInvalidCastTypeThrows()
        {
            using (var col = MakeCollider<T>())
            {
                Assert.Throws(typeof(Exception), () => { col.AsPtr<InvalidCast>(); });
            }
        }

        [Test]
        public void AsComponent_ComponentColliderPtrMatchesBlobPtr()
        {
            using (var col = MakeCollider<T>())
            {
                var cmp = col.AsComponent();

                Assert.AreEqual((ulong)cmp.ColliderPtr, (ulong)col.GetUnsafePtr());
            }
        }
    }

    class BlobAssetReferenceColliderExtensions_CompileTests
    {
        [BurstCompile]
        protected struct TestParalelForJobBox : IJobParallelFor
        {
            public void Execute(int index)
            {
                var col = BoxCollider.Create(new BoxGeometry
                {
                    Center = fp3.zero,
                    Orientation = fpquaternion.identity,
                    Size = new fp3(fp.one, fp.one, fp.one),
                    BevelRadius = fp.zero
                });

                try
                {
                    var cvx = col.As<ConvexCollider>();
                }
                finally
                {
                    col.Dispose();
                }
            }
        }
        [Test]
        public void As_WithinIParalelForJob_CanCompileSuccessfully()
        {
            Assert.DoesNotThrow(() => { var j = default(TestParalelForJobBox); j.Execute(-1); });
        }
    }
}
