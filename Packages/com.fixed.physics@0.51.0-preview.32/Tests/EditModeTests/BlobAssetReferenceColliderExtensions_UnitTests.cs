using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Fixed.Mathematics;
using Fixed.Physics.Authoring;
using Fixed.Physics.Extensions;
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
                Center = float3.zero,
                Orientation = quaternion.identity,
                Size = new float3(sfloat.One, sfloat.One, sfloat.One),
                BevelRadius = sfloat.Zero
            });
        }

        public static BlobAssetReference<Collider> MakeCapsule()
        {
            return CapsuleCollider.Create(new CapsuleGeometry { Vertex0 = math.up(), Vertex1 = -math.up(), Radius = sfloat.One });
        }

        public static BlobAssetReference<Collider> MakeCylinder()
        {
            return CylinderCollider.Create(new CylinderGeometry
            {
                Center = float3.zero,
                Orientation = quaternion.AxisAngle(new float3(sfloat.One, sfloat.Zero, sfloat.Zero), (sfloat)45.0f),
                Height = (sfloat)2f,
                Radius = (sfloat)0.25f,
                BevelRadius = (sfloat)0.05f,
                SideCount = 8
            });
        }

        public static BlobAssetReference<Collider> MakeSphere()
        {
            return SphereCollider.Create(new SphereGeometry { Center = float3.zero, Radius = sfloat.One });
        }

        public static BlobAssetReference<Collider> MakeMesh()
        {
            unsafe
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Mesh mesh = go.GetComponent<MeshFilter>().sharedMesh;

                var vertexBuff = mesh.vertices;
                var indexBuff = mesh.triangles;
                var verts = new NativeArray<float3>(vertexBuff.Length, Allocator.Temp);
                var tris = new NativeArray<int3>(indexBuff.Length / 3, Allocator.Temp);

                fixed(int* indexPtr = indexBuff)
                UnsafeUtility.MemCpy(tris.GetUnsafePtr(), indexPtr, UnsafeUtility.SizeOf<int>() * indexBuff.Length);

                UnityEngine.Object.DestroyImmediate(go);
                return MeshCollider.Create(verts, tris);
            }
        }

        public static BlobAssetReference<Collider> MakeConvex()
        {
            float3[] testPoints =
            {
                new float3((sfloat)1.45f, (sfloat)8.67f, (sfloat)3.45f),
                new float3((sfloat)8.75f, (sfloat)1.23f, (sfloat)6.44f),
                new float3((sfloat)100.34f, (sfloat)5.33f, -(sfloat)2.55f),
                new float3((sfloat)8.76f, (sfloat)4.56f, -(sfloat)4.54f),
                new float3((sfloat)9.75f, -(sfloat)0.45f, -(sfloat)8.99f),
                new float3((sfloat)7.66f, (sfloat)3.44f, sfloat.Zero)
            };

            return ConvexCollider.Create(new NativeArray<float3>(testPoints, Allocator.Temp), new ConvexHullGenerationParameters { BevelRadius = (sfloat)0.125f });
        }

        public static BlobAssetReference<Collider> MakeTerrain()
        {
            return TerrainCollider.Create(new NativeArray<sfloat>(16, Allocator.Temp), new int2(4, 4), new float3(sfloat.One, sfloat.One, sfloat.One), TerrainCollider.CollisionMethod.VertexSamples);
        }

        public static BlobAssetReference<Collider> MakeCompound()
        {
            var box = MakeBox();
            var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(4, Allocator.Temp)
            {
                [0] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = math.mul(RigidTransform.identity, new RigidTransform(quaternion.identity, new float3((sfloat)0.5f, (sfloat)0.5f, (sfloat)0.5f))) },
                [1] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = math.mul(RigidTransform.identity, new RigidTransform(quaternion.identity, new float3(-(sfloat)0.5f, (sfloat)0.5f, (sfloat)0.5f))) },
                [2] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = math.mul(RigidTransform.identity, new RigidTransform(quaternion.identity, new float3((sfloat)0.5f, -(sfloat)0.5f, (sfloat)0.5f))) },
                [3] = new CompoundCollider.ColliderBlobInstance { Collider = box, CompoundFromChild = math.mul(RigidTransform.identity, new RigidTransform(quaternion.identity, new float3((sfloat)0.5f, (sfloat)0.5f, -(sfloat)0.5f))) },
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
                    Center = float3.zero,
                    Orientation = quaternion.identity,
                    Size = new float3(sfloat.One, sfloat.One, sfloat.One),
                    BevelRadius = sfloat.Zero
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
