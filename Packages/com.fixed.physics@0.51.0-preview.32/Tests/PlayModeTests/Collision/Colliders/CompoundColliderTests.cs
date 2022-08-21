using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics.Tests.Utils;

namespace Fixed.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="CompoundCollider"/>
    /// </summary>
    class CompoundColliderTests
    {
        [Test]
        public void MassProperties_BuiltFromChildren_MatchesExpected()
        {
            void TestCompoundBox(RigidTransform transform)
            {
                // Create a unit box
                var box = BoxCollider.Create(new BoxGeometry
                {
                    Center = transform.pos,
                    Orientation = transform.rot,
                    Size = new float3(1),
                    BevelRadius = (sfloat)0.0f
                });

                // Create a compound of mini boxes, matching the volume of the single box
                var miniBox = BoxCollider.Create(new BoxGeometry
                {
                    Center = float3.zero,
                    Orientation = quaternion.identity,
                    Size = new float3((sfloat)0.5f, (sfloat)0.5f, (sfloat)0.5f),
                    BevelRadius = (sfloat)0.0f
                });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+(sfloat)0.25f, +(sfloat)0.25f, +(sfloat)0.25f))) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-(sfloat)0.25f, +(sfloat)0.25f, +(sfloat)0.25f))) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+(sfloat)0.25f, -(sfloat)0.25f, +(sfloat)0.25f))) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+(sfloat)0.25f, +(sfloat)0.25f, -(sfloat)0.25f))) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-(sfloat)0.25f, -(sfloat)0.25f, +(sfloat)0.25f))) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+(sfloat)0.25f, -(sfloat)0.25f, -(sfloat)0.25f))) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-(sfloat)0.25f, +(sfloat)0.25f, -(sfloat)0.25f))) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-(sfloat)0.25f, -(sfloat)0.25f, -(sfloat)0.25f))) }
                };
                var compound = CompoundCollider.Create(children);
                children.Dispose();

                var boxMassProperties = box.Value.MassProperties;
                var compoundMassProperties = compound.Value.MassProperties;

                TestUtils.AreEqual(compoundMassProperties.Volume, boxMassProperties.Volume, (sfloat)1e-3f);
                TestUtils.AreEqual(compoundMassProperties.AngularExpansionFactor, boxMassProperties.AngularExpansionFactor, (sfloat)1e-3f);
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.Transform.pos, boxMassProperties.MassDistribution.Transform.pos, (sfloat)1e-3f);
                //TestUtils.AreEqual(compoundMassProperties.MassDistribution.Orientation, boxMassProperties.MassDistribution.Orientation, 1e-3f);   // TODO: Figure out why this differs, and if that is a problem
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.InertiaTensor, boxMassProperties.MassDistribution.InertiaTensor, (sfloat)1e-3f);
            }

            // Compare box with compound at various transforms
            TestCompoundBox(RigidTransform.identity);
            TestCompoundBox(new RigidTransform(quaternion.identity, new float3((sfloat)1.0f, (sfloat)2.0f, (sfloat)3.0f)));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ((sfloat)0.5f, (sfloat)1.0f, (sfloat)1.5f), float3.zero));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ((sfloat)0.5f, (sfloat)1.0f, (sfloat)1.5f), new float3((sfloat)1.0f, (sfloat)2.0f, (sfloat)3.0f)));
        }

        [Test]
        public unsafe void CreateCompound_WithRepeatedInputs_ChildrenAreInstances()
        {
            BlobAssetReference<Collider> boxBlob = default;
            BlobAssetReference<Collider> capsuleBlob = default;
            BlobAssetReference<Collider> sphereBlob = default;
            BlobAssetReference<Collider> compoundBlob = default;
            try
            {
                // 3 unique instance inputs
                boxBlob = BoxCollider.Create(new BoxGeometry { Orientation = quaternion.identity, Size = new float3(1) });
                capsuleBlob = CapsuleCollider.Create(new CapsuleGeometry { Radius = (sfloat)0.5f, Vertex0 = new float3((sfloat)1f), Vertex1 = new float3(-(sfloat)1f) });
                sphereBlob = SphereCollider.Create(new SphereGeometry { Radius = (sfloat)0.5f });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)0f)) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)1f)) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)2f)) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)3f)) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)4f)) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)5f)) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)6f)) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3((sfloat)7f)) }
                };

                compoundBlob = CompoundCollider.Create(children);

                var compound = (CompoundCollider*)compoundBlob.GetUnsafePtr();
                var uniqueChildren = new HashSet<long>();
                for (var i = 0; i < compound->Children.Length; i++)
                    uniqueChildren.Add((long)compound->Children[i].Collider);
                Assert.That(uniqueChildren.Count, Is.EqualTo(3));
            }
            finally
            {
                boxBlob.Dispose();
                capsuleBlob.Dispose();
                sphereBlob.Dispose();
                if (compoundBlob.IsCreated)
                    compoundBlob.Dispose();
            }
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CompoundCollider_Create_WhenChildrenNotCreated_Throws() =>
            Assert.Throws<ArgumentException>(() => CompoundCollider.Create(default));

        [Test]
        public void CompoundCollider_Create_WhenChildrenEmpty_Throws() =>
            Assert.Throws<ArgumentException>(() => CompoundCollider.Create(new NativeArray<CompoundCollider.ColliderBlobInstance>(0, Allocator.Temp)));

        [Test]
        public void CompoundCollider_Create_WhenNestingLevelBreached_Throws()
        {
            var CreatedColliders = new NativeList<BlobAssetReference<Collider>>(Allocator.Temp);
            Assert.Throws<ArgumentException>(
                () =>
                {
                    int numSpheres = 17;
                    sfloat sphereRadius = (sfloat)0.5f;
                    sfloat ringRadius = (sfloat)2f;
                    BlobAssetReference<Collider> compound = default;
                    for (int i = 0; i < numSpheres; ++i)
                    {
                        var t = (sfloat)i / (sfloat)numSpheres * (sfloat)2f * math.PI;
                        var p = ringRadius * new float3(math.cos(t), (sfloat)0f, math.sin(t));
                        var sphere = SphereCollider.Create(new SphereGeometry { Center = p, Radius = sphereRadius });
                        CreatedColliders.Add(sphere);
                        if (compound.IsCreated)
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = RigidTransform.identity },
                                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = compound, CompoundFromChild = RigidTransform.identity }
                                });
                        }
                        else
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(1, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = RigidTransform.identity }
                                });
                        }
                        CreatedColliders.Add(compound);
                    }
                });

            for (int i = CreatedColliders.Length - 1; i >= 0; i--)
            {
                CreatedColliders[i].Dispose();
            }

            CreatedColliders.Dispose();
        }

#endif
    }
}
