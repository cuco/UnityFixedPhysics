using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;
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
            void TestCompoundBox(FpRigidTransform transform)
            {
                // Create a unit box
                var box = BoxCollider.Create(new BoxGeometry
                {
                    Center = transform.pos,
                    Orientation = transform.rot,
                    Size = new fp3(1),
                    BevelRadius = (fp)0.0f
                });

                // Create a compound of mini boxes, matching the volume of the single box
                var miniBox = BoxCollider.Create(new BoxGeometry
                {
                    Center = fp3.zero,
                    Orientation = fpquaternion.identity,
                    Size = new fp3(fp.half, fp.half, fp.half),
                    BevelRadius = (fp)0.0f
                });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(+(fp)0.25f, +(fp)0.25f, +(fp)0.25f))) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(-(fp)0.25f, +(fp)0.25f, +(fp)0.25f))) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(+(fp)0.25f, -(fp)0.25f, +(fp)0.25f))) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(+(fp)0.25f, +(fp)0.25f, -(fp)0.25f))) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(-(fp)0.25f, -(fp)0.25f, +(fp)0.25f))) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(+(fp)0.25f, -(fp)0.25f, -(fp)0.25f))) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(-(fp)0.25f, +(fp)0.25f, -(fp)0.25f))) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = fpmath.mul(transform, new FpRigidTransform(fpquaternion.identity, new fp3(-(fp)0.25f, -(fp)0.25f, -(fp)0.25f))) }
                };
                var compound = CompoundCollider.Create(children);
                children.Dispose();

                var boxMassProperties = box.Value.MassProperties;
                var compoundMassProperties = compound.Value.MassProperties;

                TestUtils.AreEqual(compoundMassProperties.Volume, boxMassProperties.Volume, (fp)1e-3f);
                TestUtils.AreEqual(compoundMassProperties.AngularExpansionFactor, boxMassProperties.AngularExpansionFactor, (fp)1e-3f);
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.Transform.pos, boxMassProperties.MassDistribution.Transform.pos, (fp)1e-3f);
                //TestUtils.AreEqual(compoundMassProperties.MassDistribution.Orientation, boxMassProperties.MassDistribution.Orientation, 1e-3f);   // TODO: Figure out why this differs, and if that is a problem
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.InertiaTensor, boxMassProperties.MassDistribution.InertiaTensor, (fp)1e-3f);
            }

            // Compare box with compound at various transforms
            TestCompoundBox(FpRigidTransform.identity);
            TestCompoundBox(new FpRigidTransform(fpquaternion.identity, new fp3((fp)1.0f, fp.two, (fp)3.0f)));
            TestCompoundBox(new FpRigidTransform(fpquaternion.EulerXYZ(fp.half, (fp)1.0f, (fp)1.5f), fp3.zero));
            TestCompoundBox(new FpRigidTransform(fpquaternion.EulerXYZ(fp.half, (fp)1.0f, (fp)1.5f), new fp3((fp)1.0f, fp.two, (fp)3.0f)));
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
                boxBlob = BoxCollider.Create(new BoxGeometry { Orientation = fpquaternion.identity, Size = new fp3(1) });
                capsuleBlob = CapsuleCollider.Create(new CapsuleGeometry { Radius = fp.half, Vertex0 = new fp3((fp)1f), Vertex1 = new fp3(-(fp)1f) });
                sphereBlob = SphereCollider.Create(new SphereGeometry { Radius = fp.half });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)0f)) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)1f)) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)2f)) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)3f)) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)4f)) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)5f)) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)6f)) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new FpRigidTransform(fpquaternion.identity, new fp3((fp)7f)) }
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
                    fp sphereRadius = fp.half;
                    fp ringRadius = (fp)2f;
                    BlobAssetReference<Collider> compound = default;
                    for (int i = 0; i < numSpheres; ++i)
                    {
                        var t = (fp)i / (fp)numSpheres * (fp)2f * fpmath.PI;
                        var p = ringRadius * new fp3(fpmath.cos(t), (fp)0f, fpmath.sin(t));
                        var sphere = SphereCollider.Create(new SphereGeometry { Center = p, Radius = sphereRadius });
                        CreatedColliders.Add(sphere);
                        if (compound.IsCreated)
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = FpRigidTransform.identity },
                                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = compound, CompoundFromChild = FpRigidTransform.identity }
                                });
                        }
                        else
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(1, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = FpRigidTransform.identity }
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
