using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;
using UnityMesh = UnityEngine.Mesh;

namespace Fixed.Physics.Tests.Authoring
{
    class HashableShapeInputs_UnitTests
    {
        [OneTimeSetUp]
        public void OneTimeSetUp() => HashUtility.Initialize();

        static UnityMesh MeshA => UnityEngine.Resources.GetBuiltinResource<UnityMesh>("New-Plane.fbx");
        static UnityMesh MeshB => UnityEngine.Resources.GetBuiltinResource<UnityMesh>("New-Cylinder.fbx");

        static Hash128 GetHash128_FromMesh(
            UnityMesh mesh, fp4x4 leafToBody,
            ConvexHullGenerationParameters convexHullGenerationParameters = default,
            Material material = default,
            CollisionFilter filter = default,
            fp4x4 shapeFromBody = default,
            uint uniqueIdentifier = default,
            int[] includedIndices = default,
            fp[] blendShapeWeights = default
        )
        {
            using (var allIncludedIndices = new NativeList<int>(0, Allocator.TempJob))
            using (var allBlendShapeWeights = new NativeList<fp>(0, Allocator.TempJob))
            using (var indices = new NativeArray<int>(includedIndices ?? Array.Empty<int>(), Allocator.TempJob))
            using (var blendWeights = new NativeArray<fp>(blendShapeWeights ?? Array.Empty<fp>(), Allocator.TempJob))
                return HashableShapeInputs.GetHash128(
                    uniqueIdentifier, convexHullGenerationParameters, material, filter, shapeFromBody,
                    new NativeArray<HashableShapeInputs>(1, Allocator.Temp)
                    {
                        [0] = HashableShapeInputs.FromSkinnedMesh(
                            mesh, leafToBody,
                            indices,
                            allIncludedIndices,
                            blendWeights,
                            allBlendShapeWeights
                        )
                    }, allIncludedIndices, allBlendShapeWeights, HashableShapeInputs.k_DefaultLinearPrecision
                );
        }

        [Test]
        public void GetHash128_WhenBothUninitialized_IsEqual()
        {
            var a = HashableShapeInputs.GetHash128(default, default, default, default, default, default, default, default, HashableShapeInputs.k_DefaultLinearPrecision);
            var b = HashableShapeInputs.GetHash128(default, default, default, default, default, default, default, default, HashableShapeInputs.k_DefaultLinearPrecision);

            Assert.That(a, Is.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenBothDefault_IsEqual()
        {
            var a = HashableShapeInputs.GetHash128(
                0u, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default, fp4x4.identity,
                new NativeArray<HashableShapeInputs>(1, Allocator.Temp) { [0] = default },
                new NativeList<int>(0, Allocator.Temp),
                new NativeList<fp>(0, Allocator.Temp), HashableShapeInputs.k_DefaultLinearPrecision
            );
            var b = HashableShapeInputs.GetHash128(
                0u, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default, fp4x4.identity,
                new NativeArray<HashableShapeInputs>(1, Allocator.Temp) { [0] = default },
                new NativeList<int>(0, Allocator.Temp),
                new NativeList<fp>(0, Allocator.Temp), HashableShapeInputs.k_DefaultLinearPrecision
            );

            Assert.That(a, Is.EqualTo(b));
        }

        static readonly TestCaseData[] k_GetHash128TestCases =
        {
            new TestCaseData(
                1u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default,
                2u, MeshA, default(ConvexHullGenerationParameters), Material.Default, CollisionFilter.Default
            ).Returns(false).SetName("Unique identifiers differ (not equal)"),
            new TestCaseData(
                0u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default,
                0u, MeshA, default(ConvexHullGenerationParameters), Material.Default, CollisionFilter.Default
            ).Returns(false).SetName("Convex hull parameters differ (not equal)"),
            new TestCaseData(
                0u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default,
                0u, MeshA, ConvexHullGenerationParameters.Default, default(Material), CollisionFilter.Default
            ).Returns(false).SetName("Materials differ (not equal)"),
            new TestCaseData(
                0u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default,
                0u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, default(CollisionFilter)
            ).Returns(false).SetName("Filters differ (not equal)"),
            new TestCaseData(
                0u, MeshA, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default,
                0u, MeshB, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default
            ).Returns(false).SetName("Mesh keys differ (not equal)")
        };

        [TestCaseSource(nameof(k_GetHash128TestCases))]
        public bool GetHash128_WhenDifferentInputs_CheckEquality(
            uint ida, UnityMesh ma, ConvexHullGenerationParameters ha, Material mta, CollisionFilter fa,
            uint idb, UnityMesh mb, ConvexHullGenerationParameters hb, Material mtb, CollisionFilter fb
        )
        {
            var a = GetHash128_FromMesh(ma, fp4x4.identity, ha, mta, fa, fp4x4.identity, uniqueIdentifier: ida);
            var b = GetHash128_FromMesh(mb, fp4x4.identity, hb, mtb, fb, fp4x4.identity, uniqueIdentifier: idb);

            return a.Equals(b);
        }

        [TestCase(1.1f, 1.1f, 1.1f, ExpectedResult = false, TestName = "Different scale (not equal)")]
        [TestCase(1.0001f, 1.0001f, 1.0001f, ExpectedResult = true, TestName = "Pretty close (equal)")]
        [TestCase(-1f, 1f, 1f, ExpectedResult = false, TestName = "Reflected X (not equal)")]
        [TestCase(1f, -1f, 1f, ExpectedResult = false, TestName = "Reflected Y (not equal)")]
        [TestCase(1f, 1f, -1f, ExpectedResult = false, TestName = "Reflectex Z (not equal)")]
        [TestCase(1f, -1f, -1f, ExpectedResult = false, TestName = "(1f, -1f, -1f) (not equal)")]
        [TestCase(-1f, 1f, -1f, ExpectedResult = false, TestName = "(-1f, 1f, -1f) (not equal)")]
        [TestCase(-1f, -1f, 1f, ExpectedResult = false, TestName = "(-1f, -1f, 1f) (not equal)")]
        public bool GetHash128_WhenDifferentScale_CheckEqualityWithIdentity(
            float x, float y, float z
        )
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.identity, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default, fp4x4.identity);
            var b = GetHash128_FromMesh(MeshA, fp4x4.identity, ConvexHullGenerationParameters.Default, Material.Default, CollisionFilter.Default, fp4x4.Scale((fp)x, (fp)y, (fp)z));

            return a.Equals(b);
        }

        [Test]
        public void GetHash128_WhenDifferentPositions_NotEqual()
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.Translate((fp)1f));
            var b = GetHash128_FromMesh(MeshA, fp4x4.Translate((fp)1.1f));

            Assert.That(a, Is.Not.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenDifferentOrientations_NotEqual()
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.RotateX(fpmath.PI / (fp)180f));
            var b = GetHash128_FromMesh(MeshA, fp4x4.RotateX(fpmath.PI / (fp)180f * (fp)1.05f));

            Assert.That(a, Is.Not.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenDifferentScales_NotEqual()
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.Scale((fp)1f));
            var b = GetHash128_FromMesh(MeshA, fp4x4.Scale((fp)1.01f));

            Assert.That(a, Is.Not.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenDifferentShears_NotEqual()
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.Scale((fp)2f));
            var b = GetHash128_FromMesh(MeshA, fpmath.mul(fp4x4.Scale((fp)2f), fp4x4.EulerZXY(fpmath.PI / (fp)4)));

            Assert.That(a, Is.Not.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenDifferentBlendShapeWeights_NotEqual(
            [Values(new[] { 1f }, new[] { 0f, 0f })] float[] otherSkinWeights
        )
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.identity, blendShapeWeights: new[] { (fp)0f });
            fp[] blend = default;
            if (otherSkinWeights != null)
            {
                blend = new fp[otherSkinWeights.Length];
                for (int i = 0; i < otherSkinWeights.Length; i++)
                {
                    blend[i] = (fp)otherSkinWeights[i];
                }
            }
            var b = GetHash128_FromMesh(MeshA, fp4x4.identity, blendShapeWeights: blend);

            Assert.That(a, Is.Not.EqualTo(b));
        }

        [Test]
        public void GetHash128_WhenDifferentIncludedIndices_NotEqual(
            [Values(new[] { 1 }, new[] { 0, 0 })] int[] otherIncludedIndices
        )
        {
            var a = GetHash128_FromMesh(MeshA, fp4x4.identity, includedIndices: new[] { 0 });
            var b = GetHash128_FromMesh(MeshA, fp4x4.identity, includedIndices: otherIncludedIndices);

            Assert.That(a, Is.Not.EqualTo(b));
        }

        HashableShapeInputs InputsWithIndicesAndBlendShapeWeights(
            int[] indices, fp[] weights, NativeList<int> allIncludedIndices, NativeList<fp> allBlendShapeWeights
        )
        {
            return HashableShapeInputs.FromSkinnedMesh(
                MeshA,
                fp4x4.identity,
                new NativeArray<int>(indices, Allocator.Temp),
                allIncludedIndices,
                new NativeArray<fp>(weights, Allocator.Temp),
                allBlendShapeWeights
            );
        }

        [Test]
        public void GetHash128_WhenMultipleInputs_WithDifferentIncludedIndices_NotEqual()
        {
            using (var allIndices = new NativeList<int>(0, Allocator.TempJob))
            using (var allWeights = new NativeList<fp>(0, Allocator.TempJob))
            {
                Hash128 a, b;

                using (var inputs = new NativeList<HashableShapeInputs>(2, Allocator.TempJob)
                   {
                       Length = 2,
                       [0] = InputsWithIndicesAndBlendShapeWeights(new[] { 0 }, Array.Empty<fp>(), allIndices, allWeights),
                       [1] = InputsWithIndicesAndBlendShapeWeights(new[] { 0, 0 }, Array.Empty<fp>(), allIndices, allWeights)
                   })
                {
                    a = HashableShapeInputs.GetHash128(
                        default, default, default, default, fp4x4.identity, inputs, allIndices, allWeights, HashableShapeInputs.k_DefaultLinearPrecision
                    );
                }

                allIndices.Clear();
                allWeights.Clear();
                using (var inputs = new NativeList<HashableShapeInputs>(2, Allocator.TempJob)
                   {
                       Length = 2,
                       [0] = InputsWithIndicesAndBlendShapeWeights(new[] { 0, 0 }, Array.Empty<fp>(), allIndices, allWeights),
                       [1] = InputsWithIndicesAndBlendShapeWeights(new[] { 0 }, Array.Empty<fp>(), allIndices, allWeights)
                   })
                {
                    b = HashableShapeInputs.GetHash128(
                        default, default, default, default, fp4x4.identity, inputs, allIndices, allWeights, HashableShapeInputs.k_DefaultLinearPrecision
                    );
                }

                Assert.That(a, Is.Not.EqualTo(b));
            }
        }

        [Test]
        public void GetHash128_WhenMultipleInputs_WithDifferentBlendShapeWeights_NotEqual()
        {
            using (var allIndices = new NativeList<int>(0, Allocator.TempJob))
            using (var allWeights = new NativeList<fp>(0, Allocator.TempJob))
            {
                Hash128 a, b;

                using (var inputs = new NativeList<HashableShapeInputs>(2, Allocator.TempJob)
                   {
                       Length = 2,
                       [0] = InputsWithIndicesAndBlendShapeWeights(Array.Empty<int>(), new[] { (fp)0f }, allIndices, allWeights),
                       [1] = InputsWithIndicesAndBlendShapeWeights(Array.Empty<int>(), new[] { (fp)0f, (fp)0f }, allIndices, allWeights)
                   })
                {
                    a = HashableShapeInputs.GetHash128(
                        default, default, default, default, fp4x4.identity, inputs, allIndices, allWeights, HashableShapeInputs.k_DefaultLinearPrecision
                    );
                }

                allIndices.Clear();
                allWeights.Clear();
                using (var inputs = new NativeList<HashableShapeInputs>(2, Allocator.TempJob)
                   {
                       Length = 2,
                       [0] = InputsWithIndicesAndBlendShapeWeights(Array.Empty<int>(), new[] { (fp)0f, (fp)0f }, allIndices, allWeights),
                       [1] = InputsWithIndicesAndBlendShapeWeights(Array.Empty<int>(), new[] { (fp)0f }, allIndices, allWeights)
                   })
                {
                    b = HashableShapeInputs.GetHash128(
                        default, default, default, default, fp4x4.identity, inputs, allIndices, allWeights, HashableShapeInputs.k_DefaultLinearPrecision
                    );
                }

                Assert.That(a, Is.Not.EqualTo(b));
            }
        }

        static readonly TestCaseData[] k_EqualsWithinToleranceTestCases =
        {
            new TestCaseData(fp4x4.identity).SetName("Identity (equal)").Returns(true),
            new TestCaseData(fp4x4.Translate((fp)0.00001f)).SetName("Small translation (equal)").Returns(true),
            new TestCaseData(fp4x4.Translate((fp)1000f)).SetName("Large translation (equal)").Returns(true),
            new TestCaseData(fp4x4.EulerZXY(fpmath.PI / (fp)720f)).SetName("Small rotation (equal)").Returns(true),
            new TestCaseData(fp4x4.EulerZXY(fpmath.PI * (fp)0.95f)).SetName("Large rotation (equal)").Returns(true),
            new TestCaseData(fp4x4.Scale((fp)0.00001f)).SetName("Small scale (equal)").Returns(true),
            new TestCaseData(fp4x4.Scale((fp)1000f)).SetName("Large scale (equal)").Returns(true),
            new TestCaseData(fp4x4.TRS(new fp3((fp)1000f), fpquaternion.EulerZXY(fpmath.PI / (fp)9f), new fp3((fp)0.1f))).SetName("Several transformations (equal)").Returns(true),
        };

        [TestCaseSource(nameof(k_EqualsWithinToleranceTestCases))]
        public bool GetHash128_WhenInputsImprecise_ReturnsExpectedValue(fp4x4 leafToBody)
        {
            var a = GetHash128_FromMesh(MeshA, leafToBody);

            var t = fp4x4.TRS(new fp3((fp)1 / (fp)9f), fpquaternion.EulerZXY(fpmath.PI / (fp)9f), new fp3((fp)1 / (fp)27f));
            t = fpmath.mul(t, fpmath.inverse(t));
            Assume.That(t, Is.Not.EqualTo(fp4x4.identity));
            t = fpmath.mul(t, leafToBody);
            Assume.That(t, Is.Not.EqualTo(leafToBody));
            var b = GetHash128_FromMesh(MeshA, t);

            return a.Equals(b);
        }

        // following are slow tests used for local regression testing only
        /*
        [Test]
        public void GetHash128_Rotated90DegreeIntervals_WhenInputsImprecise_Equal(
            [Values(-360f, -270f, -180f, -90f, 0f, 90f, 180f, 270f, 360f)]float rotateX,
            [Values(-360f, -270f, -180f, -90f, 0f, 90f, 180f, 270f, 360f)]float rotateY,
            [Values(-360f, -270f, -180f, -90f, 0f, 90f, 180f, 270f, 360f)]float rotateZ
        )
        {
            var leafToBody = fp4x4.EulerZXY(fpmath.radians(rotateZ), fpmath.radians(rotateY), fpmath.radians(rotateX));

            var equalsWithinTolerance = GetHash128_WhenInputsImprecise_ReturnsExpectedValue(leafToBody);

            Assert.That(equalsWithinTolerance, Is.True);
        }
        */
    }
}
