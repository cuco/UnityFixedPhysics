using System;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;

namespace Fixed.Physics.Tests.Authoring
{
    class PhysicsShapeExtensions_UnitTests
    {
        static readonly TestCaseData[] k_DeviantAxisTestCases =
        {
            new TestCaseData(new fp3((fp)0f,  (fp)1f, (fp)1f)).Returns(0).SetName("Smallest axis, other axes identical"),
            new TestCaseData(new fp3((fp)1f,  (fp)2f, (fp)1f)).Returns(1).SetName("Largest axis, other axes identical"),
            new TestCaseData(new fp3((fp)5f,  (fp)8f, (fp)1f)).Returns(2).SetName("Smallest axis, other axes differ"),
            new TestCaseData(new fp3((fp)9f,  (fp)2f, (fp)3f)).Returns(0).SetName("Largest axis, other axes differ"),
            new TestCaseData(new fp3(-(fp)1f, -(fp)1f, (fp)1f)).Returns(2).SetName("Only positive axis, other axes identical"),
            new TestCaseData(new fp3((fp)1f, -(fp)2f, (fp)1f)).Returns(1).SetName("Only negative axis, other axes identical")
        };

        [TestCaseSource(nameof(k_DeviantAxisTestCases))]
        public int GetDeviantAxis_ReturnsTheMostDifferentAxis(fp3 v)
        {
            return PhysicsShapeExtensions.GetDeviantAxis(v);
        }

        static readonly TestCaseData[] k_MaxAxisTestCases =
        {
            new TestCaseData(new fp3((fp)3f,  (fp)2f,  (fp)1f)).Returns(0).SetName("X-axis (all positive)"),
            new TestCaseData(new fp3((fp)3f,  (fp)2f, -(fp)4f)).Returns(0).SetName("X-axis (one negative with greater magnitude)"),
            new TestCaseData(new fp3((fp)2f,  (fp)3f,  (fp)1f)).Returns(1).SetName("Y-axis (all positive)"),
            new TestCaseData(new fp3(-(fp)4f,  (fp)3f,  (fp)2f)).Returns(1).SetName("Y-axis (one negative with greater magnitude)"),
            new TestCaseData(new fp3((fp)1f,  (fp)2f,  (fp)3f)).Returns(2).SetName("Z-axis (all positive)"),
            new TestCaseData(new fp3((fp)2f, -(fp)4f,  (fp)3f)).Returns(2).SetName("Z-axis (one negative with greater magnitude)"),
        };

        [TestCaseSource(nameof(k_MaxAxisTestCases))]
        public int GetMaxAxis_ReturnsLargestAxis(fp3 v)
        {
            return PhysicsShapeExtensions.GetMaxAxis(v);
        }

        static readonly fp k_Small = fpmath.FLT_MIN_NORMAL;
        static readonly fp k_Large = fp.max_value;

        static readonly fp4x4 k_Orthogonal = new fp4x4(fpquaternion.Euler(fpmath.PI / (fp)8f, fpmath.PI / (fp)8f, fpmath.PI / (fp)8f), (fp)0f);

        static readonly fp3 k_NonUniformScale = new fp3((fp)1f, (fp)2f, (fp)3f);

        static readonly fp4x4 k_Sheared = fpmath.mul(fp4x4.Scale(k_NonUniformScale), k_Orthogonal);

        static readonly TestCaseData[] k_ShearTestCases =
        {
            new TestCaseData(new fp4x4()).Returns(false)
                .SetName("Orthogonal (Zero)"),
            new TestCaseData(fp4x4.identity).Returns(false)
                .SetName("Orthogonal (Identity)"),
            new TestCaseData(k_Orthogonal).Returns(false)
                .SetName("Orthogonal (Rotated)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_NonUniformScale))).Returns(false)
                .SetName("Orthogonal (Rotated and scaled)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_NonUniformScale * new fp3((fp)0f, (fp)1f, (fp)1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on one axis)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_NonUniformScale * new fp3((fp)0f, (fp)0f, (fp)1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on two axes)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_Small))).Returns(false)
                .SetName("Orthogonal (Rotated and small scale)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large scale)"),
            new TestCaseData(fpmath.mul(k_Orthogonal, fp4x4.Scale(k_Small, (fp)1f, k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large difference between two axis scales)"),
            new TestCaseData(k_Sheared).Returns(true)
                .SetName("Sheared"),
            new TestCaseData(fpmath.mul(k_Sheared, fp4x4.Scale(k_Small))).Returns(true)
                .SetName("Sheared (Small scale)"),
            new TestCaseData(fpmath.mul(k_Sheared, fp4x4.Scale(k_Large / fpmath.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large scale)"),
            new TestCaseData(fpmath.mul(k_Sheared, fp4x4.Scale(k_Small, (fp)1f, k_Large / fpmath.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large difference between two axis scales)")
        };

        [TestCaseSource(nameof(k_ShearTestCases))]
        public bool HasShear_ReturnsShearedState(fp4x4 m) => m.HasShear();
    }
}
