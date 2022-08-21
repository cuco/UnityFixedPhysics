using System;
using NUnit.Framework;
using Fixed.Mathematics;
using Fixed.Physics.Authoring;

namespace Fixed.Physics.Tests.Authoring
{
    class PhysicsShapeExtensions_UnitTests
    {
        static readonly TestCaseData[] k_DeviantAxisTestCases =
        {
            new TestCaseData(new float3((sfloat)0f,  (sfloat)1f, (sfloat)1f)).Returns(0).SetName("Smallest axis, other axes identical"),
            new TestCaseData(new float3((sfloat)1f,  (sfloat)2f, (sfloat)1f)).Returns(1).SetName("Largest axis, other axes identical"),
            new TestCaseData(new float3((sfloat)5f,  (sfloat)8f, (sfloat)1f)).Returns(2).SetName("Smallest axis, other axes differ"),
            new TestCaseData(new float3((sfloat)9f,  (sfloat)2f, (sfloat)3f)).Returns(0).SetName("Largest axis, other axes differ"),
            new TestCaseData(new float3(-(sfloat)1f, -(sfloat)1f, (sfloat)1f)).Returns(2).SetName("Only positive axis, other axes identical"),
            new TestCaseData(new float3((sfloat)1f, -(sfloat)2f, (sfloat)1f)).Returns(1).SetName("Only negative axis, other axes identical")
        };

        [TestCaseSource(nameof(k_DeviantAxisTestCases))]
        public int GetDeviantAxis_ReturnsTheMostDifferentAxis(float3 v)
        {
            return PhysicsShapeExtensions.GetDeviantAxis(v);
        }

        static readonly TestCaseData[] k_MaxAxisTestCases =
        {
            new TestCaseData(new float3((sfloat)3f,  (sfloat)2f,  (sfloat)1f)).Returns(0).SetName("X-axis (all positive)"),
            new TestCaseData(new float3((sfloat)3f,  (sfloat)2f, -(sfloat)4f)).Returns(0).SetName("X-axis (one negative with greater magnitude)"),
            new TestCaseData(new float3((sfloat)2f,  (sfloat)3f,  (sfloat)1f)).Returns(1).SetName("Y-axis (all positive)"),
            new TestCaseData(new float3(-(sfloat)4f,  (sfloat)3f,  (sfloat)2f)).Returns(1).SetName("Y-axis (one negative with greater magnitude)"),
            new TestCaseData(new float3((sfloat)1f,  (sfloat)2f,  (sfloat)3f)).Returns(2).SetName("Z-axis (all positive)"),
            new TestCaseData(new float3((sfloat)2f, -(sfloat)4f,  (sfloat)3f)).Returns(2).SetName("Z-axis (one negative with greater magnitude)"),
        };

        [TestCaseSource(nameof(k_MaxAxisTestCases))]
        public int GetMaxAxis_ReturnsLargestAxis(float3 v)
        {
            return PhysicsShapeExtensions.GetMaxAxis(v);
        }

        static readonly sfloat k_Small = math.FLT_MIN_NORMAL;
        static readonly sfloat k_Large = sfloat.MaxValue;

        static readonly float4x4 k_Orthogonal = new float4x4(quaternion.Euler(math.PI / (sfloat)8f, math.PI / (sfloat)8f, math.PI / (sfloat)8f), (sfloat)0f);

        static readonly float3 k_NonUniformScale = new float3((sfloat)1f, (sfloat)2f, (sfloat)3f);

        static readonly float4x4 k_Sheared = math.mul(float4x4.Scale(k_NonUniformScale), k_Orthogonal);

        static readonly TestCaseData[] k_ShearTestCases =
        {
            new TestCaseData(new float4x4()).Returns(false)
                .SetName("Orthogonal (Zero)"),
            new TestCaseData(float4x4.identity).Returns(false)
                .SetName("Orthogonal (Identity)"),
            new TestCaseData(k_Orthogonal).Returns(false)
                .SetName("Orthogonal (Rotated)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale))).Returns(false)
                .SetName("Orthogonal (Rotated and scaled)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale * new float3((sfloat)0f, (sfloat)1f, (sfloat)1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on one axis)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale * new float3((sfloat)0f, (sfloat)0f, (sfloat)1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on two axes)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Small))).Returns(false)
                .SetName("Orthogonal (Rotated and small scale)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large scale)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Small, (sfloat)1f, k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large difference between two axis scales)"),
            new TestCaseData(k_Sheared).Returns(true)
                .SetName("Sheared"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Small))).Returns(true)
                .SetName("Sheared (Small scale)"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Large / math.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large scale)"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Small, (sfloat)1f, k_Large / math.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large difference between two axis scales)")
        };

        [TestCaseSource(nameof(k_ShearTestCases))]
        public bool HasShear_ReturnsShearedState(float4x4 m) => m.HasShear();
    }
}
