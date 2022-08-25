using NUnit.Framework;
using Unity.Burst;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;
using Unity.Mathematics;
using UnityEngine;

namespace Fixed.Physics.Tests.Authoring
{
    class EulerAngles_UnitTests
    {
        [BurstCompile(CompileSynchronously = true)]
        struct SetValueFromBurstJob : IJob
        {
            public void Execute() => new EulerAngles().SetValue(fpquaternion.identity);
        }

        [Test]
        public void SetValue_WhenCalledFromBurstJob_DoesNotThrow() => new SetValueFromBurstJob().Run();

        static readonly fpquaternion k_NotIdentityQuaternion = fpmath.mul(
            fpmath.mul(
                fpquaternion.AxisAngle(new fp3 { z = (fp)1f }, fpmath.radians((fp)45f)),
                fpquaternion.AxisAngle(new fp3 { y = (fp)1f }, fpmath.radians((fp)45f))
                ),  fpquaternion.AxisAngle(new fp3 { x = (fp)1f }, fpmath.radians((fp)45f))
        );

        static readonly TestCaseData[] k_TestCases =
        {
            new TestCaseData(math.RotationOrder.XYZ, fpquaternion.identity, fp3.zero).SetName("XYZ (identity)"),
            new TestCaseData(math.RotationOrder.YZX, fpquaternion.identity, fp3.zero).SetName("YZX (identity)"),
            new TestCaseData(math.RotationOrder.ZXY, fpquaternion.identity, fp3.zero).SetName("ZXY (identity)"),
            new TestCaseData(math.RotationOrder.XZY, fpquaternion.identity, fp3.zero).SetName("XZY (identity)"),
            new TestCaseData(math.RotationOrder.YXZ, fpquaternion.identity, fp3.zero).SetName("YXZ (identity)"),
            new TestCaseData(math.RotationOrder.ZYX, fpquaternion.identity, fp3.zero).SetName("ZYX (identity)"),
            new TestCaseData(math.RotationOrder.XYZ, k_NotIdentityQuaternion, new fp3((fp)45f, (fp)45f, (fp)45f)).SetName("XYZ (not identity)"),
            new TestCaseData(math.RotationOrder.YZX, k_NotIdentityQuaternion, new fp3((fp)30.36119f, (fp)59.63881f, (fp)8.421058f)).SetName("YZX (not identity)"),
            new TestCaseData(math.RotationOrder.ZXY, k_NotIdentityQuaternion, new fp3((fp)8.421058f, (fp)59.63881f, (fp)30.36119f)).SetName("ZXY (not identity)"),
            new TestCaseData(math.RotationOrder.XZY, k_NotIdentityQuaternion, new fp3((fp)9.735609f, (fp)54.73561f, (fp)30f)).SetName("XZY (not identity)"),
            new TestCaseData(math.RotationOrder.YXZ, k_NotIdentityQuaternion, new fp3((fp)30f, (fp)54.73561f, (fp)9.735609f)).SetName("YXZ (not identity)"),
            new TestCaseData(math.RotationOrder.ZYX, k_NotIdentityQuaternion, new fp3((fp)16.32495f, (fp)58.60028f, (fp)16.32495f)).SetName("ZYX (not identity)")
        };

        [TestCaseSource(nameof(k_TestCases))]
        public void SetValue_WhenRotationOrder_ReturnsExpectedValue(
            math.RotationOrder rotationOrder, fpquaternion value, fp3 expectedEulerAngles
        )
        {
            var eulerAngles = new EulerAngles { RotationOrder = rotationOrder };

            eulerAngles.SetValue(value);

            Assert.That(eulerAngles.Value, Is.PrettyCloseTo(expectedEulerAngles));
        }

        [Test]
        public void EulerToQuaternion_QuaternionToEuler_ResultingOrientationIsCloseToOriginal(
            [Values] math.RotationOrder rotationOrder,
            [Values(-90f, -45, 0f, 45, 90f)] float x,
            [Values(-90f, -45, 0f, 45, 90f)] float y,
            [Values(-90f, -45, 0f, 45, 90f)] float z
        )
        {
            var inputEulerAngles = new EulerAngles { RotationOrder = rotationOrder, Value = new fp3((fp)x, (fp)y, (fp)z) };
            var inputQuaternion = (fpquaternion)inputEulerAngles;
            Assume.That(fpmath.abs(fpmath.length(inputQuaternion.value)), Is.EqualTo(1.0f).Within(1e-05));

            EulerAngles outputEulerAngles = new EulerAngles { RotationOrder = inputEulerAngles.RotationOrder };
            outputEulerAngles.SetValue(inputQuaternion);
            fpquaternion outputQuaternion = (fpquaternion)outputEulerAngles;
            Assume.That(fpmath.abs(fpmath.length(outputQuaternion.value)), Is.EqualTo(1.0f).Within(1e-05));

            Assert.That(outputQuaternion, Is.OrientedEquivalentTo(inputQuaternion));
        }
    }
}
