using System.Linq;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Editor;
using UnityEngine;

namespace Fixed.Physics.Tests.Editor
{
    class ManipulatorUtility_UnitTests
    {
        static TestCaseData[] k_GetMatrixStateReturnValueTestCases =
        {
            new TestCaseData(new fp4x4()).Returns(MatrixState.NotValidTRS).SetName("Not valid TRS"),
            new TestCaseData(new fp4x4 { c3 = new fp4 { w = (fp)1f } }).Returns(MatrixState.ZeroScale).SetName("Zero scale"),
            new TestCaseData(fp4x4.Scale((fp)3f)).Returns(MatrixState.UniformScale).SetName("Uniform scale"),
            new TestCaseData(fp4x4.Scale((fp)3f, (fp)2f, (fp)1f)).Returns(MatrixState.NonUniformScale).SetName("Non-uniform scale")
        };

        [TestCaseSource(nameof(k_GetMatrixStateReturnValueTestCases))]
        public MatrixState GetMatrixState_ReturnsExpectedState(fp4x4 localToWorld)
        {
            return ManipulatorUtility.GetMatrixState(ref localToWorld);
        }

        static TestCaseData[] k_GetMatrixStatMutateTestCases => k_GetMatrixStateReturnValueTestCases
        .Select(testCase => new TestCaseData(testCase.Arguments).SetName($"{testCase.TestName} does not mutate"))
        .ToArray();

        [TestCaseSource(nameof(k_GetMatrixStatMutateTestCases))]
        public void GetMatrixState_DoesNotMutateLTWArgument(fp4x4 localToWorld)
        {
            var previous = localToWorld;
            ManipulatorUtility.GetMatrixState(ref localToWorld);
            Assert.That(localToWorld, Is.EqualTo(previous));
        }
    }
}
