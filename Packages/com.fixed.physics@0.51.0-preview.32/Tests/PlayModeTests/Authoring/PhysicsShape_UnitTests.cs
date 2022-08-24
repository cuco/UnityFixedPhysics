using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;
using UnityEngine;

namespace Fixed.Physics.Tests.Authoring
{
    class PhysicsShape_UnitTests
    {
        static readonly fp k_Tolerance = (fp)0.001f;

        PhysicsShapeAuthoring m_Shape;

        [SetUp]
        public void SetUp() => m_Shape = new GameObject("Shape").AddComponent<PhysicsShapeAuthoring>();

        [TearDown]
        public void TearDown()
        {
            if (m_Shape != null)
                GameObject.DestroyImmediate(m_Shape.gameObject);
        }

        [Test]
        public void SetBoxProperties_WithSizeLessThanZero_ClampsToZero()
        {
            m_Shape.SetBox(new BoxGeometry { Size = -(fp)3f, Orientation = fpquaternion.identity });

            var box = m_Shape.GetBoxProperties();

            Assert.That(box.Size, Is.EqualTo(new fp3((fp)0f)));
        }

        [Test]
        public void GetCapsuleProperties_WhenShapeIsBox_HeightIsMaxDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            m_Shape.SetBox(new BoxGeometry { Size = size, Orientation = fpquaternion.identity });

            var capsule = m_Shape.GetCapsuleProperties();

            var height = capsule.Height;
            Assert.That(height, Is.EqualTo(fpmath.cmax(size)));
        }

        [Test]
        public void GetCapsuleProperties_WhenShapeIsBox_RadiusIsHalfSecondMaxDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            m_Shape.SetBox(new BoxGeometry { Size = size, Orientation = fpquaternion.identity });

            var capsule = m_Shape.GetCapsuleProperties();

            var cmaxI = size.GetMaxAxis();
            var expectedRadius = fp.half * fpmath.cmax(cmaxI == 0 ? size.yz : cmaxI == 1 ? size.xz : size.xy);
            Assert.That(capsule.Radius, Is.EqualTo(expectedRadius));
        }

        static readonly TestCaseData[] k_CapsuleOrientationTestCases =
        {
            new TestCaseData(new fp3((fp)2f, (fp)1f, (fp)1f), new fp3((fp)1f, (fp)0f, (fp)0f)).SetName("Aligned to x-axis"),
            new TestCaseData(new fp3((fp)1f, (fp)2f, (fp)1f), new fp3((fp)0f, (fp)1f, (fp)0f)).SetName("Aligned to y-axis"),
            new TestCaseData(new fp3((fp)1f, (fp)1f, (fp)2f), new fp3((fp)0f, (fp)0f, (fp)1f)).SetName("Aligned to z-axis")
        };
        [TestCaseSource(nameof(k_CapsuleOrientationTestCases))]
        public void GetCapsuleProperties_WhenShapeIsElongatedBox_OrientationPointsDownLongAxis(fp3 boxSize, fp3 expectedLookVector)
        {
            m_Shape.SetBox(new BoxGeometry { Size = boxSize, Orientation = fpquaternion.identity });

            var capsule = m_Shape.GetCapsuleProperties();

            var lookVector = fpmath.mul(capsule.Orientation, new fp3((fp)0f, (fp)0f, (fp)1f));
            Assert.That(
                fpmath.dot(lookVector, expectedLookVector), Is.EqualTo((fp)1f).Within(k_Tolerance),
                $"Expected {expectedLookVector} but got {lookVector}"
            );
        }

        [Test]
        public void GetCapsuleProperties_WhenShapeIsRotatedElongatedBox_MidpointIsBoxCenter(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            var orientation = fpquaternion.LookRotation(new fp3((fp)1f), fpmath.up());
            var expectedCenter = new fp3((fp)4f, (fp)5f, (fp)6f);
            m_Shape.SetBox(new BoxGeometry { Size = size, Center = expectedCenter, Orientation = orientation });

            var capsule = m_Shape.GetCapsuleProperties();

            Assert.That(capsule.Center, Is.EqualTo(expectedCenter));
        }

        [Test]
        public void GetCylinderProperties_WhenShapeIsBox_HeightIsDeviantDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            m_Shape.SetBox(new BoxGeometry { Size = size, Orientation = fpquaternion.identity });

            var cylinder = m_Shape.GetCylinderProperties();

            var heightAxis = size.GetDeviantAxis();
            Assert.That(cylinder.Height, Is.EqualTo(size[heightAxis]));
        }

        [Test]
        public void GetCylinderProperties_WhenShapeIsBox_RadiusIsHalfMaxHomogenousDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            m_Shape.SetBox(new BoxGeometry { Size = size, Orientation = fpquaternion.identity });

            var cylinder = m_Shape.GetCylinderProperties();

            var heightAxis = size.GetDeviantAxis();
            var expectedRadius = fp.half * fpmath.cmax(heightAxis == 0 ? size.yz : heightAxis == 1 ? size.xz : size.xy);
            Assert.That(cylinder.Radius, Is.EqualTo(expectedRadius));
        }

        [Test]
        public void GetSphereProperties_WhenShapeIsBox_RadiusIsHalfMaxDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new fp3((fp)sizeX, (fp)sizeY, (fp)sizeZ);
            m_Shape.SetBox(new BoxGeometry { Size = size, Orientation = fpquaternion.identity });

            var sphere = m_Shape.GetSphereProperties(out fpquaternion _);

            var expectedRadius = fp.half * fpmath.cmax(size);
            Assert.That(sphere.Radius, Is.EqualTo(expectedRadius));
        }

        static readonly TestCaseData[] k_PlaneSizeTestCases =
        {
            new TestCaseData(new fp3((fp)2f, (fp)3f, (fp)1f), 0, 1).SetName("xy"),
            new TestCaseData(new fp3((fp)2f, (fp)1f, (fp)3f), 0, 2).SetName("xz"),
            new TestCaseData(new fp3((fp)1f, (fp)2f, (fp)3f), 1, 2).SetName("yz")
        };

        [TestCaseSource(nameof(k_PlaneSizeTestCases))]
        public void GetPlaneProperties_WhenShapeIsBox_SizeIsTwoGreatestDimensions(fp3 boxSize, int ax1, int ax2)
        {
            m_Shape.SetBox(new BoxGeometry { Size = boxSize, Orientation = fpquaternion.identity });

            m_Shape.GetPlaneProperties(out _, out var size, out fpquaternion _);

            Assert.That(
                new[] { size.x, size.y }, Is.EquivalentTo(new[] { boxSize[ax1], boxSize[ax2] }),
                "Plane dimensions did not match two greatest dimensions of original box"
            );
        }

        static readonly TestCaseData[] k_PlaneOrientationTestCases =
        {
            new TestCaseData(new fp3((fp)3f, (fp)2f, (fp)1f), fpquaternion.LookRotation(new fp3((fp)1f, (fp)0f, (fp)0f), new fp3((fp)0f, (fp)0f, (fp)1f))).SetName("look x, up z"),
            new TestCaseData(new fp3((fp)2f, (fp)3f, (fp)1f), fpquaternion.LookRotation(new fp3((fp)0f, (fp)1f, (fp)0f), new fp3((fp)0f, (fp)0f, (fp)1f))).SetName("look y, up z"),
            new TestCaseData(new fp3((fp)3f, (fp)1f, (fp)2f), fpquaternion.LookRotation(new fp3((fp)1f, (fp)0f, (fp)0f), new fp3((fp)0f, (fp)1f, (fp)0f))).SetName("look x, up y"),
            new TestCaseData(new fp3((fp)2f, (fp)1f, (fp)3f), fpquaternion.LookRotation(new fp3((fp)0f, (fp)0f, (fp)1f), new fp3((fp)0f, (fp)1f, (fp)0f))).SetName("look z, up y"),
            new TestCaseData(new fp3((fp)1f, (fp)3f, (fp)2f), fpquaternion.LookRotation(new fp3((fp)0f, (fp)1f, (fp)0f), new fp3((fp)1f, (fp)0f, (fp)0f))).SetName("look y, up x"),
            new TestCaseData(new fp3((fp)1f, (fp)2f, (fp)3f), fpquaternion.LookRotation(new fp3((fp)0f, (fp)0f, (fp)1f), new fp3((fp)1f, (fp)0f, (fp)0f))).SetName("look z, up x")
        };

        [TestCaseSource(nameof(k_PlaneOrientationTestCases))]
        public void GetPlaneProperties_WhenShapeIsBox_OrientationPointsDownLongAxisUpFlatAxis(fp3 boxSize, fpquaternion expected)
        {
            m_Shape.SetBox(new BoxGeometry { Size = boxSize, Orientation = fpquaternion.identity });

            m_Shape.GetPlaneProperties(out _, out _, out fpquaternion orientation);

            var expectedLook = fpmath.mul(expected, new fp3 { z = (fp)1f });
            var expectedUp = fpmath.mul(expected, new fp3 { y = (fp)1f });
            var actualLook = fpmath.mul(orientation, new fp3 { z = (fp)1f });
            var actualUp = fpmath.mul(orientation, new fp3 { y = (fp)1f });
            var dotProducts = fpmath.abs(new fp3(
                fpmath.dot(expectedLook, actualLook),
                fpmath.dot(expectedUp, actualUp),
                (fp)0f
            ));
            Assert.That(
                dotProducts, Is.PrettyCloseTo(new fp3((fp)1f, (fp)1f, (fp)0f)),
                $"Expected look axis to be parallel to {expectedLook} and up axis to be parallel to {expectedUp} but got {actualLook} and {actualUp}"
            );
        }
    }
}
