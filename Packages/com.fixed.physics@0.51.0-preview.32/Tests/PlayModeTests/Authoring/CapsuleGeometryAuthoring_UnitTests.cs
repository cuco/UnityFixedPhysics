using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using Fixed.Physics.Authoring;
using UnityEngine;

namespace Fixed.Physics.Tests.Authoring
{
    class CapsuleGeometryAuthoring_UnitTests
    {
        [Test]
        public void SetOrientation_DoesNotThrow()
        {
            Assert.DoesNotThrow(() => new CapsuleGeometryAuthoring { Orientation = fpquaternion.identity });
        }
    }
}
