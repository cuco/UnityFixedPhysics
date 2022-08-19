using NUnit.Framework;
using Fixed.Mathematics;
using Fixed.Physics.Authoring;
using UnityEngine;

namespace Fixed.Physics.Tests.Authoring
{
    class CapsuleGeometryAuthoring_UnitTests
    {
        [Test]
        public void SetOrientation_DoesNotThrow()
        {
            Assert.DoesNotThrow(() => new CapsuleGeometryAuthoring { Orientation = quaternion.identity });
        }
    }
}
