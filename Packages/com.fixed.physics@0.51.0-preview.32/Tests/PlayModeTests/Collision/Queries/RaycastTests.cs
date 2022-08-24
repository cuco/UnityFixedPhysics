using System;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.Tests.Collision.Queries
{
    class RayCastTests
    {
        [Test]
        public void RayVsTriangle()
        {
            // triangle
            var v1 = new fp3(-(fp)1, -(fp)1, (fp)0);
            var v2 = new fp3((fp)0, (fp)1, (fp)0);
            var v3 = new fp3((fp)1, -(fp)1, (fp)0);

            {
                var origin = new fp3((fp)0, (fp)0, -(fp)2);
                var direction = new fp3((fp)0, (fp)0, (fp)4);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (fp)0.5);
            }

            {
                var origin = new fp3((fp)0, (fp)0, (fp)2);
                var direction = new fp3((fp)0, (fp)0, -(fp)4);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (fp)0.5);
            }

            {
                var origin = new fp3((fp)1, -(fp)1, -(fp)2);
                var direction = new fp3((fp)0, (fp)0, (fp)4);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (fp)0.5);
            }

            {
                var origin = new fp3((fp)2, (fp)0, -(fp)2);
                var direction = new fp3((fp)0, (fp)0, (fp)4);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsFalse(hit);
            }

            {
                var origin = new fp3((fp)2, (fp)0, -(fp)2);
                var direction = new fp3((fp)0, (fp)0, -(fp)4);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsFalse(hit);
                Assert.IsTrue(math.all(normal == new fp3((fp)0, (fp)0, (fp)0)));
            }

            {
                v1 = new fp3(-(fp)4, (fp)0, (fp)0);
                v2 = new fp3(-(fp)5, (fp)0, -(fp)1);
                v3 = new fp3(-(fp)4, (fp)0, -(fp)1);

                var origin = new fp3(-(fp)4.497f, (fp)0.325f, -(fp)0.613f);
                var direction = new fp3((fp)0f, -(fp)10f, (fp)0f);

                fp fraction = (fp)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out fp3 normal);
                Assert.IsTrue(hit);
            }
        }
    }
}
