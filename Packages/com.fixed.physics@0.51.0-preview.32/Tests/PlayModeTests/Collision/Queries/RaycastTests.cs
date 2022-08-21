using System;
using NUnit.Framework;
using Fixed.Mathematics;

namespace Fixed.Physics.Tests.Collision.Queries
{
    class RayCastTests
    {
        [Test]
        public void RayVsTriangle()
        {
            // triangle
            var v1 = new float3(-(sfloat)1, -(sfloat)1, (sfloat)0);
            var v2 = new float3((sfloat)0, (sfloat)1, (sfloat)0);
            var v3 = new float3((sfloat)1, -(sfloat)1, (sfloat)0);

            {
                var origin = new float3((sfloat)0, (sfloat)0, -(sfloat)2);
                var direction = new float3((sfloat)0, (sfloat)0, (sfloat)4);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (sfloat)0.5);
            }

            {
                var origin = new float3((sfloat)0, (sfloat)0, (sfloat)2);
                var direction = new float3((sfloat)0, (sfloat)0, -(sfloat)4);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (sfloat)0.5);
            }

            {
                var origin = new float3((sfloat)1, -(sfloat)1, -(sfloat)2);
                var direction = new float3((sfloat)0, (sfloat)0, (sfloat)4);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == (sfloat)0.5);
            }

            {
                var origin = new float3((sfloat)2, (sfloat)0, -(sfloat)2);
                var direction = new float3((sfloat)0, (sfloat)0, (sfloat)4);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsFalse(hit);
            }

            {
                var origin = new float3((sfloat)2, (sfloat)0, -(sfloat)2);
                var direction = new float3((sfloat)0, (sfloat)0, -(sfloat)4);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsFalse(hit);
                Assert.IsTrue(math.all(normal == new float3((sfloat)0, (sfloat)0, (sfloat)0)));
            }

            {
                v1 = new float3(-(sfloat)4, (sfloat)0, (sfloat)0);
                v2 = new float3(-(sfloat)5, (sfloat)0, -(sfloat)1);
                v3 = new float3(-(sfloat)4, (sfloat)0, -(sfloat)1);

                var origin = new float3(-(sfloat)4.497f, (sfloat)0.325f, -(sfloat)0.613f);
                var direction = new float3((sfloat)0f, -(sfloat)10f, (sfloat)0f);

                sfloat fraction = (sfloat)1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
            }
        }
    }
}
