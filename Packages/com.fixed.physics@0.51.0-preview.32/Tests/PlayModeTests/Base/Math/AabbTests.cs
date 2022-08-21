using System;
using NUnit.Framework;
using static Fixed.Mathematics.math;
using Assert = UnityEngine.Assertions.Assert;
using float3 = Fixed.Mathematics.float3;
using quaternion = Fixed.Mathematics.quaternion;
using Random = Fixed.Mathematics.Random;
using RigidTransform = Fixed.Mathematics.RigidTransform;
using TestUtils = Fixed.Physics.Tests.Utils.TestUtils;

namespace Fixed.Physics.Tests.Base.Math
{
    class AabbTests
    {
        static readonly sfloat k_pi2 = (sfloat)1.57079632679489f;

        [Test]
        public void TestAabb()
        {
            float3 v0 = float3((sfloat)100, (sfloat)200, (sfloat)300);
            float3 v1 = float3((sfloat)200, (sfloat)300, (sfloat)400);
            float3 v2 = float3((sfloat)50, (sfloat)100, (sfloat)350);

            Aabb a0; a0.Min = float3.zero; a0.Max = v0;
            Aabb a1; a1.Min = float3.zero; a1.Max = v1;
            Aabb a2; a2.Min = v2; a2.Max = v1;
            Aabb a3; a3.Min = v2; a3.Max = v0;

            Assert.IsTrue(a0.IsValid);
            Assert.IsTrue(a1.IsValid);
            Assert.IsTrue(a2.IsValid);
            Assert.IsFalse(a3.IsValid);

            Assert.IsTrue(a1.Contains(a0));
            Assert.IsFalse(a0.Contains(a1));
            Assert.IsTrue(a1.Contains(a2));
            Assert.IsFalse(a2.Contains(a1));
            Assert.IsFalse(a0.Contains(a2));
            Assert.IsFalse(a2.Contains(a0));

            // Test Union / Intersect
            {
                Aabb unionAabb = a0;
                unionAabb.Include(a1);
                Assert.IsTrue(unionAabb.Min.x == sfloat.Zero);
                Assert.IsTrue(unionAabb.Min.y == sfloat.Zero);
                Assert.IsTrue(unionAabb.Min.z == sfloat.Zero);
                Assert.IsTrue(unionAabb.Max.x == a1.Max.x);
                Assert.IsTrue(unionAabb.Max.y == a1.Max.y);
                Assert.IsTrue(unionAabb.Max.z == a1.Max.z);

                Aabb intersectAabb = a2;
                intersectAabb.Intersect(a3);
                Assert.IsTrue(intersectAabb.Min.x == (sfloat)50);
                Assert.IsTrue(intersectAabb.Min.y == (sfloat)100);
                Assert.IsTrue(intersectAabb.Min.z == (sfloat)350);
                Assert.IsTrue(intersectAabb.Max.x == a3.Max.x);
                Assert.IsTrue(intersectAabb.Max.y == a3.Max.y);
                Assert.IsTrue(intersectAabb.Max.z == a3.Max.z);
            }

            // Test Expand / Contains
            {
                Aabb a5; a5.Min = v2; a5.Max = v1;
                float3 testPoint = float3(v2.x - (sfloat)1.0f, v1.y + (sfloat)1.0f, (sfloat)0.5f * (v2.z + v1.z));
                Assert.IsFalse(a5.Contains(testPoint));

                a5.Expand((sfloat)1.5f);
                Assert.IsTrue(a5.Contains(testPoint));
            }

            // Test transform
            {
                Aabb ut; ut.Min = v0; ut.Max = v1;

                // Identity transform should not modify aabb
                Aabb outAabb = Fixed.Physics.Math.TransformAabb(RigidTransform.identity, ut);

                TestUtils.AreEqual(ut.Min, outAabb.Min, (sfloat)1e-3f);

                // Test translation
                outAabb = Fixed.Physics.Math.TransformAabb(new RigidTransform(quaternion.identity, float3((sfloat)100.0f, (sfloat)0.0f, (sfloat)0.0f)), ut);

                Assert.AreEqual(outAabb.Min.x, (sfloat)200);
                Assert.AreEqual(outAabb.Min.y, (sfloat)200);
                Assert.AreEqual(outAabb.Max.x, (sfloat)300);
                Assert.AreEqual(outAabb.Max.z, (sfloat)400);

                // Test rotation
                quaternion rot = quaternion.EulerXYZ((sfloat)0.0f, (sfloat)0.0f, k_pi2);
                outAabb = Fixed.Physics.Math.TransformAabb(new RigidTransform(rot, float3.zero), ut);

                TestUtils.AreEqual(outAabb.Min, float3(-(sfloat)300.0f, (sfloat)100.0f, (sfloat)300.0f), (sfloat)1e-3f);
                TestUtils.AreEqual(outAabb.Max, float3(-(sfloat)200.0f, (sfloat)200.0f, (sfloat)400.0f), (sfloat)1e-3f);
                TestUtils.AreEqual(outAabb.SurfaceArea, ut.SurfaceArea, (sfloat)1e-2f);
            }
        }

        [Test]
        public void TestAabbTransform()
        {
            Random rnd = new Random(0x12345678);
            for (int i = 0; i < 100; i++)
            {
                quaternion r = rnd.NextQuaternionRotation();
                float3 t = rnd.NextFloat3();

                Aabb orig = new Aabb();
                orig.Include(rnd.NextFloat3());
                orig.Include(rnd.NextFloat3());

                Aabb outAabb1 = Fixed.Physics.Math.TransformAabb(new RigidTransform(r, t), orig);

                Physics.Math.MTransform bFromA = new Physics.Math.MTransform(r, t);
                Aabb outAabb2 = Fixed.Physics.Math.TransformAabb(bFromA, orig);

                TestUtils.AreEqual(outAabb1.Min, outAabb2.Min, (sfloat)1e-3f);
                TestUtils.AreEqual(outAabb1.Max, outAabb2.Max, (sfloat)1e-3f);
            }
        }
    }
}
