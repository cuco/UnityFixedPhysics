using System;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using static Unity.Mathematics.FixedPoint.fpmath;
using Assert = UnityEngine.Assertions.Assert;
using fp3 = Unity.Mathematics.FixedPoint.fp3;
using fpquaternion = Unity.Mathematics.FixedPoint.fpquaternion;
using Random = Unity.Mathematics.FixedPoint.Random;
using FpRigidTransform = Unity.Mathematics.FixedPoint.FpRigidTransform;
using TestUtils = Fixed.Physics.Tests.Utils.TestUtils;

namespace Fixed.Physics.Tests.Base.Math
{
    class AabbTests
    {
        static readonly fp k_pi2 = (fp)1.57079632679489f;

        [Test]
        public void TestAabb()
        {
            fp3 v0 = fp3((fp)100, (fp)200, (fp)300);
            fp3 v1 = fp3((fp)200, (fp)300, (fp)400);
            fp3 v2 = fp3((fp)50, (fp)100, (fp)350);

            Aabb a0; a0.Min = fp3.zero; a0.Max = v0;
            Aabb a1; a1.Min = fp3.zero; a1.Max = v1;
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
                Assert.IsTrue(unionAabb.Min.x == fp.zero);
                Assert.IsTrue(unionAabb.Min.y == fp.zero);
                Assert.IsTrue(unionAabb.Min.z == fp.zero);
                Assert.IsTrue(unionAabb.Max.x == a1.Max.x);
                Assert.IsTrue(unionAabb.Max.y == a1.Max.y);
                Assert.IsTrue(unionAabb.Max.z == a1.Max.z);

                Aabb intersectAabb = a2;
                intersectAabb.Intersect(a3);
                Assert.IsTrue(intersectAabb.Min.x == (fp)50);
                Assert.IsTrue(intersectAabb.Min.y == (fp)100);
                Assert.IsTrue(intersectAabb.Min.z == (fp)350);
                Assert.IsTrue(intersectAabb.Max.x == a3.Max.x);
                Assert.IsTrue(intersectAabb.Max.y == a3.Max.y);
                Assert.IsTrue(intersectAabb.Max.z == a3.Max.z);
            }

            // Test Expand / Contains
            {
                Aabb a5; a5.Min = v2; a5.Max = v1;
                fp3 testPoint = fp3(v2.x - (fp)1.0f, v1.y + (fp)1.0f, fp.half * (v2.z + v1.z));
                Assert.IsFalse(a5.Contains(testPoint));

                a5.Expand((fp)1.5f);
                Assert.IsTrue(a5.Contains(testPoint));
            }

            // Test transform
            {
                Aabb ut; ut.Min = v0; ut.Max = v1;

                // Identity transform should not modify aabb
                Aabb outAabb = Fixed.Physics.Math.TransformAabb(FpRigidTransform.identity, ut);

                TestUtils.AreEqual(ut.Min, outAabb.Min, (fp)1e-3f);

                // Test translation
                outAabb = Fixed.Physics.Math.TransformAabb(new FpRigidTransform(fpquaternion.identity, fp3((fp)100.0f, (fp)0.0f, (fp)0.0f)), ut);

                Assert.AreEqual(outAabb.Min.x, (fp)200);
                Assert.AreEqual(outAabb.Min.y, (fp)200);
                Assert.AreEqual(outAabb.Max.x, (fp)300);
                Assert.AreEqual(outAabb.Max.z, (fp)400);

                // Test rotation
                fpquaternion rot = fpquaternion.EulerXYZ((fp)0.0f, (fp)0.0f, k_pi2);
                outAabb = Fixed.Physics.Math.TransformAabb(new FpRigidTransform(rot, fp3.zero), ut);

                TestUtils.AreEqual(outAabb.Min, fp3(-(fp)300.0f, (fp)100.0f, (fp)300.0f), (fp)1e-3f);
                TestUtils.AreEqual(outAabb.Max, fp3(-(fp)200.0f, (fp)200.0f, (fp)400.0f), (fp)1e-3f);
                TestUtils.AreEqual(outAabb.SurfaceArea, ut.SurfaceArea, (fp)1e-2f);
            }
        }

        [Test]
        public void TestAabbTransform()
        {
            Random rnd = new Random(0x12345678);
            for (int i = 0; i < 100; i++)
            {
                fpquaternion r = rnd.NextQuaternionRotation();
                fp3 t = rnd.Nextfp3();

                Aabb orig = new Aabb();
                orig.Include(rnd.Nextfp3());
                orig.Include(rnd.Nextfp3());

                Aabb outAabb1 = Fixed.Physics.Math.TransformAabb(new FpRigidTransform(r, t), orig);

                Physics.Math.MTransform bFromA = new Physics.Math.MTransform(r, t);
                Aabb outAabb2 = Fixed.Physics.Math.TransformAabb(bFromA, orig);

                TestUtils.AreEqual(outAabb1.Min, outAabb2.Min, (fp)1e-3f);
                TestUtils.AreEqual(outAabb1.Max, outAabb2.Max, (fp)1e-3f);
            }
        }
    }
}
