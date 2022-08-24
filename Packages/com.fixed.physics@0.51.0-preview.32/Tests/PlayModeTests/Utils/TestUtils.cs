using System;
using System.Linq;
using System.Text;
using NUnit.Framework.Constraints;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using Assert = UnityEngine.Assertions.Assert;
using Unity.Entities;
using Unity.Collections;
using Unity.Jobs;
using Random = Unity.Mathematics.FixedPoint.Random;

namespace Fixed.Physics.Tests
{
    static class TestExtensions
    {
        public static string ToReadableString(this EntityQueryDesc query)
        {
            var sb = new StringBuilder();

            if (query.All.Any())
                sb.Append($"with [{string.Join(", ", query.All)}]");

            if (query.Any.Any())
            {
                if (sb.Length > 0) sb.Append(", ");
                sb.Append($"with any of [{string.Join(", ", query.Any)}]");
            }
            if (query.None.Any())
            {
                if (sb.Length > 0) sb.Append(", ");
                sb.Append($"without [{string.Join(", ", query.None)}]");
            }

            if (query.Options != EntityQueryOptions.Default)
                sb.Append($" ({string.Join(", ", Enum.GetValues(typeof(EntityQueryOptions)).Cast<EntityQueryOptions>().Where(v => (query.Options & v) == v))})");

            return sb.ToString();
        }

        static MatrixPrettyCloseConstraint PrettyCloseTo(this ConstraintExpression expression, fp4x4 expected)
        {
            var constraint = new MatrixPrettyCloseConstraint(expected);
            expression.Append(constraint);
            return constraint;
        }

        static QuaternionOrientationEquivalentConstraint OrientedEquivalentTo(
            this ConstraintExpression expression, fpquaternion expected
        )
        {
            var constraint = new QuaternionOrientationEquivalentConstraint(expected);
            expression.Append(constraint);
            return constraint;
        }
    }

    class Is : NUnit.Framework.Is
    {
        public static fp3PrettyCloseConstraint PrettyCloseTo(fp3 actual) =>
            new fp3PrettyCloseConstraint(actual);

        public static MatrixPrettyCloseConstraint PrettyCloseTo(fp4x4 actual) =>
            new MatrixPrettyCloseConstraint(actual);

        public static QuaternionOrientationEquivalentConstraint OrientedEquivalentTo(fpquaternion actual) =>
            new QuaternionOrientationEquivalentConstraint(actual);
    }

    class fp3PrettyCloseConstraint : NUnit.Framework.Constraints.Constraint
    {
        public static readonly fp DefaultTolerance = (fp)0.0001f;

        readonly fp3 m_Expected;
        fp m_ToleranceSq = DefaultTolerance * DefaultTolerance;

        public fp3PrettyCloseConstraint(fp3 expected) : base((object)expected) => m_Expected = expected;

        public override string Description => $"value to be within {fpmath.sqrt(m_ToleranceSq)} of {m_Expected}";

        public override ConstraintResult ApplyTo(object actual) =>
            new ConstraintResult(this, actual, fpmath.lengthsq((fp3)actual - m_Expected) < m_ToleranceSq);

        public fp3PrettyCloseConstraint Within(fp tolerance)
        {
            m_ToleranceSq = tolerance * tolerance;
            return this;
        }
    }

    class MatrixPrettyCloseConstraint : NUnit.Framework.Constraints.Constraint
    {
        public static readonly fp DefaultTolerance = (fp)0.0001f;

        readonly fp4x4 m_Expected;
        fp m_ToleranceSq = DefaultTolerance * DefaultTolerance;

        public MatrixPrettyCloseConstraint(fp4x4 expected) : base((object)expected) => m_Expected = expected;

        public override string Description => $"each column to be within {fpmath.sqrt(m_ToleranceSq)} of {m_Expected}";

        public override ConstraintResult ApplyTo(object actual)
        {
            var m = (fp4x4)actual;
            var cmp =
                fpmath.lengthsq(m.c0 - m_Expected.c0) < m_ToleranceSq
                && fpmath.lengthsq(m.c1 - m_Expected.c1) < m_ToleranceSq
                && fpmath.lengthsq(m.c2 - m_Expected.c2) < m_ToleranceSq
                && fpmath.lengthsq(m.c3 - m_Expected.c3) < m_ToleranceSq;
            return new ConstraintResult(this, actual, cmp);
        }

        public MatrixPrettyCloseConstraint EachAxisWithin(fp tolerance)
        {
            m_ToleranceSq = tolerance * tolerance;
            return this;
        }
    }

    class QuaternionOrientationEquivalentConstraint : NUnit.Framework.Constraints.Constraint
    {
        public static readonly fp DefaultTolerance = (fp)0.0005f;

        readonly fpquaternion m_Expected;
        fp m_ToleranceSq = DefaultTolerance * DefaultTolerance;

        public QuaternionOrientationEquivalentConstraint(fpquaternion expected) => m_Expected = expected;

        public override string Description => $"each axis to be within {fpmath.sqrt(m_ToleranceSq)} of {new fp3x3(m_Expected)}";

        public override ConstraintResult ApplyTo(object actual)
        {
            var q = (fpquaternion)actual;
            var m = new fp3x3(q);
            var expected = new fp3x3(m_Expected);
            var cmp =
                fpmath.lengthsq(m.c0 - expected.c0) < m_ToleranceSq
                && fpmath.lengthsq(m.c1 - expected.c1) < m_ToleranceSq
                && fpmath.lengthsq(m.c2 - expected.c2) < m_ToleranceSq;
            return new ConstraintResult(this, actual, cmp);
        }

        public QuaternionOrientationEquivalentConstraint EachAxisWithin(fp tolerance)
        {
            m_ToleranceSq = tolerance * tolerance;
            return this;
        }
    }
}

namespace Fixed.Physics.Tests.Utils
{
    class TestUtils
    {
        public static void AreEqual(bool a, bool b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(int a, int b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(uint a, uint b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(long a, long b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(ulong a, ulong b)
        {
            Assert.AreEqual(a, b);
        }

        public static void AreEqual(float a, float b, float delta = 0.0f)
        {
            Assert.AreApproximatelyEqual(a, b, delta);
        }

        public static void AreEqual(fp a, fp b, fp delta)
        {
            Assert.AreApproximatelyEqual((float)a, (float)b, (float)delta);
        }

        public static void AreEqual(fp a, fp b, int maxUlp, bool signedZeroEqual)
        {
            if (signedZeroEqual && a == b)
                return;

            if (fpmath.isfinite(a) && fpmath.isfinite(b))
            {
                int ia = fpmath.asint(a);
                int ib = fpmath.asint(b);
                if ((ia ^ ib) < 0)
                    Assert.AreEqual(true, false);
                int ulps = fpmath.abs(ia - ib);
                Assert.AreEqual(true, ulps <= maxUlp);
            }
            else
            {
                if (a != b && (!fpmath.isnan(a) || !fpmath.isnan(b)))
                    Assert.AreEqual(true, false);
            }
        }

        public static void AreEqual(double a, double b, double delta = 0.0)
        {
            Assert.IsTrue(fpmath.abs(a - b) < delta);
        }

        public static void AreEqual(double a, double b, int maxUlp, bool signedZeroEqual)
        {
            if (signedZeroEqual && a == b)
                return;

            if (fpmath.isfinite(a) && fpmath.isfinite(b))
            {
                long la = fpmath.aslong(a);
                long lb = fpmath.aslong(b);
                if ((la ^ lb) < 0)
                    Assert.AreEqual(true, false);
                long ulps = la > lb ? la - lb : lb - la;
                Assert.AreEqual(true, ulps <= maxUlp);
            }
            else
            {
                if (a != b && (!fpmath.isnan(a) || !fpmath.isnan(b)))
                    Assert.AreEqual(true, false);
            }
        }

        // bool
        public static void AreEqual(bool2 a, bool2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(bool3 a, bool3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(bool4 a, bool4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }

        public static void AreEqual(bool2x2 a, bool2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(bool3x2 a, bool3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(bool4x2 a, bool4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(bool2x3 a, bool2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(bool3x3 a, bool3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(bool4x3 a, bool4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(bool2x4 a, bool2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(bool3x4 a, bool3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(bool4x4 a, bool4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        // int
        public static void AreEqual(int2 a, int2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(int3 a, int3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(int4 a, int4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }

        public static void AreEqual(int2x2 a, int2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(int3x2 a, int3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(int4x2 a, int4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(int2x3 a, int2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(int3x3 a, int3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(int4x3 a, int4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(int2x4 a, int2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(int3x4 a, int3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(int4x4 a, int4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        // uint
        public static void AreEqual(uint2 a, uint2 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
        }

        public static void AreEqual(uint3 a, uint3 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
        }

        public static void AreEqual(uint4 a, uint4 b)
        {
            AreEqual(a.x, b.x);
            AreEqual(a.y, b.y);
            AreEqual(a.z, b.z);
            AreEqual(a.w, b.w);
        }

        public static void AreEqual(uint2x2 a, uint2x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(uint3x2 a, uint3x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(uint4x2 a, uint4x2 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
        }

        public static void AreEqual(uint2x3 a, uint2x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(uint3x3 a, uint3x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(uint4x3 a, uint4x3 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
        }

        public static void AreEqual(uint2x4 a, uint2x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(uint3x4 a, uint3x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        public static void AreEqual(uint4x4 a, uint4x4 b)
        {
            AreEqual(a.c0, b.c0);
            AreEqual(a.c1, b.c1);
            AreEqual(a.c2, b.c2);
            AreEqual(a.c3, b.c3);
        }

        // float
        public static void AreEqual(fp2 a, fp2 b, fp delta)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
        }

        public static void AreEqual(fp2 a, fp2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp3 a, fp3 b, fp delta)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
        }

        public static void AreEqual(fp3 a, fp3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp4 a, fp4 b, fp delta)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
            AreEqual(a.w, b.w, delta);
        }

        public static void AreEqual(fp4 a, fp4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
            AreEqual(a.w, b.w, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp2x2 a, fp2x2 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(fp2x2 a, fp2x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp3x2 a, fp3x2 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(fp3x2 a, fp3x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp4x2 a, fp4x2 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(fp4x2 a, fp4x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp2x3 a, fp2x3 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(fp2x3 a, fp2x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp3x3 a, fp3x3 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(fp3x3 a, fp3x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp4x3 a, fp4x3 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(fp4x3 a, fp4x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp2x4 a, fp2x4 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(fp2x4 a, fp2x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp3x4 a, fp3x4 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(fp3x4 a, fp3x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fp4x4 a, fp4x4 b, fp delta)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(fp4x4 a, fp4x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        // double
        public static void AreEqual(double2 a, double2 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
        }

        public static void AreEqual(double2 a, double2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3 a, double3 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
        }

        public static void AreEqual(double3 a, double3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4 a, double4 b, double delta = 0.0)
        {
            AreEqual(a.x, b.x, delta);
            AreEqual(a.y, b.y, delta);
            AreEqual(a.z, b.z, delta);
            AreEqual(a.w, b.w, delta);
        }

        public static void AreEqual(double4 a, double4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.x, b.x, maxUlp, signedZeroEqual);
            AreEqual(a.y, b.y, maxUlp, signedZeroEqual);
            AreEqual(a.z, b.z, maxUlp, signedZeroEqual);
            AreEqual(a.w, b.w, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double2x2 a, double2x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double2x2 a, double2x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x2 a, double3x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double3x2 a, double3x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x2 a, double4x2 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
        }

        public static void AreEqual(double4x2 a, double4x2 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double2x3 a, double2x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double2x3 a, double2x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x3 a, double3x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double3x3 a, double3x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x3 a, double4x3 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
        }

        public static void AreEqual(double4x3 a, double4x3 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double2x4 a, double2x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double2x4 a, double2x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double3x4 a, double3x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double3x4 a, double3x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(double4x4 a, double4x4 b, double delta = 0.0)
        {
            AreEqual(a.c0, b.c0, delta);
            AreEqual(a.c1, b.c1, delta);
            AreEqual(a.c2, b.c2, delta);
            AreEqual(a.c3, b.c3, delta);
        }

        public static void AreEqual(double4x4 a, double4x4 b, int maxUlp, bool signedZeroEqual)
        {
            AreEqual(a.c0, b.c0, maxUlp, signedZeroEqual);
            AreEqual(a.c1, b.c1, maxUlp, signedZeroEqual);
            AreEqual(a.c2, b.c2, maxUlp, signedZeroEqual);
            AreEqual(a.c3, b.c3, maxUlp, signedZeroEqual);
        }

        public static void AreEqual(fpquaternion a, fpquaternion b, fp delta)
        {
            AreEqual(a.value, b.value, delta);
        }

        public static void AreEqual(FpRigidTransform a, FpRigidTransform b, fp delta)
        {
            AreEqual(a.rot, b.rot, delta);
            AreEqual(a.pos, b.pos, delta);
        }

        //
        // Random generation
        //

        public static BlobAssetReference<Collider> GenerateRandomMesh(ref Random rnd)
        {
            int numTriangles = rnd.NextInt(1, 250);
            var vertices = new NativeArray<fp3>(numTriangles * 3, Allocator.Temp);
            var triangles = new NativeArray<int3>(numTriangles, Allocator.Temp);

            int nextIndex = 0;

            while (numTriangles > 0)
            {
                var featureTriangles = fpmath.min(rnd.NextInt(1, 100), numTriangles);

                int featureType = rnd.NextInt(0, 3);
                switch (featureType)
                {
                    case 0:
                    {
                        // Soup
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            fp size = rnd.NextFloat((fp)0.1f, (fp)2.5f);
                            size *= size;

                            fp3 center = rnd.Nextfp3(-(fp)10.0f, (fp)10.0f);
                            for (int j = 0; j < 3; j++)
                            {
                                vertices[nextIndex++] = center + rnd.Nextfp3(-size, size);
                            }
                        }
                        break;
                    }

                    case 1:
                    {
                        // Fan
                        fp3 center = rnd.Nextfp3(-(fp)10.0f, (fp)10.0f);
                        fp3 arm = rnd.Nextfp3Direction() * rnd.NextFloat((fp)0.1f, (fp)1.0f);
                        fp3 axis;
                        {
                            fp3 unused;
                            Math.CalculatePerpendicularNormalized(fpmath.normalize(arm), out axis, out unused);
                        }
                        fp arc = rnd.NextFloat((fp)0.1f, fp.two * (fp)fpmath.PI);
                        arc = fpmath.min(arc, (fp)featureTriangles * (fp)fpmath.PI / fp.two); // avoid degenerate triangles
                        featureTriangles = (int)fpmath.min((fp)featureTriangles, (arc / (fp)0.025f)); // avoid degenerate triangles
                        fpquaternion q = Unity.Mathematics.FixedPoint.fpquaternion.AxisAngle(axis, arc / (fp)numTriangles);
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            vertices[nextIndex++] = center;
                            vertices[nextIndex++] = center + arm;
                            arm = fpmath.mul(q, arm);
                            vertices[nextIndex++] = center + arm;
                        }
                        break;
                    }

                    case 2:
                    {
                        // Strip
                        fp3 v0 = rnd.Nextfp3(-(fp)10.0f, (fp)10.0f);
                        fp3 v1 = v0 + rnd.Nextfp3(-fp.half, fp.half);
                        fp3 dir;
                        {
                            fp3 unused;
                            Math.CalculatePerpendicularNormalized(fpmath.normalize(v1 - v0), out dir, out unused);
                        }
                        for (int i = 0; i < featureTriangles; i++)
                        {
                            fp3 v2 = v0 + rnd.NextFloat((fp)0.25f, fp.half) * dir;
                            dir = fpmath.mul(Unity.Mathematics.FixedPoint.fpquaternion.AxisAngle(rnd.Nextfp3Direction(), rnd.NextFloat((fp)0.0f, (fp)0.3f)), dir);

                            vertices[nextIndex++] = v0;
                            vertices[nextIndex++] = v1;
                            vertices[nextIndex++] = v2;

                            v0 = v1;
                            v1 = v2;
                        }
                        break;
                    }

                    case 3:
                    {
                        // Grid
                        int quads = featureTriangles / 2;
                        if (quads == 0)
                        {
                            featureTriangles = 0; // Too small, try again for a different feature
                            break;
                        }

                        int rows = rnd.NextInt(1, (int)fpmath.sqrt((fp)quads));
                        int cols = quads / rows;
                        quads = rows * cols;
                        featureTriangles = quads * 2;

                        fp3 origin = rnd.Nextfp3(-(fp)10.0f, (fp)10.0f);
                        fp3 x = rnd.Nextfp3(-fp.half, fp.half);
                        fp3 y = rnd.Nextfp3(-fp.half, fp.half);
                        for (int i = 0; i < rows; i++)
                        {
                            for (int j = 0; j < cols; j++)
                            {
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 0);
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 1);
                                vertices[nextIndex++] = origin + x * (i + 1) + y * (j + 1);

                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 0);
                                vertices[nextIndex++] = origin + x * (i + 1) + y * (j + 1);
                                vertices[nextIndex++] = origin + x * (i + 0) + y * (j + 1);
                            }
                        }

                        break;
                    }
                }

                numTriangles -= featureTriangles;
            }

            for (int i = 0; i < triangles.Length; i++)
            {
                triangles[i] = new int3(3 * i, 3 * i + 1, 3 * i + 2);
            }

            var ret = MeshCollider.Create(vertices, triangles);

            vertices.Dispose();
            triangles.Dispose();
            return ret;
        }

        public static BlobAssetReference<Collider> GenerateRandomTerrain(ref Random rnd)
        {
            int2 size = rnd.NextInt2(2, 50);
            fp3 scale = rnd.Nextfp3((fp)0.1f, new fp3((fp)1, (fp)10, (fp)1));

            int numSamples = size.x * size.y;
            var heights = new NativeArray<fp>(numSamples, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < numSamples; i++)
            {
                heights[i] = rnd.NextFloat((fp)0, (fp)1);
            }

            // CollisionMethod.Vertices will fail the unit tests, because it is a low-quality mode
            // that produces inaccurate manifolds. For now we just test CollisionMethod.Triangles
            return TerrainCollider.Create(heights, size, scale, TerrainCollider.CollisionMethod.Triangles);
        }

        public static BlobAssetReference<Collider> GenerateRandomCompound(ref Random rnd)
        {
            int numChildren = rnd.NextInt(1, 10);
            var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(numChildren, Allocator.Temp);
            for (int i = 0; i < numChildren; i++)
            {
                children[i] = new CompoundCollider.ColliderBlobInstance
                {
                    CompoundFromChild = new FpRigidTransform
                    {
                        pos = (rnd.NextInt(10) > 0) ? rnd.Nextfp3(-(fp)5.0f, (fp)5.0f) : fp3.zero,
                        rot = (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : fpquaternion.identity
                    },
                    Collider = GenerateRandomCollider(ref rnd)
                };
            }

            return CompoundCollider.Create(children);
        }

        public static BlobAssetReference<Collider> GenerateRandomConvex(ref Random rnd)
        {
            ColliderType colliderType = (ColliderType)rnd.NextInt((int)ColliderType.Cylinder + 1);
            fp convexRadius = (rnd.NextInt(4) > 0) ? rnd.NextFloat(fp.half) : (fp)0.0f;
            switch (colliderType)
            {
                case ColliderType.Convex:
                {
                    int numPoints = rnd.NextInt(1, 16);
                    if (numPoints == 3) // TODO - hull builder doesn't build faces for flat shapes, work around it for now to run the test
                    {
                        numPoints++;
                    }
                    var points = new NativeArray<fp3>(numPoints, Allocator.TempJob);
                    for (int i = 0; i < numPoints; i++)
                    {
                        points[i] = rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
                    }
                    var generationParameters = ConvexHullGenerationParameters.Default;
                    generationParameters.BevelRadius = convexRadius;
                    var collider = ConvexCollider.Create(points, generationParameters, CollisionFilter.Default);
                    points.Dispose();
                    return collider;
                }

                case ColliderType.Sphere:
                {
                    return SphereCollider.Create(new SphereGeometry
                    {
                        Center = (rnd.NextInt(4) > 0) ? fp3.zero : rnd.Nextfp3(-fp.half, fp.half),
                        Radius = rnd.NextFloat((fp)0.01f, fp.half)
                    });
                }

                case ColliderType.Capsule:
                {
                    fp3 point0 = rnd.Nextfp3((fp)0.0f, (fp)1.0f);
                    fp3 point1 = (rnd.NextInt(4) > 0) ? -point0 : rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
                    return CapsuleCollider.Create(new CapsuleGeometry
                    {
                        Vertex0 = point0,
                        Vertex1 = point1,
                        Radius = rnd.NextFloat((fp)0.01f, fp.half)
                    });
                }

                case ColliderType.Triangle:
                {
                    return PolygonCollider.CreateTriangle(rnd.Nextfp3(-(fp)1.0f, (fp)1.0f), rnd.Nextfp3(-(fp)1.0f, (fp)1.0f), rnd.Nextfp3(-(fp)1.0f, (fp)1.0f));
                }

                case ColliderType.Quad:
                {
                    // Pick 3 totally random points, then choose a fourth that makes a flat and convex quad
                    fp3 point0 = rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
                    fp3 point1 = rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
                    fp3 point3 = rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
                    fp t0 = rnd.NextFloat((fp)0.0f, (fp)1.0f);
                    fp t1 = rnd.NextFloat((fp)0.0f, (fp)1.0f);
                    fp3 e = point1 + point1 - point0;
                    fp3 a = fpmath.lerp(point1, e, t0);
                    fp3 b = fpmath.lerp(point3, point3 + point3 - point0, t0);
                    fp3 point2 = fpmath.lerp(a, b, t1);

                    return PolygonCollider.CreateQuad(point0, point1, point2, point3);
                }

                case ColliderType.Box:
                {
                    fp minSize = (fp)0.05f; // TODO - work around hull builder problems with small faces, sometimes doesn't extend 1D->2D based on face area
                    var boxGeometry = new BoxGeometry
                    {
                        Center = (rnd.NextInt(4) > 0) ? fp3.zero : rnd.Nextfp3(-fp.half, fp.half),
                        Orientation = (rnd.NextInt(4) > 0) ? fpquaternion.identity : rnd.NextQuaternionRotation(),
                        Size = rnd.Nextfp3(minSize, (fp)1.0f)
                    };

                    fp maxBevelRadius = fpmath.max(fpmath.cmin((boxGeometry.Size - minSize) / ((fp)2.0f * ((fp)1.0f + fp.Epsilon))), (fp)0.0f);
                    boxGeometry.BevelRadius = fpmath.min(maxBevelRadius, convexRadius);

                    return BoxCollider.Create(boxGeometry);
                }

                case ColliderType.Cylinder:
                {
                    fp minSize = (fp)0.01f; // TODO - cylinder gets degenerate faces if radius-convexRadius=0 or height/2-convexRadius=0, decide how to handle this in CylinderCollider
                    var cylinderGeometry = new CylinderGeometry
                    {
                        Center = (rnd.NextInt(4) > 0) ? fp3.zero : rnd.Nextfp3(-fp.half, fp.half),
                        Orientation = (rnd.NextInt(4) > 0) ? fpquaternion.identity : rnd.NextQuaternionRotation(),
                        Height = rnd.NextFloat((fp)2.0f * minSize, (fp)1f),
                        Radius = rnd.NextFloat(minSize, (fp)1.0f),
                        SideCount = 20
                    };

                    var maxBevelRadius = fpmath.max(fpmath.min(cylinderGeometry.Height / (fp)2, cylinderGeometry.Radius) - minSize, (fp)0.0f);
                    cylinderGeometry.BevelRadius = fpmath.min(maxBevelRadius, convexRadius);

                    return CylinderCollider.Create(cylinderGeometry);
                }

                default:
                    throw new NotImplementedException();
            }
        }

        public static BlobAssetReference<Collider> GenerateRandomCollider(ref Random rnd)
        {
            if (rnd.NextInt(10) > 0)
            {
                return GenerateRandomConvex(ref rnd); // 90% convex
            }
            else if (rnd.NextInt(4) > 0)
            {
                if (rnd.NextInt(2) > 0)
                {
                    return GenerateRandomMesh(ref rnd); // 3.25% mesh
                }
                else
                {
                    return GenerateRandomTerrain(ref rnd); // 3.25% terrain
                }
            }
            return GenerateRandomCompound(ref rnd); // 2.5% compound
        }

        public static unsafe PhysicsWorld GenerateRandomWorld(ref Random rnd, int numBodies, fp size, int numThreadsHint)
        {
            // Create the world
            PhysicsWorld world = new PhysicsWorld(numBodies, 0, 0);

            // Create bodies
            NativeArray<RigidBody> bodies = world.StaticBodies;
            for (int i = 0; i < numBodies; i++)
            {
                bodies[i] = new RigidBody
                {
                    WorldFromBody = new FpRigidTransform
                    {
                        pos = rnd.Nextfp3(-size, size),
                        rot = (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : fpquaternion.identity
                    },
                    Collider = GenerateRandomCollider(ref rnd),   // Not safe, could be garbage collected
                    Entity = Entity.Null,
                    CustomTags = 0
                };
            }

            // Build the broadphase
            if (numThreadsHint == -1)
            {
                world.CollisionWorld.Broadphase.Build(world.StaticBodies, world.DynamicBodies, world.MotionVelocities,
                    world.CollisionWorld.CollisionTolerance, (fp)1.0f, -(fp)9.81f * fpmath.up());
            }
            else
            {
                var buildStaticTree = new NativeArray<int>(1, Allocator.TempJob);
                buildStaticTree[0] = 1;
                world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, timeStep: (fp)1.0f, gravity: -(fp)9.81f * fpmath.up(),
                    buildStaticTree, inputDeps: new JobHandle(), numThreadsHint > 0).Complete();
                buildStaticTree.Dispose();
            }

            return world;
        }
    }
}
