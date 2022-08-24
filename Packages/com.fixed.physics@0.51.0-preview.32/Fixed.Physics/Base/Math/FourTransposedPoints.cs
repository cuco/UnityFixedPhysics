using System;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public static partial class Math
    {
        // 4 transposed 3D vertices
        [Serializable]
        public struct FourTransposedPoints
        {
            private fp4x3 m_TransposedPoints;

            public fp4 X => m_TransposedPoints.c0;
            public fp4 Y => m_TransposedPoints.c1;
            public fp4 Z => m_TransposedPoints.c2;

            public FourTransposedPoints V0000 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.xxxx, Y.xxxx, Z.xxxx) };
            public FourTransposedPoints V1111 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.yyyy, Y.yyyy, Z.yyyy) };
            public FourTransposedPoints V2222 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.zzzz, Y.zzzz, Z.zzzz) };
            public FourTransposedPoints V3333 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.wwww, Y.wwww, Z.wwww) };
            public FourTransposedPoints V1230 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.yzwx, Y.yzwx, Z.yzwx) };
            public FourTransposedPoints V3012 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.wxyz, Y.wxyz, Z.wxyz) };
            public FourTransposedPoints V1203 => new FourTransposedPoints { m_TransposedPoints = new fp4x3(X.yzxw, Y.yzxw, Z.yzxw) };

            public FourTransposedPoints(fp3 v)
            {
                m_TransposedPoints.c0 = v.xxxx;
                m_TransposedPoints.c1 = v.yyyy;
                m_TransposedPoints.c2 = v.zzzz;
            }

            public FourTransposedPoints(fp3 v0, fp3 v1, fp3 v2, fp3 v3)
            {
                m_TransposedPoints.c0 = new fp4(v0.x, v1.x, v2.x, v3.x);
                m_TransposedPoints.c1 = new fp4(v0.y, v1.y, v2.y, v3.y);
                m_TransposedPoints.c2 = new fp4(v0.z, v1.z, v2.z, v3.z);
            }

            public static FourTransposedPoints operator+(FourTransposedPoints lhs, FourTransposedPoints rhs)
            {
                return new FourTransposedPoints { m_TransposedPoints = lhs.m_TransposedPoints + rhs.m_TransposedPoints };
            }

            public static FourTransposedPoints operator-(FourTransposedPoints lhs, FourTransposedPoints rhs)
            {
                return new FourTransposedPoints { m_TransposedPoints = lhs.m_TransposedPoints - rhs.m_TransposedPoints };
            }

            public FourTransposedPoints MulT(fp4 v)
            {
                return new FourTransposedPoints { m_TransposedPoints = new fp4x3(X * v, Y * v, Z * v) };
            }

            public FourTransposedPoints Cross(FourTransposedPoints a)
            {
                return new FourTransposedPoints { m_TransposedPoints = new fp4x3(Y * a.Z - Z * a.Y, Z * a.X - X * a.Z, X * a.Y - Y * a.X) };
            }

            public fp4 Dot(fp3 v) => X * v.x + Y * v.y + Z * v.z;
            public fp4 Dot(FourTransposedPoints a) => X * a.X + Y * a.Y + Z * a.Z;

            public fp3 GetPoint(int index) => new fp3(X[index], Y[index], Z[index]);
            public fp4 GetComponent(int index) => m_TransposedPoints[index];
        }
    }
}
