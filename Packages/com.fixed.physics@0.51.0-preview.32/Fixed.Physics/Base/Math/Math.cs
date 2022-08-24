using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine.Assertions;

namespace Fixed.Physics
{
    // Common fpmath helper functions
    [DebuggerStepThrough]
    public static partial class Math
    {
        // Constants
        [DebuggerStepThrough]
        public static class Constants
        {
            public static fp4 One4F => new fp4(1);
            public static fp4 Min4F => new fp4(fp.min_value);
            public static fp4 Max4F => new fp4(fp.max_value);
            public static fp3 Min3F => new fp3(fp.min_value);
            public static fp3 Max3F => new fp3(fp.max_value);
            public static fp3 MaxDisplacement3F => new fp3(fp.max_value * fp.half);

            // Smallest fp such that 1.0 + eps != 1.0
            // Different from fp.Epsilon which is the smallest value greater than zero.
            public static fp Eps => fp.FromRaw(0x34000000);//1.192092896e-07F;

            // These constants are identical to the ones in the Unity Mathf library, to ensure identical behaviour
            internal static fp UnityEpsilonNormalSqrt => fp.FromRaw(0x26901d7d);//1e-15F;
            internal static fp UnityEpsilon => fp.FromRaw(0x3727c5ac);//0.00001F;

            public static readonly fp Tau = fp.two * fpmath.PI;
            public static readonly fp OneOverTau = fp.one / Tau;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf16(int input) => ((input + 15) >> 4) << 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf16(ulong input) => ((input + 15) >> 4) << 4;

        /// Note that alignment must be a power of two for this to work.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf(int input, int alignment) => (input + (alignment - 1)) & (~(alignment - 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf(ulong input, ulong alignment) => (input + (alignment - 1)) & (~(alignment - 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(fp2 v) => v.x < v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(fp3 v) => v.x < v.y ? ((v.x < v.z) ? 0 : 2) : ((v.y < v.z) ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(fp4 v) => math.cmax(math.select(new int4(0, 1, 2, 3), new int4(-1), fpmath.cmin(v) < v));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(fp2 v) => v.x > v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(fp3 v) => v.x > v.y ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(fp4 v) => math.cmax(math.select(new int4(0, 1, 2, 3), new int4(-1), fpmath.cmax(v) > v));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp HorizontalMul(fp3 v) => v.x * v.y * v.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp HorizontalMul(fp4 v) => (v.x * v.y) * (v.z * v.w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp Dotxyz1(fp4 lhs, fp3 rhs) => fpmath.dot(lhs, new fp4(rhs, fp.one));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dotxyz1(double4 lhs, double3 rhs) => math.dot(lhs, new double4(rhs, 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp Det(fp3 a, fp3 b, fp3 c) => fpmath.dot(fpmath.cross(a, b), c); // TODO: use fpmath.determinant()?

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp RSqrtSafe(fp v) => fpmath.select(fpmath.rsqrt(v), fp.zero, fpmath.abs(v) < fp.FromRaw(0x2edbe6ff));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampToMaxLength(fp maxLength, ref fp3 vector)
        {
            fp lengthSq = fpmath.lengthsq(vector);
            bool maxExceeded = lengthSq > maxLength * maxLength;
            if (maxExceeded)
            {
                fp invLen = fpmath.rsqrt(lengthSq);
                fp3 rescaledVector = maxLength * invLen * vector;
                vector = rescaledVector;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp NormalizeWithLength(fp3 v, out fp3 n)
        {
            fp lengthSq = fpmath.lengthsq(v);
            fp invLength = fpmath.rsqrt(lengthSq);
            n = v * invLength;
            return lengthSq * invLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsNormalized(fp3 v)
        {
            fp lenZero = fpmath.lengthsq(v) - fp.one;
            fp absLenZero = fpmath.abs(lenZero);
            return absLenZero < Constants.UnityEpsilon;
        }

        // Return two normals perpendicular to the input vector
        public static void CalculatePerpendicularNormalized(fp3 v, out fp3 p, out fp3 q)
        {
            fp3 vSquared = v * v;
            fp3 lengthsSquared = vSquared + vSquared.xxx; // y = ||j x v||^2, z = ||k x v||^2
            fp3 invLengths = fpmath.rsqrt(lengthsSquared);

            // select first direction, j x v or k x v, whichever has greater magnitude
            fp3 dir0 = new fp3(-v.y, v.x, fp.zero);
            fp3 dir1 = new fp3(-v.z, fp.zero, v.x);
            bool cmp = (lengthsSquared.y > lengthsSquared.z);
            fp3 dir = fpmath.select(dir1, dir0, cmp);

            // normalize and get the other direction
            fp invLength = fpmath.select(invLengths.z, invLengths.y, cmp);
            p = dir * invLength;
            fp3 cross = fpmath.cross(v, dir);
            q = cross * invLength;
        }

        // Calculate the eigenvectors and eigenvalues of a symmetric 3x3 matrix
        public static void DiagonalizeSymmetricApproximation(fp3x3 a, out fp3x3 eigenVectors, out fp3 eigenValues)
        {
            fp GetMatrixElement(fp3x3 m, int row, int col)
            {
                switch (col)
                {
                    case 0: return m.c0[row];
                    case 1: return m.c1[row];
                    case 2: return m.c2[row];
                    default: UnityEngine.Assertions.Assert.IsTrue(false); return fp.zero;
                }
            }

            void SetMatrixElement(ref fp3x3 m, int row, int col, fp x)
            {
                switch (col)
                {
                    case 0: m.c0[row] = x; break;
                    case 1: m.c1[row] = x; break;
                    case 2: m.c2[row] = x; break;
                    default: UnityEngine.Assertions.Assert.IsTrue(false); break;
                }
            }

            eigenVectors = fp3x3.identity;
            fp epsSq = fp.FromRaw(0x283424dc) * (fpmath.lengthsq(a.c0) + fpmath.lengthsq(a.c1) + fpmath.lengthsq(a.c2));
            const int maxIterations = 10;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                // Find the row (p) and column (q) of the off-diagonal entry with greater magnitude
                int p = 0, q = 1;
                {
                    fp maxEntry = fpmath.abs(a.c1[0]);
                    fp mag02 = fpmath.abs(a.c2[0]);
                    fp mag12 = fpmath.abs(a.c2[1]);
                    if (mag02 > maxEntry)
                    {
                        maxEntry = mag02;
                        p = 0;
                        q = 2;
                    }
                    if (mag12 > maxEntry)
                    {
                        maxEntry = mag12;
                        p = 1;
                        q = 2;
                    }

                    // Terminate if it's small enough
                    if (maxEntry * maxEntry < epsSq)
                    {
                        break;
                    }
                }

                // Calculate jacobia rotation
                fp3x3 j = fp3x3.identity;
                {
                    fp apq = GetMatrixElement(a, p, q);
                    fp tau = (GetMatrixElement(a, q, q) - GetMatrixElement(a, p, p)) / ((fp)2.0f * apq);
                    fp t = fpmath.sqrt(fp.one + tau * tau);
                    if (tau > fp.zero)
                    {
                        t = fp.one / (tau + t);
                    }
                    else
                    {
                        t = fp.one / (tau - t);
                    }
                    fp c = fpmath.rsqrt(fp.one + t * t);
                    fp s = t * c;

                    SetMatrixElement(ref j, p, p, c);
                    SetMatrixElement(ref j, q, q, c);
                    SetMatrixElement(ref j, p, q, s);
                    SetMatrixElement(ref j, q, p, -s);
                }

                // Rotate a
                a = fpmath.mul(fpmath.transpose(j), fpmath.mul(a, j));
                eigenVectors = fpmath.mul(eigenVectors, j);
            }
            eigenValues = new fp3(a.c0.x, a.c1.y, a.c2.z);
        }

        // Returns the twist angle of the swing-twist decomposition of q about i, j, or k corresponding to index = 0, 1, or 2 respectively.
        public static fp CalculateTwistAngle(fpquaternion q, int twistAxisIndex)
        {
            // q = swing * twist, twist = normalize(twistAxis * twistAxis dot q.xyz, q.w)
            fp dot = q.value[twistAxisIndex];
            fp w = q.value.w;
            fp lengthSq = dot * dot + w * w;
            fp invLength = RSqrtSafe(lengthSq);
            fp sinHalfAngle = dot * invLength;
            fp cosHalfAngle = w * invLength;
            fp halfAngle = fpmath.atan2(sinHalfAngle, cosHalfAngle);
            return halfAngle + halfAngle;
        }

        // Returns a fpquaternion q with q * from = to
        public static fpquaternion FromToRotation(fp3 from, fp3 to)
        {
            Assert.IsTrue(fpmath.abs(fpmath.lengthsq(from) - fp.one) < fp.FromRaw(0x38d1b717));
            Assert.IsTrue(fpmath.abs(fpmath.lengthsq(to) - fp.one) < fp.FromRaw(0x38d1b717));
            fp3 cross = fpmath.cross(from, to);
            CalculatePerpendicularNormalized(from, out fp3 safeAxis, out fp3 unused); // for when angle ~= 180
            fp dot = fpmath.dot(from, to);
            fp3 squares = new fp3(fp.half - new fp2(dot, -dot) * fp.half, fpmath.lengthsq(cross));
            fp3 inverses = fpmath.select(fpmath.rsqrt(squares), fp.zero, squares < fp.FromRaw(0x2edbe6ff));
            fp2 sinCosHalfAngle = squares.xy * inverses.xy;
            fp3 axis = fpmath.select(cross * inverses.z, safeAxis, squares.z < fp.FromRaw(0x2edbe6ff));
            return new fpquaternion(new fp4(axis * sinCosHalfAngle.x, sinCosHalfAngle.y));
        }

        // Note: taken from Unity.Animation/Core/MathExtensions.cs, which will be moved to Unity.Mathematics.FixedPoint at some point
        //       after that, this should be removed and the Mathematics version should be used
        #region toEuler
        static fp3 toEuler(fpquaternion q, math.RotationOrder order = math.RotationOrder.Default)
        {
            //const fp epsilon = 1e-6f;

            //prepare the data
            var qv = q.value;
            var d1 = qv * qv.wwww * new fp4((fp)2.0f); //xw, yw, zw, ww
            var d2 = qv * qv.yzxw * new fp4((fp)2.0f); //xy, yz, zx, ww
            var d3 = qv * qv;
            var euler = new fp3(fp.zero);

            //const fp epsilon = 1e-6f;
            //const fp CUTOFF = (1.0f - 2.0f * epsilon) * (1.0f - 2.0f * epsilon);
            const uint CUTOFF_U32 = 0x3f7fffbd;
            fp CUTOFF = fp.FromRaw(CUTOFF_U32);

            switch (order)
            {
                case math.RotationOrder.ZYX:
                {
                    var y1 = d2.z + d1.y;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.x + d1.z;
                        var x2 = d3.x + d3.w - d3.y - d3.z;
                        var z1 = -d2.y + d1.x;
                        var z2 = d3.z + d3.w - d3.y - d3.x;
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else //zxz
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.z, d1.y, d2.y, d1.x);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(-1, 1, -1, 1));
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fp.zero);
                    }

                    break;
                }

                case math.RotationOrder.ZXY:
                {
                    var y1 = d2.y - d1.x;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.x + d1.z;
                        var x2 = d3.y + d3.w - d3.x - d3.z;
                        var z1 = d2.z + d1.y;
                        var z2 = d3.z + d3.w - d3.x - d3.y;
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else //zxz
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.z, d1.y, d2.y, d1.x);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(fp.minusOne, fp.one, fp.minusOne, fp.one));
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fp.zero);
                    }

                    break;
                }

                case math.RotationOrder.YXZ:
                {
                    var y1 = d2.y + d1.x;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.z + d1.y;
                        var x2 = d3.z + d3.w - d3.x - d3.y;
                        var z1 = -d2.x + d1.z;
                        var z2 = d3.y + d3.w - d3.z - d3.x;
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else //yzy
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.x, d1.z, d2.y, d1.x);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(fp.minusOne, fp.one, fp.minusOne, fp.one));
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fp.zero);
                    }

                    break;
                }

                case math.RotationOrder.YZX:
                {
                    var y1 = d2.x - d1.z;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.z + d1.y;
                        var x2 = d3.x + d3.w - d3.z - d3.y;
                        var z1 = d2.y + d1.x;
                        var z2 = d3.y + d3.w - d3.x - d3.z;
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else //yxy
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.x, d1.z, d2.y, d1.x);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(fp.minusOne, fp.one, fp.minusOne, fp.one));
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fp.zero);
                    }

                    break;
                }

                case math.RotationOrder.XZY:
                {
                    var y1 = d2.x + d1.z;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.y + d1.x;
                        var x2 = d3.y + d3.w - d3.z - d3.x;
                        var z1 = -d2.z + d1.y;
                        var z2 = d3.x + d3.w - d3.y - d3.z;
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else //xyx
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.x, d1.z, d2.z, d1.y);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(fp.minusOne, fp.one, fp.minusOne, fp.one));
                        euler = new fp3(fpmath.atan2(x1, x2), fp.Asin(y1), fp.zero);
                    }

                    break;
                }

                case math.RotationOrder.XYZ:
                {
                    var y1 = d2.z - d1.y;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.y + d1.x;
                        var x2 = d3.z + d3.w - d3.y - d3.x;
                        var z1 = d2.x + d1.z;
                        var z2 = d3.x + d3.w - d3.y - d3.z;
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fpmath.atan2(z1, z2));
                    }
                    else   //xzx
                    {
                        y1 = fpmath.clamp(y1, fp.minusOne, fp.one);
                        var abcd = new fp4(d2.z, d1.y, d2.x, d1.z);
                        var x1 = fp.two * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = fpmath.csum(abcd * abcd * new fp4(fp.minusOne, fp.one, fp.minusOne, fp.one));
                        euler = new fp3(fpmath.atan2(x1, x2), -fp.Asin(y1), fp.zero);
                    }

                    break;
                }
            }

            return eulerReorderBack(euler, order);
        }

        static fp3 eulerReorderBack(fp3 euler, math.RotationOrder order)
        {
            switch (order)
            {
                case math.RotationOrder.XZY:
                    return euler.xzy;
                case math.RotationOrder.YZX:
                    return euler.zxy;
                case math.RotationOrder.YXZ:
                    return euler.yxz;
                case math.RotationOrder.ZXY:
                    return euler.yzx;
                case math.RotationOrder.ZYX:
                    return euler.zyx;
                case math.RotationOrder.XYZ:
                default:
                    return euler;
            }
        }

        #endregion

        /// <summary>
        /// Convert a fpquaternion orientation to Euler angles.
        /// Use this method to calculate angular velocity needed to achieve a target orientation.
        /// </summary>
        /// <param name="q">An orientation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fp3 ToEulerAngles(this fpquaternion q, math.RotationOrder order = math.RotationOrder.XYZ)
        {
            return toEuler(q, order);
        }

        // Returns the angle in degrees between /from/ and /to/. This is always the smallest
        internal static fp Angle(fp3 from, fp3 to)
        {
            // sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
            var denominator = fpmath.sqrt(fpmath.lengthsq(from) * fpmath.lengthsq(to));
            if (denominator < Constants.UnityEpsilonNormalSqrt)
                return fp.zero;

            var dot = fpmath.clamp(fpmath.dot(from, to) / denominator, fp.minusOne, fp.one);
            return fpmath.degrees(fp.Acos(dot));
        }

        // The smaller of the two possible angles between the two vectors is returned, therefore the result will never be greater than 180 degrees or smaller than -180 degrees.
        // If you imagine the from and to vectors as lines on a piece of paper, both originating from the same point, then the /axis/ vector would point up out of the paper.
        // The measured angle between the two vectors would be positive in a clockwise direction and negative in an anti-clockwise direction.
        internal static fp SignedAngle(fp3 from, fp3 to, fp3 axis)
        {
            var unsignedAngle = Angle(from, to);
            var sign = fpmath.sign(fpmath.dot(fpmath.cross(from, to), axis));
            return unsignedAngle * sign;
        }

        // Projects a vector onto a plane defined by a normal orthogonal to the plane.
        internal static fp3 ProjectOnPlane(fp3 vector, fp3 planeNormal)
        {
            var sqrMag = fpmath.dot(planeNormal, planeNormal);
            if (sqrMag < Constants.UnityEpsilon)
                return vector;

            var dot = fpmath.dot(vector, planeNormal);
            return vector - planeNormal * (dot / sqrMag);
        }

        /// <summary>
        /// Physics internally represents all rigid bodies in world space.
        /// If a static body is in a hierarchy, its local-to-world matrix must be decomposed when building the physics world.
        /// This method returns a world-space FpRigidTransform that would be decomposed for such a rigid body.
        /// </summary>
        /// <returns>A world-space FpRigidTransform as used by physics.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FpRigidTransform DecomposeRigidBodyTransform(in fp4x4 localToWorld) =>
            new FpRigidTransform(DecomposeRigidBodyOrientation(localToWorld), localToWorld.c3.xyz);

        /// <summary>
        /// Physics internally represents all rigid bodies in world space.
        /// If a static body is in a hierarchy, its local-to-world matrix must be decomposed when building the physics world.
        /// This method returns a world-space orientation that would be decomposed for such a rigid body.
        /// </summary>
        /// <returns>A world-space orientation as used by physics.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion DecomposeRigidBodyOrientation(in fp4x4 localToWorld) =>
            fpquaternion.LookRotationSafe(localToWorld.c2.xyz, localToWorld.c1.xyz);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fp3 DecomposeScale(this fp4x4 matrix) =>
            new fp3(fpmath.length(matrix.c0.xyz), fpmath.length(matrix.c1.xyz), fpmath.length(matrix.c2.xyz));
    }
}
