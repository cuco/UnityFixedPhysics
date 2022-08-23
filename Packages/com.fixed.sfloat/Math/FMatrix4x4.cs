using System;
using MI = System.Runtime.CompilerServices.MethodImplAttribute;
using O = System.Runtime.CompilerServices.MethodImplOptions;

namespace Morefun.LockStep
{
    public struct FMatrix4x4 : IEquatable<FMatrix4x4>
    {
        //       col0 col1 col2 col3
        //  ----+--------------------
        // row0 | m00, m01, m02, m03
        // row1 | m10, m11, m12, m13
        // row2 | m20, m21, m22, m23
        // row3 | m30, m31, m32, m33
        
        public FScalar m00, m01, m02, m03;
        public FScalar m10, m11, m12, m13;
        public FScalar m20, m21, m22, m23;
        public FScalar m30, m31, m32, m33;

        [MI(O.AggressiveInlining)] public FMatrix4x4 (
            FScalar m00, FScalar m01, FScalar m02, FScalar m03, 
            FScalar m10, FScalar m11, FScalar m12, FScalar m13,
            FScalar m20, FScalar m21, FScalar m22, FScalar m23, 
            FScalar m30, FScalar m31, FScalar m32, FScalar m33) {
            this.m00 = m00; this.m01 = m01; this.m02 = m02; this.m03 = m03;
            this.m10 = m10; this.m11 = m11; this.m12 = m12; this.m13 = m13;
            this.m20 = m20; this.m21 = m21; this.m22 = m22; this.m23 = m23;
            this.m30 = m30; this.m31 = m31; this.m32 = m32; this.m33 = m33;
        }

        public override string ToString () {
            return $"((m00:{this.m00}, m01:{this.m01}, m02:{this.m02}, m03:{this.m03}), "
                 + $"(m10:{this.m10}, m11:{this.m11}, m12:{this.m12}, m13:{this.m13}), "
                 + $"(m20:{this.m20}, m21:{this.m21}, m22:{this.m22}, m23:{this.m23}), "
                 + $"(m30:{this.m30}, m31:{this.m31}, m32:{this.m32}, m33:{this.m33}))";
        }

        [MI(O.AggressiveInlining)] public override int GetHashCode () {
            return m00.GetHashCode ()                  ^ m01.GetHashCode ().ShiftAndWrap (2)
                ^ m02.GetHashCode ().ShiftAndWrap (4)  ^ m03.GetHashCode ().ShiftAndWrap (6)
                ^ m10.GetHashCode ().ShiftAndWrap (8)  ^ m11.GetHashCode ().ShiftAndWrap (10)
                ^ m12.GetHashCode ().ShiftAndWrap (12) ^ m13.GetHashCode ().ShiftAndWrap (14)
                ^ m20.GetHashCode ().ShiftAndWrap (16) ^ m21.GetHashCode ().ShiftAndWrap (18)
                ^ m22.GetHashCode ().ShiftAndWrap (20) ^ m23.GetHashCode ().ShiftAndWrap (22)
                ^ m30.GetHashCode ().ShiftAndWrap (24) ^ m31.GetHashCode ().ShiftAndWrap (26)
                ^ m32.GetHashCode ().ShiftAndWrap (28) ^ m33.GetHashCode ().ShiftAndWrap (30);
        }

        [MI(O.AggressiveInlining)] public override bool Equals (object obj) { return (obj is FMatrix4x4 other) && this.Equals (other); }

        [MI(O.AggressiveInlining)] public bool Equals (FMatrix4x4 other) { Equals (ref this, ref other, out var r); return r; }

        //[MI(O.AggressiveInlining)] public Boolean ApproximateEquals (FMatrix4x4 other) { Boolean r; ApproximateEquals (ref this, ref other, out r); return r; }

        [MI(O.AggressiveInlining)] public bool IsSymmetric () {
            FMatrix4x4 transpose = this;
            Transpose (ref transpose, out transpose);
            return transpose.Equals (this);
        }

        [MI(O.AggressiveInlining)] public bool IsSkewSymmetric () {
            FMatrix4x4 transpose = this;
            Transpose (ref transpose, out transpose);
            Negate (ref transpose, out transpose);
            return transpose.Equals (this);
        }

        // Accessors //-------------------------------------------------------//

        public FVector3 Up          { get => new FVector3 ( m10,  m11,  m12); set { m10 =  value.x; m11 =  value.y; m12 =  value.z; } }
        public FVector3 Down        { get => new FVector3 (-m10, -m11, -m12); set { m10 = -value.x; m11 = -value.y; m12 = -value.z; } }
        public FVector3 Right       { get => new FVector3 ( m00,  m01,  m02); set { m00 =  value.x; m01 =  value.y; m02 =  value.z; } }
        public FVector3 Left        { get => new FVector3 (-m00, -m01, -m02); set { m00 = -value.x; m01 = -value.y; m02 = -value.z; } }
        public FVector3 Forward     { get => new FVector3 (-m20, -m21, -m22); set { m20 = -value.x; m21 = -value.y; m22 = -value.z; } }
        public FVector3 Backward    { get => new FVector3 ( m20,  m21,  m22); set { m20 =  value.x; m21 =  value.y; m22 =  value.z; } }
        public FVector3 Translation { get => new FVector3 ( m30,  m31,  m32); set { m30 =  value.x; m31 =  value.y; m32 =  value.z; } }

        // Constants //-------------------------------------------------------//

        static FMatrix4x4 identity, zero;

        static FMatrix4x4 () {
            identity = new FMatrix4x4 (
                1, 0, 0, 0, 
                0, 1, 0, 0, 
                0, 0, 1, 0, 
                0, 0, 0, 1);
            zero     = new FMatrix4x4 (
                0, 0, 0, 0, 
                0, 0, 0, 0, 
                0, 0, 0, 0, 
                0, 0, 0, 0);
        }

        public static FMatrix4x4 Identity => identity;
        public static FMatrix4x4 Zero => zero;

        // Operators //-------------------------------------------------------//

        [MI(O.AggressiveInlining)] public static void Equals (ref FMatrix4x4 a, ref FMatrix4x4 b, out bool r) {
            r = (a.m00 == b.m00) && (a.m11 == b.m11) &&
                     (a.m22 == b.m22) && (a.m33 == b.m33) &&
                     (a.m01 == b.m01) && (a.m02 == b.m02) &&
                     (a.m03 == b.m03) && (a.m10 == b.m10) &&
                     (a.m12 == b.m12) && (a.m13 == b.m13) &&
                     (a.m20 == b.m20) && (a.m21 == b.m21) &&
                     (a.m23 == b.m23) && (a.m30 == b.m30) &&
                     (a.m31 == b.m31) && (a.m32 == b.m32);
        }

        [MI(O.AggressiveInlining)] public static void Add (ref FMatrix4x4 a, ref FMatrix4x4 b, out FMatrix4x4 r) {
            r.m00 = a.m00 + b.m00; r.m01 = a.m01 + b.m01;
            r.m02 = a.m02 + b.m02; r.m03 = a.m03 + b.m03;
            r.m10 = a.m10 + b.m10; r.m11 = a.m11 + b.m11;
            r.m12 = a.m12 + b.m12; r.m13 = a.m13 + b.m13;
            r.m20 = a.m20 + b.m20; r.m21 = a.m21 + b.m21;
            r.m22 = a.m22 + b.m22; r.m23 = a.m23 + b.m23;
            r.m30 = a.m30 + b.m30; r.m31 = a.m31 + b.m31;
            r.m32 = a.m32 + b.m32; r.m33 = a.m33 + b.m33;
        }

        [MI(O.AggressiveInlining)] public static void Subtract (ref FMatrix4x4 a, ref FMatrix4x4 b, out FMatrix4x4 r) {
            r.m00 = a.m00 - b.m00; r.m01 = a.m01 - b.m01;
            r.m02 = a.m02 - b.m02; r.m03 = a.m03 - b.m03;
            r.m10 = a.m10 - b.m10; r.m11 = a.m11 - b.m11;
            r.m12 = a.m12 - b.m12; r.m13 = a.m13 - b.m13;
            r.m20 = a.m20 - b.m20; r.m21 = a.m21 - b.m21;
            r.m22 = a.m22 - b.m22; r.m23 = a.m23 - b.m23;
            r.m30 = a.m30 - b.m30; r.m31 = a.m31 - b.m31;
            r.m32 = a.m32 - b.m32; r.m33 = a.m33 - b.m33;
        }

        [MI(O.AggressiveInlining)] public static void Negate (ref FMatrix4x4 m, out FMatrix4x4 r) {
            r.m00 = -m.m00; r.m01 = -m.m01;
            r.m02 = -m.m02; r.m03 = -m.m03;
            r.m10 = -m.m10; r.m11 = -m.m11;
            r.m12 = -m.m12; r.m13 = -m.m13;
            r.m20 = -m.m20; r.m21 = -m.m21;
            r.m22 = -m.m22; r.m23 = -m.m23;
            r.m30 = -m.m30; r.m31 = -m.m31;
            r.m32 = -m.m32; r.m33 = -m.m33;
        }

        [MI(O.AggressiveInlining)] public static void Product (ref FMatrix4x4 a, ref FMatrix4x4 b, out FMatrix4x4 r) {
            FScalar m00 = (a.m00 * b.m00) + (a.m01 * b.m10) + (a.m02 * b.m20) + (a.m03 * b.m30);
            FScalar m01 = (a.m00 * b.m01) + (a.m01 * b.m11) + (a.m02 * b.m21) + (a.m03 * b.m31);
            FScalar m02 = (a.m00 * b.m02) + (a.m01 * b.m12) + (a.m02 * b.m22) + (a.m03 * b.m32);
            FScalar m03 = (a.m00 * b.m03) + (a.m01 * b.m13) + (a.m02 * b.m23) + (a.m03 * b.m33);
            FScalar m10 = (a.m10 * b.m00) + (a.m11 * b.m10) + (a.m12 * b.m20) + (a.m13 * b.m30);
            FScalar m11 = (a.m10 * b.m01) + (a.m11 * b.m11) + (a.m12 * b.m21) + (a.m13 * b.m31);
            FScalar m12 = (a.m10 * b.m02) + (a.m11 * b.m12) + (a.m12 * b.m22) + (a.m13 * b.m32);
            FScalar m13 = (a.m10 * b.m03) + (a.m11 * b.m13) + (a.m12 * b.m23) + (a.m13 * b.m33);
            FScalar m20 = (a.m20 * b.m00) + (a.m21 * b.m10) + (a.m22 * b.m20) + (a.m23 * b.m30);
            FScalar m21 = (a.m20 * b.m01) + (a.m21 * b.m11) + (a.m22 * b.m21) + (a.m23 * b.m31);
            FScalar m22 = (a.m20 * b.m02) + (a.m21 * b.m12) + (a.m22 * b.m22) + (a.m23 * b.m32);
            FScalar m23 = (a.m20 * b.m03) + (a.m21 * b.m13) + (a.m22 * b.m23) + (a.m23 * b.m33);
            FScalar m30 = (a.m30 * b.m00) + (a.m31 * b.m10) + (a.m32 * b.m20) + (a.m33 * b.m30);
            FScalar m31 = (a.m30 * b.m01) + (a.m31 * b.m11) + (a.m32 * b.m21) + (a.m33 * b.m31);
            FScalar m32 = (a.m30 * b.m02) + (a.m31 * b.m12) + (a.m32 * b.m22) + (a.m33 * b.m32);
            FScalar m33 = (a.m30 * b.m03) + (a.m31 * b.m13) + (a.m32 * b.m23) + (a.m33 * b.m33);
            r.m00 = m00; r.m01 = m01; r.m02 = m02; r.m03 = m03;
            r.m10 = m10; r.m11 = m11; r.m12 = m12; r.m13 = m13;
            r.m20 = m20; r.m21 = m21; r.m22 = m22; r.m23 = m23;
            r.m30 = m30; r.m31 = m31; r.m32 = m32; r.m33 = m33; 
        }

        [MI(O.AggressiveInlining)] public static void Multiply (ref FMatrix4x4 m, ref FScalar f, out FMatrix4x4 r) {
            r.m00 = m.m00 * f; r.m01 = m.m01 * f;
            r.m02 = m.m02 * f; r.m03 = m.m03 * f;
            r.m10 = m.m10 * f; r.m11 = m.m11 * f;
            r.m12 = m.m12 * f; r.m13 = m.m13 * f;
            r.m20 = m.m20 * f; r.m21 = m.m21 * f;
            r.m22 = m.m22 * f; r.m23 = m.m23 * f;
            r.m30 = m.m30 * f; r.m31 = m.m31 * f;
            r.m32 = m.m32 * f; r.m33 = m.m33 * f;
        }

        [MI(O.AggressiveInlining)] public static bool  operator == (FMatrix4x4 a, FMatrix4x4 b) {
            Equals    (ref a, ref b, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static bool  operator != (FMatrix4x4 a, FMatrix4x4 b) {
            Equals    (ref a, ref b, out var r); return !r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  + (FMatrix4x4 a, FMatrix4x4 b) {
            Add       (ref a, ref b, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  - (FMatrix4x4 a, FMatrix4x4 b) {
            Subtract  (ref a, ref b, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  - (FMatrix4x4 m)             {
            Negate    (ref m,        out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  * (FMatrix4x4 a, FMatrix4x4 b) {
            Product   (ref a, ref b, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  * (FMatrix4x4 m, FScalar f)   {
            Multiply  (ref m, ref f, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 operator  * (FScalar f, FMatrix4x4 m)   {
            Multiply  (ref m, ref f, out var r); return  r; }
        [MI(O.AggressiveInlining)] public static FVector3  operator  * (FVector3 v, FMatrix4x4 m)  {
            Transform (ref m, ref v, out var r); return  r; }
        //[MI(O.AggressiveInlining)] public static Vector4  operator  * (Vector4 v, FMatrix4x4 m)  { Vector4  r; Transform (ref m, ref v, out r); return  r; }
        [MI(O.AggressiveInlining)] public static FVector3  operator  * (FMatrix4x4 m, FVector3 v)  {
            Transform (ref m, ref v, out var r); return  r; }
        //[MI(O.AggressiveInlining)] public static Vector4  operator  * (FMatrix4x4 m, Vector4 v)  { Vector4  r; Transform (ref m, ref v, out r); return  r; }

        [MI(O.AggressiveInlining)] public static bool  Equals            (FMatrix4x4 a, FMatrix4x4 b) {
            Equals            (ref a, ref b, out var r); return r; }
        //[MI(O.AggressiveInlining)] public static Boolean  ApproximateEquals (FMatrix4x4 a, FMatrix4x4 b) { Boolean  r; ApproximateEquals (ref a, ref b, out r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Add               (FMatrix4x4 a, FMatrix4x4 b) {
            Add               (ref a, ref b, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Subtract          (FMatrix4x4 a, FMatrix4x4 b) {
            Subtract          (ref a, ref b, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Negate            (FMatrix4x4 m)             {
            Negate            (ref m,        out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Product           (FMatrix4x4 a, FMatrix4x4 b) {
            Product           (ref a, ref b, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Multiply          (FMatrix4x4 m, FScalar f)   {
            Multiply          (ref m, ref f, out var r); return r; }

        // Utilities //-------------------------------------------------------//

        [MI(O.AggressiveInlining)] public static void Lerp (ref FMatrix4x4 a, ref FMatrix4x4 b, ref FScalar amount, out FMatrix4x4 r) {
            LockstepDebug.Assert (amount > 0 && amount <= 1);
            r.m00 = a.m00 + ((b.m00 - a.m00) * amount);
            r.m01 = a.m01 + ((b.m01 - a.m01) * amount);
            r.m02 = a.m02 + ((b.m02 - a.m02) * amount);
            r.m03 = a.m03 + ((b.m03 - a.m03) * amount);
            r.m10 = a.m10 + ((b.m10 - a.m10) * amount);
            r.m11 = a.m11 + ((b.m11 - a.m11) * amount);
            r.m12 = a.m12 + ((b.m12 - a.m12) * amount);
            r.m13 = a.m13 + ((b.m13 - a.m13) * amount);
            r.m20 = a.m20 + ((b.m20 - a.m20) * amount);
            r.m21 = a.m21 + ((b.m21 - a.m21) * amount);
            r.m22 = a.m22 + ((b.m22 - a.m22) * amount);
            r.m23 = a.m23 + ((b.m23 - a.m23) * amount);
            r.m30 = a.m30 + ((b.m30 - a.m30) * amount);
            r.m31 = a.m31 + ((b.m31 - a.m31) * amount);
            r.m32 = a.m32 + ((b.m32 - a.m32) * amount);
            r.m33 = a.m33 + ((b.m33 - a.m33) * amount);
        }
        
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Lerp (FMatrix4x4 a, FMatrix4x4 b, FScalar amount) {
            Lerp (ref a, ref b, ref amount, out var r); return r; }

        // Maths //-----------------------------------------------------------//

        [MI(O.AggressiveInlining)] public static void Transpose (ref FMatrix4x4 m, out FMatrix4x4 r) {
            r.m00 = m.m00; r.m11 = m.m11;
            r.m22 = m.m22; r.m33 = m.m33;
            FScalar t = m.m01; r.m01 = m.m10; r.m10 = t;
                   t = m.m02; r.m02 = m.m20; r.m20 = t;
                   t = m.m03; r.m03 = m.m30; r.m30 = t;
                   t = m.m12; r.m12 = m.m21; r.m21 = t;
                   t = m.m13; r.m13 = m.m31; r.m31 = t;
                   t = m.m23; r.m23 = m.m32; r.m32 = t;
        }

        // [MI(O.AggressiveInlining)] public static void Decompose (ref FMatrix4x4 matrix, out FVector3 scale, out FQuaternion rotation, out FVector3 translation, out Boolean r) {
        //     translation.x = matrix.m30; translation.y = matrix.m31; translation.z = matrix.m32;
        //     FVector3 a = new FVector3 (matrix.m00, matrix.m10, matrix.m20);
        //     FVector3 b = new FVector3 (matrix.m01, matrix.m11, matrix.m21);
        //     FVector3 c = new FVector3 (matrix.m02, matrix.m12, matrix.m22);
        //     FScalar aLen = a.magnitude; scale.x = aLen;
        //     FScalar bLen = b.magnitude; scale.y = bLen;
        //     FScalar cLen = c.magnitude; scale.z = cLen;
        //     if (FScalar.IsApproximatelyZero (scale.x) || FScalar.IsApproximatelyZero (scale.y) || FScalar.IsApproximatelyZero (scale.z)) {
        //         rotation = FQuaternion.identity;
        //         r = false;
        //     }
        //     a.Normalize();
        //     b.Normalize();
        //     c.Normalize();
        //     
        //     FVector3 right = new FVector3 (a.x, b.x, c.x);
        //     FVector3 up = new FVector3 (a.y, b.y, c.y);
        //     FVector3 backward = new FVector3 (a.z, b.z, c.z);
        //     
        //     if (right.Equals (FVector3.zero)) right = FVector3.right;
        //     if (up.Equals (FVector3.zero)) up = FVector3.up;
        //     if (backward.Equals (FVector3.zero)) backward = FVector3.back;
        //     
        //     right.Normalize();
        //     up.Normalize();
        //     backward.Normalize();
        //     FMatrix4x4 rotMat;
        //     FMatrix4x4.CreateFromCartesianAxes (ref right, ref up, ref backward, out rotMat);
        //     FQuaternion.CreateFromRotationMatrix (ref rotMat, out rotation);
        //     r = true;
        // }

        [MI(O.AggressiveInlining)] public static void Determinant (ref FMatrix4x4 m, out FScalar r) {
            r = + m.m03 * m.m12 * m.m21 * m.m30 - m.m02 * m.m13 * m.m21 * m.m30
                     - m.m03 * m.m11 * m.m22 * m.m30 + m.m01 * m.m13 * m.m22 * m.m30
                     + m.m02 * m.m11 * m.m23 * m.m30 - m.m01 * m.m12 * m.m23 * m.m30
                     - m.m03 * m.m12 * m.m20 * m.m31 + m.m02 * m.m13 * m.m20 * m.m31
                     + m.m03 * m.m10 * m.m22 * m.m31 - m.m00 * m.m13 * m.m22 * m.m31
                     - m.m02 * m.m10 * m.m23 * m.m31 + m.m00 * m.m12 * m.m23 * m.m31
                     + m.m03 * m.m11 * m.m20 * m.m32 - m.m01 * m.m13 * m.m20 * m.m32
                     - m.m03 * m.m10 * m.m21 * m.m32 + m.m00 * m.m13 * m.m21 * m.m32
                     + m.m01 * m.m10 * m.m23 * m.m32 - m.m00 * m.m11 * m.m23 * m.m32
                     - m.m02 * m.m11 * m.m20 * m.m33 + m.m01 * m.m12 * m.m20 * m.m33
                     + m.m02 * m.m10 * m.m21 * m.m33 - m.m00 * m.m12 * m.m21 * m.m33
                     - m.m01 * m.m10 * m.m22 * m.m33 + m.m00 * m.m11 * m.m22 * m.m33;
        }

        [MI(O.AggressiveInlining)] public static void Invert (ref FMatrix4x4 m, out FMatrix4x4 r) {
            Determinant (ref m, out var d); FScalar s = 1 / d;
            FScalar m00 = m.m12 * m.m23 * m.m31 - m.m13 * m.m22 * m.m31 + m.m13 * m.m21 * m.m32 - m.m11 * m.m23 * m.m32 - m.m12 * m.m21 * m.m33 + m.m11 * m.m22 * m.m33;
            FScalar m01 = m.m03 * m.m22 * m.m31 - m.m02 * m.m23 * m.m31 - m.m03 * m.m21 * m.m32 + m.m01 * m.m23 * m.m32 + m.m02 * m.m21 * m.m33 - m.m01 * m.m22 * m.m33;
            FScalar m02 = m.m02 * m.m13 * m.m31 - m.m03 * m.m12 * m.m31 + m.m03 * m.m11 * m.m32 - m.m01 * m.m13 * m.m32 - m.m02 * m.m11 * m.m33 + m.m01 * m.m12 * m.m33;
            FScalar m03 = m.m03 * m.m12 * m.m21 - m.m02 * m.m13 * m.m21 - m.m03 * m.m11 * m.m22 + m.m01 * m.m13 * m.m22 + m.m02 * m.m11 * m.m23 - m.m01 * m.m12 * m.m23;
            FScalar m10 = m.m13 * m.m22 * m.m30 - m.m12 * m.m23 * m.m30 - m.m13 * m.m20 * m.m32 + m.m10 * m.m23 * m.m32 + m.m12 * m.m20 * m.m33 - m.m10 * m.m22 * m.m33;
            FScalar m11 = m.m02 * m.m23 * m.m30 - m.m03 * m.m22 * m.m30 + m.m03 * m.m20 * m.m32 - m.m00 * m.m23 * m.m32 - m.m02 * m.m20 * m.m33 + m.m00 * m.m22 * m.m33;
            FScalar m12 = m.m03 * m.m12 * m.m30 - m.m02 * m.m13 * m.m30 - m.m03 * m.m10 * m.m32 + m.m00 * m.m13 * m.m32 + m.m02 * m.m10 * m.m33 - m.m00 * m.m12 * m.m33;
            FScalar m13 = m.m02 * m.m13 * m.m20 - m.m03 * m.m12 * m.m20 + m.m03 * m.m10 * m.m22 - m.m00 * m.m13 * m.m22 - m.m02 * m.m10 * m.m23 + m.m00 * m.m12 * m.m23;
            FScalar m20 = m.m11 * m.m23 * m.m30 - m.m13 * m.m21 * m.m30 + m.m13 * m.m20 * m.m31 - m.m10 * m.m23 * m.m31 - m.m11 * m.m20 * m.m33 + m.m10 * m.m21 * m.m33;
            FScalar m21 = m.m03 * m.m21 * m.m30 - m.m01 * m.m23 * m.m30 - m.m03 * m.m20 * m.m31 + m.m00 * m.m23 * m.m31 + m.m01 * m.m20 * m.m33 - m.m00 * m.m21 * m.m33;
            FScalar m22 = m.m01 * m.m13 * m.m30 - m.m03 * m.m11 * m.m30 + m.m03 * m.m10 * m.m31 - m.m00 * m.m13 * m.m31 - m.m01 * m.m10 * m.m33 + m.m00 * m.m11 * m.m33;
            FScalar m23 = m.m03 * m.m11 * m.m20 - m.m01 * m.m13 * m.m20 - m.m03 * m.m10 * m.m21 + m.m00 * m.m13 * m.m21 + m.m01 * m.m10 * m.m23 - m.m00 * m.m11 * m.m23;
            FScalar m30 = m.m12 * m.m21 * m.m30 - m.m11 * m.m22 * m.m30 - m.m12 * m.m20 * m.m31 + m.m10 * m.m22 * m.m31 + m.m11 * m.m20 * m.m32 - m.m10 * m.m21 * m.m32;
            FScalar m31 = m.m01 * m.m22 * m.m30 - m.m02 * m.m21 * m.m30 + m.m02 * m.m20 * m.m31 - m.m00 * m.m22 * m.m31 - m.m01 * m.m20 * m.m32 + m.m00 * m.m21 * m.m32;
            FScalar m32 = m.m02 * m.m11 * m.m30 - m.m01 * m.m12 * m.m30 - m.m02 * m.m10 * m.m31 + m.m00 * m.m12 * m.m31 + m.m01 * m.m10 * m.m32 - m.m00 * m.m11 * m.m32;
            FScalar m33 = m.m01 * m.m12 * m.m20 - m.m02 * m.m11 * m.m20 + m.m02 * m.m10 * m.m21 - m.m00 * m.m12 * m.m21 - m.m01 * m.m10 * m.m22 + m.m00 * m.m11 * m.m22;
            r.m00 = m00; r.m01 = m01; r.m02 = m02; r.m03 = m03;
            r.m10 = m10; r.m11 = m11; r.m12 = m12; r.m13 = m13;
            r.m20 = m20; r.m21 = m21; r.m22 = m22; r.m23 = m23;
            r.m30 = m30; r.m31 = m31; r.m32 = m32; r.m33 = m33; 
            Multiply (ref r, ref s, out r);
        }

        // [MI(O.AggressiveInlining)] public static void Transform (ref FMatrix4x4 m, ref FQuaternion q, out FMatrix4x4 r) {
        //     Boolean qIsUnit; FQuaternion.IsUnit (ref q, out qIsUnit);
        //     LockstepDebug.Assert (qIsUnit);
        //     FScalar twoI = q.I + q.I, twoJ = q.J + q.J, twoK = q.K + q.K;
        //     FScalar twoUI = q.U * twoI, twoUJ = q.U * twoJ, twoUK = q.U * twoK;
        //     FScalar twoII = q.I * twoI, twoIJ = q.I * twoJ, twoIK = q.I * twoK;
        //     FScalar twoJJ = q.J * twoJ, twoJK = q.J * twoK, twoKK = q.K * twoK;
        //     FScalar tR0C0 = 1 - twoJJ - twoKK;
        //     FScalar tR1C0 = twoIJ - twoUK;
        //     FScalar tR2C0 = twoIK + twoUJ;
        //     FScalar tR0C1 = twoIJ + twoUK;
        //     FScalar tR1C1 = 1 - twoII - twoKK;
        //     FScalar tR2C1 = twoJK - twoUI;
        //     FScalar tR0C2 = twoIK - twoUJ;
        //     FScalar tR1C2 = twoJK + twoUI;
        //     FScalar tR2C2 = 1 - twoII - twoJJ;
        //     FScalar r0c0 = m.m00 * tR0C0 + m.m01 * tR1C0 + m.m02 * tR2C0;
        //     FScalar r0c1 = m.m00 * tR0C1 + m.m01 * tR1C1 + m.m02 * tR2C1;
        //     FScalar r0c2 = m.m00 * tR0C2 + m.m01 * tR1C2 + m.m02 * tR2C2;
        //     FScalar r1c0 = m.m10 * tR0C0 + m.m11 * tR1C0 + m.m12 * tR2C0;
        //     FScalar r1c1 = m.m10 * tR0C1 + m.m11 * tR1C1 + m.m12 * tR2C1;
        //     FScalar r1c2 = m.m10 * tR0C2 + m.m11 * tR1C2 + m.m12 * tR2C2;
        //     FScalar r2c0 = m.m20 * tR0C0 + m.m21 * tR1C0 + m.m22 * tR2C0;
        //     FScalar r2c1 = m.m20 * tR0C1 + m.m21 * tR1C1 + m.m22 * tR2C1;
        //     FScalar r2c2 = m.m20 * tR0C2 + m.m21 * tR1C2 + m.m22 * tR2C2;
        //     FScalar r3c0 = m.m30 * tR0C0 + m.m31 * tR1C0 + m.m32 * tR2C0;
        //     FScalar r3c1 = m.m30 * tR0C1 + m.m31 * tR1C1 + m.m32 * tR2C1;
        //     FScalar r3c2 = m.m30 * tR0C2 + m.m31 * tR1C2 + m.m32 * tR2C2;
        //     r.m00 = r0c0; r.m01 = r0c1; r.m02 = r0c2; r.m03 = m.m03;
        //     r.m10 = r1c0; r.m11 = r1c1; r.m12 = r1c2; r.m13 = m.m13;
        //     r.m20 = r2c0; r.m21 = r2c1; r.m22 = r2c2; r.m23 = m.m23;
        //     r.m30 = r3c0; r.m31 = r3c1; r.m32 = r3c2; r.m33 = m.m33; 
        // }

        [MI(O.AggressiveInlining)] public static void Transform (ref FMatrix4x4 m, ref FVector3 v, out FVector3 r) {
            FScalar x = (v.x * m.m00) + (v.y * m.m10) + (v.z * m.m20) + m.m30;
            FScalar y = (v.x * m.m01) + (v.y * m.m11) + (v.z * m.m21) + m.m31;
            FScalar z = (v.x * m.m02) + (v.y * m.m12) + (v.z * m.m22) + m.m32;
            FScalar w = (v.x * m.m03) + (v.y * m.m13) + (v.z * m.m23) + m.m33;
            r.x = x / w; r.y = y / w; r.z = z / w;
        }

        // [MI(O.AggressiveInlining)] public static void Transform (ref FMatrix4x4 m, ref Vector4 v, out Vector4 r) {
        //     FScalar x = (v.x * m.m00) + (v.y * m.m10) + (v.z * m.m20) + (v.W * m.m30);
        //     FScalar y = (v.x * m.m01) + (v.y * m.m11) + (v.z * m.m21) + (v.W * m.m31);
        //     FScalar z = (v.x * m.m02) + (v.y * m.m12) + (v.z * m.m22) + (v.W * m.m32);
        //     FScalar w = (v.x * m.m03) + (v.y * m.m13) + (v.z * m.m23) + (v.W * m.m33);
        //     r.x = x; r.y = y; r.z = z; r.W = w;
        // }

        [MI(O.AggressiveInlining)] public FScalar   Determinant ()                    {
            Determinant (ref this, out var r); return r; }
        [MI(O.AggressiveInlining)] public FMatrix4x4 Transpose   ()                    { Transpose (ref this, out this); return this; }
        [MI(O.AggressiveInlining)] public FMatrix4x4 Invert      ()                    { Invert (ref this, out this); return this; }
        //[MI(O.AggressiveInlining)] public FMatrix4x4 Transform   (Quaternion rotation) { FMatrix4x4 r; Transform (ref this, ref rotation, out r); return r; }
        [MI(O.AggressiveInlining)] public FVector3  Transform   (FVector3 v)           {
            Transform (ref this, ref v, out var r); return r; } 
        //[MI(O.AggressiveInlining)] public Vector4  Transform   (Vector4 v)           { Vector4 r; Transform (ref this, ref v, out r); return r; } 

        [MI(O.AggressiveInlining)] public static FScalar   Determinant (FMatrix4x4 matrix)                      {
            Determinant (ref matrix, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Transpose   (FMatrix4x4 input)                       {
            Transpose (ref input, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 Invert      (FMatrix4x4 matrix)                      {
            Invert (ref matrix, out var r); return r; }
        //[MI(O.AggressiveInlining)] public static FMatrix4x4 Transform   (FMatrix4x4 matrix, Quaternion rotation) { FMatrix4x4 r; Transform (ref matrix, ref rotation, out r); return r; }
        [MI(O.AggressiveInlining)] public static FVector3  Transform   (FMatrix4x4 matrix, FVector3 v)           {
            Transform (ref matrix, ref v, out var r); return r; } 
        //[MI(O.AggressiveInlining)] public static Vector4  Transform   (FMatrix4x4 matrix, Vector4 v)           { Vector4 r; Transform (ref matrix, ref v, out r); return r; } 

        // Creation //--------------------------------------------------------//

        [MI(O.AggressiveInlining)] public static void CreateTranslation (ref FVector3 position, out FMatrix4x4 r) {
            r.m00 = 1;          r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = 1;          r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;          r.m21 = 0;          r.m22 = 1;          r.m23 = 0;
            r.m30 = position.x; r.m31 = position.y; r.m32 = position.z; r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateTranslation (ref FScalar x, ref FScalar y, ref FScalar z, out FMatrix4x4 r) {
            r.m00 = 1;          r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = 1;          r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;          r.m21 = 0;          r.m22 = 1;          r.m23 = 0;
            r.m30 = x;          r.m31 = y;          r.m32 = z;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateScale (ref FVector3 scale, out FMatrix4x4 r) {
            r.m00 = scale.x;    r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = scale.y;    r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;          r.m21 = 0;          r.m22 = scale.z;    r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateScale (ref FScalar x, ref FScalar y, ref FScalar z, out FMatrix4x4 r) {
            r.m00 = x;          r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = y;          r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;          r.m21 = 0;          r.m22 = z;          r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateScale (ref FScalar scale, out FMatrix4x4 r) {
            r.m00 = scale;      r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = scale;      r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;          r.m21 = 0;          r.m22 = scale;      r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateRotationX (ref FScalar turn, out FMatrix4x4 r) {
            FScalar cos = FScalar.Cos (turn), sin = FScalar.Sin (turn);
            r.m00 = 1;          r.m01 = 0;          r.m02 = 0;          r.m03 = 0;
            r.m10 = 0;          r.m11 = cos;        r.m12 = sin;        r.m13 = 0;
            r.m20 = 0;          r.m21 = -sin;       r.m22 = cos;        r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateRotationY (ref FScalar turn, out FMatrix4x4 r) {
            FScalar cos = FScalar.Cos (turn), sin = FScalar.Sin (turn);
            r.m00 = cos;        r.m01 = 0;          r.m02 = -sin;       r.m03 = 0;
            r.m10 = 0;          r.m11 = 1;          r.m12 = 0;          r.m13 = 0;
            r.m20 = sin;        r.m21 = 0;          r.m22 = cos;        r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateRotationZ (ref FScalar turn, out FMatrix4x4 r) {
            FScalar cos = FScalar.Cos (turn), sin = FScalar.Sin (turn);
            r.m00 = cos;       r.m01 = sin;         r.m02 = 0;          r.m03 = 0;
            r.m10 = -sin;      r.m11 = cos;         r.m12 = 0;          r.m13 = 0;
            r.m20 = 0;         r.m21 = 0;           r.m22 = 1;          r.m23 = 0;
            r.m30 = 0;         r.m31 = 0;           r.m32 = 0;          r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static void CreateFromAxisAngle (ref FVector3 axis, ref FScalar turn, out FMatrix4x4 r) {
            FScalar x = axis.x, y = axis.y, z = axis.z;
            FScalar sin = FScalar.Sin (turn), cos = FScalar.Cos (turn);
            FScalar xx = x * x, yy = y * y, zz = z * z;
            FScalar xy = x * y, xz = x * z, yz = y * z;
            r.m00 = xx + (cos * (1 - xx));       r.m01 = xy - (cos * xy) + (sin * z); r.m02 = xz - (cos * xz) - (sin * y); r.m03 = 0;
            r.m10 = xy - (cos * xy) - (sin * z); r.m11 = yy + (cos * (1 - yy));       r.m12 = yz - (cos * yz) + (sin * x); r.m13 = 0;
            r.m20 = xz - (cos * xz) + (sin * y); r.m21 = yz - (cos * yz) - (sin * x); r.m22 = zz + (cos * (1 - zz));       r.m23 = 0;
            r.m30 = 0;                           r.m31 = 0;                           r.m32 = 0;                           r.m33 = 1;
        }

        // Axes must be pair-wise perpendicular and have unit length.
        [MI(O.AggressiveInlining)] public static void CreateFromCartesianAxes (ref FVector3 right, ref FVector3 up, ref FVector3 backward, out FMatrix4x4 r) {
            r.m00 = right.x;    r.m01 = right.y;    r.m02 = right.z;    r.m03 = 0;
            r.m10 = up.x;       r.m11 = up.y;       r.m12 = up.z;       r.m13 = 0;
            r.m20 = backward.x; r.m21 = backward.y; r.m22 = backward.z; r.m23 = 0;
            r.m30 = 0;          r.m31 = 0;          r.m32 = 0;          r.m33 = 1;
        }

        // [MI(O.AggressiveInlining)] public static void CreateWorld (ref FVector3 position, ref FVector3 forward, ref FVector3 up, out FMatrix4x4 r) {
        //     FVector3 backward; FVector3.Negate (ref forward, out backward); FVector3.Normalise (ref backward, out backward);
        //     FVector3 right; FVector3.Cross (ref up, ref backward, out right); FVector3.Normalise (ref right, out right);
        //     FVector3 finalUp; FVector3.Cross (ref right, ref backward, out finalUp); FVector3.Normalise (ref finalUp, out finalUp);
        //     r.m00 = right.x;    r.m01 = right.y;    r.m02 = right.z;    r.m03 = 0;
        //     r.m10 = finalUp.x;  r.m11 = finalUp.y;  r.m12 = finalUp.z;  r.m13 = 0;
        //     r.m20 = backward.x; r.m21 = backward.y; r.m22 = backward.z; r.m23 = 0;
        //     r.m30 = position.x; r.m31 = position.y; r.m32 = position.z; r.m33 = 1;
        // }

        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        [MI(O.AggressiveInlining)] public static void CreateFromQuaternion (ref FQuaternion q, out FMatrix4x4 r) {
            Boolean qIsUnit = FQuaternion.IsUnit (q); LockstepDebug.Assert (qIsUnit);
            FScalar twoX = q.x + q.x, twoY = q.y + q.y, twoZ = q.z + q.z;
            FScalar twoWI = q.w * twoX, twoWY = q.w * twoY, twoWZ = q.w * twoZ;
            FScalar twoXX = q.x * twoX, twoXY = q.x * twoY, twoXZ = q.x * twoZ;
            FScalar twoYY = q.y * twoY, twoYZ = q.y * twoZ, twoZZ = q.z * twoZ;
            r.m00 = 1 - twoYY - twoZZ; r.m10 = twoXY - twoWZ;     r.m20 = twoXZ + twoWY;     r.m30 = 0;
            r.m01 = twoXY + twoWZ;     r.m11 = 1 - twoXX - twoZZ; r.m21 = twoYZ - twoWI;     r.m31 = 0;
            r.m02 = twoXZ - twoWY;     r.m12 = twoYZ + twoWI;     r.m22 = 1 - twoXX - twoYY; r.m32 = 0;
            r.m03 = 0;                 r.m13 = 0;                 r.m23 = 0;                 r.m33 = 1;
        }

        // Angle of rotation, in radians. Angles are measured anti-clockwise when viewed from the rotation axis (positive side) toward the origin.
        [MI(O.AggressiveInlining)] public static void CreateFromYawPitchRoll (ref FScalar yaw, ref FScalar pitch, ref FScalar roll, out FMatrix4x4 r) {
            FScalar cy = FScalar.Cos (yaw), sy = FScalar.Sin (yaw);
            FScalar cx = FScalar.Cos (pitch), sx = FScalar.Sin (pitch);
            FScalar cz = FScalar.Cos (roll), sz = FScalar.Sin (roll);
            r.m00 =  cz*cy+sz*sx*sy; r.m01 =  sz*cx; r.m02 = -cz*sy+sz*sx*cy; r.m03 = 0;
            r.m10 = -sz*cy+cz*sx*sy; r.m11 =  cz*cx; r.m12 = -cz*sy+sz*sx*cy; r.m13 = 0;
            r.m20 =  cx*sy;          r.m21 = -sx;    r.m22 =  cx*cy;          r.m23 = 0;
            r.m30 = 0;               r.m31 = 0;      r.m32 = 0;               r.m33 = 1;
        }

        // http://msdn.microsoft.com/en-us/library/bb205351(v=vs.85).aspx
        // [MI(O.AggressiveInlining)] public static void CreatePerspectiveFieldOfView (ref FScalar fieldOfView, ref FScalar aspectRatio, ref FScalar nearPlaneDistance, ref FScalar farPlaneDistance, out FMatrix4x4 r) {
        //     LockstepDebug.Assert (fieldOfView > 0 && fieldOfView < FScalar.Pi);
        //     LockstepDebug.Assert (nearPlaneDistance > 0);
        //     LockstepDebug.Assert (farPlaneDistance > 0);
        //     LockstepDebug.Assert (nearPlaneDistance < farPlaneDistance);
        //     FScalar yScale = (FScalar) 1 / (Maths.Tan (fieldOfView * Maths.Half));
        //     FScalar xScale = yScale / aspectRatio;
        //     FScalar f1 = farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
        //     FScalar f2 = (nearPlaneDistance * farPlaneDistance) / (nearPlaneDistance - farPlaneDistance);
        //     r.m00 = xScale; r.m01 = 0;      r.m02 = 0;  r.m03 =  0;
        //     r.m10 = 0;      r.m11 = yScale; r.m12 = 0;  r.m13 =  0;
        //     r.m20 = 0;      r.m21 = 0;      r.m22 = f1; r.m23 = -1;
        //     r.m30 = 0;      r.m31 = 0;      r.m32 = f2; r.m33 =  0;
        // }

        // http://msdn.microsoft.com/en-us/library/bb205355(v=vs.85).aspx
        [MI(O.AggressiveInlining)] public static void CreatePerspective (ref FScalar width, ref FScalar height, ref FScalar nearPlaneDistance, ref FScalar farPlaneDistance, out FMatrix4x4 r) {
            LockstepDebug.Assert (nearPlaneDistance > 0);
            LockstepDebug.Assert (farPlaneDistance > 0);
            LockstepDebug.Assert (nearPlaneDistance < farPlaneDistance);
            r.m00 = (nearPlaneDistance * 2) / width;
            r.m01 = r.m02 = r.m03 = 0;
            r.m11 = (nearPlaneDistance * 2) / height;
            r.m10 = r.m12 = r.m13 = 0;
            r.m22 = farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
            r.m20 = r.m21 = 0;
            r.m23 = -1;
            r.m30 = r.m31 = r.m33 = 0;
            r.m32 = (nearPlaneDistance * farPlaneDistance) / (nearPlaneDistance - farPlaneDistance);
        }

        // http://msdn.microsoft.com/en-us/library/bb205354(v=vs.85).aspx
        [MI(O.AggressiveInlining)] public static void CreatePerspectiveOffCenter (ref FScalar left, ref FScalar right, ref FScalar bottom, ref FScalar top, ref FScalar nearPlaneDistance, ref FScalar farPlaneDistance, out FMatrix4x4 r) {
            LockstepDebug.Assert (nearPlaneDistance > 0);
            LockstepDebug.Assert (farPlaneDistance > 0);
            LockstepDebug.Assert (nearPlaneDistance < farPlaneDistance);
            r.m00 = (nearPlaneDistance * 2) / (right - left);
            r.m01 = r.m02 = r.m03 = 0;
            r.m11 = (nearPlaneDistance * 2) / (top - bottom);
            r.m10 = r.m12 = r.m13 = 0;
            r.m20 = (left + right) / (right - left);
            r.m21 = (top + bottom) / (top - bottom);
            r.m22 = farPlaneDistance / (nearPlaneDistance - farPlaneDistance);
            r.m23 = -1;
            r.m32 = (nearPlaneDistance * farPlaneDistance) / (nearPlaneDistance - farPlaneDistance);
            r.m30 = r.m31 = r.m33 = 0;
        }

        // http://msdn.microsoft.com/en-us/library/bb205349(v=vs.85).aspx
        [MI(O.AggressiveInlining)] public static void CreateOrthographic (ref FScalar width, ref FScalar height, ref FScalar zNearPlane, ref FScalar zFarPlane, out FMatrix4x4 r) {
            r.m00 = 2 / width;
            r.m01 = r.m02 = r.m03 = 0;
            r.m11 = 2 / height;
            r.m10 = r.m12 = r.m13 = 0;
            r.m22 = 1 / (zNearPlane - zFarPlane);
            r.m20 = r.m21 = r.m23 = 0;
            r.m30 = r.m31 = 0;
            r.m32 = zNearPlane / (zNearPlane - zFarPlane);
            r.m33 = 1;
        }

        // http://msdn.microsoft.com/en-us/library/bb205348(v=vs.85).aspx
        [MI(O.AggressiveInlining)] public static void CreateOrthographicOffCenter (ref FScalar left, ref FScalar right, ref FScalar bottom, ref FScalar top, ref FScalar zNearPlane, ref FScalar zFarPlane, out FMatrix4x4 r) {
            r.m00 = 2 / (right - left);
            r.m01 = r.m02 = r.m03 = 0;
            r.m11 = 2 / (top - bottom);
            r.m10 = r.m12 = r.m13 = 0;
            r.m22 = 1 / (zNearPlane - zFarPlane);
            r.m20 = r.m21 = r.m23 = 0;
            r.m30 = (left + right) / (left - right);
            r.m31 = (top + bottom) / (bottom - top);
            r.m32 = zNearPlane / (zNearPlane - zFarPlane);
            r.m33 = 1;
        }

        // http://msdn.microsoft.com/en-us/library/bb205343(v=VS.85).aspx
        [MI(O.AggressiveInlining)] public static void CreateLookAt (ref FVector3 cameraPosition, ref FVector3 cameraTarget, ref FVector3 cameraUpVector, out FMatrix4x4 r)
        {
            FVector3 forward = cameraPosition - cameraTarget; forward.Normalize();
            FVector3 right = cameraUpVector - forward; right.Normalize();
            FVector3 up = FVector3.Cross (forward, right); up.Normalize();
            FScalar a = FVector3.Dot(right, cameraPosition);
            FScalar b = FVector3.Dot (up, cameraPosition);
            FScalar c = FVector3.Dot (forward, cameraPosition);
            r.m00 = right.x;    r.m01 = up.x;       r.m02 = forward.x;  r.m03 = 0;
            r.m10 = right.y;    r.m11 = up.y;       r.m12 = forward.y;  r.m13 = 0;
            r.m20 = right.z;    r.m21 = up.z;       r.m22 = forward.z;  r.m23 = 0;
            r.m30 = -a;         r.m31 = -b;         r.m32 = -c;         r.m33 = 1;
        }

        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateTranslation            (FScalar xPosition, FScalar yPosition, FScalar zPosition) {
            CreateTranslation (ref xPosition, ref yPosition, ref zPosition, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateTranslation            (FVector3 position) {
            CreateTranslation (ref position, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateScale                  (FScalar xScale, FScalar yScale, FScalar zScale) {
            CreateScale (ref xScale, ref yScale, ref zScale, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateScale                  (FVector3 scales) {
            CreateScale (ref scales, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateScale                  (FScalar scale) {
            CreateScale (ref scale, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateRotationX              (FScalar turn) {
            CreateRotationX (ref turn, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateRotationY              (FScalar turn) {
            CreateRotationY (ref turn, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateRotationZ              (FScalar turn) {
            CreateRotationZ (ref turn, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateFromAxisAngle          (FVector3 axis, FScalar turn) {
            CreateFromAxisAngle (ref axis, ref turn, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateFromCartesianAxes      (FVector3 right, FVector3 up, FVector3 backward) {
            CreateFromCartesianAxes (ref right, ref up, ref backward, out var r); return r; }
        //[MI(O.AggressiveInlining)] public static FMatrix4x4 CreateWorld                  (FVector3 position, FVector3 forward, FVector3 up) { FMatrix4x4 r; CreateWorld (ref position, ref forward, ref up, out r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateFromQuaternion         (FQuaternion quaternion) { FMatrix4x4 r; CreateFromQuaternion (ref quaternion, out r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateFromYawPitchRoll       (FScalar yaw, FScalar pitch, FScalar roll) {
            CreateFromYawPitchRoll (ref yaw, ref pitch, ref roll, out var r); return r; }
        //[MI(O.AggressiveInlining)] public static FMatrix4x4 CreatePerspectiveFieldOfView (FScalar fieldOfView,  FScalar aspectRatio, FScalar nearPlane, FScalar farPlane) { FMatrix4x4 r; CreatePerspectiveFieldOfView (ref fieldOfView, ref aspectRatio, ref nearPlane, ref farPlane, out r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreatePerspective            (FScalar width, FScalar height, FScalar nearPlane, FScalar farPlane) {
            CreatePerspective (ref width, ref height, ref nearPlane, ref farPlane, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreatePerspectiveOffCenter   (FScalar left, FScalar right, FScalar bottom, FScalar top, FScalar nearPlane, FScalar farPlane) {
            CreatePerspectiveOffCenter (ref left, ref right, ref bottom, ref top, ref nearPlane, ref farPlane, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateOrthographic           (FScalar width, FScalar height, FScalar nearPlane, FScalar farPlane) {
            CreateOrthographic (ref width, ref height, ref nearPlane, ref farPlane, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateOrthographicOffCenter  (FScalar left, FScalar right, FScalar bottom, FScalar top, FScalar nearPlane, FScalar farPlane) {
            CreateOrthographicOffCenter (ref left, ref right, ref bottom, ref top, ref nearPlane, ref farPlane, out var r); return r; }
        [MI(O.AggressiveInlining)] public static FMatrix4x4 CreateLookAt                 (FVector3 cameraPosition, FVector3 cameraTarget, FVector3 cameraUpVector) {
            CreateLookAt (ref cameraPosition, ref cameraTarget, ref cameraUpVector, out var r); return r; }

    }
}