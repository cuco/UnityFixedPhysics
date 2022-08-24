//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------
using System;
using System.Runtime.CompilerServices;
using static Unity.Mathematics.math;

#pragma warning disable 0660, 0661

namespace Unity.Mathematics.FixedPoint
{
    [System.Serializable]
    public partial struct fp4x4 : System.IEquatable<fp4x4>, IFormattable
    {
        public fp4 c0;
        public fp4 c1;
        public fp4 c2;
        public fp4 c3;

        /// <summary>fp4x4 identity transform.</summary>
        public static readonly fp4x4 identity = new fp4x4((fp)1, (fp)0, (fp)0, (fp)0,   (fp)0, (fp)1, (fp)0, (fp)0,   (fp)0, (fp)0, (fp)1, (fp)0,   (fp)0, (fp)0, (fp)0, (fp)1);

        /// <summary>fp4x4 zero value.</summary>
        public static readonly fp4x4 zero;

        /// <summary>Constructs a fp4x4 matrix from four fp4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(fp4 c0, fp4 c1, fp4 c2, fp4 c3)
        {
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
        }

        /// <summary>Constructs a fp4x4 matrix from 16 fp values given in row-major order.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(fp m00, fp m01, fp m02, fp m03,
                     fp m10, fp m11, fp m12, fp m13,
                     fp m20, fp m21, fp m22, fp m23,
                     fp m30, fp m31, fp m32, fp m33)
        {
            this.c0 = new fp4(m00, m10, m20, m30);
            this.c1 = new fp4(m01, m11, m21, m31);
            this.c2 = new fp4(m02, m12, m22, m32);
            this.c3 = new fp4(m03, m13, m23, m33);
        }

        /// <summary>Constructs a fp4x4 matrix from a single fp value by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(fp v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
        }

        /// <summary>Constructs a fp4x4 matrix from a single int value by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(int v)
        {
            this.c0 = (fp4)v;
            this.c1 = (fp4)v;
            this.c2 = (fp4)v;
            this.c3 = (fp4)v;
        }

        /// <summary>Constructs a fp4x4 matrix from a int4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(int4x4 v)
        {
            this.c0 = (fp4)v.c0;
            this.c1 = (fp4)v.c1;
            this.c2 = (fp4)v.c2;
            this.c3 = (fp4)v.c3;
        }

        /// <summary>Constructs a fp4x4 matrix from a single uint value by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(uint v)
        {
            this.c0 = (fp4)v;
            this.c1 = (fp4)v;
            this.c2 = (fp4)v;
            this.c3 = (fp4)v;
        }

        /// <summary>Constructs a fp4x4 matrix from a uint4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fp4x4(uint4x4 v)
        {
            this.c0 = (fp4)v.c0;
            this.c1 = (fp4)v.c1;
            this.c2 = (fp4)v.c2;
            this.c3 = (fp4)v.c3;
        }


        /// <summary>Implicitly converts a single fp value to a fp4x4 matrix by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fp4x4(fp v) { return new fp4x4(v); }

        /// <summary>Explicitly converts a single int value to a fp4x4 matrix by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator fp4x4(int v) { return new fp4x4(v); }

        /// <summary>Explicitly converts a int4x4 matrix to a fp4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator fp4x4(int4x4 v) { return new fp4x4(v); }

        /// <summary>Explicitly converts a single uint value to a fp4x4 matrix by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator fp4x4(uint v) { return new fp4x4(v); }

        /// <summary>Explicitly converts a uint4x4 matrix to a fp4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator fp4x4(uint4x4 v) { return new fp4x4(v); }


        /// <summary>Returns the result of a componentwise multiplication operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator * (fp4x4 lhs, fp4x4 rhs) { return new fp4x4 (lhs.c0 * rhs.c0, lhs.c1 * rhs.c1, lhs.c2 * rhs.c2, lhs.c3 * rhs.c3); }

        /// <summary>Returns the result of a componentwise multiplication operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator * (fp4x4 lhs, fp rhs) { return new fp4x4 (lhs.c0 * rhs, lhs.c1 * rhs, lhs.c2 * rhs, lhs.c3 * rhs); }

        /// <summary>Returns the result of a componentwise multiplication operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator * (fp lhs, fp4x4 rhs) { return new fp4x4 (lhs * rhs.c0, lhs * rhs.c1, lhs * rhs.c2, lhs * rhs.c3); }


        /// <summary>Returns the result of a componentwise addition operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator + (fp4x4 lhs, fp4x4 rhs) { return new fp4x4 (lhs.c0 + rhs.c0, lhs.c1 + rhs.c1, lhs.c2 + rhs.c2, lhs.c3 + rhs.c3); }

        /// <summary>Returns the result of a componentwise addition operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator + (fp4x4 lhs, fp rhs) { return new fp4x4 (lhs.c0 + rhs, lhs.c1 + rhs, lhs.c2 + rhs, lhs.c3 + rhs); }

        /// <summary>Returns the result of a componentwise addition operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator + (fp lhs, fp4x4 rhs) { return new fp4x4 (lhs + rhs.c0, lhs + rhs.c1, lhs + rhs.c2, lhs + rhs.c3); }


        /// <summary>Returns the result of a componentwise subtraction operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator - (fp4x4 lhs, fp4x4 rhs) { return new fp4x4 (lhs.c0 - rhs.c0, lhs.c1 - rhs.c1, lhs.c2 - rhs.c2, lhs.c3 - rhs.c3); }

        /// <summary>Returns the result of a componentwise subtraction operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator - (fp4x4 lhs, fp rhs) { return new fp4x4 (lhs.c0 - rhs, lhs.c1 - rhs, lhs.c2 - rhs, lhs.c3 - rhs); }

        /// <summary>Returns the result of a componentwise subtraction operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator - (fp lhs, fp4x4 rhs) { return new fp4x4 (lhs - rhs.c0, lhs - rhs.c1, lhs - rhs.c2, lhs - rhs.c3); }


        /// <summary>Returns the result of a componentwise division operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator / (fp4x4 lhs, fp4x4 rhs) { return new fp4x4 (lhs.c0 / rhs.c0, lhs.c1 / rhs.c1, lhs.c2 / rhs.c2, lhs.c3 / rhs.c3); }

        /// <summary>Returns the result of a componentwise division operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator / (fp4x4 lhs, fp rhs) { return new fp4x4 (lhs.c0 / rhs, lhs.c1 / rhs, lhs.c2 / rhs, lhs.c3 / rhs); }

        /// <summary>Returns the result of a componentwise division operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator / (fp lhs, fp4x4 rhs) { return new fp4x4 (lhs / rhs.c0, lhs / rhs.c1, lhs / rhs.c2, lhs / rhs.c3); }


        /// <summary>Returns the result of a componentwise modulus operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator % (fp4x4 lhs, fp4x4 rhs) { return new fp4x4 (lhs.c0 % rhs.c0, lhs.c1 % rhs.c1, lhs.c2 % rhs.c2, lhs.c3 % rhs.c3); }

        /// <summary>Returns the result of a componentwise modulus operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator % (fp4x4 lhs, fp rhs) { return new fp4x4 (lhs.c0 % rhs, lhs.c1 % rhs, lhs.c2 % rhs, lhs.c3 % rhs); }

        /// <summary>Returns the result of a componentwise modulus operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator % (fp lhs, fp4x4 rhs) { return new fp4x4 (lhs % rhs.c0, lhs % rhs.c1, lhs % rhs.c2, lhs % rhs.c3); }


        /// <summary>Returns the result of a componentwise increment operation on a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator ++ (fp4x4 val) { return new fp4x4 (++val.c0, ++val.c1, ++val.c2, ++val.c3); }


        /// <summary>Returns the result of a componentwise decrement operation on a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator -- (fp4x4 val) { return new fp4x4 (--val.c0, --val.c1, --val.c2, --val.c3); }


        /// <summary>Returns the result of a componentwise less than operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 < rhs.c0, lhs.c1 < rhs.c1, lhs.c2 < rhs.c2, lhs.c3 < rhs.c3); }

        /// <summary>Returns the result of a componentwise less than operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 < rhs, lhs.c1 < rhs, lhs.c2 < rhs, lhs.c3 < rhs); }

        /// <summary>Returns the result of a componentwise less than operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs < rhs.c0, lhs < rhs.c1, lhs < rhs.c2, lhs < rhs.c3); }


        /// <summary>Returns the result of a componentwise less or equal operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 <= rhs.c0, lhs.c1 <= rhs.c1, lhs.c2 <= rhs.c2, lhs.c3 <= rhs.c3); }

        /// <summary>Returns the result of a componentwise less or equal operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 <= rhs, lhs.c1 <= rhs, lhs.c2 <= rhs, lhs.c3 <= rhs); }

        /// <summary>Returns the result of a componentwise less or equal operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs <= rhs.c0, lhs <= rhs.c1, lhs <= rhs.c2, lhs <= rhs.c3); }


        /// <summary>Returns the result of a componentwise greater than operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 > rhs.c0, lhs.c1 > rhs.c1, lhs.c2 > rhs.c2, lhs.c3 > rhs.c3); }

        /// <summary>Returns the result of a componentwise greater than operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 > rhs, lhs.c1 > rhs, lhs.c2 > rhs, lhs.c3 > rhs); }

        /// <summary>Returns the result of a componentwise greater than operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs > rhs.c0, lhs > rhs.c1, lhs > rhs.c2, lhs > rhs.c3); }


        /// <summary>Returns the result of a componentwise greater or equal operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 >= rhs.c0, lhs.c1 >= rhs.c1, lhs.c2 >= rhs.c2, lhs.c3 >= rhs.c3); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 >= rhs, lhs.c1 >= rhs, lhs.c2 >= rhs, lhs.c3 >= rhs); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs >= rhs.c0, lhs >= rhs.c1, lhs >= rhs.c2, lhs >= rhs.c3); }


        /// <summary>Returns the result of a componentwise unary minus operation on a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator - (fp4x4 val) { return new fp4x4 (-val.c0, -val.c1, -val.c2, -val.c3); }


        /// <summary>Returns the result of a componentwise unary plus operation on a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 operator + (fp4x4 val) { return new fp4x4 (+val.c0, +val.c1, +val.c2, +val.c3); }


        /// <summary>Returns the result of a componentwise equality operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 == rhs.c0, lhs.c1 == rhs.c1, lhs.c2 == rhs.c2, lhs.c3 == rhs.c3); }

        /// <summary>Returns the result of a componentwise equality operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 == rhs, lhs.c1 == rhs, lhs.c2 == rhs, lhs.c3 == rhs); }

        /// <summary>Returns the result of a componentwise equality operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs == rhs.c0, lhs == rhs.c1, lhs == rhs.c2, lhs == rhs.c3); }


        /// <summary>Returns the result of a componentwise not equal operation on two fp4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (fp4x4 lhs, fp4x4 rhs) { return new bool4x4 (lhs.c0 != rhs.c0, lhs.c1 != rhs.c1, lhs.c2 != rhs.c2, lhs.c3 != rhs.c3); }

        /// <summary>Returns the result of a componentwise not equal operation on a fp4x4 matrix and a fp value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (fp4x4 lhs, fp rhs) { return new bool4x4 (lhs.c0 != rhs, lhs.c1 != rhs, lhs.c2 != rhs, lhs.c3 != rhs); }

        /// <summary>Returns the result of a componentwise not equal operation on a fp value and a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (fp lhs, fp4x4 rhs) { return new bool4x4 (lhs != rhs.c0, lhs != rhs.c1, lhs != rhs.c2, lhs != rhs.c3); }



        /// <summary>Returns the fp4 element at a specified index.</summary>
        unsafe public ref fp4 this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 4)
                    throw new System.ArgumentException("index must be between[0...3]");
#endif
                fixed (fp4x4* array = &this) { return ref ((fp4*)array)[index]; }
            }
        }

        /// <summary>Returns true if the fp4x4 is equal to a given fp4x4, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(fp4x4 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2) && c3.Equals(rhs.c3); }

        /// <summary>Returns true if the fp4x4 is equal to a given fp4x4, false otherwise.</summary>
        public override bool Equals(object o) { return Equals((fp4x4)o); }


        /// <summary>Returns a hash code for the fp4x4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() { return (int)fpmath.hash(this); }


        /// <summary>Returns a string representation of the fp4x4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("fp4x4({0}, {1}, {2}, {3},  {4}, {5}, {6}, {7},  {8}, {9}, {10}, {11},  {12}, {13}, {14}, {15})", c0.x, c1.x, c2.x, c3.x, c0.y, c1.y, c2.y, c3.y, c0.z, c1.z, c2.z, c3.z, c0.w, c1.w, c2.w, c3.w);
        }

        /// <summary>Returns a string representation of the fp4x4 using a specified format and culture-specific format information.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("fp4x4({0}, {1}, {2}, {3},  {4}, {5}, {6}, {7},  {8}, {9}, {10}, {11},  {12}, {13}, {14}, {15})", c0.x.ToString(format, formatProvider), c1.x.ToString(format, formatProvider), c2.x.ToString(format, formatProvider), c3.x.ToString(format, formatProvider), c0.y.ToString(format, formatProvider), c1.y.ToString(format, formatProvider), c2.y.ToString(format, formatProvider), c3.y.ToString(format, formatProvider), c0.z.ToString(format, formatProvider), c1.z.ToString(format, formatProvider), c2.z.ToString(format, formatProvider), c3.z.ToString(format, formatProvider), c0.w.ToString(format, formatProvider), c1.w.ToString(format, formatProvider), c2.w.ToString(format, formatProvider), c3.w.ToString(format, formatProvider));
        }

    }

    public static partial class fpmath
    {
        /// <summary>Returns a fp4x4 matrix constructed from four fp4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(fp4 c0, fp4 c1, fp4 c2, fp4 c3) { return new fp4x4(c0, c1, c2, c3); }

        /// <summary>Returns a fp4x4 matrix constructed from from 16 fp values given in row-major order.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(fp m00, fp m01, fp m02, fp m03,
                                  fp m10, fp m11, fp m12, fp m13,
                                  fp m20, fp m21, fp m22, fp m23,
                                  fp m30, fp m31, fp m32, fp m33)
        {
            return new fp4x4(m00, m01, m02, m03,
                             m10, m11, m12, m13,
                             m20, m21, m22, m23,
                             m30, m31, m32, m33);
        }

        /// <summary>Returns a fp4x4 matrix constructed from a single fp value by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(fp v) { return new fp4x4(v); }

        /// <summary>Returns a fp4x4 matrix constructed from a single int value by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(int v) { return new fp4x4(v); }

        /// <summary>Return a fp4x4 matrix constructed from a int4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(int4x4 v) { return new fp4x4(v); }

        /// <summary>Returns a fp4x4 matrix constructed from a single uint value by converting it to fp and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(uint v) { return new fp4x4(v); }

        /// <summary>Return a fp4x4 matrix constructed from a uint4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(uint4x4 v) { return new fp4x4(v); }

        /// <summary>Return the fp4x4 transpose of a fp4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 transpose(fp4x4 v)
        {
            return fp4x4(
                v.c0.x, v.c0.y, v.c0.z, v.c0.w,
                v.c1.x, v.c1.y, v.c1.z, v.c1.w,
                v.c2.x, v.c2.y, v.c2.z, v.c2.w,
                v.c3.x, v.c3.y, v.c3.z, v.c3.w);
        }

        /// <summary>Return the result of rotating a fp3 vector by a fp4x4 matrix</summary>
        /// <param name ="a">Left hand side matrix argument that specifies the rotation.</param>
        /// <param name ="b">Right hand side vector argument to be rotated.</param>
        /// <returns>The rotated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3 rotate(fp4x4 a, fp3 b)
        {
            return (a.c0 * b.x + a.c1 * b.y + a.c2 * b.z).xyz;
        }

        /// <summary>Return the result of transforming a fp3 point by a fp4x4 matrix</summary>
        /// <param name ="a">Left hand side matrix argument that specifies the transformation.</param>
        /// <param name ="b">Right hand side point argument to be transformed.</param>
        /// <returns>The transformed point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3 transform(fp4x4 a, fp3 b)
        {
            return (a.c0 * b.x + a.c1 * b.y + a.c2 * b.z + a.c3).xyz;
        }

        /// <summary>Returns the fp4x4 full inverse of a fp4x4 matrix.</summary>
        /// <param name="m">Matrix to invert.</param>
        /// <returns>The inverted matrix.</returns>
        public static fp4x4 inverse(fp4x4 m)
        {
            fp4 c0 = m.c0;
            fp4 c1 = m.c1;
            fp4 c2 = m.c2;
            fp4 c3 = m.c3;

            fp4 r0y_r1y_r0x_r1x = movelh(c1, c0);
            fp4 r0z_r1z_r0w_r1w = movelh(c2, c3);
            fp4 r2y_r3y_r2x_r3x = movehl(c0, c1);
            fp4 r2z_r3z_r2w_r3w = movehl(c3, c2);

            fp4 r1y_r2y_r1x_r2x = shuffle(c1, c0, ShuffleComponent.LeftY, ShuffleComponent.LeftZ, ShuffleComponent.RightY, ShuffleComponent.RightZ);
            fp4 r1z_r2z_r1w_r2w = shuffle(c2, c3, ShuffleComponent.LeftY, ShuffleComponent.LeftZ, ShuffleComponent.RightY, ShuffleComponent.RightZ);
            fp4 r3y_r0y_r3x_r0x = shuffle(c1, c0, ShuffleComponent.LeftW, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightX);
            fp4 r3z_r0z_r3w_r0w = shuffle(c2, c3, ShuffleComponent.LeftW, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightX);

            fp4 r0_wzyx = shuffle(r0z_r1z_r0w_r1w, r0y_r1y_r0x_r1x, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);
            fp4 r1_wzyx = shuffle(r0z_r1z_r0w_r1w, r0y_r1y_r0x_r1x, ShuffleComponent.LeftW, ShuffleComponent.LeftY, ShuffleComponent.RightY, ShuffleComponent.RightW);
            fp4 r2_wzyx = shuffle(r2z_r3z_r2w_r3w, r2y_r3y_r2x_r3x, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);
            fp4 r3_wzyx = shuffle(r2z_r3z_r2w_r3w, r2y_r3y_r2x_r3x, ShuffleComponent.LeftW, ShuffleComponent.LeftY, ShuffleComponent.RightY, ShuffleComponent.RightW);
            fp4 r0_xyzw = shuffle(r0y_r1y_r0x_r1x, r0z_r1z_r0w_r1w, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);

            // Calculate remaining inner term pairs. inner terms have zw=-xy, so we only have to calculate xy and can pack two pairs per vector.
            fp4 inner12_23 = r1y_r2y_r1x_r2x * r2z_r3z_r2w_r3w - r1z_r2z_r1w_r2w * r2y_r3y_r2x_r3x;
            fp4 inner02_13 = r0y_r1y_r0x_r1x * r2z_r3z_r2w_r3w - r0z_r1z_r0w_r1w * r2y_r3y_r2x_r3x;
            fp4 inner30_01 = r3z_r0z_r3w_r0w * r0y_r1y_r0x_r1x - r3y_r0y_r3x_r0x * r0z_r1z_r0w_r1w;

            // Expand inner terms back to 4 components. zw signs still need to be flipped
            fp4 inner12 = shuffle(inner12_23, inner12_23, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            fp4 inner23 = shuffle(inner12_23, inner12_23, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            fp4 inner02 = shuffle(inner02_13, inner02_13, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            fp4 inner13 = shuffle(inner02_13, inner02_13, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            // Calculate minors
            fp4 minors0 = r3_wzyx * inner12 - r2_wzyx * inner13 + r1_wzyx * inner23;

            fp4 denom = r0_xyzw * minors0;

            // Horizontal sum of denominator. Free sign flip of z and w compensates for missing flip in inner terms.
            denom = denom + shuffle(denom, denom, ShuffleComponent.LeftY, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightZ);   // x+y        x+y            z+w            z+w
            denom = denom - shuffle(denom, denom, ShuffleComponent.LeftZ, ShuffleComponent.LeftZ, ShuffleComponent.RightX, ShuffleComponent.RightX);   // x+y-z-w  x+y-z-w        z+w-x-y        z+w-x-y

            fp4 rcp_denom_ppnn = fp4(fp.one) / denom;
            fp4x4 res;
            res.c0 = minors0 * rcp_denom_ppnn;

            fp4 inner30 = shuffle(inner30_01, inner30_01, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            fp4 inner01 = shuffle(inner30_01, inner30_01, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            fp4 minors1 = r2_wzyx * inner30 - r0_wzyx * inner23 - r3_wzyx * inner02;
            res.c1 = minors1 * rcp_denom_ppnn;

            fp4 minors2 = r0_wzyx * inner13 - r1_wzyx * inner30 - r3_wzyx * inner01;
            res.c2 = minors2 * rcp_denom_ppnn;

            fp4 minors3 = r1_wzyx * inner02 - r0_wzyx * inner12 + r2_wzyx * inner01;
            res.c3 = minors3 * rcp_denom_ppnn;
            return res;
        }

        /// <summary>Fast matrix inverse for rigid transforms (orthonormal basis and translation)</summary>
        /// <param name="m">Matrix to invert.</param>
        /// <returns>The inverted matrix.</returns>
        public static fp4x4 fastinverse(fp4x4 m)
        {
            fp4 c0 = m.c0;
            fp4 c1 = m.c1;
            fp4 c2 = m.c2;
            fp4 pos = m.c3;

            fp4 zero = fp4(0);

            fp4 t0 = unpacklo(c0, c2);
            fp4 t1 = unpacklo(c1, zero);
            fp4 t2 = unpackhi(c0, c2);
            fp4 t3 = unpackhi(c1, zero);

            fp4 r0 = unpacklo(t0, t1);
            fp4 r1 = unpackhi(t0, t1);
            fp4 r2 = unpacklo(t2, t3);

            pos = -(r0 * pos.x + r1 * pos.y + r2 * pos.z);
            pos.w = fp.one;

            return fp4x4(r0, r1, r2, pos);
        }

        /// <summary>Returns the determinant of a fp4x4 matrix.</summary>
        /// <param name="m">Matrix to use when computing determinant.</param>
        /// <returns>The determinant of the matrix.</returns>
        public static fp determinant(fp4x4 m)
        {
            fp4 c0 = m.c0;
            fp4 c1 = m.c1;
            fp4 c2 = m.c2;
            fp4 c3 = m.c3;

            fp m00 = c1.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c1.z * c3.w - c1.w * c3.z) + c3.y * (c1.z * c2.w - c1.w * c2.z);
            fp m01 = c0.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c2.w - c0.w * c2.z);
            fp m02 = c0.y * (c1.z * c3.w - c1.w * c3.z) - c1.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c1.w - c0.w * c1.z);
            fp m03 = c0.y * (c1.z * c2.w - c1.w * c2.z) - c1.y * (c0.z * c2.w - c0.w * c2.z) + c2.y * (c0.z * c1.w - c0.w * c1.z);

            return c0.x * m00 - c1.x * m01 + c2.x * m02 - c3.x * m03;
        }

        /// <summary>Returns a uint hash code of a fp4x4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint hash(fp4x4 v)
        {
            return math.csum(fpmath.asuint(v.c0) * uint4(0x68EEE0F5u, 0xBC3B0A59u, 0x816EFB5Du, 0xA24E82B7u) +
                        fpmath.asuint(v.c1) * uint4(0x45A22087u, 0xFC104C3Bu, 0x5FFF6B19u, 0x5E6CBF3Bu) +
                        fpmath.asuint(v.c2) * uint4(0xB546F2A5u, 0xBBCF63E7u, 0xC53F4755u, 0x6985C229u) +
                        fpmath.asuint(v.c3) * uint4(0xE133B0B3u, 0xC3E0A3B9u, 0xFE31134Fu, 0x712A34D7u)) + 0x9D77A59Bu;
        }

        /// <summary>
        /// Returns a uint4 vector hash code of a fp4x4 vector.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 hashwide(fp4x4 v)
        {
            return (fpmath.asuint(v.c0) * uint4(0x4942CA39u, 0xB40EC62Du, 0x565ED63Fu, 0x93C30C2Bu) +
                    fpmath.asuint(v.c1) * uint4(0xDCAF0351u, 0x6E050B01u, 0x750FDBF5u, 0x7F3DD499u) +
                    fpmath.asuint(v.c2) * uint4(0x52EAAEBBu, 0x4599C793u, 0x83B5E729u, 0xC267163Fu) +
                    fpmath.asuint(v.c3) * uint4(0x67BC9149u, 0xAD7C5EC1u, 0x822A7D6Du, 0xB492BF15u)) + 0xD37220E3u;
        }

    }
}
