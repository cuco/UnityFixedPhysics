using System;
using System.Runtime.CompilerServices;
using System.Text;
using System.Runtime.InteropServices;
namespace Morefun.LockStep
{
	/// <summary>
    /// 32-bit Fixed point number with 19.13 format.
	/// </summary>
    /// <description>
    /// The range is -262144.0 to 262143.9998779296875.
    /// The precision (epsilon) is 1/8192 = 0.0001220703125 within the whole range.
    /// </description>
    /// <remarks>
    /// Lauha, Jetro. "The neglected art of Fixed Point arithmetic." Retrieved August 12 (2006): 2011.
    /// ftp://chiptune.untergrund.net/users/in4kadmin/files/The_neglected_art_of_Fixed_Point_arithmetic_20060913.pdf
    /// </remarks>
    [Serializable, StructLayout(LayoutKind.Sequential)]
	public partial struct FScalar : IComparable<FScalar>, IEquatable<FScalar>
	{
		public const int fractionBits = 13;
        public const int wholeBits = 19;
        public const uint fractionMask = (1 << fractionBits) - 1;
        public const uint wholeMask = ~fractionMask;
        public const int minWholeValue = -(1 << (wholeBits - 1));
        public const int maxWholeValue = (1 << (wholeBits - 1)) - 1;
        public const uint minFractionValue = 0;
        public const uint maxFractionValue = (1 << fractionBits) - 1;
        public const float floatScale = 1 << fractionBits;
        public const double doubleScale = 1 << fractionBits;
        
        public const int minRawValue = int.MinValue;
        public const int maxRawValue = int.MaxValue;
        public const float minFloatValue = minRawValue / floatScale;
        public const float maxFloatValue = 262143.999f; // need to specify explicitly //maxRawValue / floatScale;
        public const double minDoubleValue = minRawValue / doubleScale;
        public const double maxDoubleValue = maxRawValue / doubleScale;

        public static readonly FScalar zero = FScalar.FromRaw(0);
        public static readonly FScalar half = FScalar.FromRaw(1 << (fractionBits - 1));
        public static readonly FScalar quarter = FScalar.FromRaw(1 << (fractionBits - 2));
        public static readonly FScalar eighth = FScalar.FromRaw(1 << (fractionBits - 3));
        public static readonly FScalar one = FScalar.FromRaw(1 << fractionBits);
        public static readonly FScalar two = FScalar.FromRaw(1 << (fractionBits + 1));
        public static readonly FScalar maxValue = FScalar.FromRaw(maxRawValue);
        public static readonly FScalar minValue = FScalar.FromRaw(minRawValue);
        public static readonly FScalar epsilon = FScalar.FromRaw(1);
        public static readonly FScalar angleEpsilon = new FScalar(0, 360, 8192);
        public static readonly FScalar pi = new FScalar(3, 1416, 10000); //rad 3.1416
        public static readonly FScalar deg360 = 360; //reg 360

		public int rawValue;

		public int whole
		{
			get
			{
                if (rawValue >= 0)
				    return (rawValue >> fractionBits);
                else
                    return -(int)((-(long)rawValue) >> fractionBits);
			}
			set
			{
				rawValue = (int)((((uint)rawValue) & fractionMask) | ((uint)value << fractionBits));
			}
		}

		public uint fraction
		{
			get
			{
                if (rawValue >= 0)
    				return (uint)rawValue & fractionMask;
                else
                    return (uint)-rawValue & fractionMask;
			}
			set
			{
                if (rawValue >= 0)
				    rawValue = (int)(((uint)rawValue & wholeMask) | value);
                else
                    rawValue = -(int)(((uint)-rawValue & wholeMask) | value);
            }
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public FScalar(int whole) {
            LockstepDebug.Assert(whole >= minWholeValue);
            LockstepDebug.Assert(whole <= maxWholeValue);
            rawValue = whole << fractionBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static implicit operator FScalar(int whole)
        {
            return new FScalar(whole);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar FromRaw(int rawValue)
        {
            //return new FScalar(rawValue);
            FScalar s = new FScalar();
            s.rawValue = rawValue;
            return s;
        }

        /// Construct by whole and fraction parts.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public FScalar(int whole, uint fraction)
        {
            LockstepDebug.Assert(whole >= minWholeValue);
            LockstepDebug.Assert(whole <= maxWholeValue);
            LockstepDebug.Assert(fraction >= minFractionValue);
            LockstepDebug.Assert(fraction <= maxFractionValue);
            if (whole >= 0)
                rawValue = (int)((((uint)whole << fractionBits)) | fraction);
            else
                rawValue = -(int)((((uint)-whole) << fractionBits) | fraction);
        }

        /// <summary>
        /// Construct by mixed fraction a + b/c.
        /// </summary>
        /// <param name="a">The whole number.</param>
        /// <param name="b">The numerator.</param>
        /// <param name="c">The denominator.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public FScalar(int a, uint b, uint c)
        {
            LockstepDebug.Assert(a >= minWholeValue);
            LockstepDebug.Assert(a <= maxWholeValue);
            LockstepDebug.Assert(b <= c);
            LockstepDebug.Assert(c > 0);

            if (a >= 0)
                rawValue = (int)(((uint)a << fractionBits) | (((long)b << fractionBits) / c));
            else
                rawValue = -(int)((((uint)-a << fractionBits) | (((long)b << fractionBits) / c)));
        }

		public static FScalar FromFloat(float value)
		{
            return FromDouble(value);   // Need double precision conversion to prevent precision problem.
		}

        public static FScalar FromDouble(double value)
        {
            LockstepDebug.Assert (value >= minDoubleValue);
            LockstepDebug.Assert (value <= maxDoubleValue);

            if (value >= 0)
                return FromRaw((int)(Math.Min(value * doubleScale + 0.5, maxRawValue)));
            else
                return FromRaw((int)(Math.Max(value * doubleScale - 0.5, minRawValue)));
        }

        public static FScalar LongDivideLong(long longA, long longB)
        {
            LockstepDebug.Assert(longA <= int.MaxValue && longA > int.MinValue);
            LockstepDebug.Assert(longB <= int.MaxValue && longB > int.MinValue);
            if (longA >= 0)
            {
                return FromRaw((int)((longA << fractionBits) / longB));
            }
            return -FromRaw((int)((-longA << fractionBits) / longB));
        }

        /// <summary>
        /// Convert to float.
        /// </summary>
        /// <remarks>This may lose precision.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public float ToFloat()
		{
#if UNITY_EDITOR
            return (float)Math.Round(rawValue / floatScale, 4);
#else
            return rawValue / floatScale;
#endif
			
		}

        /// <summary>
        /// Convert to double.
        /// </summary>
        /// <remarks>All precision is preserved.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public double ToDouble()
        {
            return rawValue / doubleScale;
        }

        /// <summary>
        /// Parse a string into FScalar.
        /// </summary>
        /// <description>
        /// The syntax is [ \t]*[+-]?(0|[1-9][0-9]*)(\.[0-9]+)[ \t]* in regular expression.
        /// Valid examples: 1, -2, 0.5, +1.25
        /// Invalid examples: 01, 1., 12e5
        /// </description>
        /// <exception cref="ArgumentNullException"></exception> 
        /// <exception cref="FormatException"></exception>
        /// <exception cref="OverflowException"></exception> 
        public static FScalar Parse(string s)
        {
            FScalar result;
            switch (ParseInternal(s, out result))
            {
                case ParseResult.ErrorArgumentNull: throw new ArgumentNullException();
                case ParseResult.ErrorFormat: throw new FormatException();
                case ParseResult.ErrorOverflow: throw new OverflowException();
            }
            return result;
        }

        /// <summary>
        /// Parse a string into FScalar, without exception throwing.
        /// </summary>
        public static bool TryParse(string s, out FScalar result)
        {
            return ParseInternal(s, out result) == ParseResult.Success;
        }

        private enum ParseResult
        {
            Success,
            ErrorArgumentNull,
            ErrorFormat,
            ErrorOverflow
        }

        private static ParseResult ParseInternal(string s, out FScalar result)
        {
            result = zero;

            if (s == null)
                return ParseResult.ErrorArgumentNull;
            
            int i = 0;

            // Skip whitespaces
            while (i < s.Length && (s[i] == ' ' || s[i] == '\t'))
                i++;

            if (i == s.Length)
                return ParseResult.ErrorFormat;
            
            // Optional sign
            bool sign = false;
            if (s[i] == '+')
                i++;
            else if (s[i] == '-')
            {
                sign = true;
                i++;
            }

            if (i == s.Length)
                return ParseResult.ErrorFormat;

            // Whole part
            ulong mantissa = 0;
            ulong overflowValue = sign ? (ulong)-minWholeValue : (ulong)maxWholeValue;

            if (s[i] == '0')
                i++;
            else
            {
                if (s[i] < '1' || s[i] > '9')
                    return ParseResult.ErrorFormat;

                mantissa = (ulong)(s[i++] - '0');

                while (i < s.Length && s[i] >= '0' && s[i] <= '9')
                {
                    mantissa = mantissa * 10 + (ulong)(s[i++] - '0');
                    if (mantissa > overflowValue)
                        return ParseResult.ErrorOverflow;
                }
            }

            // fraction part
            ulong divisor = 1;
            if (i < s.Length && s[i] == '.')
            {
                i++;
                if (i == s.Length)
                    return ParseResult.ErrorFormat;
                
                while (i < s.Length && s[i] >= '0' && s[i] <= '9')
                {
                    mantissa = mantissa * 10 + (ulong)(s[i++] - '0');
                    divisor *= 10;

                    // Skip the following digits if 64-bit mantissa cannot handle
                    if (mantissa > (ulong.MaxValue >> fractionBits) / 10)
                    {
                        while (i < s.Length && s[i] >= '0' && s[i] <= '9')
                            i++;
                        break;
                    }
                }
            }

            // Skip whitespaces
            while (i < s.Length && (s[i] == ' ' || s[i] == '\t'))
                i++;

            if (i != s.Length)
                return ParseResult.ErrorFormat; // some trailing non-whitespace characters

            if (divisor == 1)
                result = new FScalar(sign ? -(int)mantissa : (int)mantissa);
            else
            {
                ulong y = mantissa << fractionBits;
                ulong x = y / (divisor / 2);
                if ((x & 1) == 1)  // rounding
                    x++;
                x >>= 1;         // divide by 2
                if ((sign && x > -(long)int.MinValue) || (!sign && x > int.MaxValue))
                    return ParseResult.ErrorOverflow;
                result = FScalar.FromRaw(sign ? -(int)x : (int)x);
            }

            return ParseResult.Success;
        }

        #region operators

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator+(FScalar a)
		{
			return a;
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator-(FScalar a)
		{
            LockstepDebug.Assert(a.rawValue != minRawValue);

            a.rawValue = -a.rawValue;
            return a;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator+(FScalar a, FScalar b)
		{
            // Overflow
            LockstepDebug.Assert(b >= zero ? 
                         (long)a.rawValue + (long)b.rawValue <= (long)maxRawValue : 
                         (long)a.rawValue + (long)b.rawValue >= (long)minRawValue);

            a.rawValue = a.rawValue + b.rawValue;
            return a;
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator-(FScalar a, FScalar b)
		{
            // Overflow
            LockstepDebug.Assert(b >= zero ? 
                         (long)a.rawValue - (long)b.rawValue >= (long)minRawValue : 
                         (long)a.rawValue - (long)b.rawValue <= (long)maxRawValue);

            a.rawValue = a.rawValue - b.rawValue;
            return a;
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator*(FScalar a, FScalar b)
		{
            long ret = ((long)a.rawValue * (long)b.rawValue) >> fractionBits;

            LockstepDebug.Assert(ret >= minRawValue && ret <= maxRawValue); // Overflow

            a.rawValue = (int)ret;
            return a;
		}

        /// <summary>
        /// Multiplication with integer (slightly faster)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator*(FScalar a, int b)
        {
            long ret = (long)a.rawValue * (long)b;

            LockstepDebug.Assert(ret >= minRawValue && ret <= maxRawValue); // Overflow

            a.rawValue = (int)ret;
            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator*(int a, FScalar b)
        {
            return b * a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static long MultiplyRaw(FScalar a, FScalar b)
        {
            return (long)a.rawValue * (long)b.rawValue;
        }

        /// <summary>
        /// 勾股定理 rawValue a*a + b*b
        /// </summary>
        /// <param name="a">直角三角直边a</param>
        /// <param name="b">直角三角直边b</param>
        /// <returns>直角三角斜边的平方 rawValue</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static long HypotenuseRaw(FScalar a, FScalar b)
        {
            return (long) a.rawValue * (long) a.rawValue + (long) b.rawValue * (long) b.rawValue;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Hypotenuse(FScalar a, FScalar b)
        {
            return Sqrt(a * a + b * b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator/(FScalar a, FScalar b)
		{
			LockstepDebug.Assert(b.rawValue != 0);  // Division by zero

            long ret = ((long)a.rawValue << fractionBits) / b.rawValue;

            LockstepDebug.Assert(ret >= minRawValue && ret <= maxRawValue); // Overflow

            a.rawValue = (int)ret;
            return a;
		}

        /// <summary>
        /// Division by integer (slightly faster)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator/(FScalar a, int b)
        {
            LockstepDebug.Assert(b != 0);
            a.rawValue = a.rawValue / b;// Never overflow since b >= 1
            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar operator%(FScalar a, FScalar b)
		{
			LockstepDebug.Assert (b.rawValue != 0); // Division by zero

            a.rawValue = a.rawValue % b.rawValue;// Never overflow since b >= 1
            return a;
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator==(FScalar x, FScalar y)
		{
			return x.rawValue == y.rawValue;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator!=(FScalar x, FScalar y)
		{
			return x.rawValue != y.rawValue;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator>(FScalar x, FScalar y)
		{
			return x.rawValue > y.rawValue;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator<(FScalar x, FScalar y) 
		{
			return x.rawValue < y.rawValue;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator>=(FScalar x, FScalar y) 
		{
			return x.rawValue >= y.rawValue;
		}
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool operator<=(FScalar x, FScalar y) 
		{
			return x.rawValue <= y.rawValue;
		}
        
        #endregion

        #region IComparable<FScalar>

		public int CompareTo(FScalar rhs) {
			return rawValue.CompareTo(rhs.rawValue);
		}
        
        // public int CompareTo(object obj)
        // {
        //     if (obj is FScalar)
        //     {
        //         return rawValue.CompareTo(((FScalar) obj).rawValue);
        //     }
        //     throw new Exception("FScalar can be compared to FScalar only!!!");
        // }

        #endregion

		#region IEquatable implementation

		public bool Equals (FScalar other)
		{
			return other.rawValue == rawValue;
		}

		#endregion

		// Object
		
        #region Object implementation

		public override bool Equals(object obj)
		{
            return obj is FScalar && ((FScalar)obj).rawValue == rawValue;
		}
		
		public override int GetHashCode()
		{
			return rawValue.GetHashCode();
		}
		
		public override string ToString()
		{
            StringBuilder s = new StringBuilder();
            if (rawValue < 0) {
                s.Append('-');
                s.Append((int)((-(long)rawValue) >> fractionBits));
            }
            else
                s.Append(rawValue >> fractionBits);
            
            s.Append('.');
            uint f = fraction;
            //小数部分四位有效
            uint validNum = 4;
            do
            {
                validNum--;
                f *= 10;
                s.Append((char)('0' + (f >> fractionBits)));
                f &= fractionMask;
            }
            while (validNum > 0 && f > 0);
            return s.ToString();
	    }

        #endregion

        // Rounding and clamping functions

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Abs(FScalar x) 
        {
            return x.rawValue > 0 ? x : FScalar.FromRaw(-x.rawValue);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Sgn(FScalar x)
        {
            if (x.rawValue > 0)
                return one;
            else if (x.rawValue < 0)
                return -one;
            else
                return zero;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Min(FScalar a, FScalar b)
        {
            return a.rawValue < b.rawValue ? a : b;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Max(FScalar a, FScalar b)
        {
            return a.rawValue > b.rawValue ? a : b;
        }

        public static FScalar Saturate(FScalar x)
        {
            return Min(Max(x, zero), one);
        }

		public static FScalar Clamp(FScalar value, FScalar min, FScalar max)
		{
			return Min(Max(value, min), max);
		}

        public static FScalar Truncate(FScalar x)
        {
            if (x.rawValue >= 0)
                return FScalar.FromRaw((int)((uint)x.rawValue & wholeMask));
            else
                return FScalar.FromRaw(-(int)((uint)-x.rawValue & wholeMask));
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Floor(FScalar x)
        {
            return FScalar.FromRaw((int)((uint)x.rawValue & wholeMask));
        }
    
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static FScalar Ceil(FScalar x)
        {
            return FScalar.FromRaw((int)((uint)(x.rawValue + (1 << fractionBits) - 1) & wholeMask));
        }

        public static FScalar Round(FScalar x)
        {
            if (x.rawValue >= 0)
                return FScalar.FromRaw((int)((uint)(x.rawValue + half.rawValue) & wholeMask));
            else
                return FScalar.FromRaw(-(int)((uint)-(x.rawValue - half.rawValue) & wholeMask));
        }

        /// <summary>
        /// 是否异号，0被当做正数
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static bool OppositeSigns(FScalar x, FScalar y)
        {
            return ((x.rawValue ^ y.rawValue) >> 31) != 0;
        }

        public static FScalar Pow(FScalar x, int p)
		{
			LockstepDebug.Assert(p >= 0);

			FScalar px = 1;
			for(int i = 0; i < p; i++)
			{
				px *= x;
			}
			return px;
		}

        public static FScalar Sqrt(FScalar x)
        {
            return FScalar.FromRaw(FMath.SqrtRaw((long) x.rawValue << FScalar.fractionBits));
        }

        public static FScalar SqrtRaw(long x)
        {
            return FScalar.FromRaw(FMath.SqrtRaw(x));
        }
        
        public static FScalar InvSqrt(FScalar x)
        {
            LockstepDebug.Assert(x > zero);
            // TODO: optimization
            FScalar sx = Max(Sqrt(x), epsilon);
            return one / sx;
        }

        // x in 44.20
        public static FScalar InvSqrtRaw(long x)
        {
            LockstepDebug.Assert(x > 0);

            if (x > (1L << (fractionBits * 4)))
                return zero;

            // TODO: optimization
            return one / SqrtRaw(x);
        }

        /// <summary>
        /// Converts turn (angle unit) to degrees.
        /// </summary>
        public static FScalar Turn2Deg(FScalar turn)
        {
            return turn * deg360;
        }

        /// <summary>
        /// Converts turn (angle unit) to radians in float.
        /// </summary>
        public static FScalar Turn2Rad(FScalar turn)
        {
            return two * pi * turn;
        }

        /// <summary>
        /// converts degrees to turn
        /// </summary>
        /// <returns></returns>
        public static FScalar Deg2Turn(FScalar deg)
        {
            return deg / deg360;
        }

        /// <summary>
        /// converts rad to turn
        /// </summary>
        /// <param name="rad"></param>
        /// <returns></returns>
        public static FScalar Rad2Turn(FScalar rad)
        {
            return rad / (two * pi);
        }
        
        /// <summary>
        /// Converts turn (angle unit) to degrees in float.
        /// </summary>
        public static float Turn2DegFloat(FScalar turn)
        {
            return 360.0f * turn.ToFloat();
        }

        /// <summary>
        /// Converts turn (angle unit) to degrees in double.
        /// </summary>
        public static double Turn2DegDouble(FScalar turn)
        {
            return 360.0 * turn.ToDouble();
        }

        /// <summary>
        /// Converts turn (angle unit) to radians in float.
        /// </summary>
        public static float Turn2RadFloat(FScalar turn)
        {
            return (float)(2.0f * Math.PI) * turn.ToFloat();
        }

        /// <summary>
        /// Converts turn (angle unit) to radians in double.
        /// </summary>
        public static double Turn2RadDouble(FScalar turn)
        {
            return 2.0 * Math.PI * turn.ToDouble();
        }

        /// <summary>
        /// Sine in terms of turns (not radian).
        /// </summary>
        /// <param name="turn">Turn (0 = 0 degree, 0.25 = 90 degrees, 0.5 = 180 degrees, 1.0 = 360 degrees).</param>
        public static FScalar Sin(FScalar turn)
        {
            int quadrant = (turn.rawValue >> (fractionBits - 2)) & 0x3;
            int index = turn.rawValue & (int)(fractionMask >> 2);
            int rightAngle = 1 << (fractionBits - 2);

            switch (quadrant)
            {
                case 0: return FScalar.FromRaw(c_sinTable[index]);
                case 1: return FScalar.FromRaw(c_sinTable[rightAngle - index]);
                case 2: return FScalar.FromRaw(-c_sinTable[index]);
                default: return FScalar.FromRaw(-c_sinTable[rightAngle - index]);
            }
        }

        /// <summary>
        /// Cosine in terms of turns (not radian).
        /// </summary>
        /// <param name="turn">Turn (0 = 0 degree, 0.25 = 90 degrees, 0.5 = 180 degrees, 1.0 = 360 degrees).</param>
        public static FScalar Cos(FScalar turn)
        {
            int quadrant = (turn.rawValue >> (fractionBits - 2)) & 0x3;
            int index = turn.rawValue & (int)(fractionMask >> 2);
            int rightAngle = 1 << (fractionBits - 2);

            switch (quadrant)
            {
                case 0: return FScalar.FromRaw(c_sinTable[rightAngle - index]);
                case 1: return FScalar.FromRaw(-c_sinTable[index]);
                case 2: return FScalar.FromRaw(-c_sinTable[rightAngle - index]);
                default: return FScalar.FromRaw(c_sinTable[index]);
            }
        }

        /// <summary>
        /// Sine and Cosine in terms of turns (not radian).
        /// </summary>
        /// <param name="turn">Turn (0 = 0 degree, 0.25 = 90 degrees, 0.5 = 180 degrees, 1.0 = 360 degrees).</param>
        /// <remarks>
        /// This should be slightly faster than calling Sin(x) and Cos(x) independently.
        /// </remarks>
        public static void SinCos(FScalar turn, out FScalar s, out FScalar c)
        {
            int quadrant = (turn.rawValue >> (fractionBits - 2)) & 0x3;
            int index = turn.rawValue & (int)(fractionMask >> 2);
            int rightAngle = 1 << (fractionBits - 2);

            switch (quadrant)
            {
                case 0:
                    s = FScalar.FromRaw(c_sinTable[index]);
                    c = FScalar.FromRaw(c_sinTable[rightAngle - index]);
                    break;
                case 1:
                    s = FScalar.FromRaw(c_sinTable[rightAngle - index]);
                    c = FScalar.FromRaw(-c_sinTable[index]);
                    break;
                case 2:
                    s = FScalar.FromRaw(-c_sinTable[index]);
                    c = FScalar.FromRaw(-c_sinTable[rightAngle - index]);
                    break;
                default:
                    s = FScalar.FromRaw(-c_sinTable[rightAngle - index]);
                    c = FScalar.FromRaw(c_sinTable[index]);
                    break;
            }
        }

        /// <summary>
        /// Tangent in terms of turns (not radian).
        /// </summary>
        /// <param name="turn">Turn (0 = 0 degree, 0.25 = 90 degrees, 0.5 = 180 degrees, 1.0 = 360 degrees).</param>
        public static FScalar Tan(FScalar turn)
        {
            int quadrantParity = (turn.rawValue >> (fractionBits - 2)) & 0x1;
            int index = turn.rawValue & (int)(fractionMask >> 2);
            int rightAngle = 1 << (fractionBits - 2);

            LockstepDebug.Assert(!(quadrantParity == 1 && index == 0));

            if (quadrantParity == 0)
                return FScalar.FromRaw(c_tanTable[index]);
            else
                return FScalar.FromRaw(-c_tanTable[rightAngle - index]);
        }

        /// <summary>
        /// Arcsine in terms of turns (not radian).
        /// </summary>
        /// <param name="x">Scalar in [-1, 1]</param>
        /// <returns>Angle in turns in [-0.25, 0.25]</returns>
        public static FScalar Asin(FScalar x)
        {
            LockstepDebug.Assert(x >= -one && x <= one);

            if (x >= zero)
                return FScalar.FromRaw(c_asinTable[x.rawValue]);
            else
                return FScalar.FromRaw(-c_asinTable[-x.rawValue]);
        }

        /// <summary>
        /// Arccosine in terms of turns (not radian).
        /// </summary>
        /// <param name="x">Scalar in [-1, 1]</param>
        /// <returns>Angle in turns in [0, 0.5]</returns>
        public static FScalar Acos(FScalar x)
        {
            LockstepDebug.Assert(x >= -one && x <= one);

            if (x >= zero)
                return quarter - FScalar.FromRaw(c_asinTable[x.rawValue]);
            else
                return quarter + FScalar.FromRaw(c_asinTable[-x.rawValue]);
        }

        /// <summary>
        /// Arctangent.
        /// </summary>
        /// <param name="x">Any scalar</param>
        /// <returns>Angle [-0.5, 0.5] in terms of turns (not radian)</returns>
        public static FScalar Atan(FScalar x)
        {
            if (Abs(x) < one)
                return x * (FromRaw(1376) - FromRaw(352) * Abs(x));
            else
            {
                FScalar r = Abs(one / x);
                FScalar t = r * (FromRaw(1376) - FromRaw(352) * r);
                if (x > zero)
                    return quarter - t;
                else
                    return t - quarter;
            }
        }

        /// <summary>
        /// Arctangent with two arguments.
        /// </summary>
        /// <param name="y">The y coordinate.</param>
        /// <param name="x">The x coordinate.</param>
        /// <returns>Turn in (-0.5, 0.5].</returns>
        public static FScalar Atan2(FScalar y, FScalar x)
        {
            if (x == zero)
            {
                if (y == zero)
                    return zero;
                else
                    return y > zero ? quarter : -quarter;
            }
            else if (y == zero)
                return x > zero ? zero : half;

            FScalar absx = Abs(x);
            FScalar absy = Abs(y);

            // http://geekshavefeelings.com/posts/fixed-point-atan2
            // \tan^{-1}(x) \approx \frac{\pi}{4} x + 0.273 x (1 - |x|)
            // after radian -> turn conversion:
            // \approx x \left ( \frac{1}{8} + \frac{0.273}{2\pi} - \frac{0.273}{2\pi} |x| \right )
            // \frac{1}{8} + \frac{0.273}{2\pi} \approx 172 / 1024
            // \frac{0.273}{2\pi} \approx 44

            if (absx > absy)            // octants I, IV, V, VIII
            {
                FScalar r = y / x;
                FScalar t = r * (FromRaw(1376) - FromRaw(352) * Abs(r));
                if (x > zero)           // octants I, VIII
                    return t;
                else                    // octants IV, V
                    return y > zero ? t + half : t - half;
            }
            else                        // octants II, III, VI, VII
            {
                FScalar r = x / y;
                FScalar t = r * (FromRaw(1376) - FromRaw(352) * Abs(r));
                if (y > zero)           // octants II, III
                    return quarter - t;
                else                    // octants VI, VII
                    return -quarter - t;
            }
        }

        /// <summary>
        /// Log base-2 of x.
        /// </summary>
        /// <param name="x">Positive number.</param>
        /// <remarks>
        /// C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal Processing Mag., pp. 124,140, Sep. 2010.
        /// https://github.com/dmoulding/log2fix
        /// </remarks>
        public static FScalar Log2(FScalar x)
        {
            LockstepDebug.Assert(x.rawValue > 0);

            int rawOne = 1 << fractionBits;
            int rawTwo = 2 << fractionBits;

            int z = x.rawValue;
            int y = 0;
            while (z < rawOne) {
                z <<= 1;
                y -= rawOne;
            }

            while (z >= rawTwo) {
                z >>= 1;
                y += rawOne;
            }

            int b = 1 << (fractionBits - 1);
            do {
                z = (int)(((long)z * (long)z) >> fractionBits);
                if (z >= rawTwo) {
                    z >>= 1;
                    y |= b;
                }
                b >>= 1;
            } while (b > 0);

            return FScalar.FromRaw(y);
        }

        public static FScalar Log(FScalar x)
        {
            return FScalar.FromRaw((int)(((long)(Log2(x).rawValue) * 0x58b90bfc) >> 31));
        }

        public static FScalar Log10(FScalar x)
        {
            return FScalar.FromRaw((int)(((long)(Log2(x).rawValue) * 0x268826a1) >> 31));
        }

        public static void Swap(ref FScalar a, ref FScalar b)
        {
            var temp = a.rawValue;
            a.rawValue = b.rawValue;
            b.rawValue = temp;
        }

        public static bool IsApproximatelyZero(FScalar value)
        {
            return Abs(value) < epsilon;
        }
    }
}
