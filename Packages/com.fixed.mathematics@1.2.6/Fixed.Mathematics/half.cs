using System;
using System.Runtime.CompilerServices;
using Unity.IL2CPP.CompilerServices;

namespace Fixed.Mathematics
{
    /// <summary>
    /// A half precision sfloat that uses 16 bits instead of 32 bits.
    /// </summary>
    [Il2CppEagerStaticClassConstruction]
    [Serializable]
    public struct half : System.IEquatable<half>, IFormattable
    {
        /// <summary>
        /// The raw 16 bit value of the half.
        /// </summary>
        public ushort value;

        /// <summary>half zero value.</summary>
        public static readonly half zero = new half();

        /// <summary>
        /// The maximum finite half value as a single precision sfloat.
        /// </summary>
        public static sfloat MaxValue { get { return (sfloat)65504; } }

        /// <summary>
        /// The minimum finite half value as a single precision sfloat.
        /// </summary>
        public static sfloat MinValue { get { return -(sfloat)65504; } }

        // /// <summary>
        // /// The maximum finite half value as a half.
        // /// </summary>
        // public static half MaxValueAsHalf => new half(MaxValue);
        //
        // /// <summary>
        // /// The minimum finite half value as a half.
        // /// </summary>
        // public static half MinValueAsHalf => new half(MinValue);

        /// <summary>Constructs a half value from a half value.</summary>
        /// <param name="x">The input half value to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public half(half x)
        {
            value = x.value;
        }

        // /// <summary>Constructs a half value from a sfloat value.</summary>
        // /// <param name="v">The single precision sfloat value to convert to half.</param>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public half(sfloat v)
        // {
        //     value = (ushort)math.f32tof16(v);
        // }
        //
        // /// <summary>Constructs a half value from a double value.</summary>
        // /// <param name="v">The double precision sfloat value to convert to half.</param>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public half(double v)
        // {
        //     value = (ushort)math.f32tof16((sfloat)v);
        // }

        // /// <summary>Explicitly converts a sfloat value to a half value.</summary>
        // /// <param name="v">The single precision sfloat value to convert to half.</param>
        // /// <returns>The converted half value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static explicit operator half(sfloat v) { return new half(v); }
        //
        // /// <summary>Explicitly converts a double value to a half value.</summary>
        // /// <param name="v">The double precision sfloat value to convert to half.</param>
        // /// <returns>The converted half value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static explicit operator half(double v) { return new half(v); }
        //
        // /// <summary>Implicitly converts a half value to a sfloat value.</summary>
        // /// <param name="d">The half value to convert to a single precision sfloat.</param>
        // /// <returns>The converted single precision sfloat value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static implicit operator sfloat(half d) { return math.f16tof32(d.value); }

        // /// <summary>Implicitly converts a half value to a double value.</summary>
        // /// <param name="d">The half value to convert to double precision sfloat.</param>
        // /// <returns>The converted double precision sfloat value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static implicit operator double(half d) { return math.f16tof32(d.value); }


        /// <summary>Returns whether two half values are bitwise equivalent.</summary>
        /// <param name="lhs">Left hand side half value to use in comparison.</param>
        /// <param name="rhs">Right hand side half value to use in comparison.</param>
        /// <returns>True if the two half values are bitwise equivalent, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(half lhs, half rhs) { return lhs.value == rhs.value; }

        /// <summary>Returns whether two half values are not bitwise equivalent.</summary>
        /// <param name="lhs">Left hand side half value to use in comparison.</param>
        /// <param name="rhs">Right hand side half value to use in comparison.</param>
        /// <returns>True if the two half values are not bitwise equivalent, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(half lhs, half rhs) { return lhs.value != rhs.value; }


        /// <summary>Returns true if the half is bitwise equivalent to a given half, false otherwise.</summary>
        /// <param name="rhs">Right hand side half value to use in comparison.</param>
        /// <returns>True if the half value is bitwise equivalent to the input, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(half rhs) { return value == rhs.value; }

        /// <summary>Returns true if the half is equal to a given half, false otherwise.</summary>
        /// <param name="o">Right hand side object to use in comparison.</param>
        /// <returns>True if the object is of type half and is bitwise equivalent, false otherwise.</returns>
        public override bool Equals(object o) { return o is half converted && Equals(converted); }

        /// <summary>Returns a hash code for the half.</summary>
        /// <returns>The computed hash code of the half.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() { return (int)value; }

        /// <summary>Returns a string representation of the half.</summary>
        /// <returns>The string representation of the half.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return value.ToString();
        }

        /// <summary>Returns a string representation of the half using a specified format and culture-specific format information.</summary>
        /// <param name="format">The format string to use during string formatting.</param>
        /// <param name="formatProvider">The format provider to use during string formatting.</param>
        /// <returns>The string representation of the half.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return value.ToString(format, formatProvider);
        }
    }

    public static partial class math
    {
        /// <summary>Returns a half value constructed from a half values.</summary>
        /// <param name="x">The input half value to copy.</param>
        /// <returns>The constructed half value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static half half(half x) { return new half(x); }

        // /// <summary>Returns a half value constructed from a sfloat value.</summary>
        // /// <param name="v">The single precision sfloat value to convert to half.</param>
        // /// <returns>The constructed half value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static half half(sfloat v) { return new half(v); }
        //
        // /// <summary>Returns a half value constructed from a double value.</summary>
        // /// <param name="v">The double precision sfloat value to convert to half.</param>
        // /// <returns>The constructed half value.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static half half(double v) { return new half(v); }

        /// <summary>Returns a uint hash code of a half value.</summary>
        /// <param name="v">The half value to hash.</param>
        /// <returns>The computed hash code of the half value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint hash(half v)
        {
            return v.value * 0x745ED837u + 0x816EFB5Du;
        }
    }
}
