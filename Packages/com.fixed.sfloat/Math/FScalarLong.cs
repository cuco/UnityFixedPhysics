using System.Runtime.CompilerServices;

namespace Morefun.LockStep
{
    /// <summary>
    /// FScalar与long之间的乘法
    /// </summary>
    public partial struct FScalar
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static long operator*(FScalar a, long b)
        {
            long ret = ((long)a.rawValue * b) >> fractionBits;
            return ret;
        }
		
        [MethodImpl(MethodImplOptions.AggressiveInlining)]public static long operator*(long a, FScalar b) 
        {
            long ret = ((long)b.rawValue * a) >> fractionBits;
            return ret;
        }
		
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]public static long operator/(long a, FScalar b) 
        // {
        //     long ret = (a << fractionBits) / b.rawValue;
        //     return ret;
        // }
    }
}