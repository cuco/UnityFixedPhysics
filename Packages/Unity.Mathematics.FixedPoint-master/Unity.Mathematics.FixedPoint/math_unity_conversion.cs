#if !UNITY_DOTSPLAYER
using UnityEngine;

#pragma warning disable 0660, 0661

namespace Unity.Mathematics.FixedPoint
{
    public partial struct fp2
    {
        /// <summary>
        /// Converts a float2 to Vector2.
        /// </summary>
        /// <param name="v">float2 to convert.</param>
        /// <returns>The converted Vector2.</returns>
        public static implicit operator Vector2(fp2 v)     { return new Vector2((float)v.x, (float)v.y); }

        /// <summary>
        /// Converts a Vector2 to float2.
        /// </summary>
        /// <param name="v">Vector2 to convert.</param>
        /// <returns>The converted float2.</returns>
        public static implicit operator fp2(Vector2 v)     { return new fp2((fp)v.x, (fp)v.y); }
    }

    public partial struct fp3
    {
        /// <summary>
        /// Converts a float3 to Vector3.
        /// </summary>
        /// <param name="v">float3 to convert.</param>
        /// <returns>The converted Vector3.</returns>
        public static implicit operator Vector3(fp3 v)     { return new Vector3((float)v.x, (float)v.y, (float)v.z); }

        /// <summary>
        /// Converts a Vector3 to float3.
        /// </summary>
        /// <param name="v">Vector3 to convert.</param>
        /// <returns>The converted float3.</returns>
        public static implicit operator fp3(Vector3 v)     { return new fp3((fp)v.x, (fp)v.y, (fp)v.z); }
    }

    public partial struct fp4
    {
        /// <summary>
        /// Converts a Vector4 to float4.
        /// </summary>
        /// <param name="v">Vector4 to convert.</param>
        /// <returns>The converted float4.</returns>
        public static implicit operator fp4(Vector4 v)     { return new fp4((fp)v.x, (fp)v.y, (fp)v.z, (fp)v.w); }

        /// <summary>
        /// Converts a float4 to Vector4.
        /// </summary>
        /// <param name="v">float4 to convert.</param>
        /// <returns>The converted Vector4.</returns>
        public static implicit operator Vector4(fp4 v)     { return new Vector4((float)v.x, (float)v.y, (float)v.z, (float)v.w); }
    }

    public partial struct fpquaternion
    {
        /// <summary>
        /// Converts a quaternion to Quaternion.
        /// </summary>
        /// <param name="q">quaternion to convert.</param>
        /// <returns>The converted Quaternion.</returns>
        public static implicit operator Quaternion(fpquaternion q)  { return new Quaternion((float)q.value.x, (float)q.value.y, (float)q.value.z, (float)q.value.w); }

        /// <summary>
        /// Converts a Quaternion to quaternion.
        /// </summary>
        /// <param name="q">Quaternion to convert.</param>
        /// <returns>The converted quaternion.</returns>
        public static implicit operator fpquaternion(Quaternion q)  { return new fpquaternion((fp)q.x, (fp)q.y, (fp)q.z, (fp)q.w); }
    }

    public partial struct fp4x4
    {
        /// <summary>
        /// Converts a Matrix4x4 to float4x4.
        /// </summary>
        /// <param name="m">Matrix4x4 to convert.</param>
        /// <returns>The converted float4x4.</returns>
        public static implicit operator fp4x4(Matrix4x4 m) { return new fp4x4(m.GetColumn(0), m.GetColumn(1), m.GetColumn(2), m.GetColumn(3)); }

        /// <summary>
        /// Converts a float4x4 to Matrix4x4.
        /// </summary>
        /// <param name="m">float4x4 to convert.</param>
        /// <returns>The converted Matrix4x4.</returns>
        public static implicit operator Matrix4x4(fp4x4 m) { return new Matrix4x4(m.c0, m.c1, m.c2, m.c3); }
    }
}
#endif
