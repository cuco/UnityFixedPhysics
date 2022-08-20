using System;
using Fixed.Mathematics;
using UnityEngine;

namespace Fixed.Physics.Editor
{
    enum MatrixState
    {
        UniformScale,
        NonUniformScale,
        ZeroScale,
        NotValidTRS
    }

    static class ManipulatorUtility
    {
        public static MatrixState GetMatrixState(ref float4x4 localToWorld)
        {
            if (
                localToWorld.c0.w != sfloat.Zero
                || localToWorld.c1.w != sfloat.Zero
                || localToWorld.c2.w != sfloat.Zero
                || localToWorld.c3.w != sfloat.One
            )
                return MatrixState.NotValidTRS;

            var m = new float3x3(localToWorld.c0.xyz, localToWorld.c1.xyz, localToWorld.c2.xyz);
            var lossyScale = new float3(math.length(m.c0.xyz), math.length(m.c1.xyz), math.length(m.c2.xyz));
            if (math.determinant(m) < sfloat.Zero)
                lossyScale.x *= sfloat.MinusOne;
            if (math.lengthsq(lossyScale) == sfloat.Zero)
                return MatrixState.ZeroScale;
            return math.abs(math.cmax(lossyScale)) - math.abs(math.cmin(lossyScale)) > (sfloat)0.000001f
                ? MatrixState.NonUniformScale
                : MatrixState.UniformScale;
        }
    }
}
