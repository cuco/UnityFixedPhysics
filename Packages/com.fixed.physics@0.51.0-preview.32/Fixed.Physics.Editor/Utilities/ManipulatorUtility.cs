using System;
using Unity.Mathematics.FixedPoint;
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
        public static MatrixState GetMatrixState(ref fp4x4 localToWorld)
        {
            if (
                localToWorld.c0.w != fp.zero
                || localToWorld.c1.w != fp.zero
                || localToWorld.c2.w != fp.zero
                || localToWorld.c3.w != fp.one
            )
                return MatrixState.NotValidTRS;

            var m = new fp3x3(localToWorld.c0.xyz, localToWorld.c1.xyz, localToWorld.c2.xyz);
            var lossyScale = new fp3(fpmath.length(m.c0.xyz), fpmath.length(m.c1.xyz), fpmath.length(m.c2.xyz));
            if (fpmath.determinant(m) < fp.zero)
                lossyScale.x *= fp.minusOne;
            if (fpmath.lengthsq(lossyScale) == fp.zero)
                return MatrixState.ZeroScale;
            return fpmath.abs(fpmath.cmax(lossyScale)) - fpmath.abs(fpmath.cmin(lossyScale)) > (fp)0.000001f
                ? MatrixState.NonUniformScale
                : MatrixState.UniformScale;
        }
    }
}
