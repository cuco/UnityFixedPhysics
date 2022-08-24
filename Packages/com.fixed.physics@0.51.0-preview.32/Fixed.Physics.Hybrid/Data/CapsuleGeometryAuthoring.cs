using System;
using Unity.Mathematics.FixedPoint;
using UnityEngine;
using UnityComponent = UnityEngine.Component;
using UnityMesh = UnityEngine.Mesh;

namespace Fixed.Physics.Authoring
{
    /// <summary>
    /// A structure for storing authoring data for a capsule shape.
    /// In contrast to the CapsuleGeometry struct in the run-time, this structure permits storing stable orientation values, as well as height values that can be retained when the source data are defined with respect to a non-uniformly scaled object.
    /// </summary>
    [Serializable]
    public struct CapsuleGeometryAuthoring : IEquatable<CapsuleGeometryAuthoring>
    {
        /// <summary>
        /// The local orientation of the capsule. It is aligned with the forward axis (z) when it is identity.
        /// </summary>
        public fpquaternion Orientation { get => m_OrientationEuler; set => m_OrientationEuler.SetValue(value); }
        internal EulerAngles OrientationEuler { get => m_OrientationEuler; set => m_OrientationEuler = value; }
        [SerializeField]
        EulerAngles m_OrientationEuler;

        /// <summary>
        /// The local position offset of the capsule.
        /// </summary>
        public fp3 Center { get => m_Center; set => m_Center = value; }
        [SerializeField]
        fp3 m_Center;

        /// <summary>
        /// The height of the capsule. It may store any value, but will ultimately always be converted into a value that is at least twice the radius.
        /// </summary>
        public fp Height { get => m_Height; set => m_Height = value; }
        [SerializeField]
        fp m_Height;

        /// <summary>
        /// The radius of the capsule.
        /// </summary>
        public fp Radius { get => m_Radius; set => m_Radius = value; }
        [SerializeField]
        fp m_Radius;

        public bool Equals(CapsuleGeometryAuthoring other)
        {
            return m_Height.Equals(other.m_Height)
                && m_Center.Equals(other.m_Center)
                && m_Radius.Equals(other.m_Radius)
                && m_OrientationEuler.Equals(other.m_OrientationEuler);
        }

        public override int GetHashCode()
        {
            return unchecked((int)fpmath.hash(
                new fp3x3(
                    Center,
                    m_OrientationEuler.Value,
                    new fp3((fp)(int)m_OrientationEuler.RotationOrder, m_Height, m_Radius)
                )
            ));
        }
    }

    public static class CapsuleGeometryAuthoringExtensions
    {
        /// <summary>
        /// Construct a CapsuleGeometryAuthoring instance from a run-time CapsuleGeometry instance.
        /// </summary>
        /// <param name="input">A run-time CapsuleGeometry instance.</param>
        public static CapsuleGeometryAuthoring ToAuthoring(this CapsuleGeometry input)
        {
            var orientationEuler = EulerAngles.Default;
            orientationEuler.SetValue(fpquaternion.LookRotationSafe(input.Vertex1 - input.Vertex0, fpmath.up()));
            return new CapsuleGeometryAuthoring
            {
                Height = input.GetHeight(),
                OrientationEuler = orientationEuler,
                Center = input.GetCenter(),
                Radius = input.Radius
            };
        }

        /// <summary>
        /// Construct a run-time CapsuleGeometry instance from a CapsuleGeometryAuthoring instance.
        /// </summary>
        /// <param name="input">A CapsuleGeometryAuthoring instance.</param>
        public static CapsuleGeometry ToRuntime(this CapsuleGeometryAuthoring input)
        {
            var halfHeight   = fp.half * input.Height;
            var halfDistance = halfHeight - input.Radius;
            var axis         = fpmath.normalize(fpmath.mul(input.Orientation, new fp3 { z = fp.one }));
            var halfAxis     = axis * halfDistance;
            var vertex0      = input.Center + halfAxis;
            var vertex1      = input.Center - halfAxis;
            return new CapsuleGeometry
            {
                Vertex0 = vertex0,
                Vertex1 = vertex1,
                Radius  = input.Radius
            };
        }
    }
}
