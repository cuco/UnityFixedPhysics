using System;
using Unity.Mathematics.FixedPoint;
using UnityEditor;
using UnityEngine;
using static UnityEditor.IMGUI.Controls.PrimitiveBoundsHandle;

namespace Fixed.Physics.Editor
{
    static class PhysicsBoundsHandleUtility
    {
        internal static readonly fp kBackfaceAlphaMultiplier = (fp)0.2f;
        static readonly fp kDegreeEpsilon = (fp)0.001f;
        public static readonly fp kDistanceEpsilon = (fp)0.0001f;

        public static bool IsBackfaced(fp3 localPos, fp3 localTangent, fp3 localBinormal, Axes axes, bool isCameraInsideBox)
        {
            // if inside the box then ignore back facing alpha multiplier (otherwise all handles will look disabled)
            if (isCameraInsideBox || axes != Axes.All)
                return false;

            // use tangent and binormal to calculate normal in case handle matrix is skewed
            fp3 worldTangent = fpmath.normalize(Handles.matrix.MultiplyVector(localTangent));
            fp3 worldBinormal = fpmath.normalize(Handles.matrix.MultiplyVector(localBinormal));
            fp3 worldDir = fpmath.normalize(fpmath.cross(worldTangent, worldBinormal));

            // adjust color if handle is back facing
            float cosV;

            var currentCamera = Camera.current;
            if (currentCamera != null && !currentCamera.orthographic)
            {
                fp3 cameraPos = currentCamera.transform.position;
                fp3 worldPos = Handles.matrix.MultiplyPoint(localPos);
                cosV = (float)fpmath.dot(fpmath.normalize(cameraPos - worldPos), worldDir);
            }
            else
            {
                fp3 cameraForward = currentCamera == null ? Vector3.forward : currentCamera.transform.forward;
                cosV = (float)fpmath.dot(-cameraForward, worldDir);
            }

            return cosV < -0.0001f;
        }

        public static Color GetStateColor(bool isBackfaced)
        {
            float alphaMultiplier = isBackfaced ? (float)kBackfaceAlphaMultiplier : 1f;
            return Handles.color * new Color(1f, 1f, 1f, alphaMultiplier);
        }

        static void AdjustMidpointHandleColor(bool isBackfaced)
        {
            Handles.color = GetStateColor(isBackfaced);
        }

        static readonly Vector3[] s_FacePoints = new Vector3[8];
        static readonly Vector3[] s_LinePoints = new Vector3[2];

        static readonly int[] k_NextAxis = { 1, 2, 0 };

        public static void DrawFace(fp3 center, fp3 size, float cornerRadius, int normalAxis, Axes axes, bool isCameraInsideBox)
        {
            // 0 = 0 1 2
            // 1 = 1 2 0
            // 2 = 2 0 1

            int a = normalAxis;
            int b = k_NextAxis[a];
            int c = k_NextAxis[b];

            cornerRadius = Mathf.Abs(cornerRadius);
            size *= fp.half;
            var normal = new fp3 { [a] = size[a] };
            var ctr = center + normal;
            size -= new fp3((fp)cornerRadius);

            // check if our face is a point
            if (fpmath.abs(size[c]) < kDistanceEpsilon &&
                fpmath.abs(size[b]) < kDistanceEpsilon)
                return;

            Vector3[] points;
            // check if our face is a line or not
            if (fpmath.abs(size[c]) >= kDistanceEpsilon &&
                fpmath.abs(size[b]) >= kDistanceEpsilon)
            {
                var i = 0;
                s_FacePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = size[c] };
                s_FacePoints[i++] = ctr + new fp3 { [b] = -size[b], [c] = size[c] };

                s_FacePoints[i++] = ctr + new fp3 { [b] = -size[b], [c] = size[c] };
                s_FacePoints[i++] = ctr + new fp3 { [b] = -size[b], [c] = -size[c] };

                s_FacePoints[i++] = ctr + new fp3 { [b] = -size[b], [c] = -size[c] };
                s_FacePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = -size[c] };

                s_FacePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = -size[c] };
                s_FacePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = size[c] };
                points = s_FacePoints;
            }
            else if (fpmath.abs(size[c]) >= kDistanceEpsilon)
            {
                var i = 0;
                s_LinePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = size[c] };
                s_LinePoints[i++] = ctr + new fp3 { [b] = size[b], [c] = -size[c] };
                points = s_LinePoints;
            }
            else
            {
                var i = 0;
                s_LinePoints[i++] = ctr + new fp3 { [c] = size[c], [b] = size[b] };
                s_LinePoints[i++] = ctr + new fp3 { [c] = size[c], [b] = -size[b] };
                points = s_LinePoints;
            }

            fp3 tangent, biNormal;
            if (size[c] > fp.zero)
            {
                tangent = fpmath.cross(normal, fpmath.normalizesafe(new fp3 { [c] = size[c] }));
                biNormal = fpmath.cross(normal, tangent);
            }
            else
            {
                tangent = fpmath.cross(normal, fpmath.normalizesafe(new fp3 { [b] = size[b] }));
                biNormal = fpmath.cross(normal, tangent);
            }

            using (new Handles.DrawingScope(Handles.color))
            {
                AdjustMidpointHandleColor(IsBackfaced(ctr, tangent, biNormal, axes, isCameraInsideBox));
                Handles.DrawLines(points);
            }
        }

        public struct Corner
        {
            public fp3 angle;
            public fp3x2 intersections;
            public fp3x2 points;
            public fp3x3 axes;
            public fp3x3 normals;
            public bool3x2 splitAxis;
            public int splitCount;

            public fp3 position;
            public float radius;
            public bool isBackFaced;

            public fp3 cameraForward;
        }

        public static void CalculateCornerHorizon(fp3 cornerPosition, fpquaternion orientation, fp3 cameraCenter, fp3 cameraForward, bool cameraOrtho, float radius, out Corner corner)
        {
            var axisx = new fp3(fp.one, fp.zero, fp.zero);
            var axisy = new fp3(fp.zero, fp.one, fp.zero);
            var axisz = new fp3(fp.zero, fp.zero, fp.one);

            // a vector pointing away from the center of the corner
            var cornerNormal = fpmath.normalize(fpmath.mul(orientation, new fp3(fp.one, fp.one, fp.one)));

            var axes = fpmath.mul(new fp3x3(orientation), new fp3x3(axisx, axisy, axisz));
            CalculateCornerHorizon(cornerPosition,
                axes,
                cornerNormal,
                cameraCenter, cameraForward, cameraOrtho,
                radius, out corner);
        }

        public static void CalculateCornerHorizon(fp3 cornerPosition, fp3x3 axes, fp3 cornerNormal, fp3 cameraCenter, fp3 cameraForward, bool cameraOrtho, float radius, out Corner corner)
        {
            var cameraToCenter          = cornerPosition - cameraCenter; // vector from camera to center
            var sqrRadius               = (fp)(radius * radius);
            var sqrDistCameraToCenter   = fpmath.lengthsq(cameraToCenter);
            var sqrOffset               = (sqrRadius * sqrRadius / sqrDistCameraToCenter);  // squared distance from actual center to drawn disc center

            if (!cameraOrtho)
                cameraForward = cameraToCenter;

            var normals = new fp3x3
            {
                c0 = fpmath.normalize(fpmath.cross(axes[1], axes[2])),
                c1 = fpmath.normalize(fpmath.cross(axes[2], axes[0])),
                c2 = fpmath.normalize(fpmath.cross(axes[0], axes[1]))
            };

            corner = new Corner
            {
                angle           = new fp3(
                    Math.Angle(axes[0], axes[1]),
                    Math.Angle(axes[1], axes[2]),
                    Math.Angle(axes[2], axes[0])
                    ),
                intersections   = default,
                points          = default,
                splitAxis       = default,

                axes            = axes,
                normals         = normals,

                position        = cornerPosition,
                radius          = radius,
                cameraForward   = cameraForward,
                isBackFaced     = fpmath.dot(cornerNormal, cameraForward) > fp.zero,
                splitCount      = 0
            };

            if (fpmath.abs(sqrDistCameraToCenter) <= sqrRadius)
                return;

            for (int n = 0, sign = -1; n < 2; n++, sign += 2)
            {
                for (int i = 0; i < 3; i++)
                {
                    var axis1 = normals[i] * sign;
                    var axis2 = axes[(i + 1) % 3] * sign;
                    var axis3 = axes[(i + 2) % 3] * sign;

                    var Q = Math.Angle(cameraForward, axis1);
                    var f = fpmath.tan(fpmath.radians((fp)90f - fpmath.min(Q, (fp)180.0f - Q)));
                    var g = sqrOffset + f * f * sqrOffset;
                    if (g >= sqrRadius)
                        continue;

                    var e                       = fpmath.degrees(fp.Asin(fpmath.sqrt(g) / (fp)radius));
                    var vectorToPointOnHorizon  = Quaternion.AngleAxis((float)e, axis1) * fpmath.normalize(fpmath.cross(axis1, cameraForward));

                    vectorToPointOnHorizon = fpmath.normalize(Math.ProjectOnPlane(vectorToPointOnHorizon, axis1));

                    var intersectionDirection = vectorToPointOnHorizon;
                    var angle1 = Math.SignedAngle(axis2, intersectionDirection, axis1);
                    var angle2 = Math.SignedAngle(axis3, intersectionDirection, axis1);

                    if (angle1 <= fp.zero || angle2 >= fp.zero)
                        continue;

                    var point = corner.position + (fp3)(intersectionDirection * radius);

                    if (corner.splitCount < 2)
                    {
                        corner.splitAxis[corner.splitCount][i] = true;
                        corner.intersections[corner.splitCount] = intersectionDirection;
                        corner.points[corner.splitCount] = point;

                        corner.splitCount++;
                    }
                }
            }

            if (!math.any(corner.splitAxis[0]) &&
                !math.any(corner.splitAxis[1]))
            {
                corner.splitCount = 0;
                corner.splitAxis[0] = false;
                corner.splitAxis[1] = false;
            }
        }

        public static void DrawCorner(Corner corner, bool3 showAxis)
        {
            var color           = Handles.color;
            var axes            = corner.axes;
            var intersections   = corner.intersections;
            var normals         = corner.normals;
            var origin          = corner.position;
            var radius          = corner.radius;

            if (corner.splitCount <= 1)
            {
                AdjustMidpointHandleColor(corner.isBackFaced);
                if (showAxis[0]) Handles.DrawWireArc(origin, normals[0], axes[1], (float)corner.angle[1], radius);
                if (showAxis[1]) Handles.DrawWireArc(origin, normals[1], axes[2], (float)corner.angle[2], radius);
                if (showAxis[2]) Handles.DrawWireArc(origin, normals[2], axes[0], (float)corner.angle[0], radius);
            }
            else
            {
                var angleLength = Math.SignedAngle(Math.ProjectOnPlane(intersections[0], corner.cameraForward),
                    Math.ProjectOnPlane(intersections[1], corner.cameraForward), corner.cameraForward);
                bool reversePolarity = angleLength < fp.zero;
                if (reversePolarity)
                    Handles.DrawWireArc(origin, corner.cameraForward, corner.points[1] - origin, -(float)angleLength, radius);
                else
                    Handles.DrawWireArc(origin, corner.cameraForward, corner.points[0] - origin, (float)angleLength, radius);


                var backfacedColor = GetStateColor(true);

                var axesBackfaced = new bool3(fpmath.length(intersections[0] - axes[0]) < kDistanceEpsilon || fpmath.length(intersections[1] - axes[0]) < kDistanceEpsilon,
                    fpmath.length(intersections[0] - axes[1]) < kDistanceEpsilon || fpmath.length(intersections[1] - axes[1]) < kDistanceEpsilon,
                    fpmath.length(intersections[0] - axes[2]) < kDistanceEpsilon || fpmath.length(intersections[1] - axes[2]) < kDistanceEpsilon);

                var color1 = reversePolarity ? color : backfacedColor;
                var color2 = reversePolarity ? backfacedColor : color;

                for (int A = 1, B = 2, C = 0; C < 3; A = B, B = C, C++)
                {
                    if (corner.splitAxis[0][C] == corner.splitAxis[1][C])
                    {
                        if (!axesBackfaced[A]) { angleLength = Math.Angle(intersections[0], axes[A]); axesBackfaced[A] = (angleLength<kDegreeEpsilon || angleLength> corner.angle[C] - kDegreeEpsilon); }
                        if (!axesBackfaced[B]) { angleLength = Math.Angle(intersections[1], axes[A]); axesBackfaced[B] = (angleLength<kDegreeEpsilon || angleLength> corner.angle[C] - kDegreeEpsilon); }
                    }
                    else if (corner.splitAxis[0][C])
                    {
                        if (showAxis[C])
                        {
                            angleLength = Math.Angle(intersections[0], axes[A]);
                            Handles.color = color1; Handles.DrawWireArc(origin, normals[C], intersections[0],                 -(float)angleLength, radius);
                            Handles.color = color2; Handles.DrawWireArc(origin, normals[C], intersections[0], (float)corner.angle[A] - (float)angleLength, radius);
                        }
                        axesBackfaced[A] = true;
                    }
                    else
                    //if (corner.splitAxis[1][C])
                    {
                        if (showAxis[C])
                        {
                            angleLength = Math.Angle(intersections[1], axes[A]);
                            Handles.color = color2; Handles.DrawWireArc(origin, normals[C], intersections[1],                 -(float)angleLength, radius);
                            Handles.color = color1; Handles.DrawWireArc(origin, normals[C], intersections[1], (float)corner.angle[A] - (float)angleLength, radius);
                        }
                        axesBackfaced[B] = true;
                    }
                }

                // check for singularity
                if (math.all(axesBackfaced))
                    axesBackfaced = corner.isBackFaced;

                for (int A = 1, B = 2, C = 0; C < 3; A = B, B = C, C++)
                {
                    if (!showAxis[C])
                        continue;

                    if (corner.splitAxis[0][C] == corner.splitAxis[1][C])
                    {
                        Handles.color = (axesBackfaced[B] && axesBackfaced[A]) ? color1 : color2;
                        Handles.DrawWireArc(origin, normals[C], axes[A], (float)corner.angle[A], radius);
                    }
                }
            }
            Handles.color = color;
        }
    }
}
