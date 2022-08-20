using System;
using Fixed.Mathematics;
using UnityEditor;
using UnityEngine;
using static UnityEditor.IMGUI.Controls.PrimitiveBoundsHandle;

namespace Fixed.Physics.Editor
{
    static class PhysicsBoundsHandleUtility
    {
        internal static readonly sfloat kBackfaceAlphaMultiplier = (sfloat)0.2f;
        static readonly sfloat kDegreeEpsilon = (sfloat)0.001f;
        public static readonly sfloat kDistanceEpsilon = (sfloat)0.0001f;

        public static bool IsBackfaced(float3 localPos, float3 localTangent, float3 localBinormal, Axes axes, bool isCameraInsideBox)
        {
            // if inside the box then ignore back facing alpha multiplier (otherwise all handles will look disabled)
            if (isCameraInsideBox || axes != Axes.All)
                return false;

            // use tangent and binormal to calculate normal in case handle matrix is skewed
            float3 worldTangent = math.normalize(Handles.matrix.MultiplyVector(localTangent));
            float3 worldBinormal = math.normalize(Handles.matrix.MultiplyVector(localBinormal));
            float3 worldDir = math.normalize(math.cross(worldTangent, worldBinormal));

            // adjust color if handle is back facing
            float cosV;

            var currentCamera = Camera.current;
            if (currentCamera != null && !currentCamera.orthographic)
            {
                float3 cameraPos = currentCamera.transform.position;
                float3 worldPos = Handles.matrix.MultiplyPoint(localPos);
                cosV = (float)math.dot(math.normalize(cameraPos - worldPos), worldDir);
            }
            else
            {
                float3 cameraForward = currentCamera == null ? Vector3.forward : currentCamera.transform.forward;
                cosV = (float)math.dot(-cameraForward, worldDir);
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

        public static void DrawFace(float3 center, float3 size, float cornerRadius, int normalAxis, Axes axes, bool isCameraInsideBox)
        {
            // 0 = 0 1 2
            // 1 = 1 2 0
            // 2 = 2 0 1

            int a = normalAxis;
            int b = k_NextAxis[a];
            int c = k_NextAxis[b];

            cornerRadius = Mathf.Abs(cornerRadius);
            size *= (sfloat)0.5f;
            var normal = new float3 { [a] = size[a] };
            var ctr = center + normal;
            size -= new float3((sfloat)cornerRadius);

            // check if our face is a point
            if (math.abs(size[c]) < kDistanceEpsilon &&
                math.abs(size[b]) < kDistanceEpsilon)
                return;

            Vector3[] points;
            // check if our face is a line or not
            if (math.abs(size[c]) >= kDistanceEpsilon &&
                math.abs(size[b]) >= kDistanceEpsilon)
            {
                var i = 0;
                s_FacePoints[i++] = ctr + new float3 { [b] = size[b], [c] = size[c] };
                s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = size[c] };

                s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = size[c] };
                s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = -size[c] };

                s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = -size[c] };
                s_FacePoints[i++] = ctr + new float3 { [b] = size[b], [c] = -size[c] };

                s_FacePoints[i++] = ctr + new float3 { [b] = size[b], [c] = -size[c] };
                s_FacePoints[i++] = ctr + new float3 { [b] = size[b], [c] = size[c] };
                points = s_FacePoints;
            }
            else if (math.abs(size[c]) >= kDistanceEpsilon)
            {
                var i = 0;
                s_LinePoints[i++] = ctr + new float3 { [b] = size[b], [c] = size[c] };
                s_LinePoints[i++] = ctr + new float3 { [b] = size[b], [c] = -size[c] };
                points = s_LinePoints;
            }
            else
            {
                var i = 0;
                s_LinePoints[i++] = ctr + new float3 { [c] = size[c], [b] = size[b] };
                s_LinePoints[i++] = ctr + new float3 { [c] = size[c], [b] = -size[b] };
                points = s_LinePoints;
            }

            float3 tangent, biNormal;
            if (size[c] > sfloat.Zero)
            {
                tangent = math.cross(normal, math.normalizesafe(new float3 { [c] = size[c] }));
                biNormal = math.cross(normal, tangent);
            }
            else
            {
                tangent = math.cross(normal, math.normalizesafe(new float3 { [b] = size[b] }));
                biNormal = math.cross(normal, tangent);
            }

            using (new Handles.DrawingScope(Handles.color))
            {
                AdjustMidpointHandleColor(IsBackfaced(ctr, tangent, biNormal, axes, isCameraInsideBox));
                Handles.DrawLines(points);
            }
        }

        public struct Corner
        {
            public float3 angle;
            public float3x2 intersections;
            public float3x2 points;
            public float3x3 axes;
            public float3x3 normals;
            public bool3x2 splitAxis;
            public int splitCount;

            public float3 position;
            public float radius;
            public bool isBackFaced;

            public float3 cameraForward;
        }

        public static void CalculateCornerHorizon(float3 cornerPosition, quaternion orientation, float3 cameraCenter, float3 cameraForward, bool cameraOrtho, float radius, out Corner corner)
        {
            var axisx = new float3(sfloat.One, sfloat.Zero, sfloat.Zero);
            var axisy = new float3(sfloat.Zero, sfloat.One, sfloat.Zero);
            var axisz = new float3(sfloat.Zero, sfloat.Zero, sfloat.One);

            // a vector pointing away from the center of the corner
            var cornerNormal = math.normalize(math.mul(orientation, new float3(sfloat.One, sfloat.One, sfloat.One)));

            var axes = math.mul(new float3x3(orientation), new float3x3(axisx, axisy, axisz));
            CalculateCornerHorizon(cornerPosition,
                axes,
                cornerNormal,
                cameraCenter, cameraForward, cameraOrtho,
                radius, out corner);
        }

        public static void CalculateCornerHorizon(float3 cornerPosition, float3x3 axes, float3 cornerNormal, float3 cameraCenter, float3 cameraForward, bool cameraOrtho, float radius, out Corner corner)
        {
            var cameraToCenter          = cornerPosition - cameraCenter; // vector from camera to center
            var sqrRadius               = (sfloat)(radius * radius);
            var sqrDistCameraToCenter   = math.lengthsq(cameraToCenter);
            var sqrOffset               = (sqrRadius * sqrRadius / sqrDistCameraToCenter);  // squared distance from actual center to drawn disc center

            if (!cameraOrtho)
                cameraForward = cameraToCenter;

            var normals = new float3x3
            {
                c0 = math.normalize(math.cross(axes[1], axes[2])),
                c1 = math.normalize(math.cross(axes[2], axes[0])),
                c2 = math.normalize(math.cross(axes[0], axes[1]))
            };

            corner = new Corner
            {
                angle           = new float3(
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
                isBackFaced     = math.dot(cornerNormal, cameraForward) > sfloat.Zero,
                splitCount      = 0
            };

            if (math.abs(sqrDistCameraToCenter) <= sqrRadius)
                return;

            for (int n = 0, sign = -1; n < 2; n++, sign += 2)
            {
                for (int i = 0; i < 3; i++)
                {
                    var axis1 = normals[i] * sign;
                    var axis2 = axes[(i + 1) % 3] * sign;
                    var axis3 = axes[(i + 2) % 3] * sign;

                    var Q = Math.Angle(cameraForward, axis1);
                    var f = math.tan(math.radians((sfloat)90f - math.min(Q, (sfloat)180.0f - Q)));
                    var g = sqrOffset + f * f * sqrOffset;
                    if (g >= sqrRadius)
                        continue;

                    var e                       = math.degrees(math.asin(math.sqrt(g) / (sfloat)radius));
                    var vectorToPointOnHorizon  = Quaternion.AngleAxis((float)e, axis1) * math.normalize(math.cross(axis1, cameraForward));

                    vectorToPointOnHorizon = math.normalize(Math.ProjectOnPlane(vectorToPointOnHorizon, axis1));

                    var intersectionDirection = vectorToPointOnHorizon;
                    var angle1 = Math.SignedAngle(axis2, intersectionDirection, axis1);
                    var angle2 = Math.SignedAngle(axis3, intersectionDirection, axis1);

                    if (angle1 <= sfloat.Zero || angle2 >= sfloat.Zero)
                        continue;

                    var point = corner.position + (float3)(intersectionDirection * radius);

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
                bool reversePolarity = angleLength < sfloat.Zero;
                if (reversePolarity)
                    Handles.DrawWireArc(origin, corner.cameraForward, corner.points[1] - origin, -(float)angleLength, radius);
                else
                    Handles.DrawWireArc(origin, corner.cameraForward, corner.points[0] - origin, (float)angleLength, radius);


                var backfacedColor = GetStateColor(true);

                var axesBackfaced = new bool3(math.length(intersections[0] - axes[0]) < kDistanceEpsilon || math.length(intersections[1] - axes[0]) < kDistanceEpsilon,
                    math.length(intersections[0] - axes[1]) < kDistanceEpsilon || math.length(intersections[1] - axes[1]) < kDistanceEpsilon,
                    math.length(intersections[0] - axes[2]) < kDistanceEpsilon || math.length(intersections[1] - axes[2]) < kDistanceEpsilon);

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
