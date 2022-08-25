using System;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Fixed.Physics.Editor
{
    class PhysicsCapsuleBoundsHandle : CapsuleBoundsHandle
    {
        static PhysicsBoundsHandleUtility.Corner[] s_Corners = new PhysicsBoundsHandleUtility.Corner[8];

        protected override void DrawWireframe()
        {
            if (this.radius <= 0f)
            {
                base.DrawWireframe();
                return;
            }

            var cameraPos = default(fp3);
            var cameraFwd = new fp3 { z = fp.one };
            var cameraOrtho = true;
            if (Camera.current != null)
            {
                cameraPos = Camera.current.transform.position;
                cameraFwd = Camera.current.transform.forward;
                cameraOrtho = Camera.current.orthographic;
            }

            var size        = new fp3((fp)this.radius * (fp)2f, (fp)this.radius * (fp)2f, (fp)height);
            var radius      = this.radius;
            var origin      = (fp3)this.center;
            var bounds      = new Bounds(this.center, size);

            // Since the geometry is transformed by Handles.matrix during rendering, we transform the camera position
            // by the inverse matrix so that the two-shaded wireframe will have the proper orientation.
            var invMatrix       = Handles.inverseMatrix;
            var cameraCenter    = (fp3)invMatrix.MultiplyPoint(cameraPos);
            var cameraForward   = (fp3)invMatrix.MultiplyVector(cameraFwd);

            bool isCameraInsideBox = Camera.current != null
                && bounds.Contains(invMatrix.MultiplyPoint(cameraPos));

            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), radius, 0, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.minusOne,  fp.one,  fp.one), radius, 0, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), radius, 1, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one, fp.minusOne,  fp.one), radius, 1, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), radius, 2, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one, fp.minusOne), radius, 2, axes, isCameraInsideBox);

            var corner = fp.half * size - new fp3(fp.one) * (fp)radius;
            var axisx = new fp3(fp.one, fp.zero, fp.zero);
            var axisy = new fp3(fp.zero, fp.one, fp.zero);
            var axisz = new fp3(fp.zero, fp.zero, fp.one);

            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne,  fp.one, fp.minusOne), fpquaternion.LookRotation(-axisz,  axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[0]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne,  fp.one,  fp.one), fpquaternion.LookRotation(-axisx,  axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[1]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one,  fp.one,  fp.one), fpquaternion.LookRotation(axisz,  axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[2]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one,  fp.one, fp.minusOne), fpquaternion.LookRotation(axisx,  axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[3]);

            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne, fp.minusOne, fp.minusOne), fpquaternion.LookRotation(-axisx, -axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[4]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne, fp.minusOne,  fp.one), fpquaternion.LookRotation(axisz, -axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[5]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one, fp.minusOne,  fp.one), fpquaternion.LookRotation(axisx, -axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[6]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one, fp.minusOne, fp.minusOne), fpquaternion.LookRotation(-axisz, -axisy), cameraCenter, cameraForward, cameraOrtho, radius, out s_Corners[7]);

            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[0], new bool3(false, true,  true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[3], new bool3(true,  false, true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[4], new bool3(true,  false, true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[7], new bool3(false, true,  true));

            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[1], new bool3(true,  false, true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[2], new bool3(false, true,  true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[5], new bool3(false, true,  true));
            PhysicsBoundsHandleUtility.DrawCorner(s_Corners[6], new bool3(true,  false, true));

            // Draw the horizon edges between the corners
            for (int upA = 3, upB = 0; upB < 4; upA = upB, upB++)
            {
                int dnA = upA + 4;
                int dnB = upB + 4;

                if (s_Corners[upA].splitAxis[0].z && s_Corners[upB].splitAxis[1].x) Handles.DrawLine(s_Corners[upA].points[0], s_Corners[upB].points[1]);
                if (s_Corners[upA].splitAxis[1].z && s_Corners[upB].splitAxis[0].x) Handles.DrawLine(s_Corners[upA].points[1], s_Corners[upB].points[0]);

                if (s_Corners[dnA].splitAxis[0].x && s_Corners[dnB].splitAxis[1].z) Handles.DrawLine(s_Corners[dnA].points[0], s_Corners[dnB].points[1]);
                if (s_Corners[dnA].splitAxis[1].x && s_Corners[dnB].splitAxis[0].z) Handles.DrawLine(s_Corners[dnA].points[1], s_Corners[dnB].points[0]);

                if (s_Corners[dnA].splitAxis[0].y && s_Corners[upA].splitAxis[1].y) Handles.DrawLine(s_Corners[dnA].points[0], s_Corners[upA].points[1]);
                if (s_Corners[dnA].splitAxis[1].y && s_Corners[upA].splitAxis[0].y) Handles.DrawLine(s_Corners[dnA].points[1], s_Corners[upA].points[0]);
            }
        }
    }
}
