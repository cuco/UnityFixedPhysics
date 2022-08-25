using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Fixed.Physics.Editor
{
    class BeveledBoxBoundsHandle : BoxBoundsHandle
    {
        public float bevelRadius
        {
            get => Mathf.Min(m_BevelRadius, (float)math.cmin(GetSize()) * 0.5f);
            set
            {
                if (!m_IsDragging)
                    m_BevelRadius = Mathf.Max(0f, value);
            }
        }
        float m_BevelRadius = (float)ConvexHullGenerationParameters.Default.BevelRadius;
        bool m_IsDragging = false;

        static PhysicsBoundsHandleUtility.Corner[] s_Corners = new PhysicsBoundsHandleUtility.Corner[8];

        public new void DrawHandle()
        {
            int prevHotControl = GUIUtility.hotControl;
            if (prevHotControl == 0)
                m_IsDragging = false;
            base.DrawHandle();
            int currHotcontrol = GUIUtility.hotControl;
            if (currHotcontrol != prevHotControl)
                m_IsDragging = currHotcontrol != 0;
        }

        protected override void DrawWireframe()
        {
            if (this.bevelRadius <= 0f)
            {
                base.DrawWireframe();
                return;
            }

            var cameraPosition = fp3.zero;
            var cameraForward = new fp3 { z = 1 };
            if (Camera.current != null)
            {
                cameraPosition = (fp3)Camera.current.transform.position;
                cameraForward = (fp3)Camera.current.transform.forward;
            }

            var bounds = new Bounds(this.center, this.size);
            bool isCameraInsideBox = Camera.current != null && bounds.Contains(Handles.inverseMatrix.MultiplyPoint(cameraPosition));
            var bevelRadius = this.bevelRadius;
            var origin      = (fp3)this.center;
            var size        = (fp3)this.size;

            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), bevelRadius, 0, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.minusOne,  fp.one,  fp.one), bevelRadius, 0, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), bevelRadius, 1, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one, fp.minusOne,  fp.one), bevelRadius, 1, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one,  fp.one), bevelRadius, 2, axes, isCameraInsideBox);
            PhysicsBoundsHandleUtility.DrawFace(origin, size * new fp3(fp.one,  fp.one, fp.minusOne), bevelRadius, 2, axes, isCameraInsideBox);

            var corner = fp.half * size - new fp3(fp.one) * (fp)bevelRadius;
            var axisx = new fp3(fp.one, fp.zero, fp.zero);
            var axisy = new fp3(fp.zero, fp.one, fp.zero);
            var axisz = new fp3(fp.zero, fp.zero, fp.one);

            // Since the geometry is transformed by Handles.matrix during rendering, we transform the camera position
            // by the inverse matrix so that the two-shaded wireframe will have the proper orientation.
            var invMatrix   = Handles.inverseMatrix;
            cameraPosition  = (fp3)invMatrix.MultiplyPoint((Vector3)cameraPosition);
            cameraForward   = (fp3)invMatrix.MultiplyVector((Vector3)cameraForward);
            var cameraOrtho = Camera.current == null || Camera.current.orthographic;

            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne,  fp.one, fp.minusOne), fpquaternion.LookRotation(-axisz,  axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[0]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne,  fp.one,  fp.one), fpquaternion.LookRotation(-axisx,  axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[1]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one,  fp.one,  fp.one), fpquaternion.LookRotation(axisz,  axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[2]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one,  fp.one, fp.minusOne), fpquaternion.LookRotation(axisx,  axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[3]);

            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne, fp.minusOne, fp.minusOne), fpquaternion.LookRotation(-axisx, -axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[4]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.minusOne, fp.minusOne,  fp.one), fpquaternion.LookRotation(axisz, -axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[5]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one, fp.minusOne,  fp.one), fpquaternion.LookRotation(axisx, -axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[6]);
            PhysicsBoundsHandleUtility.CalculateCornerHorizon(origin + corner * new fp3(fp.one, fp.minusOne, fp.minusOne), fpquaternion.LookRotation(-axisz, -axisy), cameraPosition, cameraForward, cameraOrtho, bevelRadius, out s_Corners[7]);

            for (int i = 0; i < s_Corners.Length; i++)
                PhysicsBoundsHandleUtility.DrawCorner(s_Corners[i], true);

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
