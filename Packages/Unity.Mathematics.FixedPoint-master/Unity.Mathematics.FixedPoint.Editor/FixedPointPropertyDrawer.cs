using System;
using UnityEditor;
using UnityEngine;

namespace Unity.Mathematics.FixedPoint.Editor
{
    [CustomPropertyDrawer(typeof(fp))]
    public class FixedPointPropertyDrawer : PropertyDrawer
    {
        private static GUIStyle s_overlay;

        public static GUIStyle OverlayStyle {
            get {
                if (s_overlay == null)
                {
                    s_overlay = new GUIStyle(EditorStyles.miniLabel);
                    s_overlay.alignment = TextAnchor.MiddleRight;
                    s_overlay.contentOffset = new Vector2(-2, 0);

                    Color c = EditorGUIUtility.isProSkin ? Color.yellow : Color.blue;
                    c.a = 0.2f;
                    s_overlay.normal.textColor = c;
                }
                return s_overlay;
            }
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            property.Next(true);
            bool hideLabel = false;
            if (fieldInfo != null)
            {
                var propertyAttributes = fieldInfo.GetCustomAttributes(false);
                for (int i = 0; i < propertyAttributes.Length; i++)
                {
                    if (propertyAttributes[i].GetType().Name == "HideLabelAttribute")
                    {
                        hideLabel = true;
                        break;
                    }
                }
            }
            if(hideLabel)
                Draw(position, property, null);
            else
                Draw(position, property, label);
        }

        public static void Draw(Rect position, SerializedProperty property, GUIContent label)
        {
            var f = fp.FromRaw(property.longValue);
            var v = (float) f;

            try
            {
                var n = label == null ? EditorGUI.FloatField(position, v) : EditorGUI.FloatField(position, label, v);
                if (n != v)
                {
                    property.longValue = ((fp)n).RawValue;
                }
                GUI.Label(position, "FP", OverlayStyle);
            }
            catch (FormatException exn)
            {
                if (exn.Message != ".") Debug.LogException(exn);
            }
        }

        /// <summary>
        /// 为了显示一致，提供一个通用的FScalar变量编辑的静态方法
        /// </summary>
        /// <param name="label"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        public static fp FixedPointField(string label, fp value)
        {
            Rect rect = EditorGUILayout.GetControlRect();
            var v = (float) value;
            var n = label == null ? EditorGUI.FloatField(rect, v) : EditorGUI.FloatField(rect, label, v);
            var ret = n == v ? value : (fp)(n);
            GUI.Label(rect, "FP", OverlayStyle);
            return ret;
        }

        /// <summary>
        /// 为了显示一致，提供一个通用的FScalar变量编辑的静态方法
        /// </summary>
        /// <param name="label"></param>
        /// <param name="value"></param>
        /// <returns></returns>
        public static fp LayoutFScalarField(string label, fp value, params GUILayoutOption[] options)
        {
            var v = (float) value;
            var n = label == null
                ? EditorGUILayout.FloatField(v, options)
                : EditorGUILayout.FloatField(label, v, options);
            Rect rect = GUILayoutUtility.GetLastRect();
            var ret = n == v ? value : (fp)(n);
            GUI.Label(rect, "FP", OverlayStyle);
            return ret;
        }
    }
}