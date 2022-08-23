using System;
using System.Runtime.InteropServices;
using Morefun.LockStep;

namespace Morefun.LockStep
{
    public class FMathWrapper
    {
        #if UNITY_STANDALONE_WIN || UNITY_EDITOR || UNITY_STANDALONE_OSX
            //public const string PluginName = "FixedBullet_d";
            public const string PluginName = "FixedBullet";
#else
#if UNITY_IPHONE
                public const string PluginName = "__Internal";
#else
                // Other platforms load plugins dynamically, so pass the name
                // of the plugin's dynamic library.
                public const string PluginName = "FixedBullet";
#endif
#endif

        [DllImport(PluginName)]
        public static extern bool PointInBox(
            FVector3 point,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend);

        [DllImport(PluginName)]
        public static extern bool PointInSphere(
            FVector3 point,
            FVector3 sphere_center,
            FScalar sphere_radius);

        [DllImport(PluginName)]
        public static extern bool PointInCapsule(
            FVector3 point,
            FVector3 capsule_center,
            FQuaternion capsule_rot,
            FScalar capsule_radius,
            FScalar capsule_height);

        [DllImport(PluginName)]
        public static extern bool PointInCylinder(
            FVector3 point,
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar radius,
            FScalar height);

        [DllImport(PluginName)]
        public static extern bool PointInCylindricalSector(
            FVector3 point,
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle);

        [DllImport(PluginName)]
        public unsafe static extern bool BoxAndBoxIntersect(
            FVector3 box1_center,
            FQuaternion box1_rot,
            FVector3 box1_extend,
            FVector3 box2_center,
            FQuaternion box2_rot,
            FVector3 box2_extend,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool SphereAndSphereIntersect(
            FVector3 sphere1_center,
            FScalar sphere1_radius,
            FVector3 sphere2_center,
            FScalar sphere2_radius,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool BoxAndSphereIntersect(
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            FVector3 sphere_center,
            FScalar sphere_radius,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool HollowBoxAndBoxIntersect(
            FVector3 box1_center,
            FQuaternion box1_rot,
            FVector3 box1_inextend,
            FVector3 box1_outextend,
            FVector3 box2_center,
            FQuaternion box2_rot,
            FVector3 box2_extend,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CapsuleAndCapsuleIntersect(
            FVector3 cap1_center,
            FQuaternion cap1_rot,
            FScalar cap1_radius,
            FScalar cap1_height,
            FVector3 cap2_center,
            FQuaternion cap2_rot,
            FScalar cap2_radius,
            FScalar cap2_height,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool BoxAndCapsuleIntersect(
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            FVector3 cap_center,
            FQuaternion cap_rot,
            FScalar cap_radius,
            FScalar cap_height,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylinderAndSphereIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 sphere_center,
            FScalar sphere_radius,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylinderAndBoxIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylinderAndCapsuleIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 capsule_center,
            FQuaternion capsule_rot,
            FScalar capsule_radius,
            FScalar capsule_height,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylindricalSectorAndSphereIntersect(
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle,
            FVector3 sphere_center,
            FScalar sphere_radius,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylindricalSectorAndBoxIntersect(
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extent,
            FScalar* outarr);

        [DllImport(PluginName)]
        public unsafe static extern bool CylindricalSectorAndCapsuleIntersect(
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle,
            FVector3 capsule_center,
            FQuaternion capsule_rot,
            FScalar capsule_radius,
            FScalar capsule_height,
            FScalar* outarr);
    }
}
