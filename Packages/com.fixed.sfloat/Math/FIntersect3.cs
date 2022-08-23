namespace Morefun.LockStep
{
    public class FIntersect3
    {
        public static bool PointInBox(
            FVector3 point,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend)
        {
            return FMathWrapper.PointInBox(point, box_center, box_rot, box_extend);
        }

        public static bool PointInSphere(
            FVector3 point,
            FVector3 sphere_center,
            FScalar sphere_radius)
        {
            return FMathWrapper.PointInSphere(point, sphere_center, sphere_radius);
        }

        public static bool PointInCapsule(
            FVector3 point,
            FVector3 capsule_center,
            FQuaternion capsule_rot,
            FScalar capsule_radius,
            FScalar capsule_height)
        {
            return FMathWrapper.PointInCapsule(point, capsule_center, capsule_rot, capsule_radius, capsule_height);
        }

        public static bool PointInCylinder(
            FVector3 point,
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar radius,
            FScalar height)
        {
            return FMathWrapper.PointInCylinder(point, cylinder_center, cylinder_rot, radius, height);
        }

        public static bool PointInCylindricalSector(
            FVector3 point,
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle)
        {
            return FMathWrapper.PointInCylindricalSector(point, sector_center, sector_rot, in_radius, out_radius, height, angle);
        }

        public static bool BoxAndBoxIntersect(
                    FVector3 box1_center,
                    FQuaternion box1_rot,
                    FVector3 box1_extend,
                    FVector3 box2_center,
                    FQuaternion box2_rot,
                    FVector3 box2_extend,
                    ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.BoxAndBoxIntersect(
                     box1_center,
                     box1_rot,
                     box1_extend,
                     box2_center,
                     box2_rot,
                     box2_extend,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool SphereAndSphereIntersect(
                    FVector3 sphere1_center,
                    FScalar sphere1_radius,
                    FVector3 sphere2_center,
                    FScalar sphere2_radius,
                    ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.SphereAndSphereIntersect(
                     sphere1_center,
                     sphere1_radius,
                     sphere2_center,
                     sphere2_radius,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool BoxAndSphereIntersect(
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            FVector3 sphere_center,
            FScalar sphere_radius,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.BoxAndSphereIntersect(
                     box_center,
                     box_rot,
                     box_extend,
                     sphere_center,
                     sphere_radius,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool HollowBoxAndBoxIntersect(
            FVector3 box1_center,
            FQuaternion box1_rot,
            FVector3 box1_inextend,
            FVector3 box1_outextend,
            FVector3 box2_center,
            FQuaternion box2_rot,
            FVector3 box2_extend,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.HollowBoxAndBoxIntersect(
                     box1_center,
                     box1_rot,
                     box1_inextend,
                     box1_outextend,
                     box2_center,
                     box2_rot,
                     box2_extend,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CapsuleAndCapsuleIntersect(
            FVector3 cap1_center,
            FQuaternion cap1_rot,
            FScalar cap1_radius,
            FScalar cap1_height,
            FVector3 cap2_center,
            FQuaternion cap2_rot,
            FScalar cap2_radius,
            FScalar cap2_height,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CapsuleAndCapsuleIntersect(
                     cap1_center,
                     cap1_rot,
                     cap1_radius,
                     cap1_height,
                     cap2_center,
                     cap2_rot,
                     cap2_radius,
                     cap2_height,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool BoxAndCapsuleIntersect(
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            FVector3 cap_center,
            FQuaternion cap_rot,
            FScalar cap_radius,
            FScalar cap_height,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.BoxAndCapsuleIntersect(
                     box_center,
                     box_rot,
                     box_extend,
                     cap_center,
                     cap_rot,
                     cap_radius,
                     cap_height,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylinderAndSphereIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 sphere_center,
            FScalar sphere_radius,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylinderAndSphereIntersect(
                     cylinder_center,
                     cylinder_rot,
                     cylinder_radius,
                     cylinder_height,
                     sphere_center,
                     sphere_radius,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylinderAndBoxIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extend,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylinderAndBoxIntersect(
                     cylinder_center,
                     cylinder_rot,
                     cylinder_radius,
                     cylinder_height,
                     box_center,
                     box_rot,
                     box_extend,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylinderAndCapsuleIntersect(
            FVector3 cylinder_center,
            FQuaternion cylinder_rot,
            FScalar cylinder_radius,
            FScalar cylinder_height,
            FVector3 capsule_center,
            FQuaternion capsule_rot,
            FScalar capsule_radius,
            FScalar capsule_height,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylinderAndCapsuleIntersect(
                     cylinder_center,
                     cylinder_rot,
                     cylinder_radius,
                     cylinder_height,
                     capsule_center,
                     capsule_rot,
                     capsule_radius,
                     capsule_height,
                     arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylindricalSectorAndSphereIntersect(
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle,
            FVector3 sphere_center,
            FScalar sphere_radius,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylindricalSectorAndSphereIntersect(
                                 sector_center,
                                 sector_rot,
                                 in_radius,
                                 out_radius,
                                 height,
                                 angle,
                                 sphere_center,
                                 sphere_radius,
                                 arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylindricalSectorAndBoxIntersect(
            FVector3 sector_center,
            FQuaternion sector_rot,
            FScalar in_radius,
            FScalar out_radius,
            FScalar height,
            FScalar angle,
            FVector3 box_center,
            FQuaternion box_rot,
            FVector3 box_extent,
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylindricalSectorAndBoxIntersect(
                                 sector_center,
                                 sector_rot,
                                 in_radius,
                                 out_radius,
                                 height,
                                 angle,
                                 box_center,
                                 box_rot,
                                 box_extent,
                                 arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }

        public static bool CylindricalSectorAndCapsuleIntersect(
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
            ref FVector3 hitPoint)
        {
            unsafe
            {
                FScalar* arr = stackalloc FScalar[3];

                bool rt = FMathWrapper.CylindricalSectorAndCapsuleIntersect(
                                 sector_center,
                                 sector_rot,
                                 in_radius,
                                 out_radius,
                                 height,
                                 angle,
                                 capsule_center,
                                 capsule_rot,
                                 capsule_radius,
                                 capsule_height,
                                 arr);
                if (rt)
                {
                    hitPoint.x = arr[0];
                    hitPoint.y = arr[1];
                    hitPoint.z = arr[2];
                }
                return rt;
            }
        }
    }
}
