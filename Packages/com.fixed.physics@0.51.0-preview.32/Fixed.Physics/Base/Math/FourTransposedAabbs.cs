using Unity.Mathematics.FixedPoint;
using Unity.Mathematics;

namespace Fixed.Physics
{
    // 4 transposed AABBs
    public struct FourTransposedAabbs
    {
        public fp4 Lx, Hx;    // Lower and upper bounds along the X axis.
        public fp4 Ly, Hy;    // Lower and upper bounds along the Y axis.
        public fp4 Lz, Hz;    // Lower and upper bounds along the Z axis.

        public static FourTransposedAabbs Empty => new FourTransposedAabbs
        {
            Lx = new fp4(fp.max_value),
            Hx = new fp4(fp.min_value),
            Ly = new fp4(fp.max_value),
            Hy = new fp4(fp.min_value),
            Lz = new fp4(fp.max_value),
            Hz = new fp4(fp.min_value)
        };

        public void SetAllAabbs(Aabb aabb)
        {
            Lx = new fp4(aabb.Min.x);
            Ly = new fp4(aabb.Min.y);
            Lz = new fp4(aabb.Min.z);
            Hx = new fp4(aabb.Max.x);
            Hy = new fp4(aabb.Max.y);
            Hz = new fp4(aabb.Max.z);
        }

        public void SetAabb(int index, Aabb aabb)
        {
            Lx[index] = aabb.Min.x;
            Hx[index] = aabb.Max.x;

            Ly[index] = aabb.Min.y;
            Hy[index] = aabb.Max.y;

            Lz[index] = aabb.Min.z;
            Hz[index] = aabb.Max.z;
        }

        public Aabb GetAabb(int index) => new Aabb
        {
            Min = new fp3(Lx[index], Ly[index], Lz[index]),
            Max = new fp3(Hx[index], Hy[index], Hz[index])
        };

        public FourTransposedAabbs GetAabbT(int index) => new FourTransposedAabbs
        {
            Lx = new fp4(Lx[index]),
            Ly = new fp4(Ly[index]),
            Lz = new fp4(Lz[index]),
            Hx = new fp4(Hx[index]),
            Hy = new fp4(Hy[index]),
            Hz = new fp4(Hz[index])
        };

        public Aabb GetCompoundAabb() => new Aabb
        {
            Min = new fp3(fpmath.cmin(Lx), fpmath.cmin(Ly), fpmath.cmin(Lz)),
            Max = new fp3(fpmath.cmax(Hx), fpmath.cmax(Hy), fpmath.cmax(Hz))
        };

        public bool4 Overlap1Vs4(ref FourTransposedAabbs aabbT)
        {
            bool4 lc = (aabbT.Lx <= Hx) & (aabbT.Ly <= Hy) & (aabbT.Lz <= Hz);
            bool4 hc = (aabbT.Hx >= Lx) & (aabbT.Hy >= Ly) & (aabbT.Hz >= Lz);
            bool4 c = lc & hc;
            return c;
        }

        public bool4 Overlap1Vs4(ref FourTransposedAabbs other, int index)
        {
            FourTransposedAabbs aabbT = other.GetAabbT(index);
            return Overlap1Vs4(ref aabbT);
        }

        public fp4 DistanceFromPointSquared(ref Math.FourTransposedPoints tranposedPoint)
        {
            fp4 px = fpmath.max(tranposedPoint.X, Lx);
            px = fpmath.min(px, Hx) - tranposedPoint.X;

            fp4 py = fpmath.max(tranposedPoint.Y, Ly);
            py = fpmath.min(py, Hy) - tranposedPoint.Y;

            fp4 pz = fpmath.max(tranposedPoint.Z, Lz);
            pz = fpmath.min(pz, Hz) - tranposedPoint.Z;

            return px * px + py * py + pz * pz;
        }

        public fp4 DistanceFromPointSquared(ref Math.FourTransposedPoints tranposedPoint, fp3 scale)
        {
            fp4 px = fpmath.max(tranposedPoint.X, Lx);
            px = (fpmath.min(px, Hx) - tranposedPoint.X) * scale.x;

            fp4 py = fpmath.max(tranposedPoint.Y, Ly);
            py = (fpmath.min(py, Hy) - tranposedPoint.Y) * scale.y;

            fp4 pz = fpmath.max(tranposedPoint.Z, Lz);
            pz = (fpmath.min(pz, Hz) - tranposedPoint.Z) * scale.z;

            return px * px + py * py + pz * pz;
        }

        public fp4 DistanceFromAabbSquared(ref FourTransposedAabbs tranposedAabb)
        {
            fp4 px = fpmath.max(fp4.zero, tranposedAabb.Lx - Hx);
            px = fpmath.min(px, tranposedAabb.Hx - Lx);

            fp4 py = fpmath.max(fp4.zero, tranposedAabb.Ly - Hy);
            py = fpmath.min(py, tranposedAabb.Hy - Ly);

            fp4 pz = fpmath.max(fp4.zero, tranposedAabb.Lz - Hz);
            pz = fpmath.min(pz, tranposedAabb.Hz - Lz);

            return px * px + py * py + pz * pz;
        }

        public fp4 DistanceFromAabbSquared(ref FourTransposedAabbs tranposedAabb, fp3 scale)
        {
            fp4 px = fpmath.max(fp4.zero, tranposedAabb.Lx - Hx);
            px = fpmath.min(px, tranposedAabb.Hx - Lx) * scale.x;

            fp4 py = fpmath.max(fp4.zero, tranposedAabb.Ly - Hy);
            py = fpmath.min(py, tranposedAabb.Hy - Ly) * scale.y;

            fp4 pz = fpmath.max(fp4.zero, tranposedAabb.Lz - Hz);
            pz = fpmath.min(pz, tranposedAabb.Hz - Lz) * scale.z;

            return px * px + py * py + pz * pz;
        }

        public bool4 Raycast(Ray ray, fp maxFraction, out fp4 fractions)
        {
            fp4 lx = Lx - new fp4(ray.Origin.x);
            fp4 hx = Hx - new fp4(ray.Origin.x);
            fp4 nearXt = lx * new fp4(ray.ReciprocalDisplacement.x);
            fp4 farXt = hx * new fp4(ray.ReciprocalDisplacement.x);

            fp4 ly = Ly - new fp4(ray.Origin.y);
            fp4 hy = Hy - new fp4(ray.Origin.y);
            fp4 nearYt = ly * new fp4(ray.ReciprocalDisplacement.y);
            fp4 farYt = hy * new fp4(ray.ReciprocalDisplacement.y);

            fp4 lz = Lz - new fp4(ray.Origin.z);
            fp4 hz = Hz - new fp4(ray.Origin.z);
            fp4 nearZt = lz * new fp4(ray.ReciprocalDisplacement.z);
            fp4 farZt = hz * new fp4(ray.ReciprocalDisplacement.z);

            fp4 nearX = fpmath.min(nearXt, farXt);
            fp4 farX = fpmath.max(nearXt, farXt);

            fp4 nearY = fpmath.min(nearYt, farYt);
            fp4 farY = fpmath.max(nearYt, farYt);

            fp4 nearZ = fpmath.min(nearZt, farZt);
            fp4 farZ = fpmath.max(nearZt, farZt);

            fp4 nearMax = fpmath.max(fpmath.max(fpmath.max(nearX, nearY), nearZ), fp4.zero);
            fp4 farMin = fpmath.min(fpmath.min(fpmath.min(farX, farY), farZ), new fp4(maxFraction));

            fractions = nearMax;

            return (nearMax <= farMin) & (lx <= hx);
        }
    }
}
