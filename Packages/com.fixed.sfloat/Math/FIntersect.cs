using System;

namespace Morefun.LockStep
{
    public interface IFIntersect
    {
        bool DiskDisk(FVector2 c1, FScalar r1, FVector2 c2, FScalar r2);
        bool LineDisk(FVector2 lPoint1, FVector2 lPoint2, FVector2 dCenter, FScalar dRadius);
        bool CapsuleAABB(FVector2 origin, FVector2 direction, FScalar radius, FVector2 p, FVector2 h, out FScalar t);
        bool DiskSector(FVector2 diskCenter,FScalar  diskRadius, FVector2 secCenter,FVector2 secDir,FVector2 secDirVertical,
            FVector2 VTLeftOutPoint,FVector2 VTLeftInPoint,	FScalar secRadiusOut, FScalar secRadiusIn, FScalar cosHalfTheta );
        bool DiskOBB(FVector2 c, FScalar r, FVector2 p, FVector2 h, FVector2 u);
        bool DiskAABB(FVector2 c, FScalar r, FVector2 p, FVector2 h);
        bool OBBOBB(FVector2 p1, FVector2 h1, FVector2 u1, FVector2 p2, FVector2 h2, FVector2 u2);
        bool AABBAABB(FVector2 p1, FVector2 h1, FVector2 p2, FVector2 h2);
        bool RayDisk(FVector2 o, FVector2 d, FVector2 c, FScalar r, out FScalar t);
        bool RayAABB(FVector2 o, FVector2 d, FVector2 p, FVector2 h, out FScalar t);
        bool RayOBB(FVector2 o, FVector2 d, FVector2 p, FVector2 h, FVector2 u, out FScalar t);
    }
	public class FIntersect
    {

        public static void SetIntersectInterface( IFIntersect TIF )
        {
            m_Intersect = TIF;
        }

		public static IFIntersect m_Intersect = null;
        public static bool DiskDisk(FVector2 c1, FScalar r1, FVector2 c2, FScalar r2)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.DiskDisk (c1, r1, c2, r2);
			}
			else
			{
				FScalar rsum = r1 + r2;
				return (c1 - c2).sqrMagnitudeRaw <= FScalar.MultiplyRaw (rsum, rsum);
			}
        }

		public static bool LineDisk(FVector2 lPoint1, FVector2 lPoint2, FVector2 dCenter, FScalar dRadius)
		{
			if (m_Intersect != null)
			{
				return m_Intersect.LineDisk (lPoint1, lPoint2, dCenter, dRadius);	
			} 
			else 
			{
				FVector2 TDir = lPoint2 - lPoint1;

				FScalar uLength = TDir.sqrMagnitude;

				if (uLength == 0) 
				{
					if ((dCenter - lPoint1).sqrMagnitudeRaw > FScalar.MultiplyRaw (dRadius, dRadius)) 
					{
						return false;
					} 
					else 
					{
						return true;
					}
				} 
				else 
				{
					FScalar T = FVector2.Dot (dCenter - lPoint1, TDir) / uLength;

					FVector2 VNearest = lPoint1 + FScalar.Clamp (T, 0, 1) * TDir;

					if ((dCenter - VNearest).sqrMagnitudeRaw > FScalar.MultiplyRaw (dRadius, dRadius)) 
					{
						return false;
					}
					else 
					{
						return true;
					}
				}
			}
		}

        static bool PointAABB(FVector2 TPoint, FVector2 p, FVector2 h, FScalar radius)
        {
            if ((p.x - h.x) <= TPoint.x && TPoint.x <= (p.x + h.x) &&
                (p.y - h.y - radius) <= TPoint.y && TPoint.y <= (p.y + h.y + radius))
            {
                return true;
            }

            if ((p.x - h.x - radius) <= TPoint.x && TPoint.x <= (p.x + h.x + radius) &&
                (p.y - h.y) <= TPoint.y && TPoint.y <= (p.y + h.y))
            {
                return true;
            }

            FVector2 VCenter = new FVector2(
                 TPoint.x - p.x > 0 ? p.x + h.x : p.x - h.x,
                 TPoint.y - p.y > 0 ? p.y + h.y : p.y - h.y);

            return (VCenter - TPoint).sqrMagnitudeRaw <= FScalar.MultiplyRaw(radius, radius);
        }

        public static bool CapsuleAABB(FVector2 origin, FVector2 direction, FScalar radius, FVector2 p, FVector2 h, out FScalar t)
        {

			if (m_Intersect != null)
			{
				return m_Intersect.CapsuleAABB(origin, direction, radius, p, h, out  t);
			}
			else
			{
				FVector2 VNewHalf = new FVector2(h.x + radius, h.y + radius);

				if (PointAABB(origin, p, h, radius))
				{
					t = 0;
					return true;
				}

				if ((p.x - h.x - radius) <= origin.x && origin.x <= (p.x + h.x + radius) &&
				             (p.y - h.y - radius) <= origin.y && origin.y <= (p.y + h.y + radius))
				{
					FVector2 TCirclePos = new FVector2(
						                                  origin.x - p.x > 0 ? p.x + h.x : p.x - h.x,
						                                  origin.y - p.y > 0 ? p.y + h.y : p.y - h.y);

					return RayDisk(origin, direction, TCirclePos, radius, out t);
				}

				FScalar T1 = FScalar.zero;
				if (RayAABB(origin, direction, p, VNewHalf, out T1))
				{
					FVector2 V1 = origin + direction * T1;

					FVector2 TCirclePos = new FVector2();
					if (V1.x < (p.x - h.x))
					{
						TCirclePos.x = p.x - h.x;
					}
					else if (V1.x <= (p.x + h.x))
					{
						t = T1;
						return true;
					}
					else
					{
						TCirclePos.x = p.x + h.x;
					}

					if (V1.y < (p.y - h.y))
					{
						TCirclePos.y = p.y - h.y;
					}
					else if (V1.y <= (p.y + h.y))
					{
						t = T1;
						return true;
					}
					else
					{
						TCirclePos.y = p.y + h.y;
					}


					return RayDisk(origin, direction, TCirclePos, radius, out t);
				}
				else
				{
					t = 0;
					return false;
				}
			}
        }




        // RTR3 p.714
        public static bool RayDisk(FVector2 o, FVector2 d, FVector2 c, FScalar r, out FScalar t)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.RayDisk(o, d, c, r, out t);
			}
			else
			{
				FVector2 l = c - o;
                FScalar DLength = d.magnitude;
                if( DLength > 0 )
                {
                    FScalar s = FVector2.Dot(l, d) / DLength;
                    long ll = FVector2.DotRaw(l, l);
                    long rr = FScalar.MultiplyRaw(r, r);
                    if (s < FScalar.zero && ll > rr)
                    {
                        t = 0;
                        return false;
                    }

                    long mm = ll - FScalar.MultiplyRaw(s, s);
                    if (mm > rr)
                    {
                        t = 0;
                        return false;
                    }

                    FScalar q = FScalar.SqrtRaw(rr - mm);
                    if (ll >= rr)
                        t = s - q;
                    else
                        t = s + q;

                    return true;
                }
                else
                {
                    t = 0;
                    return false;
                }
			}
        }

        public static bool DiskOBB(FVector2 c, FScalar r, FVector2 p, FVector2 h, FVector2 u)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.DiskOBB(c, r, p, h, u);
			}
			else
			{
				// Transform disk to box's coordinate system
				FVector2 v = new FVector2(-u.y, u.x);
				FVector2 d = c - p;
				FVector2 tc = new FVector2(FVector2.Dot(d, u), FVector2.Dot(d, v));
				return DiskAABB(tc, r, FVector2.zero, h);
			}
        }

		public static bool DiskSector(
			FVector2 diskCenter,
			FScalar  diskRadius, 
			FVector2 secCenter,
			FVector2 secDir,
			FVector2 secDirVertical,
			FVector2 VTLeftOutPoint,
			FVector2 VTLeftInPoint,
			FScalar  secRadiusOut,
			FScalar  secRadiusIn,
			FScalar  cosHalfTheta )
		{
			if (m_Intersect != null)
			{
				return m_Intersect.DiskSector(
					diskCenter,
					diskRadius, 
					secCenter,
					secDir,
					secDirVertical,
					VTLeftOutPoint,
					VTLeftInPoint,
					secRadiusOut,
					secRadiusIn,
					cosHalfTheta);
			}
			else
			{

				FVector2 VDelta = diskCenter - secCenter;

				long FDis = VDelta.sqrMagnitudeRaw;
	
				if (FDis > FScalar.MultiplyRaw(secRadiusOut + diskRadius, secRadiusOut + diskRadius))
					return false;

				if (secRadiusIn > diskRadius && FDis < FScalar.MultiplyRaw(secRadiusIn - diskRadius, secRadiusIn - diskRadius))
					return false;


				FVector2 VDir = VDelta.normalized;

				FScalar px = FVector2.Dot(VDir, secDir);

				if (px > cosHalfTheta)
				{
					return true;
				}

				FScalar py = FVector2.Dot(VDelta, secDirVertical);

				FVector2 newDiskPos = new FVector2(diskCenter.x, diskCenter.y);

				if (py < 0)
				{
					newDiskPos = diskCenter - secDirVertical * py * FScalar.two;
				}
				
				return FIntersect.LineDisk(VTLeftOutPoint, VTLeftInPoint, newDiskPos, diskRadius);
			}
		}
			
        /// <summary>
        /// Intersection test between disk and AABB.
        /// </summary>
        /// <param name="c">Center of disk</param>
        /// <param name="r">Radius of disk.</param>
        /// <param name="p">Center of AABB.</param>
        /// <param name="h">Half-extent of AABB.</param>
        /// <remarks>https://www.zhihu.com/question/24251545</remarks>
        public static bool DiskAABB(FVector2 c, FScalar r, FVector2 p, FVector2 h)
        {
            FVector2 v = FVector2.Abs(p - c);
            FVector2 u = FVector2.Max(v - h, FVector2.zero);
            return FVector2.DotRaw(u, u) <= FScalar.MultiplyRaw(r, r);
        }

        public static bool OBBOBB(FVector2 p1, FVector2 h1, FVector2 u1, FVector2 p2, FVector2 h2, FVector2 u2)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.OBBOBB(p1, h1, u1, p2, h2, u2);
			}
			else
			{
				FVector2 t = p2 - p1;
				FVector2 v1 = new FVector2(-u1.y, u1.x);
				FVector2 v2 = new FVector2(-u2.y, u2.x);

				// SAT on u1
				if (FScalar.Abs(FVector2.Dot(t, u1)) > h1.x + FScalar.Abs(FVector2.Dot(u2 * h2.x, u1)) + FScalar.Abs(FVector2.Dot(v2 * h2.y, u1)))
					return false;

				// SAT on v1
				if (FScalar.Abs(FVector2.Dot(t, v1)) > h1.y + FScalar.Abs(FVector2.Dot(u2 * h2.x, v1)) + FScalar.Abs(FVector2.Dot(v2 * h2.y, v1)))
					return false;

				// SAT on u2
				if (FScalar.Abs(FVector2.Dot(t, u2)) > h2.x + FScalar.Abs(FVector2.Dot(u1 * h1.x, u2)) + FScalar.Abs(FVector2.Dot(v1 * h1.y, u2)))
					return false;

				// SAT on v2
				if (FScalar.Abs(FVector2.Dot(t, v2)) > h2.y + FScalar.Abs(FVector2.Dot(u1 * h1.x, v2)) + FScalar.Abs(FVector2.Dot(v1 * h1.y, v2)))
					return false;

				return true;
			}
        }

        public static bool AABBAABB(FVector2 p1, FVector2 h1, FVector2 p2, FVector2 h2)
        {
            FVector2 d = FVector2.Abs(p2 - p1);
            FVector2 h = h1 + h2;
            return d.x < h.x && d.y < h.y;
        }

        // RTR3 p.743
        public static bool RayOBB(FVector2 o, FVector2 d, FVector2 p, FVector2 h, FVector2 u, out FScalar t)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.RayOBB(o, d, p, h, u, out t);
			}
			else
			{

				FScalar tmin = FScalar.minValue;
				FScalar tmax = FScalar.maxValue;

				p = p - o;
				FVector2 v = new FVector2(-u.y, u.x);

				FScalar e, f;

				// u direction
				e = FVector2.Dot(u, p);
				f = FVector2.Dot(u, d);
				if (FScalar.Abs(f) > FScalar.zero)
				{
					FScalar t1 = (e + h.x) / f;
					FScalar t2 = (e - h.x) / f;

					if (t1 > t2)
						FScalar.Swap(ref t1, ref t2);
                
					if (t1 > tmin)
						tmin = t1;
                
					if (t2 < tmax)
						tmax = t2;
                
					if (tmin > tmax || tmax < FScalar.zero)
					{
						t = 0;
						return false;
					}
				}
				else if (-e - h.x > FScalar.zero || -e + h.x < FScalar.zero)
				{
					t = 0;
					return false;
				}

				// v direction
				e = FVector2.Dot(v, p);
				f = FVector2.Dot(v, d);
				if (FScalar.Abs(f) > FScalar.zero)
				{
					FScalar t1 = (e + h.y) / f;
					FScalar t2 = (e - h.y) / f;

					if (t1 > t2)
						FScalar.Swap(ref t1, ref t2);
                
					if (t1 > tmin)
						tmin = t1;
                
					if (t2 < tmax)
						tmax = t2;

					if (tmin > tmax || tmax < FScalar.zero)
					{
						t = 0;
						return false;
					}
				}
				else if (-e - h.y > FScalar.zero || -e + h.y < FScalar.zero)
				{
					t = 0;
					return false;
				}

				if (tmin > 0)
					t = tmin;
				else
					t = tmax;
				return true;
			}
        }

        public static bool RayAABB(FVector2 o, FVector2 d, FVector2 p, FVector2 h, out FScalar t)
        {
			if (m_Intersect != null)
			{
				return m_Intersect.RayAABB(o, d, p, h, out t);
			}
			else
			{
				FScalar tmin = FScalar.minValue;
				FScalar tmax = FScalar.maxValue;

				p = p - o;

				FScalar e, f;

				// u direction
				e = p.x;
				f = d.x;
				if (FScalar.Abs(f) > FScalar.zero)
				{
					FScalar t1 = (e + h.x) / f;
					FScalar t2 = (e - h.x) / f;

					if (t1 > t2)
						FScalar.Swap(ref t1, ref t2);

					if (t1 > tmin)
						tmin = t1;

					if (t2 < tmax)
						tmax = t2;

					if (tmin > tmax || tmax < FScalar.zero)
					{
						t = 0;
						return false;
					}
				}
				else if (-e - h.x > FScalar.zero || -e + h.x < FScalar.zero)
				{
					t = 0;
					return false;
				}

				// v direction
				e = p.y;
				f = d.y;
				if (FScalar.Abs(f) > FScalar.zero)
				{
					FScalar t1 = (e + h.y) / f;
					FScalar t2 = (e - h.y) / f;

					if (t1 > t2)
						FScalar.Swap(ref t1, ref t2);

					if (t1 > tmin)
						tmin = t1;

					if (t2 < tmax)
						tmax = t2;

					if (tmin > tmax || tmax < FScalar.zero)
					{
						t = 0;
						return false;
					}
				}
				else if (-e - h.y > FScalar.zero || -e + h.y < FScalar.zero)
				{
					t = 0;
					return false;
				}

				if (tmin > 0)
					t = tmin;
				else
					t = tmax;
				return true;
			}
        }

        public static FVector2 ClosestPtPointTriangle(FVector2 p, FVector2 a, FVector2 b, FVector2 c )
        {
	        // Check if P in vertex region outside A
	        FVector2 ab = b - a;
	        FVector2 ac = c - a;
	        FVector2 ap = p - a;
	        FScalar d1 = FVector2.Dot(ab, ap);
	        FScalar d2 = FVector2.Dot(ac, ap);
	        if(d1<=0 && d2<=0)
	        {
// 		        s = 0;
// 		        t = 0;
		        return a;	// Barycentric coords 1,0,0
	        }

	        // Check if P in vertex region outside B
	        FVector2 bp = p - b;
	        FScalar d3 = FVector2.Dot(ab, bp);
	        FScalar d4 = FVector2.Dot(ac, bp);
	        if(d3>=0 && d4<=d3)
	        {
// 		        s = 1;
// 		        t = 0;
		        return b;	// Barycentric coords 0,1,0
	        }

	        // Check if P in edge region of AB, if so return projection of P onto AB
	        FScalar vc = d1*d4 - d3*d2;
	        if(vc<=0 && d1>=0 && d3<=0)
	        {
                FScalar v = 0;

                if( d1 - d3 != 0 )
                {
                    v = d1 / (d1 - d3);
                }

// 		        s = v;
// 		        t = 0;
		        return a + v * ab;	// barycentric coords (1-v, v, 0)
	        }

	        // Check if P in vertex region outside C
	        FVector2 cp = p - c;
	        FScalar d5 = FVector2.Dot(ab, cp);
	        FScalar d6 = FVector2.Dot(ac, cp);
	        if(d6>=0 && d5<=d6)
	        {
// 		        s = 0;
// 		        t = 1;
		        return c;	// Barycentric coords 0,0,1
	        }

	        // Check if P in edge region of AC, if so return projection of P onto AC
	        FScalar vb = d5*d2 - d1*d6;
	        if(vb<=0 && d2>=0 && d6<=0)
	        {
                FScalar w = 0;
                if (d1 - d3 != 0)
                {
                    w = d2 / (d2 - d6);
                }

// 		        s = 0;
// 		        t = w;
		        return a + w * ac;	// barycentric coords (1-w, 0, w)
	        }

	        // Check if P in edge region of BC, if so return projection of P onto BC
	        FScalar va = d3*d6 - d5*d4;
	        if(va<=0 && (d4-d3)>=0 && (d5-d6)>=0)
	        {
		        FScalar w = 0;

                if( ((d4 - d3) + (d5-d6)) != 0 )
                {
                    w = (d4-d3) / ((d4 - d3) + (d5-d6));
                }

// 		        s = 1-w;
// 		        t = w;
		        return b + w * (c-b);	// barycentric coords (0, 1-w, w)
	        }

            return p;

	       /* // P inside face region. Compute Q through its barycentric coords (u,v,w)
	        FScalar denom = 0;

            if( (va + vb + vc) != 0)
            {
                denom = 1 / (va + vb + vc);
            }
	        FScalar vt = vb * denom;
	        FScalar wt = vc * denom;
// 	        s = vt;
// 	        t = wt;
	        return a + ab*vt + ac*wt;*/
        }

    }
}
