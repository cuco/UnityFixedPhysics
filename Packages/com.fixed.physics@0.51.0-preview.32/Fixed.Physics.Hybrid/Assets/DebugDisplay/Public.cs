using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.DebugDisplay
{
    internal struct Color
    {
        internal static ColorIndex Quantize(fp4 rgba)
        {
            var oldi = 0;
            var oldd = fpmath.lengthsq(rgba - Unmanaged.Instance.Data.m_ColorData[0]);
            for (var i = 1; i < Unmanaged.kMaxColors; ++i)
            {
                var newd = fpmath.lengthsq(rgba - Unmanaged.Instance.Data.m_ColorData[0]);
                if (newd < oldd)
                {
                    oldi = i;
                    oldd = newd;
                }
            }
            return new ColorIndex {value = oldi};
        }
    }

    internal struct Arrows : IDisposable
    {
        private Lines m_Lines;
        internal Arrows(int count)
        {
            m_Lines = new Lines(count * 5);
        }

        internal void Draw(fp3 x, fp3 v, Fixed.DebugDisplay.ColorIndex color)
        {
            var X0 = x;
            var X1 = x + v;

            m_Lines.Draw(X0, X1, color);

            fp3 dir;
            fp length = Physics.Math.NormalizeWithLength(v, out dir);
            fp3 perp, perp2;
            Physics.Math.CalculatePerpendicularNormalized(dir, out perp, out perp2);
            //TODO
            //fp3 scale = length * 0.2f;
            fp3 scale = length / (fp)5;

            m_Lines.Draw(X1, X1 + (perp - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp + dir) * scale, color);
            m_Lines.Draw(X1, X1 + (perp2 - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp2 + dir) * scale, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Arrow
    {
        internal static void Draw(fp3 x, fp3 v, Fixed.DebugDisplay.ColorIndex color)
        {
            new Arrows(1).Draw(x, v, color);
        }
    }

    internal struct Planes : IDisposable
    {
        private Lines m_Lines;
        internal Planes(int count)
        {
            m_Lines = new Lines(count * 9);
        }

        internal void Draw(fp3 x, fp3 v, Fixed.DebugDisplay.ColorIndex color)
        {
            var X0 = x;
            var X1 = x + v;

            m_Lines.Draw(X0, X1, color);

            fp3 dir;
            fp length = Physics.Math.NormalizeWithLength(v, out dir);
            fp3 perp, perp2;
            Physics.Math.CalculatePerpendicularNormalized(dir, out perp, out perp2);
            //TODO
            //fp3 scale = length * 0.2f;
            fp3 scale = length / (fp)5;

            m_Lines.Draw(X1, X1 + (perp - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp + dir) * scale, color);
            m_Lines.Draw(X1, X1 + (perp2 - dir) * scale, color);
            m_Lines.Draw(X1, X1 - (perp2 + dir) * scale, color);

            perp *= length;
            perp2 *= length;

            m_Lines.Draw(X0 + perp + perp2, X0 + perp - perp2, color);
            m_Lines.Draw(X0 + perp - perp2, X0 - perp - perp2, color);
            m_Lines.Draw(X0 - perp - perp2, X0 - perp + perp2, color);
            m_Lines.Draw(X0 - perp + perp2, X0 + perp + perp2, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Plane
    {
        internal static void Draw(fp3 x, fp3 v, Fixed.DebugDisplay.ColorIndex color)
        {
            new Planes(1).Draw(x, v, color);
        }
    }

    internal struct Arcs : IDisposable
    {
        private Lines m_Lines;
        const int res = 16;

        internal Arcs(int count)
        {
            m_Lines = new Lines(count * (2 + res));
        }

        internal void Draw(fp3 center, fp3 normal, fp3 arm, fp angle, Fixed.DebugDisplay.ColorIndex color)
        {
            fpquaternion q = fpquaternion.AxisAngle(normal, angle / (fp)res);
            fp3 currentArm = arm;
            m_Lines.Draw(center, center + currentArm, color);
            for (int i = 0; i < res; i++)
            {
                fp3 nextArm = fpmath.mul(q, currentArm);
                m_Lines.Draw(center + currentArm, center + nextArm, color);
                currentArm = nextArm;
            }
            m_Lines.Draw(center, center + currentArm, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Arc
    {
        internal static void Draw(fp3 center, fp3 normal, fp3 arm, fp angle,
            Fixed.DebugDisplay.ColorIndex color)
        {
            new Arcs(1).Draw(center, normal, arm, angle, color);
        }
    }

    internal struct Boxes : IDisposable
    {
        private Lines m_Lines;

        internal Boxes(int count)
        {
            m_Lines = new Lines(count * 12);
        }

        internal void Draw(fp3 Size, fp3 Center, fpquaternion Orientation, Fixed.DebugDisplay.ColorIndex color)
        {
            fp3x3 mat = fpmath.fp3x3(Orientation);
            fp3 x = mat.c0 * Size.x * fp.half;
            fp3 y = mat.c1 * Size.y * fp.half;
            fp3 z = mat.c2 * Size.z * fp.half;
            fp3 c0 = Center - x - y - z;
            fp3 c1 = Center - x - y + z;
            fp3 c2 = Center - x + y - z;
            fp3 c3 = Center - x + y + z;
            fp3 c4 = Center + x - y - z;
            fp3 c5 = Center + x - y + z;
            fp3 c6 = Center + x + y - z;
            fp3 c7 = Center + x + y + z;

            m_Lines.Draw(c0, c1, color); // ring 0
            m_Lines.Draw(c1, c3, color);
            m_Lines.Draw(c3, c2, color);
            m_Lines.Draw(c2, c0, color);

            m_Lines.Draw(c4, c5, color); // ring 1
            m_Lines.Draw(c5, c7, color);
            m_Lines.Draw(c7, c6, color);
            m_Lines.Draw(c6, c4, color);

            m_Lines.Draw(c0, c4, color); // between rings
            m_Lines.Draw(c1, c5, color);
            m_Lines.Draw(c2, c6, color);
            m_Lines.Draw(c3, c7, color);
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Box
    {
        internal static void Draw(fp3 Size, fp3 Center, fpquaternion Orientation, Fixed.DebugDisplay.ColorIndex color)
        {
            new Boxes(1).Draw(Size, Center, Orientation, color);
        }
    }

    internal struct Cones : IDisposable
    {
        private Lines m_Lines;
        const int res = 16;

        internal Cones(int count)
        {
            m_Lines = new Lines(count * res * 2);
        }

        internal void Draw(fp3 point, fp3 axis, fp angle, Fixed.DebugDisplay.ColorIndex color)
        {
            fp3 dir;
            fp scale = Physics.Math.NormalizeWithLength(axis, out dir);
            fp3 arm;
            {
                fp3 perp1, perp2;
                Physics.Math.CalculatePerpendicularNormalized(dir, out perp1, out perp2);
                arm = fpmath.mul(fpquaternion.AxisAngle(perp1, angle), dir) * scale;
            }
            fpquaternion q = fpquaternion.AxisAngle(dir, fp.two * (fp)fpmath.PI / (fp)res);

            for (int i = 0; i < res; i++)
            {
                fp3 nextArm = fpmath.mul(q, arm);
                m_Lines.Draw(point, point + arm, color);
                m_Lines.Draw(point + arm, point + nextArm, color);
                arm = nextArm;
            }
        }

        public void Dispose()
        {
            m_Lines.Dispose();
        }
    }
    internal struct Cone
    {
        internal static void Draw(fp3 point, fp3 axis, fp angle, Fixed.DebugDisplay.ColorIndex color)
        {
            new Cones(1).Draw(point, axis, angle, color);
        }
    }

    internal struct Lines : IDisposable
    {
        Unit m_Unit;
        internal Lines(int count)
        {
            DebugDisplay.Instantiate();
            m_Unit = Unmanaged.Instance.Data.m_LineBufferAllocations.AllocateAtomic(count);
        }

        internal void Draw(fp3 begin, fp3 end, ColorIndex color)
        {
            if (m_Unit.m_Next < m_Unit.m_End)
                Unmanaged.Instance.Data.m_LineBuffer.SetLine(begin, end, color, m_Unit.m_Next++);
        }

        public void Dispose()
        {
            while (m_Unit.m_Next < m_Unit.m_End)
                Unmanaged.Instance.Data.m_LineBuffer.ClearLine(m_Unit.m_Next++);
        }
    }
    internal struct Line
    {
        internal static void Draw(fp3 begin, fp3 end, ColorIndex color)
        {
            new Lines(1).Draw(begin, end, color);
        }
    }

    internal struct TextBox
    {
        internal static void Draw(int2 xy, int2 wh)
        {
            DebugDisplay.Instantiate();
            var unit = Unmanaged.Instance.Data.m_TextBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var text = Unmanaged.Instance.Data.m_TextBuffer;
            text.SetTextBox(xy, wh, unit.m_Next);
        }
    }

    internal struct Label
    {
        internal static void Draw(fp3 position, in FixedString128Bytes s, ColorIndex fg, ColorIndex bg)
        {
            DebugDisplay.Instantiate();
            var unit = Unmanaged.Instance.Data.m_TextBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var text = Unmanaged.Instance.Data.m_TextBuffer;
            text.SetLabel(position, new int2(0, text.m_Screen.m_Size.y + unit.m_Next), s, fg, bg, unit.m_Next);
        }
    }

    internal struct Text
    {
        internal static void Draw(int x, int y, byte color, in FixedString128Bytes f)
        {
            DebugDisplay.Instantiate();
            var xy = new int2(x, y);
            Unmanaged.Instance.Data.m_TextBuffer.m_Screen.PutChars(ref xy, f, ColorIndex.Foreground(color), ColorIndex.Background(color));
        }
    }

    internal struct Log
    {
        internal static void Info(in FixedString128Bytes f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Debug(in FixedString128Bytes f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Error(in FixedString128Bytes f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Warn(in FixedString128Bytes f)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.WriteLine(f);
        }

        internal static void Draw(Window w)
        {
            DebugDisplay.Instantiate();
            Unmanaged.Instance.Data.m_LogBuffer.CopyToWithFrame(ref Unmanaged.Instance.Data.m_TextBuffer.m_Screen, w);
        }
    }

    internal struct GraphData
    {
        Unit m_unit;

        internal int Length => m_unit.Length;

        internal GraphData(int count)
        {
            DebugDisplay.Instantiate();
            m_unit = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(count);
        }

        internal fp GetValue(int offset)
        {
            return Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Begin + (offset & (m_unit.Length - 1))];
        }

        internal void SetValue(int offset, fp value)
        {
            Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Begin + (offset & (m_unit.Length - 1))] = value;
        }

        internal unsafe ref fp this[int offset] =>
            ref ((fp*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr())[m_unit.m_Begin + (offset & (m_unit.Length - 1))];

        internal void AddValue(fp value)
        {
            if (m_unit.m_Next >= m_unit.m_End)
                m_unit.m_Next = m_unit.m_Begin;
            Unmanaged.Instance.Data.m_GraphBuffer.m_Data[m_unit.m_Next++] = value;
        }

        internal Data GetData()
        {
            return new Data { offset = m_unit.m_Begin, length = m_unit.Length };
        }

        internal unsafe void CalcMinMaxMean(out fp minValue, out fp maxValue, out fp mean)
        {
            fp* f = (fp*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr();
            minValue = f[m_unit.m_Begin];
            maxValue = f[m_unit.m_Begin];
            fp sum = f[m_unit.m_Begin];
            for (var i = m_unit.m_Begin + 1; i < m_unit.m_End; i++)
            {
                var x = f[i];
                sum += x;
                if (x < minValue) minValue = x;
                if (x > maxValue) maxValue = x;
            }

            mean = sum / (fp)m_unit.Length;
        }

        internal unsafe void CalcStatistics(out fp mean, out fp variance, out fp minValue,
            out fp maxValue)
        {
            CalcMinMaxMean(out minValue, out maxValue, out mean);
            fp* f = (fp*)Unmanaged.Instance.Data.m_GraphBuffer.m_Data.GetUnsafePtr();
            fp sum2 = fp.zero;
            for (var i = m_unit.m_Begin; i < m_unit.m_End; i++)
            {
                fp d = f[i] - mean;
                sum2 += d * d;
            }

            variance = sum2 / (fp)m_unit.Length;
        }
    }

    internal struct Graph
    {
        internal int Color;
        internal Unit DataSeries0;
        internal Unit DataSeries1;
        internal fp YMin;
        internal fp YMax;

        internal Graph(int color, fp yMin, fp yMax, int Length0, int Length1 = 0)
        {
            DebugDisplay.Instantiate();
            Color = color;
            DataSeries0 = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(Length0);
            DataSeries1 = Unmanaged.Instance.Data.m_GraphDataBufferAllocations.AllocateAtomic(Length1);
            YMin = yMin;
            YMax = yMax;
        }

        internal void Draw(Window window, int frame)
        {
            var unit = Unmanaged.Instance.Data.m_GraphBufferAllocations.AllocateAtomic(1);
            if (unit.Length == 0)
                return;
            var x = window.m_begin.x;
            var y = window.m_begin.y;
            var w = window.Size.x;
            var h = window.Size.y;
            if (DataSeries1.Length > 0)
                Unmanaged.Instance.Data.m_GraphBuffer.SetGraph(x, y, w, h, new GraphBuffer.Sample
                {
                    data = DataSeries0, ColorIndex = ColorIndex.Foreground(Color), xMin = (fp)(frame - 1 - w * 8), xMax = (fp)(frame - 1),
                    yMin = YMin, yMax = YMax
                }, new GraphBuffer.Sample
                    {
                        data = DataSeries1, ColorIndex = ColorIndex.Background(Color), xMin = (fp)(frame - 1 - w * 8), xMax = (fp)(frame - 1),
                        yMin = YMin, yMax = YMax
                    }, unit.m_Next);
            else
                Unmanaged.Instance.Data.m_GraphBuffer.SetGraph(x, y, w, h, new GraphBuffer.Sample
                {
                    data = DataSeries0, ColorIndex = ColorIndex.Foreground(Color), xMin = (fp)(frame - 1 - w * 8), xMax = (fp)(frame - 1),
                    yMin = YMin, yMax = YMax
                }, unit.m_Next);
        }
    }

    internal class DebugDisplay
    {
        internal static void Render()
        {
            Managed.Instance.CopyFromCpuToGpu();
            Managed.Instance.Render();
        }

        internal static void Clear()
        {
            Managed.Instance.Clear();
        }

        [BurstDiscard]
        internal static void Instantiate()
        {
            if (Managed.Instance == null)
                Managed.Instance = new Managed();
        }
    }
}
