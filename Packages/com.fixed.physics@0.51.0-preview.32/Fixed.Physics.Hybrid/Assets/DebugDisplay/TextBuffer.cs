using System;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.DebugDisplay
{
    internal struct TextBuffer : IDisposable
    {
        internal struct Instance
        {
            internal fp3 worldPosition;
            internal fp2 firstCell;
            internal fp2 cellSize;
            internal uint useWorldMatrix;
        }

        internal const int kMaxInstances = 128;
        internal const int kMaxColors = 16;

        internal UnsafeArray<Instance> m_Instance;
        internal CellSurface m_Screen;

        internal unsafe void ClearCells()
        {
            m_Screen.Clear();
        }

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxInstances);
            m_Screen = new CellSurface(new int2(Cell.kMaxWide, Cell.kMaxTall));
        }

        internal void ClearTextBox(int index)
        {
            m_Instance[index] = new Instance();
        }

        internal void SetTextBox(int2 cellXY, int2 cellWH, int index)
        {
            m_Instance[index] = new Instance
            {
                worldPosition = new fp3((fp)(cellXY.x * Cell.kPixelsWide), (fp)(cellXY.y * Cell.kPixelsTall), fp.zero),
                firstCell = new fp2(cellXY),
                cellSize = new fp2(cellWH),
                useWorldMatrix = 0
            };
        }

        internal void SetTextBoxSmooth(fp3 position, int2 cellXY, int2 cellWH, int index)
        {
            m_Instance[index] = new Instance
            {
                worldPosition = position,
                firstCell = new fp2(cellXY),
                cellSize = new fp2(cellWH),
                useWorldMatrix = 1
            };
        }

        internal void SetLabel(fp3 position, int2 cellXY, in FixedString128Bytes f, ColorIndex fg, ColorIndex bg,
            int index)
        {
            SetTextBoxSmooth(position, cellXY, new int2(CellSurface.WidthInCells(f), 1), index);
            m_Screen.PutChars(ref cellXY, f, fg, bg);
        }

        public void Dispose()
        {
            m_Instance.Dispose();
            m_Screen.Dispose();
        }

        internal Unit AllocateAll()
        {
            return new Unit(0, 1, 0, m_Instance.Length);
        }
    }
}
