using System;
using System.Diagnostics;
using Morefun.LockStep;
using NUnit.Framework;
using UnityEngine;

namespace SfloatTest
{
	[TestFixture()]
	public class SfloatTest
	{
		[Test()]
		public void Performance()
		{
			float f = 101.234631f;
			float t = 5.8431f;
			
			sfloat sf = (sfloat) f;
			sfloat st = (sfloat) t;

			FScalar ff = FScalar.FromFloat(f);
			FScalar ft = FScalar.FromFloat(t);
			Assert.AreEqual(f, (float) sf);
			//Assert.AreEqual(f,ff.ToFloat());

			Stopwatch sw = new Stopwatch();
			sw.Start();
			for (int i = 0; i < 1000000; i++)
			{
				f *= t;
				f /= t;
			}
			sw.Stop();
			UnityEngine.Debug.Log("float 用时：" + sw.ElapsedMilliseconds);
			
			sw.Restart();
			for (int i = 0; i < 1000000; i++)
			{
				sf *= st;
				sf /= st;
			}
			sw.Stop();
			UnityEngine.Debug.Log("sfloat 用时："+sw.ElapsedMilliseconds);
			
			sw.Restart();
			for (int i = 0; i < 1000000; i++)
			{
				ff *= ft;
				ff /= ft;
			}
			sw.Stop();
			UnityEngine.Debug.Log("Fscalar 用时："+sw.ElapsedMilliseconds);
			
			//Assert.AreEqual(f, (float) sf);
			Assert.AreEqual(f, ff.ToFloat());
		}

	}
}
