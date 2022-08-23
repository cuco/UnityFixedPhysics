using System;
using System.Diagnostics;
using System.Text;

public class LockstepDebug
{
	private static Action<string> _logCallBack;
	private static Action<string> _logWarningCallBack;
	private static Action<string> _logErrorCallBack;
	private static bool isEnabled = false;
	//private static bool isEnabledProfile = false;
	private static Action<string> _beginSample;
	private static Action _endSample;

	public static void RegisteLogCallback(
		Action<string> callBack, //
		Action<string> warningCallBack, //
		Action<string> errorCallBack //
	)
	{
		_logCallBack = callBack;
		_logWarningCallBack = warningCallBack;
		_logErrorCallBack = errorCallBack;
		isEnabled = _logCallBack != null;
		//isEnabledProfile = false;
	}
	
	/// <summary>
	/// 注册Unity Profile的回调
	/// </summary>
	/// <param name="beginSample"></param>
	/// <param name="endSample"></param>
	public static void RegisterProfilerCallback(Action<string> beginSample, Action endSample)
	{
		_beginSample = beginSample;
		_endSample = endSample;
	}
	 
	[Conditional("DEBUG")]
	public static void Log(string obj = null)
	{
        _logCallBack?.Invoke(obj);
	}

	[Conditional("DEBUG")]
	public static void LogWarning(string obj = null)
	{
		_logWarningCallBack?.Invoke (obj);
	}

	public static void DeviceLog(string obj=null)
	{
		_logErrorCallBack?.Invoke(obj);
	}

    [Conditional("DEBUG")]
	public static void LogError(string obj=null)
	{
        _logErrorCallBack?.Invoke(obj);
	}

    [Conditional("_DEBUG")]
    public static void Assert(bool expression)
	{
		if (!expression)
			LogError ("Assertion failed");
	}

    [Conditional("NATIVECHECK")]
    public static void CheckCSAndCPP(bool expression)
    {
        if (!expression)
        {
            string stackInfo = new StackTrace(false).ToString();
            _logErrorCallBack?.Invoke("NATIVECHECK failure" + "\n" + stackInfo);
        }
    }

    [Conditional("NATIVECHECK")]
    public static void CheckCSAndCPP(bool expression, object obj )
    {
        if (!expression)
        {
            string stackInfo = new StackTrace(false).ToString();
            _logErrorCallBack?.Invoke("NATIVECHECK failure" + "\n" + obj.ToString() + "\n" + stackInfo);
        }
    }
    
    /// <summary>
    /// Unity BeginSample
    /// </summary>
    /// <param name="profilerName"></param>
    [Conditional("DEBUG")]
    public static void BeginSample(string profilerName)
    {
	    _beginSample?.Invoke(profilerName);
    }

	/// <summary>
	/// Unity EndSample, 要和BeginSample成对出现
	/// </summary>
    [Conditional("DEBUG")]
    public static void EndSample()
	{
		_endSample?.Invoke();
	}
}
