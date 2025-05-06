using OpenTK.Mathematics;
using OpenTK.Platform;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT;
public readonly struct WindowInfo
{
    public readonly WindowHandle Handle;
    public readonly EventQueue EventQueue;
    public readonly Input Input;

    public WindowInfo(WindowHandle handle, EventQueue eventQueue, Input input)
    {
        Handle = handle;
        EventQueue = eventQueue;
        Input = input;
    }
}

struct OpenWindowHandle
{
    public readonly int Handle;

    public OpenWindowHandle(int handle)
    {
        Handle = handle;
    }
}

internal class WindowHandler
{
    int _createWindows = 0;
    private readonly Queue<int> _recycledHandles;
    private readonly Dictionary<int, WindowInfo> _windows;
    public WindowHandler()
    {
        _recycledHandles = new();
        _windows = [];
    }

    public WindowInfo GetInfo(OpenWindowHandle handle)
    {
        return _windows[handle.Handle];
    }

    public OpenWindowHandle Open(string? title = null)
    {
        GraphicsApiHints contextSettings = new VulkanGraphicsApiHints();
        var window = Toolkit.Window.Create(contextSettings);
        Toolkit.Window.SetTitle(window, title ?? $"{nameof(VBeamRT)} Window");
        Toolkit.Window.SetClientSize(window, new(800, 600));
        Toolkit.Window.SetBorderStyle(window, WindowBorderStyle.ResizableBorder);
        Toolkit.Window.SetMode(window, WindowMode.Normal);
        //TODO: IDK Why this is broken
        //if (Toolkit.Mouse.SupportsRawMouseMotion)
        //{
        //    Toolkit.Mouse.EnableRawMouseMotion(window, true);
        //}
        var eventQueue = EventQueue.Subscribe(window);
        int handle;
        if (_recycledHandles.Count > 0)
        {
            handle = _recycledHandles.Dequeue();
        }
        else
        {
            handle = ++_createWindows;
        }
        var input = new Input(window, eventQueue);
        _windows.Add(handle, new WindowInfo(window, eventQueue, input));
        return new OpenWindowHandle(handle);
    }

    public void Close(OpenWindowHandle handle)
    {
        if (!_windows.TryGetValue(handle.Handle, out var info))
        {
            return;
        }

        info.EventQueue.Dispose();
        Toolkit.Window.Destroy(info.Handle);

        _recycledHandles.Enqueue(handle.Handle);
    }

    public void Update()
    {
        foreach (var info in _windows.Values)
        {
            info.Input.Update();
        }
    }
}
