using OpenTK.Mathematics;
using OpenTK.Platform;
namespace VBeamRT;
public sealed class Input
{
    public readonly WindowHandle? WindowHandle;

    public Input(WindowHandle? windowHandle = null, EventQueue? eventQueue = null)
    {
        this.WindowHandle = windowHandle;
        if (eventQueue == null)
        {
            EventQueue.EventRaised += EventQueue_EventDispatched;
        }
        else
        {
            eventQueue.EventDispatched += EventQueue_EventDispatched;
        }
    }

    //Keyboard
    readonly bool[] _previousKeyboardStates = new bool[(int)Scancode.VolumeDecrement + 1];
    readonly bool[] _keyboardStates = new bool[(int)Scancode.VolumeDecrement + 1];

    public bool KeyPressed(Scancode scancode)
    {
        return _keyboardStates[(int)scancode] & !_previousKeyboardStates[(int)scancode];
    }
    public bool KeyHeld(Scancode scancode)
    {
        return _keyboardStates[(int)scancode];
    }
    public bool KeyReleased(Scancode scancode)
    {
        return !_keyboardStates[(int)scancode] & _previousKeyboardStates[(int)scancode];
    }

    //Mouse
    private Vector2 _globalMousePosition;
    public Vector2 GlobalMousePosition
    {
        get { return _globalMousePosition; }
        set
        {
            if (Toolkit.Mouse.CanSetMousePosition)
            {
                _globalMousePosition = value;
                Toolkit.Mouse.SetGlobalPosition(_globalMousePosition);
            }
        }
    }
    public Vector2 _prevMousePosition;
    public Vector2 MousePosition { get; private set; }
    public Vector2 MouseDelta { get; private set; }
    public Vector2 ScrollDelta { get; private set; }
    readonly bool[] _previousMouseButtonStates = new bool[(int)MouseButton.Button8 + 1];
    readonly bool[] _mouseButtonStates = new bool[(int)MouseButton.Button8 + 1];
    public CursorCaptureMode CursorCaptureMode
    {
        get
        {
            if (WindowHandle != null)
            {
                return Toolkit.Window.GetCursorCaptureMode(WindowHandle);
            }

            return CursorCaptureMode.Normal;
        }

        set
        {
            if (WindowHandle != null)
            {
                Toolkit.Window.SetCursorCaptureMode(WindowHandle, value);
                if (value == CursorCaptureMode.Locked)
                {
                    Toolkit.Window.SetCursor(WindowHandle, null);
                }
                else
                {
                    Toolkit.Window.SetCursor(WindowHandle, Toolkit.Cursor.Create(SystemCursorType.Default));
                }
            }
        }
    }

    public bool MouseButtonPressed(MouseButton mouseButton)
    {
        return _mouseButtonStates[(int)mouseButton] & !_previousMouseButtonStates[(int)mouseButton];
    }
    public bool MouseButtonHeld(MouseButton mouseButton)
    {
        return _mouseButtonStates[(int)mouseButton];
    }
    public bool MouseButtonReleased(MouseButton mouseButton)
    {
        return !_mouseButtonStates[(int)mouseButton] & _previousMouseButtonStates[(int)mouseButton];
    }

    private void EventQueue_EventDispatched(PalHandle? handle, PlatformEventType type, EventArgs args)
    {
        if (args is RawMouseMoveEventArgs rawMouseMoveEvent)
        {
            const float rawMouseMotionScale = 1.0f / 65536.0f;
            MouseDelta += rawMouseMoveEvent.Delta * rawMouseMotionScale;
            Console.WriteLine(rawMouseMoveEvent.Delta);
        }
        else if (args is MouseMoveEventArgs mouseMoveEventArgs)
        {
            if (WindowHandle != null && Toolkit.Mouse.SupportsRawMouseMotion && !Toolkit.Mouse.IsRawMouseMotionEnabled(WindowHandle))
            {
                MouseDelta += _prevMousePosition - mouseMoveEventArgs.ClientPosition;
            }

            MousePosition = mouseMoveEventArgs.ClientPosition;
            Toolkit.Window.ClientToScreen(mouseMoveEventArgs.Window, mouseMoveEventArgs.ClientPosition, out var gPos);
            _globalMousePosition = gPos;
        }
        else if (args is MouseButtonDownEventArgs mouseButtonDownEventArgs)
        {
            _mouseButtonStates[(int)mouseButtonDownEventArgs.Button] = true;
        }
        else if (args is MouseButtonUpEventArgs mouseButtonUpEventArgs)
        {
            _mouseButtonStates[(int)mouseButtonUpEventArgs.Button] = false;
        }
        else if (args is ScrollEventArgs scrollEventArgs)
        {
            ScrollDelta = scrollEventArgs.Delta;
        }
        else if (args is KeyDownEventArgs keyDownEventArgs && !keyDownEventArgs.IsRepeat)
        {
            _keyboardStates[(int)keyDownEventArgs.Scancode] = true;
        }
        else if (args is KeyUpEventArgs keyUpEventArgs)
        {
            _keyboardStates[(int)keyUpEventArgs.Scancode] = false;
        }
    }

    public void Update()
    {
        _mouseButtonStates.CopyTo(_previousMouseButtonStates.AsSpan());
        _keyboardStates.CopyTo(_previousKeyboardStates.AsSpan());
        MouseDelta = Vector2.Zero;
        _prevMousePosition = MousePosition;
    }
}
