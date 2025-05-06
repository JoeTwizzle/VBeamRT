using OpenTK.Graphics;
using OpenTK.Platform;
using OpenTK.Platform.Native;
using VBeamRT.Raytracing;
using VBeamRT.Raytracing.CPU.Whitted;
using VBeamRT.Raytracing.CPU.Whitted.BlockerCache;
using VBeamRT.Raytracing.CPU.Whitted.Simple;

namespace VBeamRT;
internal sealed class Game : IDisposable
{
    private readonly Metrics _metrics;
    private readonly GameLoop _gameLoop;
    private readonly WindowHandler _windowHandler;

    private OpenWindowHandle _mainWindowHandle;
    private WindowInfo _mainWindowInfo;
    private IRenderer _renderer;

    public float DeltaTime => _metrics.DeltaTime;
    public double DeltaTimeFull => _metrics.DeltaTimeFull;
    public Input Input => _mainWindowInfo.Input;
    public WindowInfo MainWindowInfo => _mainWindowInfo;
    public WindowHandler WindowHandler => _windowHandler;

    public Game()
    {
        Toolkit.Init(new ToolkitOptions() { ApplicationName = nameof(VBeamRT) });
        VKLoader.Init();
        _metrics = new Metrics();
        _gameLoop = new GameLoop();
        _windowHandler = new WindowHandler();
        _mainWindowHandle = _windowHandler.Open();
        _mainWindowInfo = _windowHandler.GetInfo(_mainWindowHandle);
        _mainWindowInfo.EventQueue.EventDispatched += MainWindowEventHandler;
        _renderer = new BlockerRaytracer(this);
        Initialize();
        _gameLoop.ShouldRun = true;
        _gameLoop.Run(_metrics, Update);
        Destroy();
    }

    private void MainWindowEventHandler(PalHandle? handle, PlatformEventType type, EventArgs args)
    {
        if (args is CloseEventArgs)
        {
            _gameLoop.ShouldRun = false;
        }
    }

    void Initialize()
    {
        _renderer.Initalize();
    }

    void Update()
    {
        _windowHandler.Update();
        Toolkit.Window.ProcessEvents(false);
        _mainWindowInfo.EventQueue.DispatchEvents();
        _renderer.Update();
    }

    void Destroy()
    {
        _renderer.Dispose();
        _windowHandler.Close(_mainWindowHandle);
    }

    public void Dispose()
    {
        _gameLoop.ShouldRun = false;
    }
}
