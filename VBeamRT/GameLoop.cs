using OpenTK.Platform;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT;
internal sealed class GameLoop
{
    public bool ShouldRun;

    public void Run(Metrics metrics, Action updateCallback)
    {
        long prev = Stopwatch.GetTimestamp();
        while (ShouldRun)
        {
            long current = Stopwatch.GetTimestamp();
            metrics.DeltaTimeFull = Stopwatch.GetElapsedTime(prev, current).TotalSeconds; //Ticks to seconds constant
            metrics.DeltaTime = (float)metrics.DeltaTimeFull;
            metrics.TimingBuffer.Add(metrics.DeltaTimeFull);
            prev = current;
            updateCallback();
        }
    }
}
