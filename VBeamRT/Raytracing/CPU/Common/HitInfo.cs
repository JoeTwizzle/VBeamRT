using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

struct HitInfo
{
    public float HitDistNear;
    public float HitDistFar;
    public HitInfo()
    {
        HitDistNear = float.PositiveInfinity;
        HitDistNear = float.PositiveInfinity;
    }

    public HitInfo(float hitDistNear, float hitDistFar)
    {
        HitDistNear = hitDistNear;
        HitDistFar = hitDistFar;
    }
}
