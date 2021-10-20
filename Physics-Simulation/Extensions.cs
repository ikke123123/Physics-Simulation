using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics_Simulation.Extended
{
    static class Extensions
    {
        public static bool IsBetween(this float value, float minVal, float maxVal) => value < maxVal && value > minVal;
    }
}
