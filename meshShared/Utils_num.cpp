#include "stdafx.h"
#include "Utils_num.h"

namespace utils
{
    namespace num
    {
        float InvSqrt(float x)
        {
            //Origin of Quake3's Fast InvSqrt https://www.beyond3d.com/content/articles/8/
            float xhalf = 0.5f*x;
            int i = *(int*)&x;
            i = 0x5f3759df - (i >> 1);
            x = *(float*)&i;
            x = x * (1.5f - xhalf * x*x);
            return x;
        }

        D  sqrtFast(D a)
        {
            //http://assemblyrequired.crashworks.org/timing-square-root/
            //https://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
            return std::sqrt(a);
            //return sqrt13(a);
        }

        D round(D d)
        {
            return floor(d + 0.5);
        }
    }

}

