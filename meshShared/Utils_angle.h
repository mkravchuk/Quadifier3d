#pragma once


namespace utils
{
    namespace angle
    {
        const D _180 = 180;
        const D PI = static_cast<D>(M_PI);
        const D _180_div_Pi = (_180 / PI);
        const D _Pi_div_180 = (PI / _180);
        __forceinline D RadiansToDegrees(D radians)
        {
            return  radians * _180_div_Pi;
        }
        __forceinline D DegreesToRadians(D degrees)
        {
            return  degrees * _Pi_div_180;
        }
        __forceinline D DegreesToCos(D degrees)
        {
            return  cos(DegreesToRadians(degrees));
        }
        __forceinline string ToString(D degrees)
        {
            return  to_string(static_cast<int>(round(degrees))) + "`";// "°";
        }
    }

}

