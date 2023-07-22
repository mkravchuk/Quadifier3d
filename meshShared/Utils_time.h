#pragma once
#include "Timer.h"
#include "Elapsed.h"


namespace utils
{
    namespace time
    {
        //using Clock = std::chrono::steady_clock;
        //using Mc = std::chrono::microseconds;
        //using Ms = std::chrono::milliseconds;
        //using Sec = std::chrono::seconds;
        //using TimePoint = std::chrono::time_point<Clock, Mc>;

        chrono::high_resolution_clock::time_point Now();
        int ElapsedNanoseconds(const chrono::high_resolution_clock::time_point&  startTime);
        int ElapsedMilliseconds(const chrono::high_resolution_clock::time_point&  startTime);
        std::string ElapsedSecondsStr(chrono::high_resolution_clock::time_point&  startTime);
        void coutTimeElapsed(string caption, int timeNanoseconds);

        string TimeToStr(const string& mask = "%d.%m.%Y %H:%M:%S");
    }

}

