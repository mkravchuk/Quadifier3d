#include "stdafx.h"
#include <ctime>

namespace utils
{
    namespace time
    {
        chrono::high_resolution_clock::time_point Now()
        {
            //COMISO::StopWatch sw; sw.start();
            //clock_t start = clock();
            return  chrono::high_resolution_clock::now();
        }
        int ElapsedNanoseconds(const chrono::high_resolution_clock::time_point&  startTime)
        {
            chrono::high_resolution_clock::time_point now = Now();
            std::chrono::high_resolution_clock::duration d = now - startTime;
            return chrono::duration_cast<chrono::nanoseconds>(d).count();
        }
        int ElapsedMilliseconds(const chrono::high_resolution_clock::time_point&  startTime)
        {
            chrono::high_resolution_clock::time_point now = Now();
            std::chrono::high_resolution_clock::duration d = now - startTime;
            return chrono::duration_cast<chrono::milliseconds>(d).count();
        }
        std::string ElapsedSecondsStr(chrono::high_resolution_clock::time_point&  startTime)
        {
            auto res = to_string(ElapsedMilliseconds(startTime) / 1000.0);
            startTime = Now(); // restart timer to simplify using this method
            return res;
        }
        void coutTimeElapsed(string caption, int timeNanoseconds)
        {
            if (timeNanoseconds < 10)  return;
            cout << "time taken  " << to_string((1.0*timeNanoseconds) / (1000 * 1000 * 1000.0)) << "  " << caption << std::endl;
        }

        string TimeToStr(const string& mask)
        {
            time_t rawtime;
            std::time(&rawtime);

            struct tm timeinfo;
            localtime_s(&timeinfo, &rawtime);

            char buffer[200];
            strftime(buffer, sizeof(buffer), "%d.%m.%Y %H:%M:%S", &timeinfo);
            std::string strDateAndTime(buffer);
            return  strDateAndTime;
        }
    }

}

