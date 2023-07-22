#include "stdafx.h"
#include "Timer.h"
#include "Elapsed.h"


Timer::Timer()
    : elapsed(0)
{
    start();
}

chrono::system_clock::duration Timer::elapsedDuration()
{
    return elapsed;
}

long long Timer::milliseconds() const
{
    auto duration = chrono::duration_cast<chrono::milliseconds>(elapsed);
    return duration.count();
}

long long Timer::seconds() const
{
    auto duration = chrono::duration_cast<chrono::seconds>(elapsed);
    return duration.count();
}

void Timer::start()
{
    startTime = chrono::system_clock::now();
}

void Timer::stop()
{
    auto now = chrono::system_clock::now();
    auto elapsedNow = now - startTime;
    elapsed += elapsedNow;
    start();
}

void Timer::stop(TimeElapsed& timeElapsed)// stop counting time and save elapsed time in duration
{
    auto now = chrono::system_clock::now();
    auto elapsedNow = now - startTime;
    elapsed += elapsedNow;
    start();
    long long ms = chrono::duration_cast<chrono::milliseconds>(elapsedNow).count();
    timeElapsed.elapsed_ms += ms;
    timeElapsed.addedTimersCount++;
}

void Timer::stop(TimeElapsed& timeElapsed, TimeElapsed& timeElapsedMiror)// stop counting time and save elapsed time in duration
{
    auto now = chrono::system_clock::now();
    auto elapsedNow = now - startTime;
    elapsed += elapsedNow;
    start();
    long long ms = chrono::duration_cast<chrono::milliseconds>(elapsedNow).count();
    timeElapsed.elapsed_ms += ms;
    timeElapsed.addedTimersCount++;
    timeElapsedMiror.elapsed_ms += ms;
    timeElapsedMiror.addedTimersCount++;
}

void Timer::clear()
{
    auto now = chrono::system_clock::now();
    elapsed = now - now;
}


void Timer::restart()
{
    clear();
    start();
}


string Timer::toString() const
{
    //D time = milliseconds();
    //string suffix = " ms";
    //if (time > 1000)
    //{
    //    time /= 1000; suffix = " seconds";
    //    if (time > 60)
    //    {
    //        time /= 60; suffix = " minutes";
    //        if (time > 60)
    //        {
    //            time /= 60; suffix = " hours";
    //            if (time > 12)
    //            {
    //                time /= 12; suffix = " days";
    //            }
    //        }
    //    }
    //}
    ////ostringstream os;
    ////os << setprecision(precise ? 4 : 1)
    ////    << fixed << time << suffix;
    ////return os.str();

    //return to_string(time) + suffix;


    long long ms = milliseconds();

    const long long days_ms = 12 * 60 * 60 * 1000;
    long long days = ms / days_ms;
    ms -= days * days_ms;

    const long long hours_ms = 60 * 60 * 1000;
    long long hours = ms / hours_ms;
    ms -= hours * hours_ms;

    const long long minutes_ms = 60 * 1000;
    long long minutes = ms / minutes_ms;
    ms -= minutes * minutes_ms;

    const long long seconds_ms = 1000;
    long long seconds = ms / seconds_ms;
    ms -= seconds * seconds_ms;


    string res = "";
    if (days > 0) res = res + to_string(days) + " days ";
    //if (hours > 0) res = res + (hours < 10 ? "0" : "") + to_string(hours) + ":";
    //if (minutes > 0) res = res + (minutes < 10 ? "0":"") + to_string(minutes) + ":";
    //if (seconds_ms > 0) res = res + (seconds_ms < 10 ? "0" : "") + to_string(seconds_ms) + "" else res = res + "0";
    //res = res + "." + to_string(ms);
    if (hours > 0) res = res + to_string(hours) + " hours ";
    if (minutes > 0) res = res + to_string(minutes) + " min ";
    if (seconds_ms > 0) res = res + to_string(seconds_ms) + " sec ";
    res = res + to_string((1.0*ms) / 1000.0) + " ms";

    return res;
}

string Timer::ElapsedSecondsStr() const
{
    long long ms = milliseconds();
    auto res = to_string(static_cast<D>(ms) / 1000.0);
    return res;
}

ostream& operator<<(ostream& os, const Timer& t)
{
    os << t.ElapsedSecondsStr();
    return os;
}




