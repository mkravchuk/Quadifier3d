#pragma once

class TimeElapsed;

class Timer
{
private:
    //chrono::high_resolution_clock::time_point start;
    chrono::system_clock::time_point startTime;
    chrono::system_clock::duration elapsed;
public:
    Timer();// automaticaly starts timer when created

    chrono::system_clock::duration elapsedDuration();
    long long milliseconds() const;
    long long seconds() const;

    void start(); // start counting time
    void stop();// stop counting time and save elapsed time in duration
    void stop(TimeElapsed& timeElapsed);// stop counting time and save elapsed time in duration
    void stop(TimeElapsed& timeElapsed, TimeElapsed& timeElapsedMiror);// stop counting time and save elapsed time in duration
    void clear();    //  zero 'elapsed' counter
    void restart(); // clear() and start()

    string toString() const; // return string represantation of counted time
    string ElapsedSecondsStr() const;
    friend ostream& operator<<(ostream& os, const Timer& t);
};




