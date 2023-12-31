#include "stdafx.h"

__declspec(noinline) void AllocateConsoleWindow()
{
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);

    // move console window to second monitor
    HWND consoleWindow = GetConsoleWindow();
    SetWindowPos(consoleWindow, 0, -1920, 7, 1920, 1200, SWP_NOZORDER); // second monitor is on left side
                                                                        //SetWindowPos(consoleWindow, 0, 1920, 0, 1920, 1200, SWP_NOZORDER);// second monitor is on right side

                                                                        // enable mouse wheel on console
    DWORD lpMode = 0;
    HANDLE hConsole = CreateFile("CONIN$", GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL); //  get console handle
    if (!GetConsoleMode(hConsole, &lpMode)) // get mode of console 
    {
        DWORD err = GetLastError();
    }
    SetConsoleMode(hConsole, lpMode & ~ENABLE_MOUSE_INPUT | ENABLE_PROCESSED_INPUT); // update mode with mouse wheel enabled mode
}

template <class T>
std::string test(const T& a, const T& a_should_be)
{
    return(a == a_should_be) ? "ok" : "fail";
}
template <class T>
__declspec(noinline) void couttest(std::string testName, const T& a, const T& a_should_be)
{
    std::string ok_fail = test(a, a_should_be);
    std::string  text = ok_fail + "  " + testName;
    cout << text << endl;
}

 //*******************************************************************************************
 //*******************************************************************************************
 //*******************************************************************************************


__declspec(noinline) void TestSSESort()
{
    cout << endl << "Testing SSE sort" << endl;

    int vid0 = rand();
    int vid1 = rand();

    //if (vid0 > vid1) std::swap(vid0, vid1);

    //cout << "vid0 = " << vid0 << endl;
    //cout << "vid1 = " << vid1 << endl;

    vid0 = vid0 < vid1 ? vid0 : vid1;
    cout << "vid0 = " << vid0 << endl;

}

int clamp(int value, int minValue, int maxValue)
{
    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;
    return value;
}

int compactOneValue(float value)
{
    assert(abs(value) > 0 && abs(value) < 1);
    int sign = value < 0 ? 1 : 0;
    int valueInt = clamp((int)(500 * abs(value)), 0, 500);
    return (valueInt << 1) + sign;
}

int compact(V3 normal)
{
    //normal.normalize();
    int x = compactOneValue(normal(0));
    int y = compactOneValue(normal(1));
    int z = compactOneValue(normal(2));
    int normalCompacted = (x << 20) + (y << 10) + (z << 0);
    return normalCompacted;
}

float uncompactOne(int value)
{
    bool minus = value & 1;
    value = value >> 1;
    float res = (1.0f * value) / 500;
    if (minus) res = -res;
    return res;
}

V3 uncompact(int normalCompacted)
{
    int x = (normalCompacted >> 20) & 1023;
    int y = (normalCompacted >> 10) & 1023;
    int z = (normalCompacted >> 0) & 1023;
    V3 normalUncompacted = V3(uncompactOne(x), uncompactOne(y), uncompactOne(z));
    return normalUncompacted;
}

void test_compacting_normals()
{
    V3 normal(0.5, 0.7, -0.5);
    int normalCompacted = compact(normal);
    V3 normal2 = uncompact(normalCompacted);
}


int main()
{
    AllocateConsoleWindow();


    cout << "------------------------------------------------" << endl;
    cout << "     TESTING UTILS" << endl;
    cout << "------------------------------------------------" << endl;

    //utils::opencl::testSort();
    utils::sse::_4::testSort();
    
    V3 v(1, 2, 3);
    D v_dot_v = utils::vector::Dot(v, v);
    Vec4f v4(1,2,3,5);
    auto res = _mm_dp_ps(v4, v4, 113).m128_f32[0];//113 = 1 + ((1+2+4) << 4)   -  store result in 1-first float, multiply 1-first float,2-second float,3th float
    cout << v_dot_v << res << endl;
    
    TestSSESort();
    test_compacting_normals();

    cout << "------------------------------------------------" << endl;
    return 0;
}

