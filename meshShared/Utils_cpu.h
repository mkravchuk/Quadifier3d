#pragma once



namespace utils
{
    class cpu
    {
    public:
        static bool isSupportedSSE;
        static bool isSupportedSSE4;
        static bool isSupportedSSE256;
        static bool isSupportedAVX512;
        static int coresCount;
        static void readCPUinfo();
    };
}