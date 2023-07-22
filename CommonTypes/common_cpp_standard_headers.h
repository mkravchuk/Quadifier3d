#undef ASSERT

// disable vector range checking and increase speed in debug mode
#ifdef _DEBUG
#else
// disable 'iterator debugging' for std library
#define _HAS_ITERATOR_DEBUGGING 0
#endif
//#define _ITERATOR_DEBUG_LEVEL 2

// C RunTime Header Files    
//For /Wall
#pragma warning(push)
#pragma warning(disable : 4820)
#pragma warning(disable : 4619)
#pragma warning(disable : 4548)
#pragma warning(disable : 4668)
#pragma warning(disable : 4365)
#pragma warning(disable : 4710)
#pragma warning(disable : 4371)
#pragma warning(disable : 4826)
#pragma warning(disable : 4061)
#pragma warning(disable : 4640)
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <list>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <xlocale>
#include <limits>
#include <exception>
#include <numeric>
#include <fstream>
#include <future>
#include <chrono>
#include <atomic>
#include <complex>
#include <cassert>
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1
#pragma warning(pop) //For /Wall