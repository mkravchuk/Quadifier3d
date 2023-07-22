// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#undef ASSERT
#include "../CommonTypes/disabled_warnings.h"

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
#if DEBUG
#include <cassert>
#endif
//#include <stdio.h>
#include <iostream>
#include <string> 
#include <type_traits>
#include <vector> // required CompactVectorVector
//#include <iostream>
//#include <algorithm>
//#include <set>
//#include <map>
//#include <list>
//#include <stdlib.h>
//#include <malloc.h>
//#include <memory.h>
//#include <tchar.h>
//#include <xlocale>
//#include <limits>
//#include <exception>
//#include <numeric>
//#include <fstream>
#include <future>
//#include <chrono>
//#include <atomic>
#include <complex>
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1
#pragma warning(pop) //For /Wall
 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

