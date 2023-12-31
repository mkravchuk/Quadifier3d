
#pragma once

#define __SSE4_2__
#define NOMINMAX                         // disable min,max macroses to give opportunite override min,max for different types
// disable vector range checking and increase speed in debug mode
//#define _HAS_ITERATOR_DEBUGGING 0
//#define _ITERATOR_DEBUG_LEVEL 2
//#define EIGEN_DEFAULT_TO_ROW_MAJOR 1
#undef ASSERT
#include "targetver.h"
#define _AFXDLL
#include "afxwin.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#include <windows.h>


#include "../CommonTypes/disabled_warnings.h"
#include "../CommonTypes/common_cpp_standard_headers.h"
#include "../CommonTypes/common_types.h"
using namespace std;


#include "Utils.h"
#include <vectorf128.h>



