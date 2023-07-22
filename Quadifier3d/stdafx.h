// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#define NOMINMAX                   // disable min,max macroses to give opportunite override min,max for different types

#include "targetver.h"
#define _AFXDLL
#include "afxwin.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#include <windows.h>
#undef min
#undef max
#undef DrawText

#include "../CommonTypes/disabled_warnings.h"
#include "../CommonTypes/common_cpp_standard_headers.h"
#include "../CommonTypes/common_types.h"
using namespace std;

#include "_MeshLogicOptions.h"
#include "ViewerDrawObjects.h"
#include <xmmintrin.h> //  for SSE
#include <emmintrin.h> //  for SSE
#include <smmintrin.h> //  for SSE 4.1
#include "Utils.h"

#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
//#include <igl/readOFF.h>
#include <igl/readOBJ.h>
//#include "../iglcomiso/nrosy.h"
#include <iomanip>


#include "../iglviewer/OpenGL_Window.h"
#include <nanogui/nanogui.h> // must be included after "..\iglviewer\OpenGL_Window.h"
#include "NanoguiThemes.h"


__declspec(noinline) void _stdafx_chache_addvariables()
{
    OpenGL_Window window;
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        cout << "changed";
    };

    int option_int = 0;
    ngui->addVariable("int", option_int, onOptionChanged);

    string option_string = "";
    ngui->addVariable("string", option_string, onOptionChanged);
}