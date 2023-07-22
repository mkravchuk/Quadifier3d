// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

//#include "targetver.h"

//#ifdef WIN32
//#define _USE_MATH_DEFINES
//#endif

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#include <windows.h>
#undef min
#undef max
#undef DrawText

#include "../CommonTypes/disabled_warnings.h"
#include "../CommonTypes/common_cpp_standard_headers.h"
#include "../CommonTypes/common_types.h"
#include <Eigen/Geometry>  // Quaterniond
using namespace std;

#include "_MeshLogicOptions.h"
#include "ViewerDrawObjects.h"
#include "Utils.h"



#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#  undef DrawText
#endif

#ifndef __APPLE__
#  define GLEW_STATIC
#  include <GL/glew.h>
#  include <GL/gl.h>
#endif

#ifdef __APPLE__
#  include <OpenGL/gl3.h>
#  define __gl_h_ /* Prevent inclusion of the old gl.h */
#endif

#undef DrawText
