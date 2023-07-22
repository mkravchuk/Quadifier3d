// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "../CommonTypes/disabled_warnings.h"
#include "../CommonTypes/common_cpp_standard_headers.h"
#include "../CommonTypes/common_types.h"
using namespace std;


#define BOOST_COMPUTE_DEBUG_KERNEL_COMPILATION
#define BOOST_COMPUTE_THREAD_SAFE
#define BOOST_COMPUTE_USE_OFFLINE_CACHE
#include <boost/compute.hpp>
#include "vectori128.h"
#include "vectorf128.h"