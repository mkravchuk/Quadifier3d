// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath> 
#include "random_quaternion.h"

template <typename Scalar>
IGL_INLINE Eigen::Quaternion<Scalar> igl::random_quaternion()
{
  const auto & unit_rand = []()->Scalar
  {
    return ((Scalar)rand() / (Scalar)RAND_MAX);
  };

  // Shoemake method 2
  const Scalar x0 = unit_rand();
  const Scalar x1 = unit_rand();
  const Scalar x2 = unit_rand();
  const Scalar r1 = sqrt(1.0 - x0);
  const Scalar r2 = sqrt(x0);
  const Scalar t1 = 2.*M_PI*x1;
  const Scalar t2 = 2.*M_PI*x2;
  const Scalar c1 = cos(t1);
  const Scalar s1 = sin(t1);
  const Scalar c2 = cos(t2);
  const Scalar s2 = sin(t2);
  return Eigen::Quaternion<Scalar>(
    s1*r1,
    c1*r1,
    s2*r2,
    c2*r2);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template Eigen::Quaternion<double, 0> igl::random_quaternion<double>();
#endif
