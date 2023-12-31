// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "view_axis.h"
#include "../opengl/OpenGL_convenience.h"

IGL_INLINE void igl::opengl2::view_axis(double * x, double * y, double * z)
{
  double mv[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, mv);
  igl::opengl2::view_axis(mv,x,y,z);
}

IGL_INLINE void igl::opengl2::view_axis(const double * mv, double * x, double * y, double * z)
{
  *x = -mv[0*4+2];
  *y = -mv[1*4+2];
  *z = -mv[2*4+2];
}

template <typename DerivedV>
IGL_INLINE void igl::opengl2::view_axis(Eigen::PlainObjectBase<DerivedV> & V)
{
  double x,y,z;
  view_axis(&x,&y,&z);
  V(0) = x;
  V(1) = y;
  V(2) = z;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::opengl2::view_axis<Eigen::Vector3d >(Eigen::PlainObjectBase<Eigen::Vector3d >&);
template void igl::opengl2::view_axis<Eigen::Matrix<double, 1, 3, 1, 1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<double, 1, 3, 1, 1, 3> >&);
#endif
