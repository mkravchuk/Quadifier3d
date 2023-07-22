// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "frustum.h"
template < typename DerivedP>
IGL_INLINE void igl::frustum(
  const typename DerivedP::Scalar left,
  const typename DerivedP::Scalar right,
  const typename DerivedP::Scalar bottom,
  const typename DerivedP::Scalar top,
  const typename DerivedP::Scalar nearVal,
  const typename DerivedP::Scalar farVal,
  Eigen::PlainObjectBase<DerivedP> & P)
{
  P.setConstant(4,4,0.);
  P(0,0) = (2.0 * nearVal) / (right - left);
  P(1,1) = (2.0 * nearVal) / (top - bottom);
  P(0,2) = (right + left) / (right - left);
  P(1,2) = (top + bottom) / (top - bottom);
  P(2,2) = -(farVal + nearVal) / (farVal - nearVal);
  P(3,2) = -1.0;
  P(2,3) = -(2.0 * farVal * nearVal) / (farVal - nearVal);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::frustum<Eigen::Matrix4f >(Eigen::Matrix4f::Scalar, Eigen::Matrix4f::Scalar, Eigen::Matrix4f::Scalar, Eigen::Matrix4f::Scalar, Eigen::Matrix4f::Scalar, Eigen::Matrix4f::Scalar, Eigen::PlainObjectBase<Eigen::Matrix4f >&);
template void igl::frustum<Eigen::Matrix4d >(Eigen::Matrix4d::Scalar, Eigen::Matrix4d::Scalar, Eigen::Matrix4d::Scalar, Eigen::Matrix4d::Scalar, Eigen::Matrix4d::Scalar, Eigen::Matrix4d::Scalar, Eigen::PlainObjectBase<Eigen::Matrix4d >&);
#endif
