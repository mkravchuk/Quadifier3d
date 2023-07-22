// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "round.h"
#include <cmath>


// http://stackoverflow.com/a/485549
template <typename DerivedX >
IGL_INLINE DerivedX igl::round(const DerivedX r)
{
  return (r > 0.0) ? std::floor(r + 0.5) : std::ceil(r - 0.5);
}

template < typename DerivedX, typename DerivedY>
IGL_INLINE void igl::round(
  const Eigen::PlainObjectBase<DerivedX>& X,
  Eigen::PlainObjectBase<DerivedY>& Y)
{
  Y.resize(X.rows(),X.cols());
  // loop over rows
  for(int i = 0;i<X.rows();i++)
  {
    // loop over cols
    for(int j = 0;j<X.cols();j++)
    {
      Y(i,j) = igl::round(X(i,j));
    }
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit instanciation
template void igl::round<Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::round<Eigen::MatrixX2d, Eigen::MatrixX2d >(Eigen::PlainObjectBase<Eigen::MatrixX2d > const&, Eigen::PlainObjectBase<Eigen::MatrixX2d >&);
template void igl::round<Eigen::MatrixX3d, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
#endif
