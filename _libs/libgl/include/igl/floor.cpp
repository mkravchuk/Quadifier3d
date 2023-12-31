// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "floor.h"
#include <cmath>
#include <iostream>

template < typename DerivedX, typename DerivedY>
IGL_INLINE void igl::floor(
  const Eigen::PlainObjectBase<DerivedX>& X,
  Eigen::PlainObjectBase<DerivedY>& Y)
{
  using namespace std;
  //Y = DerivedY::Zero(m,n);
//#pragma omp parallel for
  //for(int i = 0;i<m;i++)
  //{
  //  for(int j = 0;j<n;j++)
  //  {
  //    Y(i,j) = std::floor(X(i,j));
  //  }
  //}
  typedef typename DerivedX::Scalar Scalar;
  Y = X.unaryExpr([](const Scalar &x)->Scalar{return std::floor(x);}).template cast<typename DerivedY::Scalar >();
}

#ifdef IGL_STATIC_LIBRARY
// Explicit instanciation
template void igl::floor<Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::floor<Eigen::Array<double, -1, 1, 0, -1, 1>, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Array<double, -1, 1, 0, -1, 1> > const&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::floor<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#endif
