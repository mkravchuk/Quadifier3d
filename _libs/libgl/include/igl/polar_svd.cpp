// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "polar_svd.h"
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iostream>

// Adapted from Olga's CGAL mentee's ARAP code
template <
  typename DerivedA,
  typename DerivedR,
  typename DerivedT>
IGL_INLINE void igl::polar_svd(
  const Eigen::PlainObjectBase<DerivedA> & A,
  Eigen::PlainObjectBase<DerivedR> & R,
  Eigen::PlainObjectBase<DerivedT> & T)
{
  DerivedA U;
  DerivedA V;
  Eigen::Matrix<typename DerivedA::Scalar,DerivedA::RowsAtCompileTime,1> S;
  return igl::polar_svd(A,R,T,U,S,V);
}

template <
  typename DerivedA,
  typename DerivedR,
  typename DerivedT,
  typename DerivedU,
  typename DerivedS,
  typename DerivedV>
IGL_INLINE void igl::polar_svd(
  const Eigen::PlainObjectBase<DerivedA> & A,
  Eigen::PlainObjectBase<DerivedR> & R,
  Eigen::PlainObjectBase<DerivedT> & T,
  Eigen::PlainObjectBase<DerivedU> & U,
  Eigen::PlainObjectBase<DerivedS> & S,
  Eigen::PlainObjectBase<DerivedV> & V)
{
  using namespace std;
  Eigen::JacobiSVD<DerivedA> svd;
  svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV );
  U = svd.matrixU();
  V = svd.matrixV();
  S = svd.singularValues();
  R = U*V.transpose();
  const auto & SVT = S.asDiagonal() * V.adjoint();
  // Check for reflection
  if(R.determinant() < 0)
  {
    // Annoyingly the .eval() is necessary
    auto W = V.eval();
    W.col(V.cols()-1) *= -1.;
    R = U*W.transpose();
    T = W*SVT;
  }else
  {
    T = V*SVT;
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::polar_svd<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::VectorXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::polar_svd<Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2> >(Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&);
template void igl::polar_svd<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d >(Eigen::PlainObjectBase<Eigen::Matrix3d > const&, Eigen::PlainObjectBase<Eigen::Matrix3d >&, Eigen::PlainObjectBase<Eigen::Matrix3d >&);
template void igl::polar_svd<Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 2, 0, 2, 2> >(Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&);
template void igl::polar_svd<Eigen::MatrixXd, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2> >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&);
template void igl::polar_svd<Eigen::MatrixXd, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::polar_svd<Eigen::Matrix3f,Eigen::Matrix3f,Eigen::Matrix3f,Eigen::Matrix3f,Eigen::Matrix<float,3,1,0,3,1>,Eigen::Matrix3f >(Eigen::PlainObjectBase<Eigen::Matrix3f > const &,Eigen::PlainObjectBase<Eigen::Matrix3f > &,Eigen::PlainObjectBase<Eigen::Matrix3f > &,Eigen::PlainObjectBase<Eigen::Matrix3f > &,Eigen::PlainObjectBase<Eigen::Matrix<float,3,1,0,3,1> > &,Eigen::PlainObjectBase<Eigen::Matrix3f >&);
template void igl::polar_svd<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix3d >(Eigen::PlainObjectBase<Eigen::Matrix3d > const&, Eigen::PlainObjectBase<Eigen::Matrix3d >&, Eigen::PlainObjectBase<Eigen::Matrix3d >&, Eigen::PlainObjectBase<Eigen::Matrix3d >&, Eigen::PlainObjectBase<Eigen::Vector3d >&, Eigen::PlainObjectBase<Eigen::Matrix3d >&);
template void igl::polar_svd<Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 2, 0, 2, 2>, Eigen::Matrix<float, 2, 1, 0, 2, 1>, Eigen::Matrix<float, 2, 2, 0, 2, 2> >(Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 1, 0, 2, 1> >&, Eigen::PlainObjectBase<Eigen::Matrix<float, 2, 2, 0, 2, 2> >&);
template void igl::polar_svd<Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 2, 0, 2, 2>, Eigen::Matrix<double, 2, 1, 0, 2, 1>, Eigen::Matrix<double, 2, 2, 0, 2, 2> >(Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 1, 0, 2, 1> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, 2, 2, 0, 2, 2> >&);
template void igl::polar_svd<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::polar_svd<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
#endif
