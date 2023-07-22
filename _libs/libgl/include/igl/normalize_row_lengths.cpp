// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "normalize_row_lengths.h"

template <typename DerivedV>
IGL_INLINE void igl::normalize_row_lengths(
  const Eigen::PlainObjectBase<DerivedV>& A,
  Eigen::PlainObjectBase<DerivedV> & B)
{
  // Resize output
  B.resize(A.rows(),A.cols());

  // loop over rows
  for(int i = 0; i < A.rows();i++)
  {
    B.row(i) = A.row(i).normalized();
  }
  //// Or just:
  //B = A;
  //B.rowwise().normalize();
}
#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::normalize_row_lengths<Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::normalize_row_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&);
template void igl::normalize_row_lengths<Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
#endif
