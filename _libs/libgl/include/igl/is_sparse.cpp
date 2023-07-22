// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "is_sparse.h"
template <typename T>
IGL_INLINE bool igl::is_sparse(
  const Eigen::SparseMatrix<T> & A)
{
  return true;
}
template <typename DerivedA>
IGL_INLINE bool igl::is_sparse(
  const Eigen::PlainObjectBase<DerivedA>& A)
{
  return false;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template bool igl::is_sparse<double>(Eigen::SparseMatrix<double, 0, int> const&);
// generated by autoexplicit.sh
template bool igl::is_sparse<Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&);
#endif
