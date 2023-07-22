// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "sortrows.h"
#include "get_seconds.h"

#include "SortableRow.h"
#include "sort.h"
#include "colon.h"
#include "IndexComparison.h"

#include <vector>

// Obsolete slower version converst to vector
//template <typename DerivedX, typename DerivedIX>
//IGL_INLINE void igl::sortrows(
//  const Eigen::PlainObjectBase<DerivedX>& X,
//  const bool ascending,
//  Eigen::PlainObjectBase<DerivedX>& Y,
//  Eigen::PlainObjectBase<DerivedIX>& IX)
//{
//  using namespace std;
//  using namespace Eigen;
//  typedef Eigen::Matrix<typename DerivedX::Scalar, Eigen::Dynamic, 1> RowVector;
//  vector<SortableRow<RowVector> > rows;
//  rows.resize(X.rows());
//  // Loop over rows
//  for(int i = 0;i<X.rows();i++)
//  {
//    RowVector ri = X.row(i);
//    rows[i] = SortableRow<RowVector>(ri);
//  }
//  vector<SortableRow<RowVector> > sorted;
//  std::vector<size_t> index_map;
//  // Perform sort on rows
//  igl::sort(rows,ascending,sorted,index_map);
//  // Resize output
//  Y.resize(X.rows(),X.cols());
//  IX.resize(X.rows(),1);
//  // Convert to eigen
//  for(int i = 0;i<X.rows();i++)
//  {
//    Y.row(i) = sorted[i].data;
//    IX(i,0) = index_map[i];
//  }
//}

template <typename DerivedX, typename DerivedIX>
IGL_INLINE void igl::sortrows(
  const Eigen::PlainObjectBase<DerivedX>& X,
  const bool ascending,
  Eigen::PlainObjectBase<DerivedX>& Y,
  Eigen::PlainObjectBase<DerivedIX>& IX)
{
  // This is already 2x faster than matlab's builtin `sortrows`. I have tried
  // implementing a "multiple-pass" sort on each column, but see no performance
  // improvement.
  using namespace std;
  using namespace Eigen;
  // Resize output
  const size_t num_rows = X.rows();
  const size_t num_cols = X.cols();
  Y.resize(num_rows,num_cols);
  IX.resize(num_rows,1);
  for(int i = 0;i<num_rows;i++)
  {
    IX(i) = i;
  }
  if (ascending) {
    auto index_less_than = [&X, num_cols](size_t i, size_t j) {
      for (size_t c=0; c<num_cols; c++) {
        if (X.coeff(i, c) < X.coeff(j, c)) return true;
        else if (X.coeff(j,c) < X.coeff(i,c)) return false;
      }
      return false;
    };
      std::sort(
        IX.data(),
        IX.data()+IX.size(),
        index_less_than
        );
  } else {
    auto index_greater_than = [&X, num_cols](size_t i, size_t j) {
      for (size_t c=0; c<num_cols; c++) {
        if (X.coeff(i, c) > X.coeff(j, c)) return true;
        else if (X.coeff(j,c) > X.coeff(i,c)) return false;
      }
      return false;
    };
      std::sort(
        IX.data(),
        IX.data()+IX.size(),
        index_greater_than
        );
  }
  for (size_t j=0; j<num_cols; j++) {
      for(int i = 0;i<num_rows;i++)
      {
          Y(i,j) = X(IX(i), j);
      }
  }
}

#ifdef IGL_STATIC_LIBRARY
template void igl::sortrows<Eigen::MatrixXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::sortrows<Eigen::MatrixXd, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::sortrows<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::sortrows<Eigen::MatrixX3d, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::sortrows<Eigen::MatrixX2d, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX2d > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixX2d >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::sortrows<Eigen::MatrixX2i, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX2i > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::sortrows<Eigen::MatrixXi, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, bool, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#endif
