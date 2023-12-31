// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "bounding_box.h"
#include <iostream>

template <typename DerivedV, typename DerivedBV, typename DerivedBF>
IGL_INLINE void igl::bounding_box(
  const Eigen::PlainObjectBase<DerivedV>& V,
  Eigen::PlainObjectBase<DerivedBV>& BV,
  Eigen::PlainObjectBase<DerivedBF>& BF)
{
  using namespace std;

  const int dim = V.cols();
  const auto & minV = V.colwise().minCoeff();
  const auto & maxV = V.colwise().maxCoeff();
  // 2^n vertices
  BV.resize((1<<dim),dim);

  // Recursive lambda to generate all 2^n combinations
  const std::function<void(const int,const int,int*,int)> combos =
  [&BV,&minV,&maxV,&combos](
    const int dim,
    const int i,
    int * X,
    const int pre_index)
  {
    for(X[i] = 0;X[i]<2;X[i]++)
    {
      int index = pre_index*2+X[i];
      if((i+1)<dim)
      {
        combos(dim,i+1,X,index);
      }else
      {
        for(int d = 0;d<dim;d++)
        {
          BV(index,d) = (X[d]?minV[d]:maxV[d]);
        }
      }
    }
  };

  Eigen::VectorXi X(dim);
  combos(dim,0,X.data(),0);
  switch(dim)
  {
    case 2:
      BF.resize(4,2);
      BF<<
        3,1,
        1,0,
        0,2,
        2,3;
      break;
    case 3:
      BF.resize(12,3);
      BF<<
        2,0,6,
        0,4,6,
        5,4,0,
        5,0,1,
        6,4,5,
        5,7,6,
        3,0,2,
        1,0,3,
        3,2,6,
        6,7,3,
        5,1,3,
        3,7,5;
      break;
    default:
      assert(false && "Unsupported dimension.");
      break;
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::bounding_box<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&);
template void igl::bounding_box<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#endif
