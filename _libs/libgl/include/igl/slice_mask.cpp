// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "slice_mask.h"
#include <cassert>

template <typename DerivedX>
IGL_INLINE void igl::slice_mask(
  const Eigen::PlainObjectBase<DerivedX> & X,
  const Eigen::Array<bool,Eigen::Dynamic,1> & R,
  const Eigen::Array<bool,Eigen::Dynamic,1> & C,
  Eigen::PlainObjectBase<DerivedX> & Y)
{
  int xm = X.rows();
  int xn = X.cols();
  int ym = R.count();
  int yn = C.count();
  assert(R.size() == X.rows() && "R.size() should match X.rows()");
  assert(C.size() == X.cols() && "C.size() should match X.cols()");
  Y.resize(ym,yn);
  {
    int yi = 0;
    for(int i = 0;i<xm;i++)
    {
      if(R(i))
      {
        int yj = 0;
        for(int j = 0;j<xn;j++)
        {
          if(C(j))
          {
            Y(yi,yj) = X(i,j);
            yj++;
          }
        }
        yi++;
      }
    }
  }
}

template <typename DerivedX>
IGL_INLINE void igl::slice_mask(
  const Eigen::PlainObjectBase<DerivedX> & X,
  const Eigen::Array<bool,Eigen::Dynamic,1> & R,
  const int dim,
  Eigen::PlainObjectBase<DerivedX> & Y)
{
  switch(dim)
  {
    case 1:
    {
      const int ym = R.count();
      assert(X.rows() == R.size() && "X.rows() should match R.size()");
      Y.resize(ym,X.cols());
      {
        int yi = 0;
        for(int i = 0;i<X.rows();i++)
        {
          if(R(i))
          {
            Y.row(yi++) = X.row(i);
          }
        }
      }
      return;
    }
    case 2:
    {
      const auto & C = R;
      const int yn = C.count();
      Y.resize(X.rows(),yn);
      assert(X.cols() == R.size() && "X.cols() should match R.size()");
      {
        int yj = 0;
        for(int j = 0;j<X.cols();j++)
        {
          if(C(j))
          {
            Y.col(yj++) = X.col(j);
          }
        }
      }
      return;
    }
    default:
      assert(false && "Unsupported dimension");
      return;
  }
}

template <typename DerivedX>
IGL_INLINE DerivedX igl::slice_mask(
  const Eigen::PlainObjectBase<DerivedX> & X,
  const Eigen::Array<bool,Eigen::Dynamic,1> & R,
  const Eigen::Array<bool,Eigen::Dynamic,1> & C)
{
  DerivedX Y;
  igl::slice_mask(X,R,C,Y);
  return Y;
}

template <typename DerivedX>
IGL_INLINE DerivedX igl::slice_mask(
  const Eigen::PlainObjectBase<DerivedX>& X,
  const Eigen::Array<bool,Eigen::Dynamic,1> & R,
  const int dim)
{
  DerivedX Y;
  igl::slice_mask(X,R,dim,Y);
  return Y;
}

#ifdef IGL_STATIC_LIBRARY
template void igl::slice_mask<Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, int, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::slice_mask<Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::slice_mask<Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::slice_mask<Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::slice_mask<Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::Array<bool, -1, 1, 0, -1, 1> const&, int, Eigen::PlainObjectBase<Eigen::VectorXi >&);
#endif
