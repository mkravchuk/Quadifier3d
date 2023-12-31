// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "squared_edge_lengths.h"
#include "parallel_for.h"
#include <iostream>
  
template <typename DerivedV, typename DerivedF, typename DerivedL>
IGL_INLINE void igl::squared_edge_lengths(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  Eigen::PlainObjectBase<DerivedL>& L)
{
  using namespace std;
  const int m = F.rows();
  switch(F.cols())
  {
    case 2:
    {
      L.resize(F.rows(),1);
      for(int i = 0;i<F.rows();i++)
      {
        L(i,0) = (V.row(F(i,1))-V.row(F(i,0))).squaredNorm();
      }
      break;
    }
    case 3:
    {
      L.resize(m,3);
      // loop over faces
      parallel_for(
        m,
        [&V,&F,&L](const int i)
        {
          L(i,0) = (V.row(F(i,1))-V.row(F(i,2))).squaredNorm();
          L(i,1) = (V.row(F(i,2))-V.row(F(i,0))).squaredNorm();
          L(i,2) = (V.row(F(i,0))-V.row(F(i,1))).squaredNorm();
        },
        1000);
      break;
    }
    case 4:
    {
      L.resize(m,6);
      // loop over faces
      parallel_for(
        m,
        [&V,&F,&L](const int i)
        {
          L(i,0) = (V.row(F(i,3))-V.row(F(i,0))).squaredNorm();
          L(i,1) = (V.row(F(i,3))-V.row(F(i,1))).squaredNorm();
          L(i,2) = (V.row(F(i,3))-V.row(F(i,2))).squaredNorm();
          L(i,3) = (V.row(F(i,1))-V.row(F(i,2))).squaredNorm();
          L(i,4) = (V.row(F(i,2))-V.row(F(i,0))).squaredNorm();
          L(i,5) = (V.row(F(i,0))-V.row(F(i,1))).squaredNorm();
        },
        1000);
      break;
    }
    default:
    {
      cerr<< "squared_edge_lengths.h: Error: Simplex size ("<<F.cols()<<
        ") not supported"<<endl;
      assert(false);
    }
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3>, Eigen::MatrixX3f >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3f >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixX2f, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::MatrixX2f >(Eigen::PlainObjectBase<Eigen::MatrixX2f > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::MatrixX2f >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixX2d, Eigen::MatrixXi, Eigen::MatrixX2d >(Eigen::PlainObjectBase<Eigen::MatrixX2d > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX2d >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixX2d, Eigen::MatrixX3i, Eigen::MatrixX2d >(Eigen::PlainObjectBase<Eigen::MatrixX2d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX2d >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::Matrix<double, -1, 6, EROW, -1, 6> >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 6, EROW, -1, 6> >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
// generated by autoexplicit.sh
template void igl::squared_edge_lengths<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::Matrix<float, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> >&);
template void igl::squared_edge_lengths<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::squared_edge_lengths<Eigen::MatrixX3d, Eigen::MatrixX4i, Eigen::Matrix<double, -1, 6, EROW, -1, 6> >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX4i > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 6, EROW, -1, 6> >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::squared_edge_lengths<Eigen::MatrixX3d, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::MatrixXi, Eigen::Matrix<double, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::Matrix<double, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&);
template void igl::squared_edge_lengths<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::MatrixXi, Eigen::Matrix<float, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> >&);
template void igl::squared_edge_lengths<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::Matrix<double, -1, 6, EROW, -1, 6> >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 6, EROW, -1, 6> >&);
template void igl::squared_edge_lengths<Eigen::MatrixX3d, Eigen::MatrixX4i, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX4i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::squared_edge_lengths<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::MatrixXi, Eigen::MatrixX3f >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3f >&);
template void igl::squared_edge_lengths<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::MatrixX3f >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3f >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::squared_edge_lengths<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, -1, 3, 1, -1, 3>, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
#endif
