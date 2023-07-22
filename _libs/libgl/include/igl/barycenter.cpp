// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "barycenter.h"





template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedBC>
IGL_INLINE void igl::barycenter(
    const Eigen::PlainObjectBase<DerivedV> & V,
    const Eigen::PlainObjectBase<DerivedF> & F,
    Eigen::PlainObjectBase<DerivedBC> & BC)
{
  BC.setZero(F.rows(),V.cols());
  // Loop over faces
  for(int i = 0;i<F.rows();i++)
  {
    // loop around face
    for(int j = 0;j<F.cols();j++)
    {
      // Accumulate
      BC.row(i) += V.row(F(i,j));
    }
    // average
    BC.row(i) /= double(F.cols());
  }
}



#ifdef IGL_STATIC_LIBRARY
// Explicit instanciation
template void igl::barycenter<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX4d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX4d >&);
template void igl::barycenter<Eigen::MatrixX4d, Eigen::MatrixXi, Eigen::MatrixX4d >(Eigen::PlainObjectBase<Eigen::MatrixX4d > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX4d >&);
template void igl::barycenter<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::barycenter<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::barycenter<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::barycenter<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::barycenter<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX2d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX2d >&);
#endif
