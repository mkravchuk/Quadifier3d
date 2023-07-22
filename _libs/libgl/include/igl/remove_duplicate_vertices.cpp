// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "remove_duplicate_vertices.h"
#include "round.h"
#include "unique.h"
#include "colon.h"
#include "slice.h"
#include <functional>

template <
  typename DerivedV, 
  typename DerivedSV, 
  typename DerivedSVI, 
  typename DerivedSVJ>
IGL_INLINE void igl::remove_duplicate_vertices(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const double epsilon,
  Eigen::PlainObjectBase<DerivedSV>& SV,
  Eigen::PlainObjectBase<DerivedSVI>& SVI,
  Eigen::PlainObjectBase<DerivedSVJ>& SVJ)
{
  if(epsilon > 0)
  {
    DerivedV rV,rSV;
    round((V/(10.0*epsilon)).eval(),rV);
    unique_rows(rV,rSV,SVI,SVJ);
    slice(V,SVI,colon<int>(0,V.cols()-1),SV);
  }else
  {
    unique_rows(V,SV,SVI,SVJ);
  }
}

template <
  typename DerivedV, 
  typename DerivedF,
  typename DerivedSV, 
  typename DerivedSVI, 
  typename DerivedSVJ,
  typename DerivedSF>
IGL_INLINE void igl::remove_duplicate_vertices(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  const double epsilon,
  Eigen::PlainObjectBase<DerivedSV>& SV,
  Eigen::PlainObjectBase<DerivedSVI>& SVI,
  Eigen::PlainObjectBase<DerivedSVJ>& SVJ,
  Eigen::PlainObjectBase<DerivedSF>& SF)
{
  using namespace Eigen;
  using namespace std;
  remove_duplicate_vertices(V,epsilon,SV,SVI,SVJ);
  SF.resize(F.rows(),F.cols());
  for(int f = 0;f<F.rows();f++)
  {
    for(int c = 0;c<F.cols();c++)
    {
      SF(f,c) = SVJ(F(f,c));
    }
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit instanciation
template void igl::remove_duplicate_vertices<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, double, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::remove_duplicate_vertices<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d, Eigen::VectorXi, Eigen::VectorXi, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, double, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&);
template void igl::remove_duplicate_vertices<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, double, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#endif
