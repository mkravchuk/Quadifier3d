// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2016 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "unique_simplices.h"
#include "sort.h"
#include "unique.h"
#include "parallel_for.h"

template <
  typename DerivedF,
  typename DerivedFF,
  typename DerivedIA,
  typename DerivedIC>
IGL_INLINE void igl::unique_simplices(
  const Eigen::PlainObjectBase<DerivedF>& F,
  Eigen::PlainObjectBase<DerivedFF>& FF,
  Eigen::PlainObjectBase<DerivedIA>& IA,
  Eigen::PlainObjectBase<DerivedIC>& IC)
{
  using namespace Eigen;
  using namespace std;
  // Sort each face
  MatrixXi sortF, unusedI;
  igl::sort(F,2,true,sortF,unusedI);
  // Find unique faces
  MatrixXi C;
  igl::unique_rows(sortF,C,IA,IC);
  FF.resize(IA.size(),F.cols());
  const size_t mff = FF.rows();
  parallel_for(mff,[&F,&IA,&FF](size_t & i){FF.row(i) = F.row(IA(i));},1000ul);
}

template <
  typename DerivedF,
  typename DerivedFF>
IGL_INLINE void igl::unique_simplices(
  const Eigen::PlainObjectBase<DerivedF>& F,
  Eigen::PlainObjectBase<DerivedFF>& FF)
{
  Eigen::VectorXi IA,IC;
  return unique_simplices(F,FF,IA,IC);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specializations
template void igl::unique_simplices<Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::unique_simplices<Eigen::MatrixXi, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&);
template void igl::unique_simplices<Eigen::MatrixX2i, Eigen::MatrixX2i, Eigen::VectorXl, Eigen::VectorXl >(Eigen::PlainObjectBase<Eigen::MatrixX2i > const&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXl >&);
template void igl::unique_simplices<Eigen::MatrixXi, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::unique_simplices<Eigen::MatrixX3i, Eigen::MatrixX3i, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::unique_simplices<Eigen::MatrixX2d, Eigen::MatrixX2d, Eigen::VectorXl, Eigen::VectorXl >(Eigen::PlainObjectBase<Eigen::MatrixX2d > const&, Eigen::PlainObjectBase<Eigen::MatrixX2d >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXl >&);
template void igl::unique_simplices<Eigen::MatrixXi, Eigen::MatrixX2i, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::unique_simplices<Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#ifdef WIN32
template void __cdecl igl::unique_simplices<class Eigen::Matrix<int, -1, 2, EROW, -1, 2>, class Eigen::Matrix<int, -1, 2, EROW, -1, 2>, class Eigen::Matrix<EDITYPE, -1, 1, 0, -1, 1>, class Eigen::Matrix<EDITYPE, -1, 1, 0, -1, 1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, 2, EROW, -1, 2> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, 2, EROW, -1, 2> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<EDITYPE, -1, 1, 0, -1, 1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<EDITYPE, -1, 1, 0, -1, 1> > &);
#endif


#endif
