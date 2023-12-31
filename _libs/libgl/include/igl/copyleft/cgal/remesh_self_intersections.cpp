// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "remesh_self_intersections.h"
#include "SelfIntersectMesh.h"
#include "../../C_STR.h"
#include <list>
#include <iostream>

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedVV,
  typename DerivedFF,
  typename DerivedIF,
  typename DerivedJ,
  typename DerivedIM>
IGL_INLINE void igl::copyleft::cgal::remesh_self_intersections(
  const Eigen::PlainObjectBase<DerivedV> & V,
  const Eigen::PlainObjectBase<DerivedF> & F,
  const RemeshSelfIntersectionsParam & params,
  Eigen::PlainObjectBase<DerivedVV> & VV,
  Eigen::PlainObjectBase<DerivedFF> & FF,
  Eigen::PlainObjectBase<DerivedIF> & IF,
  Eigen::PlainObjectBase<DerivedJ> & J,
  Eigen::PlainObjectBase<DerivedIM> & IM)
{
  using namespace std;
  if(params.detect_only)
  {
    //// This is probably a terrible idea, but CGAL is throwing floating point
    //// exceptions.

//#ifdef __APPLE__
//#define IGL_THROW_FPE 11
//    const auto & throw_fpe = [](int e)
//    {
//      throw "IGL_THROW_FPE";
//    };
//    signal(SIGFPE,throw_fpe);
//#endif

    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef
      SelfIntersectMesh<
        Kernel,
        DerivedV,
        DerivedF,
        DerivedVV,
        DerivedFF,
        DerivedIF,
        DerivedJ,
        DerivedIM>
      SelfIntersectMeshK;
    SelfIntersectMeshK SIM(V,F,params,VV,FF,IF,J,IM);

//#ifdef __APPLE__
//    signal(SIGFPE,SIG_DFL);
//#endif

  }else
  {
    typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
    typedef
      SelfIntersectMesh<
        Kernel,
        DerivedV,
        DerivedF,
        DerivedVV,
        DerivedFF,
        DerivedIF,
        DerivedJ,
        DerivedIM>
      SelfIntersectMeshK;
    SelfIntersectMeshK SIM(V,F,params,VV,FF,IF,J,IM);
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixX3i, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixX3i, Eigen::MatrixX2i, Eigen::VectorXl, Eigen::VectorXl >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&, Eigen::PlainObjectBase<Eigen::MatrixX2i >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXl >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3>, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, 8, 3, 0, 8, 3>, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, 8, 3, 0, 8, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::MatrixX3d, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXl, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::copyleft::cgal::remesh_self_intersections<Eigen::Matrix<double, -1, -1, 1, -1, -1>, Eigen::MatrixXi, Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1>, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, igl::copyleft::cgal::RemeshSelfIntersectionsParam const&, Eigen::PlainObjectBase<Eigen::Matrix<CGAL::Lazy_exact_nt<CGAL::Gmpq>, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
#endif
