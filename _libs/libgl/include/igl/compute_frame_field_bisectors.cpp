// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>, Olga Diamanti <olga.diam@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#if defined(_MSC_VER)
// Make MS math.h define M_PI
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include "compute_frame_field_bisectors.h"
#include "igl/local_basis.h"

template <typename DerivedV, typename DerivedF>
IGL_INLINE void igl::compute_frame_field_bisectors(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  const Eigen::PlainObjectBase<DerivedV>& B1,
  const Eigen::PlainObjectBase<DerivedV>& B2,
  const Eigen::PlainObjectBase<DerivedV>& PD1,
  const Eigen::PlainObjectBase<DerivedV>& PD2,
  Eigen::PlainObjectBase<DerivedV>& BIS1,
  Eigen::PlainObjectBase<DerivedV>& BIS2)
{
  BIS1.resize(PD1.rows(),3);
  BIS2.resize(PD1.rows(),3);
  
#pragma omp parallel for schedule(static)
  for (int i=0; i<PD1.rows();++i)
  {
    // project onto the tangent plane and convert to angle
    // Convert to angle
    double a1 = atan2(B2.row(i).dot(PD1.row(i)),B1.row(i).dot(PD1.row(i)));
    //make it positive by adding some multiple of 2pi
    a1 += std::ceil (std::max(0., -a1) / (M_PI*2.)) * (M_PI*2.);
    //take modulo 2pi
    a1 = fmod(a1, (M_PI*2.));
    double a2 = atan2(B2.row(i).dot(PD2.row(i)),B1.row(i).dot(PD2.row(i)));
    //make it positive by adding some multiple of 2pi
    a2 += std::ceil (std::max(0., -a2) / (M_PI*2.)) * (M_PI*2.);
    //take modulo 2pi
    a2 = fmod(a2, (M_PI*2.));

    double b1 = (a1+a2)/2.0;
    //make it positive by adding some multiple of 2pi
    b1 += std::ceil (std::max(0., -b1) / (M_PI*2.)) * (M_PI*2.);
    //take modulo 2pi
    b1 = fmod(b1, (M_PI*2.));

    double b2 = b1+(M_PI/2.);
    //make it positive by adding some multiple of 2pi
    b2 += std::ceil (std::max(0., -b2) / (M_PI*2.)) * (M_PI*2.);
    //take modulo 2pi
    b2 = fmod(b2, (M_PI*2.));

    BIS1.row(i) = cos(b1) * B1.row(i) + sin(b1) * B2.row(i);
    BIS2.row(i) = cos(b2) * B1.row(i) + sin(b2) * B2.row(i);

  }
}

template <typename DerivedV, typename DerivedF>
IGL_INLINE void igl::compute_frame_field_bisectors(
                                                   const Eigen::PlainObjectBase<DerivedV>& V,
                                                   const Eigen::PlainObjectBase<DerivedF>& F,
                                                   const Eigen::PlainObjectBase<DerivedV>& PD1,
                                                   const Eigen::PlainObjectBase<DerivedV>& PD2,
                                                   Eigen::PlainObjectBase<DerivedV>& BIS1,
                                                   Eigen::PlainObjectBase<DerivedV>& BIS2)
{
  DerivedV B1, B2, B3;
  igl::local_basis(V,F,B1,B2,B3);

  compute_frame_field_bisectors( V, F, B1, B2, PD1, PD2, BIS1, BIS2);

}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::compute_frame_field_bisectors<Eigen::MatrixX3d, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::compute_frame_field_bisectors<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
#endif
