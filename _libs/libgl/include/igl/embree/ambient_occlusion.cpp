// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "ambient_occlusion.h"
#include "../ambient_occlusion.h"
#include "EmbreeIntersector.h"
#include "../Hit.h"

template <
  typename DerivedP,
  typename DerivedN,
  typename DerivedS >
IGL_INLINE void igl::embree::ambient_occlusion(
  const igl::embree::EmbreeIntersector & ei,
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<DerivedN> & N,
  const int num_samples,
  Eigen::PlainObjectBase<DerivedS> & S)
{
  const auto & shoot_ray = [&ei](
    const Eigen::Vector3f& s,
    const Eigen::Vector3f& dir)->bool
  {
    igl::Hit hit;
    const float tnear = 1e-4f;
    return ei.intersectRay(s,dir,hit,tnear);
  };
  return igl::ambient_occlusion(shoot_ray,P,N,num_samples,S);
}

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedP,
  typename DerivedN,
  typename DerivedS >
IGL_INLINE void igl::embree::ambient_occlusion(
  const Eigen::PlainObjectBase<DerivedV> & V,
  const Eigen::PlainObjectBase<DerivedF> & F,
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<DerivedN> & N,
  const int num_samples,
  Eigen::PlainObjectBase<DerivedS> & S)
{
  using namespace Eigen;
  EmbreeIntersector ei;
  ei.init(V.template cast<float>(),F.template cast<int>());
  ambient_occlusion(ei,P,N,num_samples,S);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::embree::ambient_occlusion<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd >(igl::embree::EmbreeIntersector const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template void igl::embree::ambient_occlusion<Eigen::Matrix<double, 1, 3, 1, 1, 3>, Eigen::Matrix<double, 1, 3, 1, 1, 3>, Eigen::VectorXd >(igl::embree::EmbreeIntersector const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 1, 3, 1, 1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 1, 3, 1, 1, 3> > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template void igl::embree::ambient_occlusion<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template void igl::embree::ambient_occlusion<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::VectorXd >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template void igl::embree::ambient_occlusion<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
#endif
