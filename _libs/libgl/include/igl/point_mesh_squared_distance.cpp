// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "point_mesh_squared_distance.h"
#include "AABB.h"

template <
  typename DerivedP,
  typename DerivedV,
  typename DerivedsqrD,
  typename DerivedI,
  typename DerivedC>
IGL_INLINE void igl::point_mesh_squared_distance(
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<DerivedV> & V,
  const Eigen::MatrixXi & Ele,
  Eigen::PlainObjectBase<DerivedsqrD> & sqrD,
  Eigen::PlainObjectBase<DerivedI> & I,
  Eigen::PlainObjectBase<DerivedC> & C)
{
  using namespace std;
  const size_t dim = P.cols();
  assert((dim == 2 || dim == 3) && "P.cols() should be 2 or 3");
  assert(P.cols() == V.cols() && "P.cols() should equal V.cols()");
  switch(dim)
  {
    default:
      // fall-through and pray
    case 3:
    {
      AABB<DerivedV,3> tree;
      tree.init(V,Ele);
      return tree.squared_distance(V,Ele,P,sqrD,I,C);
    }
    case 2:
    {
      AABB<DerivedV,2> tree;
      tree.init(V,Ele);
      return tree.squared_distance(V,Ele,P,sqrD,I,C);
    }
  }
}

#ifdef IGL_STATIC_LIBRARY
template void igl::point_mesh_squared_distance<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::MatrixXi const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::point_mesh_squared_distance<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXi, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::MatrixXi const&, Eigen::PlainObjectBase<Eigen::VectorXd >&, Eigen::PlainObjectBase<Eigen::VectorXi >&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::point_mesh_squared_distance<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXl, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::MatrixXi const&, Eigen::PlainObjectBase<Eigen::VectorXd >&, Eigen::PlainObjectBase<Eigen::VectorXl >&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
#endif
