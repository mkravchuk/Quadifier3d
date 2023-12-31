// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "from_cork_mesh.h"

template <
  typename DerivedV,
  typename DerivedF>
IGL_INLINE void igl::copyleft::cork::from_cork_mesh(
  const CorkTriMesh & mesh,
  Eigen::PlainObjectBase<DerivedV > & V,
  Eigen::PlainObjectBase<DerivedF > & F)
{
  using namespace std;
  F.resize(mesh.n_triangles,3);
  V.resize(mesh.n_vertices,3);
  for(size_t v = 0;v<mesh.n_vertices;v++)
  {
    for(size_t c = 0;c<3;c++)
    {
      V(v,c) = mesh.vertices[v*3+c];
    }
  }
  for(size_t f = 0;f<mesh.n_triangles;f++)
  {
    for(size_t c = 0;c<3;c++)
    {
      F(f,c) = mesh.triangles[f*3+c];
    }
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::copyleft::cork::from_cork_mesh<Eigen::MatrixXd, Eigen::MatrixXi >(CorkTriMesh const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::copyleft::cork::from_cork_mesh<Eigen::MatrixX3d, Eigen::MatrixX3i >(CorkTriMesh const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&);
#endif
