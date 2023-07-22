// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "to_cork_mesh.h"
template <
  typename DerivedV,
  typename DerivedF>
IGL_INLINE void igl::copyleft::cork::to_cork_mesh(
  const Eigen::PlainObjectBase<DerivedV > & V,
  const Eigen::PlainObjectBase<DerivedF > & F,
  CorkTriMesh & mesh)
{
  using namespace std;
  assert((F.cols() == 0 || F.cols() == 3) && "Facets should be triangles.");
  assert((V.cols() == 0 || V.cols() == 3) && "Vertices should be in 3D.");
  mesh.n_triangles = F.rows();
  mesh.n_vertices = V.rows();
  mesh.vertices = new float[mesh.n_vertices*3];
  mesh.triangles = new uint[mesh.n_triangles*3];
  for(size_t v = 0;v<mesh.n_vertices;v++)
  {
    for(size_t c = 0;c<3;c++)
    {
      mesh.vertices[v*3+c] = V(v,c);
    }
  }
  for(size_t f = 0;f<mesh.n_triangles;f++)
  {
    for(size_t c = 0;c<3;c++)
    {
      mesh.triangles[f*3+c] = F(f,c);
    }
  }
}
#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::copyleft::cork::to_cork_mesh<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, CorkTriMesh&);
template void igl::copyleft::cork::to_cork_mesh<Eigen::MatrixX3d, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, CorkTriMesh&);
#endif
