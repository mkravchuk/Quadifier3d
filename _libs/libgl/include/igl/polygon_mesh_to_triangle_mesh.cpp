// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "polygon_mesh_to_triangle_mesh.h"
#include "matrix_to_list.h"

template <typename Index, typename DerivedF>
IGL_INLINE void igl::polygon_mesh_to_triangle_mesh(
  const std::vector<std::vector<Index> > & vF,
  Eigen::PlainObjectBase<DerivedF>& F)
{
  using namespace std;
  using namespace Eigen;
  int m = 0;
  // estimate of size
  for(typename vector<vector<Index > >::const_iterator fit = vF.begin();
    fit!=vF.end();
    fit++)
  {
    if(fit->size() >= 3)
    {
      m += fit->size() - 2;
    }
  }
  // Resize output
  F.resize(m,3);
  {
    int k = 0;
    for(typename vector<vector<Index > >::const_iterator fit = vF.begin();
      fit!=vF.end();
      fit++)
    {
      if(fit->size() >= 3)
      {
        typename vector<Index >::const_iterator cit = fit->begin();
        cit++;
        typename vector<Index >::const_iterator pit = cit++;
        for(;
          cit!=fit->end();
          cit++,pit++)
        {
          F(k,0) = *(fit->begin());
          F(k,1) = *pit;
          F(k,2) = *cit;
          k++;
        }
      }
    }
    assert(k==m);
  }

}

template <typename DerivedP, typename DerivedF>
IGL_INLINE void igl::polygon_mesh_to_triangle_mesh(
  const Eigen::PlainObjectBase<DerivedP>& P,
  Eigen::PlainObjectBase<DerivedF>& F)
{
  std::vector<std::vector<typename DerivedP::Scalar> > vP;
  matrix_to_list(P,vP);
  return polygon_mesh_to_triangle_mesh(vP,F);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::polygon_mesh_to_triangle_mesh<int, Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> >&);
// generated by autoexplicit.sh
template void igl::polygon_mesh_to_triangle_mesh<int, Eigen::MatrixX3i >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i >&);
template void igl::polygon_mesh_to_triangle_mesh<int, Eigen::MatrixXi >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::polygon_mesh_to_triangle_mesh<int, Eigen::Matrix<int, -1, 3, 1, -1, 3> >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> >&);
#endif
