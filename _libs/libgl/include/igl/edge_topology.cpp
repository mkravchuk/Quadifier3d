// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "edge_topology.h"
#include "is_edge_manifold.h"
#include "triangle_triangle_adjacency.h"
#include <algorithm>

template<typename DerivedV, typename DerivedF>
IGL_INLINE void igl::edge_topology(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
    const std::vector<std::vector<int>>& TTT,    
    bool checkManifold,
    Eigen::MatrixXi& EV,
  Eigen::MatrixXi& FE,
  Eigen::MatrixXi& EF
    )
{
  // Only needs to be edge-manifold
  if (V.rows() ==0 || F.rows()==0)
  {
    EV = Eigen::MatrixXi::Constant(0,2,-1);
    FE = Eigen::MatrixXi::Constant(0,3,-1);
    EF = Eigen::MatrixXi::Constant(0,2,-1);
    return;
  }
  if (checkManifold)
  {
      assert(igl::is_edge_manifold(F, TTT));
  }

  // count the number of edges (assume manifoldness)
  int En = 1; // the last is always counted
  for(int i=0;i<int(TTT.size())-1;++i)
    if (!((TTT[i][0] == TTT[i+1][0]) && (TTT[i][1] == TTT[i+1][1])))
      ++En;

  EV = Eigen::MatrixXi::Constant((int)(En),2,-1);
  FE = Eigen::MatrixXi::Constant((int)(F.rows()),3,-1);
  EF = Eigen::MatrixXi::Constant((int)(En),2,-1);
  En = 0;
  int TTT_size_minus_1 = TTT.size() - 1;
  for(unsigned i=0;i<TTT.size();++i)
  {
    if (i == TTT_size_minus_1 ||
        !((TTT[i][0] == TTT[i+1][0]) && (TTT[i][1] == TTT[i+1][1]))
        )
    {
      // Border edge
      const std::vector<int>& r1 = TTT[i];
      EV(En,0)     = r1[0];
      EV(En,1)     = r1[1];
      EF(En,0)    = r1[2];
      FE(r1[2],r1[3]) = En;
    }
    else
    {
      const std::vector<int>& r1 = TTT[i];
      const std::vector<int>& r2 = TTT[i+1];
      EV(En,0)     = r1[0];
      EV(En,1)     = r1[1];
      EF(En,0)    = r1[2];
      EF(En,1)    = r2[2];
      FE(r1[2],r1[3]) = En;
      FE(r2[2],r2[3]) = En;
      ++i; // skip the next one
    }
    ++En;
  }

  

  // Sort the relation EF, accordingly to EV
  // the first one is the face on the left of the edge
  for(unsigned i=0; i<EF.rows(); ++i)
  {
    int fid = EF(i,0);
    bool flip = true;
    // search for edge EV.row(i)
    for (unsigned j=0; j<3; ++j)
    {
      if ((F(fid,j) == EV(i,0)) && (F(fid,(j+1)%3) == EV(i,1)))
        flip = false;
    }

    if (flip)
    {
      int tmp = EF(i,0);
      EF(i,0) = EF(i,1);
      EF(i,1) = tmp;
    }
  }


  
}

template<typename DerivedV, typename DerivedF>
IGL_INLINE void igl::edge_topology(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::MatrixXi& EV,
    Eigen::MatrixXi& FE,
    Eigen::MatrixXi& EF)
{
    std::vector<std::vector<int> > TTT;
    triangle_triangle_adjacency_preprocess(F, TTT);
    return edge_topology(V, F, TTT, true, EV, FE, EF);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::edge_topology<Eigen::MatrixX3d, Eigen::MatrixX3i >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::MatrixXi&, Eigen::MatrixXi&, Eigen::MatrixXi&);
template void igl::edge_topology<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::MatrixXi&, Eigen::MatrixXi&, Eigen::MatrixXi&);
template void igl::edge_topology<Eigen::MatrixXd, Eigen::MatrixXi >(
    Eigen::PlainObjectBase<Eigen::MatrixXd > const&, 
    Eigen::PlainObjectBase<Eigen::MatrixXi > const&, 
    std::vector<std::vector<int> >const & TTT,
    bool,
    Eigen::MatrixXi&, Eigen::MatrixXi&, Eigen::MatrixXi&);
#endif
