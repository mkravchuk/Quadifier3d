// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "components.h"
#include "adjacency_matrix.h"
#include <queue>
#include <vector>

template <typename AScalar, typename DerivedC, typename Derivedcounts>
IGL_INLINE void igl::components(
  const Eigen::SparseMatrix<AScalar> & A,
  Eigen::PlainObjectBase<DerivedC> & C,
  Eigen::PlainObjectBase<Derivedcounts> & counts)
{
  using namespace Eigen;
  using namespace std;
  assert(A.rows() == A.cols() && "A should be square.");
  const size_t n = A.rows();
  Array<bool,Dynamic,1> seen = Array<bool,Dynamic,1>::Zero(n,1);
  C.resize(n,1);
  typename DerivedC::Scalar id = 0;
  vector<typename Derivedcounts::Scalar> vcounts;
  for(int k=0; k<A.outerSize(); ++k)
  {
    if(seen(k))
    {
      continue;
    }
    queue<int> Q;
    Q.push(k);
    vcounts.push_back(0);
    while(!Q.empty())
    {
      const int f = Q.front();
      Q.pop();
      if(seen(f))
      {
        continue;
      }
      seen(f) = true;
      C(f,0) = id;
      vcounts[id]++;
      // Iterate over inside
      for(typename SparseMatrix<AScalar>::InnerIterator it (A,f); it; ++it)
      {
        const int g = it.index();
        if(!seen(g))
        {
          Q.push(g);
        }
      }
    }
    id++;
  }
  assert((size_t) id == vcounts.size());
  const size_t ncc = vcounts.size();
  assert((size_t)C.maxCoeff()+1 == ncc);
  counts.resize(ncc,1);
  for(size_t i = 0;i<ncc;i++)
  {
    counts(i) = vcounts[i];
  }
}

template <typename AScalar, typename DerivedC>
IGL_INLINE void igl::components(
  const Eigen::SparseMatrix<AScalar> & A,
  Eigen::PlainObjectBase<DerivedC> & C)
{
  Eigen::VectorXi counts;
  return components(A,C,counts);
}

template <typename DerivedF, typename DerivedC>
IGL_INLINE void igl::components(
  const Eigen::PlainObjectBase<DerivedF> & F,
  Eigen::PlainObjectBase<DerivedC> & C)
{
  Eigen::SparseMatrix<typename DerivedC::Scalar> A;
  adjacency_matrix(F,A);
  return components(A,C);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template void igl::components<Eigen::MatrixXi, Eigen::VectorXi >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::components<int, Eigen::VectorXi >(Eigen::SparseMatrix<int, 0, int> const&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::components<int, Eigen::MatrixXi >(Eigen::SparseMatrix<int, 0, int> const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
template void igl::components<double, Eigen::MatrixXi, Eigen::MatrixXi >(Eigen::SparseMatrix<double, 0, int> const&, Eigen::PlainObjectBase<Eigen::MatrixXi >&, Eigen::PlainObjectBase<Eigen::MatrixXi >&);
#endif
