// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "harmonic.h"
#include "adjacency_matrix.h"
#include "cotmatrix.h"
#include "diag.h"
#include "invert_diag.h"
#include "isdiag.h"
#include "massmatrix.h"
#include "min_quad_with_fixed.h"
#include "speye.h"
#include "sum.h"
#include <Eigen/Sparse>

template <
  typename DerivedV,
  typename DerivedF,
  typename Derivedb,
  typename Derivedbc,
  typename DerivedW>
IGL_INLINE bool igl::harmonic(
  const Eigen::PlainObjectBase<DerivedV> & V,
  const Eigen::PlainObjectBase<DerivedF> & F,
  const Eigen::PlainObjectBase<Derivedb> & b,
  const Eigen::PlainObjectBase<Derivedbc> & bc,
  const int k,
  Eigen::PlainObjectBase<DerivedW> & W)
{
  using namespace Eigen;
  typedef typename DerivedV::Scalar Scalar;
  SparseMatrix<Scalar> L,M;
  cotmatrix(V,F,L);
  massmatrix(V,F,MASSMATRIX_TYPE_DEFAULT,M);
  return harmonic(L,M,b,bc,k,W);
}

template <
  typename DerivedF,
  typename Derivedb,
  typename Derivedbc,
  typename DerivedW>
IGL_INLINE bool igl::harmonic(
  const Eigen::PlainObjectBase<DerivedF> & F,
  const Eigen::PlainObjectBase<Derivedb> & b,
  const Eigen::PlainObjectBase<Derivedbc> & bc,
  const int k,
  Eigen::PlainObjectBase<DerivedW> & W)
{
  using namespace Eigen;
  typedef typename Derivedbc::Scalar Scalar;
  SparseMatrix<Scalar> A;
  adjacency_matrix(F,A);
  // sum each row
  SparseVector<Scalar> Asum;
  sum(A,1,Asum);
  // Convert row sums into diagonal of sparse matrix
  SparseMatrix<Scalar> Adiag;
  diag(Asum,Adiag);
  SparseMatrix<Scalar> L = A-Adiag;
  SparseMatrix<Scalar> M;
  speye(L.rows(),M);
  return harmonic(L,M,b,bc,k,W);
}

template <
  typename DerivedL,
  typename DerivedM,
  typename Derivedb,
  typename Derivedbc,
  typename DerivedW>
IGL_INLINE bool igl::harmonic(
  const Eigen::SparseMatrix<DerivedL> & L,
  const Eigen::SparseMatrix<DerivedM> & M,
  const Eigen::PlainObjectBase<Derivedb> & b,
  const Eigen::PlainObjectBase<Derivedbc> & bc,
  const int k,
  Eigen::PlainObjectBase<DerivedW> & W)
{
  const int n = L.rows();
  assert(n == L.cols() && "L must be square");
  assert(n == M.cols() && "M must be same size as L");
  assert(n == M.rows() && "M must be square");
  assert(igl::isdiag(M) && "Mass matrix should be diagonal");

  Eigen::SparseMatrix<DerivedL> Q;
  igl::harmonic(L,M,k,Q);

  typedef DerivedL Scalar;
  min_quad_with_fixed_data<Scalar> data;
  min_quad_with_fixed_precompute(Q,b,Eigen::SparseMatrix<Scalar>(),true,data);
  W.resize(n,bc.cols());
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXS;
  const VectorXS B = VectorXS::Zero(n,1);
  for(int w = 0;w<bc.cols();w++)
  {
    const VectorXS bcw = bc.col(w);
    VectorXS Ww;
    if(!min_quad_with_fixed_solve(data,B,bcw,VectorXS(),Ww))
    {
      return false;
    }
    W.col(w) = Ww;
  }
  return true;
}

template <
  typename DerivedL,
  typename DerivedM,
  typename DerivedQ>
IGL_INLINE void igl::harmonic(
  const Eigen::SparseMatrix<DerivedL> & L,
  const Eigen::SparseMatrix<DerivedM> & M,
  const int k,
  Eigen::SparseMatrix<DerivedQ> & Q)
{
  assert(L.rows() == L.cols()&&"L should be square");
  assert(M.rows() == M.cols()&&"M should be square");
  assert(L.rows() == M.rows()&&"L should match M's dimensions");
  Eigen::SparseMatrix<DerivedM> Mi;
  invert_diag(M,Mi);
  Q = -L;
  for(int p = 1;p<k;p++)
  {
    Q = (Q*Mi*-L).eval();
  }
}

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedQ>
IGL_INLINE void igl::harmonic(
  const Eigen::PlainObjectBase<DerivedV> & V,
  const Eigen::PlainObjectBase<DerivedF> & F,
  const int k,
  Eigen::SparseMatrix<DerivedQ> & Q)
{
  Eigen::SparseMatrix<DerivedQ> L,M;
  cotmatrix(V,F,L);
  massmatrix(V,F,MASSMATRIX_TYPE_DEFAULT,M);
  return harmonic(L,M,k,Q);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template bool igl::harmonic<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::VectorXd > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template bool igl::harmonic<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::VectorXi, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template bool igl::harmonic<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);

template bool igl::harmonic<Eigen::MatrixXi, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::VectorXd > const&, int, Eigen::PlainObjectBase<Eigen::VectorXd >&);
template bool igl::harmonic<Eigen::MatrixXi, Eigen::VectorXi, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template bool igl::harmonic<Eigen::MatrixXi, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, int, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::harmonic<Eigen::MatrixXd, Eigen::MatrixXi, double>(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, int, Eigen::SparseMatrix<double, 0, int>&);
#endif
