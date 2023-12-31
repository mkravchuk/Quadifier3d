// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_COMISO_NROSY_H
#define IGL_COMISO_NROSY_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include "../../igl_inline.h"

;
enum  class NRosySolver
{
    SimplicialLDLT,
    gurobi_rounding,  //solve_gurobi(_A, _x, _rhs, _to_round);
    cplex_rounding,    //solve_cplex(_A, _x, _rhs, _to_round);
    no_rounding,       //solve_no_rounding(_A, _x, _rhs);
    direct_rounding,    //solve_direct_rounding( _A, _x, _rhs, _to_round);
    multiple_rounding, //solve_multiple_rounding(_A, _x, _rhs, _to_round);
    iterative                 //solve_iterative(_A, _x, _rhs, _to_round, _fixed_order);
};
namespace igl
{
  namespace copyleft
  {
  namespace comiso
  {
      

      class NRosyField
      {
      public:
          // Init
          IGL_INLINE NRosyField(const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F);

          // Generate the N-rosy field
          // N degree of the rosy field
          // roundseparately: round the integer variables one at a time, slower but higher quality
          IGL_INLINE void solve(const int N = 4, NRosySolver solverType = NRosySolver::multiple_rounding);

          // Set a hard constraint on fid
          // fid: face id
          // v: direction to fix (in 3d)
          IGL_INLINE void setConstraintHard(const int fid, const Eigen::Vector3d& v);

          // Set a soft constraint on fid
          // fid: face id
          // w: weight of the soft constraint, clipped between 0 and 1
          // v: direction to fix (in 3d)
          IGL_INLINE void setConstraintSoft(const int fid, const double w, const Eigen::Vector3d& v);

          // Set the ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
          IGL_INLINE void setSoftAlpha(double alpha);

          // Reset constraints (at least one constraint must be present or solve will fail)
          IGL_INLINE void resetConstraints();

          // Return the current field
          IGL_INLINE Eigen::MatrixXd getFieldPerFace();          

          // Return the current field (in Ahish's ffield format)
          IGL_INLINE Eigen::MatrixXd getFFieldPerFace();

          // Compute singularity indexes
          IGL_INLINE void findCones(int N);

          // Return the singularities
          IGL_INLINE Eigen::VectorXd getSingularityIndexPerVertex();


          // Compute angle differences between reference frames
          IGL_INLINE void computek();

          // Remove useless matchings
          IGL_INLINE void reduceSpace();

          // Prepare the system matrix
          IGL_INLINE void prepareSystemMatrix(const int N);

          // Solve without roundings
          IGL_INLINE void solveNoRoundings();

          // Solve with roundings using CoMIso
          IGL_INLINE void solveRoundings(NRosySolver solverType);

          // Round all p to 0 and fix
          IGL_INLINE void roundAndFixToZero();

          // Round all p and fix
          IGL_INLINE void roundAndFix();

          // Convert a vector in 3d to an angle wrt the local reference system
          IGL_INLINE double convert3DtoLocal(unsigned fid, const Eigen::Vector3d& v);

          // Convert an angle wrt the local reference system to a 3d vector
          IGL_INLINE Eigen::Vector3d convertLocalto3D(unsigned fid, double a);

          // Compute the per vertex angle defect
          IGL_INLINE Eigen::VectorXd angleDefect();

          // Temporary variable for the field
          Eigen::VectorXd angles;

          // Hard constraints
          Eigen::VectorXd hard;
          std::vector<bool> isHard;

          // Soft constraints
          Eigen::VectorXd soft;
          Eigen::VectorXd wSoft;
          double          softAlpha;

          // Face Topology
          Eigen::MatrixXi TT, TTi;

          // Edge Topology
          Eigen::MatrixXi EV, FE, EF;
          std::vector<bool> isBorderEdge;

          // Per Edge information
          // Angle between two reference frames
          Eigen::VectorXd k;

          // Jumps
          Eigen::VectorXi p;
          std::vector<bool> pFixed;

          // Mesh
          Eigen::MatrixXd V;
          Eigen::MatrixXi F;

          // Normals per face
          Eigen::MatrixXd N;

          // Singularity index
          Eigen::VectorXd singularityIndex;

      private:

          // Reference frame per triangle
          std::vector<Eigen::MatrixXd> TPs;

          // System stuff
          Eigen::SparseMatrix<double> A;
          Eigen::VectorXd b;
          Eigen::VectorXi tag_t;
          Eigen::VectorXi tag_p;

      };

    // Generate a N-RoSy field from a sparse set of constraints
    //
    // Inputs:
    //   V       #V by 3 list of mesh vertex coordinates
    //   F       #F by 3 list of mesh faces (must be triangles)
    //   b       #B by 1 list of constrained face indices
    //   bc      #B by 3 list of representative vectors for the constrained faces
    //   b_soft  #S by 1 b for soft constraints
    //   w_soft  #S by 1 weight for the soft constraints (0-1)
    //   bc_soft #S by 3 bc for soft constraints
    //   N       the degree of the N-RoSy vector field
    //   soft    the strenght of the soft contraints w.r.t. smoothness
    //           (0 -> smoothness only, 1->constraints only)
    // Outputs:
    //   R       #F by 3 the representative vectors of the interpolated field
    //   S       #V by 1 the singularity index for each vertex (0 = regular)
    IGL_INLINE void nrosy(
      const Eigen::MatrixXd& V,
      const Eigen::MatrixXi& F,
      const Eigen::VectorXi& b,
      const Eigen::MatrixXd& bc,
      const Eigen::VectorXi& b_soft,
      const Eigen::VectorXd& w_soft,
      const Eigen::MatrixXd& bc_soft,
      const int N,
      const double soft,
      Eigen::MatrixXd& R,
      Eigen::VectorXd& S
      );
    //wrapper for the case without soft constraints
    IGL_INLINE void nrosy(
     const Eigen::MatrixXd& V,
     const Eigen::MatrixXi& F,
     const Eigen::VectorXi& b,
     const Eigen::MatrixXd& bc,
     const int N,
     Eigen::MatrixXd& R,
     Eigen::VectorXd& S
      );

  }
}
}

#ifndef IGL_STATIC_LIBRARY
#  include "nrosy.cpp"
#endif

#endif
