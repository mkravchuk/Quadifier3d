// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "per_face_normals.h"
#include <Eigen/Geometry>

#define SQRT_ONE_OVER_THREE 0.57735026918962573
template <typename DerivedV, typename DerivedF, typename DerivedZ, typename DerivedN>
IGL_INLINE void igl::per_face_normals(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  const Eigen::PlainObjectBase<DerivedZ> & Z,
  Eigen::PlainObjectBase<DerivedN> & N)
{
  N.resize(F.rows(),3);
  // loop over faces
  int Frows = F.rows();
//#pragma omp parallel for if (Frows>10000)
  for(int i = 0; i < Frows;i++)
  {
    const Eigen::Matrix<typename DerivedV::Scalar, 1, 3> v1 = V.row(F(i,1)) - V.row(F(i,0));
    const Eigen::Matrix<typename DerivedV::Scalar, 1, 3> v2 = V.row(F(i,2)) - V.row(F(i,0));
    N.row(i) = v1.cross(v2);//.normalized();
    typename DerivedV::Scalar r = N.row(i).norm();
    if(r == 0)
    {
      N.row(i) = Z;
    }else
    {
      N.row(i) /= r;
    }
  }
}

template <typename DerivedV, typename DerivedF, typename DerivedN>
IGL_INLINE void igl::per_face_normals(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  Eigen::PlainObjectBase<DerivedN> & N)
{
  using namespace Eigen;
  Matrix<typename DerivedN::Scalar,3,1> Z(0,0,0);
  return per_face_normals(V,F,Z,N);
}

template <typename DerivedV, typename DerivedF, typename DerivedN>
IGL_INLINE void igl::per_face_normals_stable(
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  Eigen::PlainObjectBase<DerivedN> & N)
{
  using namespace Eigen;
  typedef Matrix<typename DerivedV::Scalar,1,3> RowVectorV3;
  typedef typename DerivedV::Scalar Scalar;

  const size_t m = F.rows();

  N.resize(F.rows(),3);
  // Grad all points
  for(size_t f = 0;f<m;f++)
  {
    const RowVectorV3 p0 = V.row(F(f,0));
    const RowVectorV3 p1 = V.row(F(f,1));
    const RowVectorV3 p2 = V.row(F(f,2));
    const RowVectorV3 n0 = (p1 - p0).cross(p2 - p0);
    const RowVectorV3 n1 = (p2 - p1).cross(p0 - p1);
    const RowVectorV3 n2 = (p0 - p2).cross(p1 - p2);

    // careful sum
    for(int d = 0;d<3;d++)
    {
      // This is a little _silly_ in terms of complexity, but its recursive
      // implementation is clean looking...
      const std::function<Scalar(Scalar,Scalar,Scalar)> sum3 =
        [&sum3](Scalar a, Scalar b, Scalar c)->Scalar
      {
        if(fabs(c)>fabs(a))
        {
          return sum3(c,b,a);
        }
        // c < a
        if(fabs(c)>fabs(b))
        {
          return sum3(a,c,b);
        }
        // c < a, c < b
        if(fabs(b)>fabs(a))
        {
          return sum3(b,a,c);
        }
        return (a+b)+c;
      };

      N(f,d) = sum3(n0(d),n1(d),n2(d));
    }
    // sum better not be sure, or else NaN
    N.row(f) /= N.row(f).norm();
  }

}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::per_face_normals<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3>, Eigen::MatrixX3f >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3f >&);
// generated by autoexplicit.sh
template void igl::per_face_normals<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3>, Eigen::Matrix<float, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> >&);
template void igl::per_face_normals<Eigen::MatrixXd, Eigen::MatrixXi >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::per_face_normals<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::Vector3d, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::Vector3d > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::per_face_normals<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::per_face_normals<Eigen::Matrix<float, -1, -1, 1, -1, -1>, Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, Eigen::Matrix<float, -1, -1, 1, -1, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<float, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, -1, 1, -1, -1> >&);
template void igl::per_face_normals<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::per_face_normals_stable<Eigen::MatrixX3d, Eigen::MatrixX3i, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::per_face_normals_stable<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::per_face_normals<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::per_face_normals_stable<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd >&);
template void igl::per_face_normals<Eigen::MatrixXd, Eigen::Matrix<int, 1, 3, 1, 1, 3>, Eigen::Matrix<double, 1, -1, 1, 1, -1> >(Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, 1, -1, 1, 1, -1> >&);
template void igl::per_face_normals<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, -1, 3, 1, -1, 3>, Eigen::Matrix<double, -1, 3, 1, -1, 3> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&);
template void igl::per_face_normals<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, -1, 3, 1, -1, 3>, Eigen::MatrixX3d >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::MatrixX3d >&);
template void igl::per_face_normals<class Eigen::Matrix<double,-1,3,EROW,-1,3>,class Eigen::Matrix<int,-1,-1, EROW,-1,-1>,class Eigen::Matrix<double,-1,-1, EROW,-1,-1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double,-1,3, EROW,-1,3> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1, EROW,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<double,-1,-1, EROW,-1,-1> > &);

#endif
