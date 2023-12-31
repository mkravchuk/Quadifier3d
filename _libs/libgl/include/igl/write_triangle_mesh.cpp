// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "write_triangle_mesh.h"
#include "pathinfo.h"
#include "writeMESH.h"
#include "writeOBJ.h"
#include "writeOFF.h"
#include "writePLY.h"
#include "writeSTL.h"
#include "writeWRL.h"

#include <iostream>

template <typename DerivedV, typename DerivedF>
IGL_INLINE bool igl::write_triangle_mesh(
  const std::string str,
  const Eigen::PlainObjectBase<DerivedV>& V,
  const Eigen::PlainObjectBase<DerivedF>& F,
  const bool ascii)
{
  using namespace std;
  // dirname, basename, extension and filename
  string d,b,e,f;
  pathinfo(str,d,b,e,f);
  // Convert extension to lower case
  std::transform(e.begin(), e.end(), e.begin(), ::tolower);
  if(e == "mesh")
  {
    assert(ascii && ".mesh only supports ascii");
    Eigen::MatrixXi _1;
    return writeMESH(str,V,_1,F);
  }else if(e == "obj")
  {
    assert(ascii && ".obj only supports ascii");
    return writeOBJ(str,V,F);
  }else if(e == "off")
  {
    assert(ascii && ".off only supports ascii");
    return writeOFF(str,V,F);
  }else if(e == "ply")
  {
    return writePLY(str,V,F,ascii);
  }else if(e == "stl")
  {
    return writeSTL(str,V,F,ascii);
  }else if(e == "wrl")
  {
    assert(ascii && ".wrl only supports ascii");
    return writeWRL(str,V,F);
  }else
  {
    assert("Unsupported file format");
    cerr<<"Unsupported file format: ."<<e<<endl;
    return false;
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template bool igl::write_triangle_mesh<Eigen::MatrixX3d, Eigen::MatrixX3i >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::MatrixX3d > const&, Eigen::PlainObjectBase<Eigen::MatrixX3i > const&, bool);
// generated by autoexplicit.sh
template bool igl::write_triangle_mesh<Eigen::MatrixXd, Eigen::MatrixXi >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::MatrixXd > const&, Eigen::PlainObjectBase<Eigen::MatrixXi > const&, const bool);
template bool igl::write_triangle_mesh<Eigen::Matrix<double, 8, 3, 0, 8, 3>, Eigen::Matrix<int, 12, 3, 0, 12, 3> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, 8, 3, 0, 8, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, 12, 3, 0, 12, 3> > const&, bool);
#endif
