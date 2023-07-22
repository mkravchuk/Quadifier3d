// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0f/.
#ifndef IGL_MATERIAL_COLORS_H
#define IGL_MATERIAL_COLORS_H
#include <Eigen/Core>
// Define constant material colors for use with opengl glMaterialfv
// Most of these colors come from IGL publications
namespace igl
{
  // Gold/Silver used in BBW/MONO/STBS/FAST
  const float GOLD_AMBIENT[4] =   {  51.0f/255.0f, 43.0f/255.0f,33.3f/255.0f,1.0f };
  const float GOLD_DIFFUSE[4] =   { 255.0f/255.0f,228.0f/255.0f,58.0f/255.0f,1.0f };
  const float GOLD_SPECULAR[4] =  { 255.0f/255.0f,235.0f/255.0f,80.0f/255.0f,1.0f };
  const float SILVER_AMBIENT[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
  const float SILVER_DIFFUSE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
  const float SILVER_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
  // Blue/Cyan more similar to Jovan Popovic's blue than to Mario Botsch's blue
  const float CYAN_AMBIENT[4] =   {  59.0f/255.0f, 68.0f/255.0f,255.0f/255.0f,1.0f };
  const float CYAN_DIFFUSE[4] =   {  94.0f/255.0f,185.0f/255.0f,238.0f/255.0f,1.0f };
  const float CYAN_SPECULAR[4] =   { 163.0f/255.0f,221.0f/255.0f,255.0f/255.0f,1.0f };
  const float DENIS_PURPLE_DIFFUSE[4] =   { 80.0f/255.0f,64.0f/255.0f,255.0f/255.0f,1.0f };
  const float LADISLAV_ORANGE_DIFFUSE[4] = {1.0f, 125.0f / 255.0f, 19.0f / 255.0f, 0.0f};
  // FAST armadillos colors
  const float FAST_GREEN_DIFFUSE[4] = { 113.0f/255.0f, 239.0f/255.0f,  46.0f/255.0f, 1.0f};
  const float FAST_RED_DIFFUSE[4]   = { 255.0f/255.0f,  65.0f/255.0f,  46.0f/255.0f, 1.0f};
  const float FAST_BLUE_DIFFUSE[4]  = { 106.0f/255.0f, 106.0f/255.0f, 255.0f/255.0f, 1.0f};
  const float FAST_GRAY_DIFFUSE[4]  = { 150.0f/255.0f, 150.0f/255.0f, 150.0f/255.0f, 1.0f};
  // Basic colors
  const float WHITE[4] =   { 255.0f/255.0f,255.0f/255.0f,255.0f/255.0f,1.0f };
  const float BLACK[4] =   { 0.0f/255.0f,0.0f/255.0f,0.0f/255.0f,1.0f };
  const float WHITE_AMBIENT[4] =   { 255.0f/255.0f,255.0f/255.0f,255.0f/255.0f,1.0f };
  const float WHITE_DIFFUSE[4] =   { 255.0f/255.0f,255.0f/255.0f,255.0f/255.0f,1.0f };
  const float WHITE_SPECULAR[4] =  { 255.0f/255.0f,255.0f/255.0f,255.0f/255.0f,1.0f };
  const float BBW_POINT_COLOR[4] = {239.f/255.f,213.f/255.f,46.f/255.f,255.0f/255.0};
  const float BBW_LINE_COLOR[4] = {106.f/255.f,106.f/255.f,255.f/255.f,255.f/255.};
  const float MIDNIGHT_BLUE_DIFFUSE[4]  = { 21.0f/255.0f, 27.0f/255.0f, 84.0f/255.0f, 1.0f};
  // Winding number colors
  const float EASTER_RED_DIFFUSE[4] = {0.603922f,0.494118f,0.603922f,1.0f};
  const float WN_OPEN_BOUNDARY_COLOR[4] = {154.f/255.f,0.f/255.f,0.f/255.f,1.0f};
  const float WN_NON_MANIFOLD_EDGE_COLOR[4] = {201.f/255.f, 51.f/255.f,255.f/255.f,1.0f};
  const Eigen::Vector4f 
    MAYA_GREEN(128.f/255.f,242.f/255.f,0.f/255.f,1.),
    MAYA_YELLOW(255.f/255.f,247.f/255.f,50.f/255.f,1.),
    MAYA_RED(234.f/255.f,63.f/255.f,52.f/255.f,1.),
    MAYA_BLUE(0.f/255.f,73.f/255.f,252.f/255.f,1.),
    MAYA_PURPLE(180.f/255.f,73.f/255.f,200.f/255.f,1.),
    MAYA_VIOLET(31.f/255.f,15.f/255.f,66.f/255.f,1.),
    MAYA_GREY(0.5,0.5,0.5,1.0),
    MAYA_CYAN(131.f/255.f,219.f/255.f,252.f/255.f,1.),
    MAYA_SEA_GREEN(70.f/255.f,252.f/255.f,167.f/255.f,1.);
}
#endif
