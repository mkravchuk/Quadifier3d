#define testAngleBoundFFSolverData

#ifdef testAngleBoundFFSolverData

#undef IGL_STATIC_LIBRARY
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/viewer/Viewer.h>
#include <igl/barycenter.h>
#include <igl/avg_edge_length.h>
#include <vector>
#include <igl/n_polyvector.h>
#include <igl/angle_bound_frame_fields.h>
#include <stdlib.h>
#include <igl/jet.h>
#include <Eigen/Sparse>

// Input mesh
Eigen::MatrixXd V;
Eigen::MatrixXi F;

// Face barycenters
Eigen::MatrixXd B;


// Scale for visualizing the fields
double global_scale;

// Input constraints
Eigen::VectorXi isConstrained;
Eigen::MatrixXd constraints;

Eigen::MatrixXd smooth_pvf;
Eigen::MatrixXd angle_bound_pvf;
igl::AngleBoundFFSolverData<Eigen::MatrixXd, Eigen::MatrixXi> *csdata;

int maxIter = 100;
int iterDone = 0;
// double lambdaInit = 100;
// double lambdaMultFactor = 1.5;
double lambdaInit = .1;
double lambdaMultFactor = 1.01;
bool doHardConstraints = false;

int mode;

void computeAngles(const Eigen::MatrixXd &ff, Eigen::VectorXd &angles)
{
    angles.resize(ff.rows(), 1);
    // int num =0;
    for (int i = 0; i<ff.rows(); ++i)
    {
        Eigen::RowVector3d u = (ff.block(i, 0, 1, 3)); u.normalize();
        Eigen::RowVector3d v = (ff.block(i, 3, 1, 3)); v.normalize();
        double s = (u.cross(v)).norm();
        double c = fabs(u.dot(v));
        angles[i] = atan2(s, c);
        //  num += (angles[i]<70*M_PI/180);
    }
    // std::cerr<<"out of bound:"<<num<<std::endl;
}

void getAngleColor(const Eigen::MatrixXd &ff, Eigen::MatrixXd &C)
{
    double MPI = 3.14159265358979323846;
    Eigen::VectorXd angles;
    computeAngles(ff, angles);
    Eigen::VectorXd val = 0.5*MPI*Eigen::VectorXd::Ones(angles.rows(), 1) - angles;
    igl::jet(val, 0, 20 * MPI / 180., C);
}

bool pre_draw(igl::viewer::Viewer & viewer)
{
    using namespace Eigen;
    using namespace std;
    double lambdaOut;

    if (mode == 1)
    {
        if (iterDone <maxIter)
        {
            igl::angle_bound_frame_fields(*csdata,
                70,
                isConstrained,
                angle_bound_pvf,
                angle_bound_pvf,
                1,
                lambdaInit,
                lambdaMultFactor,
                doHardConstraints,
                &lambdaOut);
            iterDone++;
            // lambdaInit = lambdaOut;
            lambdaInit *= 1.1;

        }
        viewer.data.lines.resize(0, 9);
        viewer.data.add_edges(B - global_scale*angle_bound_pvf.block(0, 0, F.rows(), 3),
            B + global_scale*angle_bound_pvf.block(0, 0, F.rows(), 3),
            Eigen::RowVector3d(0, 0, 1));
        viewer.data.add_edges(B - global_scale*angle_bound_pvf.block(0, 3, F.rows(), 3),
            B + global_scale*angle_bound_pvf.block(0, 3, F.rows(), 3),
            Eigen::RowVector3d(0, 0, 1));
    }
    return false;
}


bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier)
{
    using namespace std;
    using namespace Eigen;

    // Highlight in red the constrained faces
    MatrixXd CC = MatrixXd::Constant(F.rows(), 3, 1);
    for (unsigned i = 0; i<F.rows(); ++i)
        if (isConstrained[i])
            CC.row(i) << 1, 0, 0;

    if (key <'1' || key >'5')
    {
        return false;
    }
    mode = 0;
    if (key == '1')
    {
        viewer.data.lines.resize(0, 9);
        viewer.data.set_colors(CC);
        // Frame field constraints
        MatrixXd F1_t = MatrixXd::Zero(F.rows(), 3);
        MatrixXd F2_t = MatrixXd::Zero(F.rows(), 3);
        for (unsigned i = 0; i<F.rows(); ++i)
            if (isConstrained[i])
            {
                F1_t.row(i) = constraints.block(i, 0, 1, 3);
                F2_t.row(i) = constraints.block(i, 3, 1, 3);
            }
        viewer.data.add_edges(B - global_scale*F1_t, B + global_scale*F1_t, Eigen::RowVector3d(0, 0, 1));
        viewer.data.add_edges(B - global_scale*F2_t, B + global_scale*F2_t, Eigen::RowVector3d(0, 0, 1));
    }
    if (key == '2')
    {
        viewer.data.lines.resize(0, 9);
        viewer.data.add_edges(B - global_scale*smooth_pvf.block(0, 0, F.rows(), 3),
            B + global_scale*smooth_pvf.block(0, 0, F.rows(), 3),
            Eigen::RowVector3d(0, 1, 0));
        viewer.data.add_edges(B - global_scale*smooth_pvf.block(0, 3, F.rows(), 3),
            B + global_scale*smooth_pvf.block(0, 3, F.rows(), 3),
            Eigen::RowVector3d(0, 1, 0));
        viewer.data.set_colors(CC);
    }
    if (key == '3')
    {
        viewer.data.lines.resize(0, 9);
        MatrixXd C;
        getAngleColor(smooth_pvf, C);
        viewer.data.set_colors(C);
    }

    if (key == '4')
    {
        // viewer.data.add_edges (B - global_scale*angle_bound_pvf.block(0,0,F.rows(),3),
        // B + global_scale*angle_bound_pvf.block(0,0,F.rows(),3),
        // Eigen::RowVector3d(0,1,0));
        // viewer.data.add_edges (B - global_scale*angle_bound_pvf.block(0,3,F.rows(),3),
        // B + global_scale*angle_bound_pvf.block(0,3,F.rows(),3),
        // Eigen::RowVector3d(0,1,0));
        viewer.data.lines.resize(0, 9);
        viewer.data.set_colors(CC);
        mode = 1;
    }
    if (key == '5')
    {
        viewer.data.lines.resize(0, 9);
        MatrixXd C;
        getAngleColor(angle_bound_pvf, C);
        viewer.data.set_colors(C);
    }

    return false;
}

int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace std;
    // Load a mesh in OBJ format
    igl::readOBJ(TUTORIAL_SHARED_PATH "/teddy.obj", V, F);

    // Compute face barycenters
    igl::barycenter(V, F, B);

    // Compute scale for visualizing fields
    global_scale = .5*igl::avg_edge_length(V, F);

    // Load constraints
    MatrixXd temp;
    igl::readDMAT(TUTORIAL_SHARED_PATH "/teddy.dmat", temp);
    isConstrained = temp.block(0, 0, temp.rows(), 1).cast<int>();
    constraints = temp.block(0, 1, temp.rows(), temp.cols() - 1);

    int numConstrained = isConstrained.sum();
    VectorXi b(numConstrained);
    MatrixXd bc(numConstrained, constraints.cols());
    int ind = 0;
    for (int fi = 0; fi <F.rows(); ++fi)
        if (isConstrained[fi])
        {
            b[ind] = fi;
            bc.row(ind) = constraints.row(fi);
            ind++;
        }

    // Interpolated PolyVector field
    Eigen::MatrixXd pvf;
    igl::n_polyvector(V, F, b, bc, smooth_pvf);

    // Initialize conjugate field with smooth field
    csdata = new igl::AngleBoundFFSolverData<Eigen::MatrixXd, Eigen::MatrixXi>(V, F);
    angle_bound_pvf = smooth_pvf;


    igl::viewer::Viewer viewer;


    // Launch the viewer
    viewer.data.clear();
    viewer.data.set_mesh(V, F);
    viewer.core.show_lines = false;
    viewer.core.show_texture = false;
    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;
    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;

    // Plot the original mesh with a texture parametrization
    key_down(viewer, '1', 0);

    viewer.launch();
}





#else
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/viewer/Viewer.h>
#include <igl/barycenter.h>
#include <igl/avg_edge_length.h>
#include <vector>
#include <igl/n_polyvector_general.h>
#include <igl/n_polyvector.h>
#include <igl/local_basis.h>
#include <igl/writeOFF.h>
#include <stdlib.h>
#include <igl/jet.h>
#include <fstream>
#include <iostream>
// Input mesh
Eigen::MatrixXd V;
Eigen::MatrixXi F;

// Per face bases
Eigen::MatrixXd B1,B2,B3;

// Face barycenters
Eigen::MatrixXd B;

// Scale for visualizing the fields
double global_scale;

// Random length factor
double rand_factor = 5;

Eigen::VectorXi samples;

void readSamples(const std::string &fname, Eigen::VectorXi &samples)
{
  int numSamples;
  FILE *fp = fopen(fname.c_str(),"r");
  if (fscanf(fp, "%d", &numSamples)!=1)
  {
    fclose(fp);
    return;
  }
  samples.resize(numSamples,1);
  int vali;
  for (int i =0; i<numSamples; ++i)
  {
    if (fscanf(fp, "%d", &vali)!=1 || vali<0)
    {
      fclose(fp);
      samples.resize(0,1);
      return;
    }
    samples[i]=vali;
  }
  fclose(fp);
  
}

// Create a random set of tangent vectors
Eigen::VectorXd random_constraints(const
                                   Eigen::VectorXd& b1, const
                                   Eigen::VectorXd& b2, int n)
{
  Eigen::VectorXd r(n*3);
  for (unsigned i=0; i<n;++i)
  {
    double a = (double(rand())/RAND_MAX)*2*M_PI;
    double s = 1 + ((double(rand())/RAND_MAX)) * rand_factor;
    Eigen::Vector3d t = s * (cos(a) * b1 + sin(a) * b2);
    r.block(i*3,0,3,1) = t;
  }
  return r;
}

bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier)
{
  using namespace std;
  using namespace Eigen;

  if (key <'1' || key >'9')
    return false;

  viewer.data.lines.resize(0,9);

  int num = key  - '0';

  // Interpolate
  std::cerr << "Interpolating " << num  << "-PolyVector field" << std::endl;

  VectorXi b(3);
  b << 1511, 603, 506;

  int numConstraintsToGenerate;
  // if it's not a 2-PV or a 1-PV, include a line direction (2 opposite vectors)
  // in the field
  if (num>=5)
    numConstraintsToGenerate  = num-2;
  else
    if (num>=3)
      numConstraintsToGenerate  = num-1;
    else
      numConstraintsToGenerate  = num;


  MatrixXd bc(b.size(),numConstraintsToGenerate*3);
  for (unsigned i=0; i<b.size(); ++i)
  {
    VectorXd t = random_constraints(B1.row(b(i)),B2.row(b(i)),numConstraintsToGenerate);
    bc.row(i) = t;
  }
  VectorXi rootsIndex(num);
  for (int i =0; i<numConstraintsToGenerate; ++i)
    rootsIndex[i] = i+1;
  if (num>=5)
    rootsIndex[num-2] = -2;
    if (num>=3)
      rootsIndex[num-1] = -1;

  // Interpolated PolyVector field
  Eigen::MatrixXd pvf;
  igl::n_polyvector_general(V, F, b, bc, rootsIndex, pvf);

  ofstream ofs;
  ofs.open("pvf.txt", ofstream::out);
  ofs<<pvf;
  ofs.close();
  igl::writeOFF("pvf.off",V,F);
  
  // Highlight in red the constrained faces
  MatrixXd C = MatrixXd::Constant(F.rows(),3,1);
  for (unsigned i=0; i<b.size();++i)
    C.row(b(i)) << 1, 0, 0;
  viewer.data.set_colors(C);

  for (int n=0; n<num; ++n)
  {
//    const MatrixXd &VF = pvf.block(0,n*3,F.rows(),3);
    MatrixXd VF = MatrixXd::Zero(F.rows(),3);
    for (unsigned i=0; i<b.size(); ++i)
      VF.row(b[i]) = pvf.block(b[i],n*3,1,3);
    
    for (int i=0; i<samples.rows(); ++i)
      VF.row(samples[i]) = pvf.block(samples[i],n*3,1,3);
    
    VectorXd c = VF.rowwise().norm();
    MatrixXd C2;
    igl::jet(c,1,1+rand_factor,C2);
    // viewer.data.add_edges(B - global_scale*VF, B + global_scale*VF , C2);
    viewer.data.add_edges(B, B + global_scale*VF , C2);
  }


  return false;
}

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;
  // Load a mesh in OBJ format
  igl::readOBJ(TUTORIAL_SHARED_PATH "/snail.obj", V, F);
  readSamples(TUTORIAL_SHARED_PATH "/snail.samples.0.2", samples);

  // Compute local basis for faces
  igl::local_basis(V,F,B1,B2,B3);

  // Compute face barycenters
  igl::barycenter(V, F, B);

  // Compute scale for visualizing fields
  global_scale =  .1*igl::avg_edge_length(V, F);

  // Make the example deterministic
  srand(0);

  igl::viewer::Viewer viewer;
  viewer.data.set_mesh(V, F);
  viewer.callback_key_down = &key_down;
  viewer.core.show_lines = false;
  key_down(viewer,'3',0);
  std::cerr << " *** Press keys 1-9 to select number of vectors per point. ***" << std::endl;
  
  viewer.launch();
}
#endif