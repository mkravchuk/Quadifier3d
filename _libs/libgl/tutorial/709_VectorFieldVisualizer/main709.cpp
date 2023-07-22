#include <igl/barycenter.h>
#include <igl/edge_topology.h>
#include <igl/local_basis.h>
#include <igl/parula.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/polyvector_field_matchings.h>
#include <igl/read_triangle_mesh.h>
#include <igl/readOFF.h>
#include <igl/slice.h>
#include <igl/sort_vectors_ccw.h>
#include <igl/streamlines.h>
#include <igl/copyleft/comiso/nrosy.h>
#include <igl/viewer/Viewer.h>

#include <cstdlib>
#include <iostream>
#include <vector>
#include <fstream>
#include <igl/readOBJ.h>


// Mesh
Eigen::MatrixXd V;
Eigen::MatrixXi F;

igl::StreamlineData sl_data;
igl::StreamlineState sl_state;

int degree;         // degree of the vector field
int half_degree;    // degree/2 if treat_as_symmetric
bool treat_as_symmetric = true;

int anim_t = 0;
int anim_t_dir = 1;


void representative_to_nrosy(
        const Eigen::MatrixXd &V,
        const Eigen::MatrixXi &F,
        const Eigen::MatrixXd &R,
        const int N,
        Eigen::MatrixXd &Y)
{
    using namespace Eigen;
    using namespace std;
    MatrixXd B1, B2, B3;

    igl::local_basis(V, F, B1, B2, B3);

    Y.resize(F.rows(), 3 * N);
    for (unsigned i = 0; i < F.rows(); ++i)
    {
        double x = R.row(i) * B1.row(i).transpose();
        double y = R.row(i) * B2.row(i).transpose();
        double angle = atan2(y, x);

        for (unsigned j = 0; j < N; ++j)
        {
            double anglej = angle + M_PI * double(j) / double(N);
            double xj = cos(anglej);
            double yj = sin(anglej);
            Y.block(i, j * 3, 1, 3) = xj * B1.row(i) + yj * B2.row(i);
        }
    }

    /*Y.resize(F.rows()*N, 3);
    for (unsigned i = 0; i < F.rows(); ++i)
    {
        double x = R.row(i) * B1.row(i).transpose();
        double y = R.row(i) * B2.row(i).transpose();
        double angle = atan2(y, x);

        for (unsigned j = 0; j < N; ++j)
        {
            double anglej = angle + 2 * M_PI*double(j) / double(N);
            double xj = cos(anglej);
            double yj = sin(anglej);
            Y.row(i*N + j) = xj * B1.row(i) + yj * B2.row(i);
        }
    }*/
}

bool pre_draw(igl::viewer::Viewer &viewer)
{
    using namespace Eigen;
    using namespace std;

    if (!viewer.core.is_animating)
        return false;

    igl::streamlines_next(V, F, sl_data, sl_state);
    Eigen::RowVector3d color = Eigen::RowVector3d::Zero();
    double value = ((anim_t) % 100) / 100.;

    if (value > 0.5)
        value = 1 - value;
    value = value / 0.5;
    igl::parula(value, color[0], color[1], color[2]);

    viewer.data.add_edges(sl_state.start_point, sl_state.end_point, color);

    anim_t += anim_t_dir;

    return false;
}

bool key_down(igl::viewer::Viewer &viewer, unsigned char key, int modifier)
{
    if (key == ' ')
    {
        viewer.core.is_animating = !viewer.core.is_animating;
        return true;
    }
    return false;
}

using namespace std;
using namespace Eigen;
Point3d GetFaceCentroid(int faceId, const MatrixXi& F, const MatrixXd& V)
{
    Point3d sum(0, 0, 0);
    for (int j = 0; j < F.cols(); j++)
    {
        // Accumulate
        sum += V.row(F(faceId, j));
    }
    // average
    return sum /= double(F.cols());
}
Vector3d GetEdgeNormalToFaceCentroid(int edgeId, const Point3d& faceCentroid, const MatrixXi& EV, const MatrixXd& V)
{
    Point3d P1 = V.row(EV(edgeId, 0));
    Point3d P2 = V.row(EV(edgeId, 1));
    // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
    Vector3d u = P2 - P1;
    Vector3d PQ = faceCentroid - P1;
    Vector3d w2 = PQ - u*(PQ.dot(u) / u.squaredNorm());
    Point3d closestPoint = faceCentroid - w2;
    Vector3d normalOfEdgeToFace = (closestPoint - faceCentroid).normalized();
    return normalOfEdgeToFace;
}
Vector3d GetEdgeNormalToFaceCentroid(int edgeId, int faceId, const MatrixXi& F, const MatrixXi& EV, const MatrixXd& V)
{
    Vector3d faceCentroid = GetFaceCentroid(faceId, F, V);
    return GetEdgeNormalToFaceCentroid(edgeId, faceCentroid, EV, V);
}
void detectBorderConstraines(
    VectorXi& b,            //constraints - face indexes
    MatrixXd& bc,          //constraints - constrain directions
    VectorXd& weights    //constraints - weights
)
{
    // Edge Topology
    MatrixXi EV; // Relation Edge to Verex (each edge has 2 vertex indexes)
    MatrixXi FE; // Relation Face to Edge (each face has 3 edge indexes)
    MatrixXi EF; // Relation Edge to Face (each edge has 2 faces indexes, second will be -1 if edge has only 1 face)
    vector<bool> isBorderEdge;
    igl::edge_topology(V, F, EV, FE, EF); // must be called after SplitNakedTriangleWith2NakedEdge and CorrectFacesEdgeIndexes since they change V and F
    isBorderEdge.resize(EV.rows());
    for (unsigned i = 0; i < EV.rows(); ++i)
        isBorderEdge[i] = (EF(i, 0) == -1) || ((EF(i, 1) == -1));


    // Select boundary faces
    vector<int> nakedFaces;
    vector<int> nakedEdges;
    for (unsigned eid = 0; eid < EF.rows(); ++eid) // For every border edge 
    {
        if (isBorderEdge[eid])
        {
            int faceId = EF(eid, 0);
            if (faceId == -1) faceId = EF(eid, 1);
            // add faces that have only border 1 edge (for 2 border edges we need to think...)
            int nakedEdgesCount = 0;
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                auto eid2 = FE(faceId, edgeIndex);
                if (EF(eid2, 0) == -1 || EF(eid2, 1) == -1) nakedEdgesCount++;
            }
            if (nakedEdgesCount == 1)
            {
                // if (faceId == 60){
                nakedFaces.push_back(faceId);
                nakedEdges.push_back(eid);
                //                break; //DEBUG - limit to only 1 triangle
                //}
            }
        }
    }
    //sort(edgeFaces.begin(), edgeFaces.end()); //DEBUG - sort for better debugging

    // Convert to eigen vector
    b = VectorXi::Map(nakedFaces.data(), nakedFaces.size());

    bc.resize(b.size(), 3);
    bc.resize(b.size(), 3);

    // Set weights
    weights = VectorXd::Constant(bc.rows(), 1, 1);//by default make all weights equal to 1

                                                  // Detect vector constrain for each face
    bc.resize(b.size(), 3);
    //foreach naked edge detect constrain direction
    for (unsigned nakedId = 0; nakedId < nakedEdges.size(); ++nakedId)
    {
        auto faceId = nakedFaces[nakedId];
        auto edgeId = nakedEdges[nakedId];
        Vector3d normalToFaceCentroid = GetEdgeNormalToFaceCentroid(edgeId, faceId, F, EV, V);
        bc.row(nakedId) = normalToFaceCentroid;
        //weights(nakedId) = 1-1.0*nakedId/nakedEdges.size();
        //viewer.data.add_points(P1, RowVector3d(1, 0, 0)); //DEBUG
        //viewer.data.add_points(P2, RowVector3d(0, 0, 1)); //DEBUG
        //viewer.data.add_points(point, RowVector3d(0, 0, 1)); //DEBUG

        //bc.row(nakedId) << 0, 0, 1; //DEBUG set any value
    }


    //TODO: set weights depend on edge length
}


int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace std;


    // Load a mesh in OFF format
    //igl::readOFF(TUTORIAL_SHARED_PATH "/bumpy.off", V, F);
    igl::readOBJ("K:/Mesh/TestObj/qq2_low.obj", V, F);
    //igl::readOBJ("K:/Mesh/TestObj/korzyna_small_1_medium.obj", V, F);


    Eigen::VectorXd S; // unused
    Eigen::MatrixXd R, Y;

    // Create a Vector Field
    //Eigen::VectorXi b;
    //Eigen::MatrixXd bc;
    //b.resize(1);
    //b << 0;
    //bc.resize(1, 3);
    //bc << 1, 1, 1;
    //igl::copyleft::comiso::nrosy(V, F, b, bc, VectorXi(), VectorXd(), MatrixXd(), 1, 0.5, temp_field, S);


    // border constrained
    VectorXi b;
    MatrixXd bc;
    VectorXd bw;
    detectBorderConstraines(b, bc, bw);
    igl::copyleft::comiso::nrosy(V, F, VectorXi(), MatrixXd(), b,bw, bc, 4, 0.5, R, S);


    representative_to_nrosy(V, F, R, 1, Y);
    treat_as_symmetric = false;
    igl::streamlines_init(V, F, Y, treat_as_symmetric, sl_data, sl_state);


    // Viewer Settings
    igl::viewer::Viewer viewer;
    viewer.data.set_mesh(V, F);
    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;

    viewer.core.show_lines = false;

    viewer.core.is_animating = false;
    viewer.core.animation_max_fps = 30.;

    // Paint mesh grayish
    Eigen::MatrixXd C;
    C.setConstant(viewer.data.V.rows(), 3, .3);
    viewer.data.set_colors(C);


    // Draw vector field on sample points
    igl::StreamlineState sl_state0;
    sl_state0 = sl_state;
    igl::streamlines_next(V, F, sl_data, sl_state0);
    Eigen::MatrixXd v = sl_state0.end_point - sl_state0.start_point;
    v.rowwise().normalize();

    viewer.data.add_edges(sl_state0.start_point,
        //sl_state0.start_point + 0.2 * v,
        sl_state0.end_point,
                          Eigen::RowVector3d::Constant(1.0f));

    cout <<
    "Press [space] to toggle animation" << endl;
    viewer.launch();
}
