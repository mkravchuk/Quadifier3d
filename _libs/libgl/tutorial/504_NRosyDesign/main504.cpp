#include "stdafx.h"

using namespace Eigen;
using namespace std;

igl::viewer::Viewer viewer;
void nRosySolve();
 

/*********************************************************/
/**** NROSY  FIELD ***************************************/
/*********************************************************/

enum class NRosyNum {
    N1, N2, N3, N4, N5, N6, N7, N8, N9, N10, N11, N12, N13, N14, N15, N16
};
NRosyNum nRosy_N = NRosyNum::N4;
NRosySolver nRosy_Solver = NRosySolver::multiple_rounding;
double nRosy_SoftPercent = 0.5;

bool AddSolverProperties(igl::viewer::Viewer& viewer)
{
    // Add as separate window
    //viewer.ngui->addWindow(Eigen::Vector2i(320, 10), "Solver properties");
    // Add new group
    viewer.ngui->addGroup("NRosy");

    auto calbackOptionChanged = []{
        nRosySolve();
    };
    viewer.ngui->addVariable<NRosySolver>("Solver", nRosy_Solver, calbackOptionChanged)->setItems({ "SimplicialLDLT","gurobi","cplex","no rounding", "direct rounding", "multiple rounding", "iterative" });
//    viewer.ngui->addVariable<NRosySolver>("Solver", nRosy_Solver, calbackOptionChanged)->setItems({ "SimplicialLDLT","gurobi","cplex","EigenLDLT (no rounding)", "SimplicialCholesky", "COMISO", "Cholesky" });
    viewer.ngui->addVariable<NRosyNum>("N", nRosy_N, calbackOptionChanged)->setItems({ "1","2","Triple","Quad", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16" });
    viewer.ngui->addVariable("Smoothness / Edges", nRosy_SoftPercent, calbackOptionChanged);

    // Add a button
    //viewer.ngui->addButton("Print Hello", []() { std::cout << "Hello\n"; });

    // call to generate menu
    viewer.screen->performLayout();     
     
    int i = 2;   
    return false;   
};


void detectBorderConstraines(
    igl::copyleft::comiso::NRosyField& solver,
    VectorXi& b,            //constraints - face indexes
    MatrixXd& bc,          //constraints - constrain directions
    VectorXd& weights    //constraints - weights
)
{
    // Select boundary faces
    vector<int> nakedFaces;
    vector<int> nakedEdges;
    Eigen::MatrixXi EF = solver.EF;
    for (unsigned eid = 0; eid < EF.rows(); ++eid) // For every border edge 
    {
        if (solver.isBorderEdge[eid])
        {
            int faceId = EF(eid, 0);
            if (faceId == -1) faceId = EF(eid, 1);
            // add faces that have only border 1 edge (for 2 border edges we need to think...)
            int nakedEdgesCount = 0;
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                auto eid2 = solver.FE(faceId, edgeIndex);
                if (EF(eid2, 0) == -1 || EF(eid2, 1) == -1) nakedEdgesCount++;
            }
            if (nakedEdgesCount == 1) {
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
    //get barycenters
    MatrixXd Barycenters;
    igl::barycenter(solver.V, solver.F, Barycenters);
    //foreach naked edge detect constrain direction
    for (unsigned nakedId = 0; nakedId < nakedEdges.size(); ++nakedId)
    {
        auto faceId = nakedFaces[nakedId];
        auto edgeId = nakedEdges[nakedId];
        Vector3d faceBarycenter = Barycenters.row(faceId);
        Vector3d P1 = solver.V.row(solver.EV(edgeId, 0));
        Vector3d P2 = solver.V.row(solver.EV(edgeId, 1));
        //Vector3d edgeMiddlePoint = (P1 + P2) / 2;
        //Vector3d direction = (faceBarycenter - edgeMiddlePoint).normalized();

        // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
        Vector3d u = P2 - P1;
        Vector3d PQ = faceBarycenter - P1;
        Vector3d w2 = PQ - u*(PQ.dot(u) / u.squaredNorm());
        Vector3d closestPoint = faceBarycenter - w2;

        Vector3d direction = (closestPoint - faceBarycenter).normalized();
        bc.row(nakedId) = direction;
        //weights(nakedId) = 1-1.0*nakedId/nakedEdges.size();
        //viewer.data.add_points(P1, RowVector3d(1, 0, 0)); //DEBUG
        //viewer.data.add_points(P2, RowVector3d(0, 0, 1)); //DEBUG
        //viewer.data.add_points(point, RowVector3d(0, 0, 1)); //DEBUG

        //bc.row(nakedId) << 0, 0, 1; //DEBUG set any value
    }


    //TODO: set weights depend on edge length
}

IGL_INLINE void CalculateMIQNrosyField(
    const MatrixXd& V,           // Vectors
    const MatrixXi& F,            // Faces
    bool autoDetectHardConstraines,// if true - b and bc will be autodetected
    VectorXi& b,                    //hard constraints - face indexes
    MatrixXd& bc,                  //hard constraints - face vector
    bool autoDetectSoftConstraines,// if true - b and bc will be autodetected
    VectorXi& b_soft,     //soft constraints - face indexes
    VectorXd& w_soft,    //soft constraints - weight
    MatrixXd& bc_soft,   //soft constraints - face vector  
    const int N,                     //N-rosy dimension
    const double soft,             //Set the ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    MatrixXd& R,                   //Result - cross-field
    VectorXd& S                     //Result - singularities
)
{
    // Init solver
    cout << endl << "-------------------" << endl;
    cout << "----- Init solver -----" << endl;
    igl::copyleft::comiso::NRosyField solver(V, F);

    // Add hard constraints
    if (autoDetectHardConstraines)
    {
        VectorXd wTemp; // temporaly variable
        detectBorderConstraines(solver, b, bc, wTemp);
    }
    for (unsigned i = 0; i < b.size(); ++i)
    {
        solver.setConstraintHard(b(i), bc.row(i));
    }

    // Add soft constraints
    if (autoDetectSoftConstraines)
    {
        detectBorderConstraines(solver, b_soft, bc_soft, w_soft);
    }
    for (unsigned i = 0; i < b_soft.size(); ++i)
    {
        solver.setConstraintSoft(b_soft(i), w_soft(i), bc_soft.row(i));
    }

    // Set the soft constraints global weight
    solver.setSoftAlpha(soft);

    // Interpolate
    cout << "----- Solving -----" << endl;
    solver.solve(N, nRosy_Solver);

    cout << "----- Extracting results -----" << endl;
    // Copy the result back
    R = solver.getFieldPerFace();

    // Extract singularity indices
    S = solver.getSingularityIndexPerVertex();
}


/*********************************************************/
/**** FILE ************************************************/
/*********************************************************/
MatrixXd V;     // Mesh verticies
MatrixXi F;      // Mesh faces

enum class TestFiles
{
    circle, circle1, circle2, test3, korzyna_small_1, korzyna_small_2, qq1, qq2, quad, ex1, ex2, trymach1
};
vector<string>  TestFilesStr = { "circle", "circle1", "circle2", "test3", "korzyna_small_1", "korzyna_small_2" , "qq1", "qq2", "quad", "ex1", "ex2", "trymach1" };

enum class TestFilesDensity
{
    low, medium, high
};
vector<string>  TestFilesDensityStr = { "low", "medium", "high" };

TestFiles TestFile = TestFiles::qq2;
TestFilesDensity TestFileDensity = TestFilesDensity::low;

string GetTestFileName()
{
    return "K:\\Mesh\\TestObj\\" + TestFilesStr[(int)TestFile] + "_" + TestFilesDensityStr[(int)TestFileDensity] + ".obj";
}


void CorrectFacesEdgeIndexes()
{
    // Generate topological relations
    cout << "Generating topological relations for edges ...";
    Eigen::MatrixXi EV, FE, EF;
    std::vector<bool> isBorderEdge;
    igl::edge_topology(V, F, EV, FE, EF);
    // Flag border edges
    isBorderEdge.resize(EV.rows());
    for (unsigned i = 0; i < EV.rows(); ++i)
        isBorderEdge[i] = (EF(i, 0) == -1) || ((EF(i, 1) == -1));
    cout << "done." << endl;


    cout << "Select boundary faces ...";
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
            if (nakedEdgesCount == 1) {
                // if (faceId == 60){
                nakedFaces.push_back(faceId);
                nakedEdges.push_back(eid);
                //                break; //DEBUG - limit to only 1 triangle
                //}
            }
        }
    }
    cout << "done." << endl;


    for (unsigned nakedId = 0; nakedId < nakedEdges.size(); ++nakedId)
    {
        auto faceId = nakedFaces[nakedId];
        auto edgeId = nakedEdges[nakedId];

        auto ev1 = EV(edgeId, 0);
        auto ev2 = EV(edgeId, 1);
        auto fv1 = F.row(faceId)(0);
        auto fv2 = F.row(faceId)(1);
        auto fv3 = F.row(faceId)(2);
        int antiForeverLoopFlag = 0;
        while (!((ev1 == fv1 && ev2 == fv2) || (ev2 == fv1 && ev1 == fv2)) && antiForeverLoopFlag < 3) // shift vertexes until naked edge will be first
        {
            F.row(faceId) << fv2, fv3, fv1;
            antiForeverLoopFlag++;
        }
    }
}

void LoadFile(igl::viewer::Viewer& viewer, string filename, bool alignCamera)
{
    viewer.data.clear();

    string title = "SolidQUAD - " + filename;
    glfwSetWindowTitle(viewer.window, title.c_str());

    igl::readOBJ(filename, V, F);
    //CorrectFacesEdgeIndexes();
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/circle1.obj", V, F);
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/circle_0.03.obj", V, F); //high density mesh
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/circle2.obj", V, F); // small
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/qq_1_0.2.obj", V, F);//high density mesh
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/qq_1_0.3.obj", V, F);
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/qq_2_0.25.obj", V, F);//high density mesh
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/qq_2_0.7.obj", V, F);
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/korzyna_small_1_0.16.obj", V, F);
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/korzyna_small_1_0.01.obj", V, F);//high density mesh
    viewer.data.set_mesh(V, F);
    if (alignCamera) {
        viewer.core.align_camera_center(V, F);
    }
}

bool AddFileProperties(igl::viewer::Viewer& viewer)
{
    // Add as separate window
    //viewer.ngui->addWindow(Eigen::Vector2i(320, 10), "File properties");
    // Add new group
    viewer.ngui->addGroup("File");
    
    // File name
    viewer.ngui->addVariable<TestFiles>("Obj", [&](TestFiles file)
    {
        TestFile = file;
        LoadFile(viewer, GetTestFileName(), true);
        nRosySolve();
    }, [&]()
    {
        return TestFile;
    })->setItems(TestFilesStr);

    // File density
    viewer.ngui->addVariable<TestFilesDensity>("Density", [&](TestFilesDensity density)
    {
        TestFileDensity = density;
        LoadFile(viewer, GetTestFileName(), false);
        nRosySolve();
    }, [&]()
    {
        return TestFileDensity;
    })->setItems(TestFilesDensityStr);

    // Expose a variable directly ...
    //viewer.ngui->addVariable("Elapsed ms: ", Elapsed);

    // Add a button
    //viewer.ngui->addButton("Print Hello", []() { std::cout << "Hello\n"; });

    // call to generate menu
    viewer.screen->performLayout();
    return false;
};


/*********************************************************/
/*********************************************************/
/*********************************************************/



// Converts a representative vector per face in the full set of vectors that describe an N-RoSy field
void representative_to_nrosy(const MatrixXd& V, const MatrixXi& F, const MatrixXd& R, const int N, MatrixXd& Y)
{
    using namespace Eigen;
    using namespace std;
    MatrixXd B1, B2, B3;

    igl::local_basis(V, F, B1, B2, B3);

    Y.resize(F.rows()*N, 3);
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
    }
}

// Plots the mesh with an N-RoSy field and its singularities on top
// The constrained faces (b) are colored in red.
void plot_mesh_nrosy(
    igl::viewer::Viewer& viewer,
    MatrixXd& V,
    MatrixXi& F,
    int N,
    MatrixXd& PD1,
    VectorXd& S,
    VectorXi& b)
{
    cout << "Plot mesh ...";

    // Expand the representative vectors in the full vector set and plot them as lines
    double avg = igl::avg_edge_length(V, F);
    MatrixXd Y;
    representative_to_nrosy(V, F, PD1, N, Y);

    MatrixXd B;
    igl::barycenter(V, F, B);

    MatrixXd Be(B.rows()*N, 3);
    for (unsigned i = 0; i < B.rows(); ++i)
        for (unsigned j = 0; j < N; ++j)
            Be.row(i*N + j) = B.row(i);

    viewer.data.add_edges(Be, Be + Y*(avg / 2), RowVector3d(0, 1, 0));

    // Plot the singularities as colored dots (red for negative, blue for positive)
    for (unsigned i = 0; i < S.size(); ++i)
    {
        if (S(i) < -0.001)
            viewer.data.add_points(V.row(i), RowVector3d(1, 0, 0));
        else if (S(i) > 0.001)
            viewer.data.add_points(V.row(i), RowVector3d(0, 1, 0));
    }

    // Highlight in red the constrained faces
    MatrixXd bndColor = MatrixXd::Constant(F.rows(), 3, 0.8);//by default make all faces of silver color
    for (unsigned i = 0; i < b.size(); ++i) {
        bndColor.row(b(i)) << 1, 0, 0; // for boundary faces - use red color
        //auto vertexIndex1 = F.row(b(i))(0);
        //auto vertexIndex2 = F.row(b(i))(1);
        //viewer.data.add_edges(V.row(vertexIndex1), V.row(vertexIndex2), RowVector3d(1, 0, 0));
    }
    viewer.data.set_colors(bndColor);  
    cout << "done." << endl;      
}

void nRosySolve()
{
    COMISO::StopWatch sw, sw2; sw.start(); sw2.start();
    sw.start();

    auto V = viewer.data.V;
    auto F = viewer.data.F;
    VectorXi b;      // Constrained faces id
    MatrixXd bc;    // Cosntrained faces representative vector
    VectorXi bSoft;      // Constrained faces id
    VectorXd bSoftWieghts;    // Constrained faces weights
    MatrixXd bcSoft;    // Cosntrained faces representative vector
    MatrixXd R;
    VectorXd S;

    

    // Clear the mesh to redraw cross-field
    viewer.data.clear();
    viewer.data.set_mesh(V, F);

//    CalculateMIQNrosyField(V, F, true, b, bc, true, VectorXi(), VectorXd(), MatrixXd(), (int)nRosy_N + 1, nRosy_SoftPercent, R, S);
    CalculateMIQNrosyField(V, F, false, VectorXi(), MatrixXd(), true, bSoft, bSoftWieghts, bcSoft, (int)nRosy_N + 1, nRosy_SoftPercent, R, S);
    b = bSoft;

    //plot_mesh_nrosy(viewer, V, F, N, R, S, b);
    plot_mesh_nrosy(viewer, V, F, 1, R, S, b);// draw just one line instead of N - this is more undestandable
    double time = sw.stop() / 1000.0; sw.start();
    cout << "Total              " << time << std::endl << std::endl;
}

// It allows to change the degree of the field when a number is pressed
bool viewer_callback_key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier)
{

    // Change nRosy_N
    if (modifier == 0)
    switch (key)
    {
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case '0':
        if (key == '1') nRosy_N = NRosyNum::N1;
        if (key == '2') nRosy_N = NRosyNum::N2;
        if (key == '3') nRosy_N = NRosyNum::N3;
        if (key == '4') nRosy_N = NRosyNum::N4;
        if (key == '5') nRosy_N = NRosyNum::N5;
        if (key == '6') nRosy_N = NRosyNum::N6;
        if (key == '7') nRosy_N = NRosyNum::N7;
        if (key == '8') nRosy_N = NRosyNum::N8;
        if (key == '9') nRosy_N = NRosyNum::N9;
        if (key == '0') nRosy_N = NRosyNum::N10;
        viewer.ngui->refresh();
        nRosySolve();
        break;
    }

    // Change TestFileDensity
    if (modifier == 0)
    switch (key)
    {
    case 'Q':
    case 'W':
    case 'E':
        if (key == 'Q') TestFileDensity = TestFilesDensity::low;
        if (key == 'W') TestFileDensity = TestFilesDensity::medium;
        if (key == 'E') TestFileDensity = TestFilesDensity::high;
        viewer.ngui->refresh();
        LoadFile(viewer, GetTestFileName(), false);
        nRosySolve();
        break;
    }

    //viewer.screen->performLayout();
    return false;
}

bool viewer_callback_key_pressed(igl::viewer::Viewer &viewer, unsigned int unicode_key, int modifiers)
{
    //switch (unicode_key)
    //{
    //case 'X':
    //case 'x':
    //{
    //    return true;
    //}
    //}
    return false;
}

bool AddTestProperties_(igl::viewer::Viewer& viewer)
{
    // AddSolverProperties
    using nanogui::Alignment;
    using nanogui::Arcball;
    using nanogui::BoxLayout;
    using nanogui::Button;
    using nanogui::CheckBox;
    using nanogui::Color;
    using nanogui::ComboBox;
    using nanogui::GLFramebuffer;
    using nanogui::GroupLayout;
    using nanogui::ImagePanel;
    using nanogui::Label;
    using nanogui::MessageDialog;
    using nanogui::Orientation;
    using nanogui::Popup;
    using nanogui::PopupButton;
    using nanogui::ProgressBar;
    using nanogui::Screen;
    using nanogui::Slider;
    using nanogui::TextBox;
    using nanogui::ToolButton;
    using nanogui::VScrollPanel;
    using nanogui::Widget;
    using nanogui::Window;

    //viewer.screen->size().
    /* Initialize user interface */
    //nanogui::Window *window = viewer.ngui->addWindow(Eigen::Vector2i(0, 0), "Solver properties");
    //window->setLayout(new GroupLayout());
    //window->setId("SolverProps");
    //window->setPosition(Vector2i(15, 15));
    viewer.ngui->addGroup("Some group");
    nanogui::Window *window = viewer.ngui->addWindow(Eigen::Vector2i(0, 0), "Some window");
    window->setLayout(new GroupLayout());
    window->setPosition(Vector2i(1500, 0));
    viewer.screen->performLayout();  
    return false;
}

bool viewer_callback_init(igl::viewer::Viewer& viewer)
{
    //AddTestProperties_(viewer);
    AddSolverProperties(viewer);
    AddFileProperties(viewer);
    viewer.screen->performLayout();
    LoadFile(viewer, GetTestFileName(), true);
    nRosySolve();
    return false;
};

int main(int argc, char *argv[])
{
    // Set viewer callbacks
    viewer.callback_key_down = &viewer_callback_key_down;
    viewer.callback_init = &viewer_callback_init;
    //viewer.callback_key_pressed = &callback_key_pressed;

    // Disable wireframe
    viewer.core.show_lines = false;

    // Interpolate the field and plot
    //viewer_callback_key_down(viewer, '1', 0);

    // Launch the viewer
    viewer.launch(true, false, true, "SolidQUAD");
}










