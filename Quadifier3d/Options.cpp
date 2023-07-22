#include "stdafx.h"
#include "Options.h"
#include "_MeshLogicOptions.h"
#include "OpenGL_Options.h"
#include <omp.h>
#include "MeshFile.h"

MeshLogicOptions& options = meshLogicOptions;

bool IsOmpEnabled = false;
bool IsOmpHyperthreadingEnabled = true;                  // default true - improves speed by 10%
bool IsOmpDynamicThreadAdjustmentsEnabled = true; // default true - improves speed by 10%
int OmpCpuCores = 1;

void Options::Update_OmpCpuCores()
{
    omp_set_dynamic(IsOmpDynamicThreadAdjustmentsEnabled);
    if (IsOmpHyperthreadingEnabled)
    {
        OmpCpuCores = omp_get_num_procs();
    }
    else
    {
        OmpCpuCores = omp_get_num_procs() / 2;
    }
    cout << "OMP is set to use " << OmpCpuCores << " cores  (Hyperthreading is " << (IsOmpHyperthreadingEnabled ? "enabled" : "disabled") << ")" << endl;
    omp_set_num_threads(OmpCpuCores);
    cout << "omp_get_num_procs()   =  " << omp_get_num_procs() << endl;
    cout << "omp_get_max_threads() =  " << omp_get_max_threads() << endl;
    cout << "omp_get_dynamic()     =  " << omp_get_dynamic() << endl;
}

//extern "C"
//{
//    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
//    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
//}

void Options::AddOptions(OpenGL_Window& window)
{
    auto addWindow = [&](int x, int y, string title, int fixedWindowWidth = -1, int fixedInputWidth = -1)
    {
        auto w = window.nanogui->addWindow(Vector2i(x, y), title);

        // change margin - make it smaller, since we have to many windows and we have to save space as much as possible
        int margin = 2; // 10 default
        ((AdvancedGridLayout*)w->layout())->setMargin(margin);

        // set fixed window width
        if (fixedWindowWidth != -1)
        {
            w->setFixedWidth(fixedWindowWidth);
        }

        // set fixed input width
        auto fixedSize = window.nanogui->fixedSize();
        if (fixedInputWidth != -1)
        {
            window.nanogui->setFixedSize({ fixedInputWidth,fixedSize.y() });
        }
        else
        {
            window.nanogui->setFixedSize({ 0,fixedSize.y() }); // set default width - dynamic
        }


        return w;
    };

    //ngui->setFixedSize(Vector2i(130, 23)); 

    addWindow(0, 0, "", 400, 100);                    //- dont give title and set at left-top corner
    AddOptions_App(window);                           // general application options 
    AddOptions_File(window);                            // open file options
    //AddOptions_Mesh(window);                       // storing mesh in memory options
    //AddOptions_MeshHeal(window);                       
    //AddOptions_MeshTopology(window);           // connect meshes, defines join partitions for meshes, so we can say that meshA joining meshB on partitionX (partition is part of Loop curve)
    //AddOptions_Constrain(window);                 // constrains for solver (based on each border-traingle)
    //AddOptions_SolverNrosy(window, true, true);    // general options for solvers
    //AddOptions_SolverNrosy__short(window, true);
    //AddOptions_SolverAngleBound(window);            // 
    //AddOptions_SolverAngleBound__short(window, false);    // 
    //AddOptions_ConstrainUV(window);                 // constrains for solver (based on each border-traingle)
    //AddOptions_SolverUV(window);                      // 
    //AddOptions_SolverUV__short(window, true);    // 
    //AddOptions_LDLTFloatSolver(window);        // fastest linear equations solver that is used in this app (solves 2 equations at once)
    //AddOptions_MeshLinearSolver(window);      // experimental solver, but doesnt work well
    //AddOptions_LinearEquationSolver(window); // helps choise solver from few available (options visible only if 'alternative_solvers_SUPPORTED' defined in _MeshLogicOptions.h)
    //AddOptions_Divider(window);                    // produces stream based on singularity and sharp points
    //AddOptions_Divider__short(window);
    //AddOptions_StreamAdjuster(window);         // shifts streams to meet them exactly in joining point so joining is smooth  
    AddOptions_StreamTopology(window);
    //AddOptions_Joiner(window);                      // joins streams produced by divider (ucommon_i2 == 0ses StreamAdjuster algorithm)
    //AddOptions_Joiner___short(window);                      // joins streams produced by divider (ucommon_i2 == 0ses StreamAdjuster algorithm)
    //AddOptions_DividerIterator(window);            // iterates joining streams between meshes (so stream can travel from mesh to mesh)
    //AddOptions_DividerIterator__short(window);            // iterates joining streams between meshes (so stream can travel from mesh to mesh)
    AddOptions_DividerLogicConnector(window);           // make logic connections for streams loops
    //AddOptions_DividerOptimalConnector(window);           // finds optimal connections for streams between  meshes
    //AddOptions_MeshCutter(window);             // divides single surface to few surfaces base on streams (that produces divider)


      
    addWindow(404, 0, "");
    //AddOptions_Mesher(window);                    // mesh file - final algorithm to convert quad surfaces to quad mesh
    AddOptions_Mesher__short(window);                    // mesh file - final algorithm to convert quad surfaces to quad mesh
    AddOptions_Joiner___short(window);                      // joins streams produced by divider (ucommon_i2 == 0ses StreamAdjuster algorithm)
    AddOptions_DividerIterator__short(window);            // iterates joining streams between meshes (so stream can travel from mesh to mesh)
    AddOptions_SolverNrosy__short(window, false);
    AddOptions_SolverAngleBound__short(window, false);    // 

    addWindow(750, 0, "Preset Test Files");
    AddOptions_TestFilesPreset(window);                    // mesh file - final algorithm to convert quad surfaces to quad mesh

    //addWindow(1712 - 260 - 260, 0, "OpenGL (tech)", 250, 60);
    //AddOptions_OpenGL(window, true, false);
    //addWindow(1712 - 260, 0, "OpenGL (visual)", 250, 60);
    //AddOptions_OpenGL(window, false, true);
    //addWindow(1465, 0, "MeshSimplification", 250, 60);
    //AddOptions_MeshSimplification(window);   // simplify mesh by reducing triangles count - to make drawing faster

    addWindow(1212, 0, "", -1, 100);
    AddOptions_Highlight(window);


    addWindow(1423, 0, "");  //- dont give title and set at left-top corner
    AddOptions_Stream(window);                     // debug options of showing streams (produced by divider, mesher, and other algorithms)


    addWindow(1712, 0, "Viewport");  //- dont give title and set at left-top corner
    AddOptions_GuiStyle(window);
    AddOptions_Viewer_Viewport(window);
    AddOptions_Common(window);
    AddOptions_Viewer_Show(window);
    AddOptions_DrawDebugInfo(window);
}



Options::Options()
{
}

void Options::SetOpenGLOptions()
{
    // Performance
    openglOptions.Performance.use_viewport_layers = true; // improves rotation by 10%-30% for very large models, and decrease rotation by 10% for smaller models
    openglOptions.Performance.use_nanogui_layers = true;  // improves rotation by 10-30%
    openglOptions.Performance.unindex_depth_test = false; // test - slows down rotation by 30% because to many vertexes are calculated - indexes vertexes works faster
    openglOptions.Performance.compactColors = true; // pass color as 1 int instead of 3 floats //TODO implement
    openglOptions.Performance.compactNormals = true; // store normal as 4-th value of position (as position.w) - performance improvement by 1%-3%
    
    //Offsets
    openglOptions.Offsets.wireframe_factor = -1;
    openglOptions.Offsets.wireframe_units = 0;

    // Sizes
    openglOptions.Sizes.point_size = 4;
    openglOptions.Sizes.point_size_MAX = 12; // limit point size (since transparent and solid models has different point sizes)
    openglOptions.Sizes.line_size = 1.6;
    openglOptions.Sizes.lineBold_size = 4;
    openglOptions.Sizes.wireframe_size = 1.2;
    openglOptions.Sizes.wireframe_pointsize_factor = 1.2; // if wireframe is visible - increase size of points to make them more visible
    openglOptions.Sizes.wireframe_linewidth_factor = 1.4; // if wireframe is visible - increase thikness of lines to make them more visible

    // Antialising
    openglOptions.Antialiasing.Enabled = true;
    openglOptions.Antialiasing.LineSmoothingEnabled = true; // smooth lines - very good quality, comparable with MSAA samples = 8
    openglOptions.Antialiasing.LineSmoothingEnabled_inHiddenLayers = true; // improves a bit quality, and with quadro gpu there is no fps dorp down
    openglOptions.Antialiasing.PolygonSmoothingEnabled = false;
    openglOptions.Antialiasing.LineSmoothingEnabled_quality = OpenGLAntialisingQuality::high;

    // Light
    openglOptions.Light.shininess = 35.0; // how bright light emits. Transparent override this value
    openglOptions.Light.shininess_intensity = 0.05; // how big difference from direct and indirect light
    openglOptions.Light.light_position << 0,5,3;  //openglOptions.Light.light_position << 0.0, -0.30, -5.0;
    openglOptions.Light.lighting_factor = 1.5; //the lowest value the lower difference between ortogonal surfaces and rest ones
    openglOptions.Light.alpha_BackSideCoeff = 0.3;// recommended value from 0.5 to 0.8

    // Colors
    openglOptions.Colors.wireframe_blend = true;
    openglOptions.Colors.backSideColorFactor = 1;// how upfront face color will differ from front face color. for example value of 0.5 will half color produced for front face, thus make it more dark

    // Transparency
    openglOptions.Transparent.Enabled = true;
    openglOptions.Transparent.sortTriangles = false;
    openglOptions.Transparent.sortTriangles_showtime_inconsole = false;
    openglOptions.Transparent.sortTriangles_sort_algorithm = OpenGL_Options_Transparent::SortAlgorithm::Batch;

    openglOptions.Transparent.alpha_BackSideCoeff = 0.8;// recommended value from 0.5 to 0.8
    openglOptions.Transparent.shininess = 15.0;// a bit brighter from non-transparent value 35.0f - because on transparent surface light is less visible, so to balance it we need to decrease value
    openglOptions.Transparent.lighting_factor = openglOptions.Light.lighting_factor;// a bit brighter from non-transparent value  - because on transparent surface light is less visible, so to balance it we need to increase value

    openglOptions.Transparent.Layer0Depth.Enabled = true;
    
    openglOptions.Transparent.Layer1Hidden.Enabled = true;
    openglOptions.Transparent.Layer1Hidden.alpha_lines = 0.15; // recommended value from 0.1 to 0.35
    openglOptions.Transparent.Layer1Hidden.alpha_front = 0.1; // recommended value from 0.1 to 0.35
    openglOptions.Transparent.Layer1Hidden.alpha_back = 0.1; // recommended value from 0.1 to 0.35
    openglOptions.Transparent.Layer1Hidden.pointsize_factor = 0.8;  // point size factor for hidden layer

    openglOptions.Transparent.Layer2Visible.Enabled_Front = true;
    openglOptions.Transparent.Layer2Visible.Enabled_Back = true;
    openglOptions.Transparent.Layer2Visible.alpha_front = 0.35; // recommended value from 0.1 to 0.35
    openglOptions.Transparent.Layer2Visible.alpha_back = 0.75; // recommended value from 0.4 to 0.6

    // Texture
    openglOptions.Texture.Enabled = false;
}

void Options::SetViewerOptions(OpenGL_Window& window)
{
    std::cout << std::setprecision(4); // show 4 digits  (float 6 after coma, double 16, long double 32)
    cout << fixed; // avoid priting double numbers  as 3.133-e5   -  just print number as 0.00003133  http://stackoverflow.com/questions/5212018/how-to-make-c-cout-not-use-scientific-notation

   // Triks
    window.showfps = false;
    window.showDrawEventInConsole = false;
    window.skipFastDraws = false; // disable it  beacause it makes flickering - lets user have the smoother speed - lets dont worry about his GPU strongenest
    window.hide_menu = false;
    window.hide_menu_on_mouse = false;

    // Viewport settings
    window.viewport.show_faces = true;
    window.viewport.show_wireframe = false;
    window.viewport.orthographic = false;
    window.viewport.show_XYZ = false;
    window.viewport.show_rotation_XYZ = false;
    window.viewport.show_rotation_center = false;
    window.viewport.show_texture = false;
    window.viewport.rotation_speed = 4.0;
    //window.viewport.show_vertid = false; // obsolete - doesnt work for separated surfaces - use option Draw_ids_xxx
    //window.viewport.show_edgeid = false;// obsolete - doesnt work for separated surfaces - use option Draw_ids_xxx
    //window.viewport.show_faceid = false;// obsolete - doesnt work for separated surfaces - use option Draw_ids_xxx
    //window.viewport.show_srfid = false;// obsolete - doesnt work for separated surfaces - use option Draw_ids_xxx
    //window.viewport.show_srfSerialNumber = false;// obsolete - doesnt work for separated surfaces - use option Draw_ids_xxx
    window.viewport.camera_dnear = 0.1f; // this allow to zoom and see objeczt in details, but dissalow to see object inside. default value is 1. for help see https://open.gl/transformations
    //window.viewport.show_overlay_depth = false; - disabled completelly - now all depend on show_transparent option


     //Colors
    //window.viewport.wireframe_color = Vector4f(.7, .7, .7, 1.0); //  looks ugly when solid is transparent
    window.viewport.wireframe_color = Color4f(0, 0, 0, 0.1f); // look ok in solid and transparent
    //window.viewport.show_vertid_color = Color4d(.0, .0, .0, 0.9);
    //window.viewport.show_edgeid_color = Color4d(.0, .7, .0, 0.6);
    //window.viewport.show_faceid_color = Color4d(.3, .3, .3, 0.7);
    //window.viewport.show_srfid_color = Color4d(.7, .2, .2, 0.9);
    //window.viewport.show_srfSerialNumber_color = Color4d(.7, .2, .2, 0.9);
    window.viewport.labels_default_color = Color4d(.0, 0., 1.0, 1.0);
}

void Options::SetMeshLogicOptions(OpenGL_Window& window)
{
    APPName = "Quadifier3d";
    ThemeColorStyle = NanoguiTheme::NanoguiThemeColorStyle::LightBlue;
    options.showTimeElapsed = true;

    // DRAW Ids
    options.Draw.Mesh.Id_Vertex = false;
    options.Draw.Mesh.Id_Edge = false;
    options.Draw.Mesh.Id_Face = false;
    options.Draw.Mesh.Id_Mesh = false;
    options.Draw.Mesh.Id_MeshLoop = false;
    options.Draw.Mesh.Obj_id = false;
    options.Draw.Mesh.Obj_name = false;

    // DRAW  borders  
    options.Draw.Mesh.Borders = false;
    options.Draw.Mesh.BorderIndexes = false;
    options.Draw.Mesh.BorderLabels = false;
    options.Draw.Mesh.BorderSharpnessDots = false;
    options.Draw.Mesh.BorderSharpnessDotsAngle = false;
    options.Draw.Mesh.BorderAngles = false;

    // DRAW  normals  
    options.Draw.Mesh.Normals_Faces = false;
    options.Draw.Mesh.Normals_Vertixes = false;
    options.Draw.Mesh.Normals_Faces_Calculated = false;

    // DRAW   colors 
    options.Draw.Mesh.DefaultColor = Color3d(0, 0.96, 0); ;// Color3d(0.96, 0.96, 0.96);
    options.Draw.Mesh.TopologyConnection_Color_Outher = Color3d(0.1, 0.1, 0.1); // Color.Black
    options.Draw.Mesh.TopologyConnection_Color_Inner = Color3d(1, 0, 1); //Color.Magenta
    options.Draw.Mesh.TopologyConnection_Color_Connected = Color3d(0.6, 0.6, 0.6); // Color3d(0.75, 0.75, 0.75) Color.Silver
    options.Draw.Mesh.Points_Color = Color3d(0.3, 0.3, 0.3);
    options.Draw.Mesh.Id_Vertex_Color = Color4d(.0, .0, .0, 0.9);
    options.Draw.Mesh.Id_Edge_Color = Color4d(.0, .7, .0, 0.6);
    options.Draw.Mesh.Id_Face_Color = Color4d(.3, .3, .3, 0.7);
    options.Draw.Mesh.Id_Mesh_Color = Color4d(.7, .2, .2, 0.9);
    options.Draw.Mesh.Obj_Id_Color = Color4d(.7, .2, .2, 0.9);
    options.Draw.Mesh.Obj_Name_Color = options.Draw.Mesh.Obj_Id_Color;

    // DRAW higghlights
    options.Draw.Mesh.Highlight.Enabled = false;
    options.Draw.Mesh.Highlight.What = MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::Mesh;
    options.Draw.Mesh.Highlight.id = "";


    //MESH
    options.Mesh.computeK = MeshLogicOptions_Mesh::computeKType::Direct;
    options.Mesh.SSEEnabled = true;
    options.Mesh.sse__triangle_triangle_adjacency_preprocess = true;
    options.Mesh.SplitNakedTriangleWith2NakedEdge = true; // default true - helps NrosySolver define proper constrains 
    options.Mesh.OptimizeVertexIndexesForGPU = true; // default true - improves rotation speed in viewport by 1.5x-3x times
    options.Mesh.Heal = true;

    //MESH HEAL
    options.MeshHeal.PolygonMesh_DeleteZeroTriangles = true;
    options.MeshHeal.PolygonMesh_DeleteUnattachedVertexes = true;

    options.MeshHeal.MergeVertexes = true;
    options.MeshHeal.MergeVertexes_tol_edge_percent = 0.01; // 1% of the max triangle/quad  edge
    options.MeshHeal.FlipNormals = true;
    options.MeshHeal.NormalizeNormals = true;
    options.MeshHeal.FillHoles = true;
    options.MeshHeal.FillHoles_maxEdgesCount = 6;
    options.MeshHeal.DeleteUnattachedTriangles = true;
    options.MeshHeal.DeleteUnattachedTriangles_maxConectedCount = 2;

    //MESH TOPOLOGY
    options.MeshesTopology.Enabled = true;
    options.MeshesTopology.check_PerfectConnections = true;
    options.MeshesTopology.check_PerfectConnections_fast = true;
    options.MeshesTopology.tol_PerfectConnections = (D)0.01; // 1% of edge length
    options.MeshesTopology.check_PartialConnections = true;
    options.MeshesTopology.check_PartialConnections_fast = true;
    options.MeshesTopology.tol_PartialConnections = (D)0.03; // 3% of edge length
    options.MeshesTopology.check_InsideConnections = true;
    options.MeshesTopology.tol_InsideConnections = (D)0.5; //50% of edge length - distance between edges should be higher from this value
    options.MeshesTopology.tol_InsideConnectionsDist = (D)0.1; //10% of relation start/end dist to segment length - distance between partial segments should be higher from this value
    options.MeshesTopology.max_angle_between_edges_InsideConnections = 5; // 5 degrees
    options.MeshesTopology.DebugEnabled = false;
    options.MeshesTopology.debug_tracePairsInConsole = false;
    options.MeshesTopology.debug_highlighPoint_id = -1;
    options.MeshesTopology.debug_highlighConnection_id = -1;
    options.MeshesTopology.debug_highlighCurve_id = -1;
    options.MeshesTopology.debug_TestPerformanceCall = false;
    options.MeshesTopology.debug_TestPerformanceCall_repeatTimes = 10;
    options.MeshesTopology.draw.Enabled = true;
    options.MeshesTopology.draw.drawSingleCurves = true;
    options.MeshesTopology.draw.drawConnectedCurves = true;
    options.MeshesTopology.draw.drawConnectionsIds = false;
    options.MeshesTopology.draw.drawSegmenstIds = false;
    options.MeshesTopology.draw.drawCurvesIds = false;
    options.MeshesTopology.draw.drawSegmentsPointsCount = false;
    options.MeshesTopology.draw.drawDots = true;
    options.MeshesTopology.draw.drawDotsIds = false;
    options.MeshesTopology.draw.drawDotsSegmentsIds = false;
    options.MeshesTopology.draw.drawDotsCurvesIds = false;

    //STREAM
    options.MeshStream.show_dividing_streams = true;
    options.MeshStream.show_dividing_streams__show_vid_eid_fid = false;
    options.MeshStream.show_streamIndexes_atStart = false;
    options.MeshStream.show_streamIndexes_InAllPoints = false;
    options.MeshStream.show_streamIndexes_Every10Points = false;
    options.MeshStream.show_stream_iterations = false;
    options.MeshStream.show_ExtendStream_info_in_console = false;
    options.MeshStream.snap_to_vertex_tol = 0.01;  //percent of edge length - good is 0.01, 0.005, 0.007
    options.MeshStream.use_sse41 = true;

    //STREAMS TOPOLOGY
    options.MeshStreamsTopology.Enabled = true;
    options.MeshStreamsTopology.cut_streams_at_cross = true;
    options.MeshStreamsTopology.cut_streams_at_cross__show = true;
    options.MeshStreamsTopology.debug_show_streams_intersections = false;
    options.MeshStreamsTopology.debug_show_streams_segments = false;
    options.MeshStreamsTopology.debug_show_quads = false;
    options.MeshStreamsTopology.debug_show_loops = false;
    options.MeshStreamsTopology.debug_show_connections = false;
    options.MeshStreamsTopology.debug_trace_iterations_in_console = false;
    options.MeshStreamsTopology.debug_TestPerformanceCall = false;
    options.MeshStreamsTopology.debug_TestPerformanceCall_repeatTimes = 10;

    //CONSTRAIN
    options.Constrains.CorrectInAngles = true;
    options.Constrains.CorrectInAngles_Percent = 0.51;
    options.Constrains.CorrectInAngles_Percent_IsDynamic = true;
    options.Constrains.CorrectInAngles_Percent_isProgressive = true;
    options.Constrains.CorrectInAngles_straightTo90_angleX2 = false;  //not very good when surfaces connected - sharp quad connections may appear, since angle was coorrected a bit more from normal
    options.Constrains.CorrectInAngles_force_4side_to_straightTo90 = false;
    options.Constrains.CorrectInAngles_skipp_last_edges_count = 2;
    options.Constrains.CorrectInAngles_ForceTo0_90to0 = false;
    options.Constrains.Type = MeshLogicOptions_Constrains::MeshConstrainsType::AllEdgesWithCorrectionInCorners;
    options.Constrains.WeightIsRelativeToEdgeLength = false; // important for meshes with different border sizes (false because of issue #69)
    options.Constrains.CorrectInAngles_Debug = false;
    options.Constrains.AddContrainsInSharpEdges = false;
    options.Constrains.AddContrainsInSharpEdges_sharpAngle = 70;
    options.Constrains.DebugEnabled = false;
    options.Draw.Cons.ShowBorderConstrains = false;
    options.Draw.Cons.HighlightConstrainedFaces = false;
    options.Draw.Cons.ShowEdgesSharpness = false;

    //CONSTRAIN UV
    options.ConstrainsUV.DebugEnabled = true;
    options.ConstrainsUV.debug_showConstrains = true;
    options.ConstrainsUV.debug_HighlightConstrainedFaces = true;
    options.ConstrainsUV.debug_showSharpness = true;

    // MESH LINEAR SOLVER    
    #if MeshLinearSolver_SUPPORTED
    options.MeshLinearSolver.DebugEnabled = true;
    options.MeshLinearSolver.debug_debugStepNum = 1;
    #endif


    // LDLT FLOAT SOLVER    
    options.LDLTFloatSolver.fast_factorize_preordered = true;
    options.LDLTFloatSolver.fast_factorize_preordered__multithreading = true;
    options.LDLTFloatSolver.fast_factorize_preordered_use_AVX = true;// doesnt work for now, since my CPU doesnt support it
    options.LDLTFloatSolver.fast_permutation = true;
    options.LDLTFloatSolver.DebugEnabled = false;
    options.LDLTFloatSolver.debug_trace_factorize_preordered = false;
    options.LDLTFloatSolver.debug_skip_permuting = false;
    options.LDLTFloatSolver.debug_trace_iterationsCount = false;
    options.LDLTFloatSolver.debug_trace_compute_iterationsCount = -1;

    // LINEAR EQUATION SOLVER
    options.LinearEquationSolver.Solver = MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType::LDLTFloat;

    // SOLVER    
    options.Solver.Enabled = true;
    options.Solver.N = MeshLogicOptions_SolverNrosy::NRosyNum::N4;
    options.Solver.Solver = MeshLogicOptions_SolverNrosy::MeshSolverType::NPolyVectorField_Polar2x;
    #if alternative_solvers_SUPPORTED
    options.Solver.SoftPercent = 0.9999; // the best value is 0.9 - lines follows more borders rather than ortogonal to borders
    options.Solver.Tolerance = 0.0001;  // tolerance used for a iterative solvers
    options.Solver.SolveForVectorsXY = MeshLogicOptions_Solver::SolveForVectorsXYEnum::XY;
    #endif
    #if EROW
    options.Solver.fast_Generate_Result_F = true;
    #else
    options.Solver.fast_Generate_Result_F = false; // this options is valid only if we can walk our data lineary by pointer
    #endif
    options.Solver.fast_sparce_matrix_creation = true;
    options.Solver.use_sse_cin_cos = true;
    options.Solver.adjustK = false; // research method to apply surface curvature - doesnt work for now
    options.Solver.adjustK_Percent = 0.5;
    options.Solver.adjustK_useCos = true;
    options.Solver.adjustK_quadric = false;

    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    options.Solver.compute_only_once = false;
    options.Solver.ignore_x = false;
    options.Solver.ignore_x__max_correction_angle = 0.001;
    options.Solver.ignore_x_always = false;
    #endif
    options.Solver.sort_complex_vectors = true;
    options.Solver.NormalizeNrosyField_sort_field_directions = true;
    options.Solver.check_angle_correction_validity = false; // run sover twice with corrected constains and default and get best result
    options.Solver.DebugEnabled = true;
    options.Solver.debug_trace_solver_performance_per_mesh = false;
    options.Solver.TestPerformanceCall = false;
    options.Solver.TestPerformanceCall_repeatTimes = 10;

    // NROSY  FIELD Visual
    options.Draw.Solver.HighlightFixedP = false;
    options.Draw.Solver.ShowDirections = false;
    options.Draw.Solver.ShowDirectionsStyle = MeshLogicOptions_Draw_Solver::DirectionsStyle::Default;
    options.Draw.Solver.ShowSingularities = false;
    options.Draw.Solver.ShowDirectionsWeights = false;
    options.Draw.Solver.ShowBorderConstrainesIsoLines = false;
    options.Draw.Solver.ShowBorderConstrainesIsoLinesCount = 11111;
    options.Draw.Solver.ShowBorderConstrainesIsoLinesCount_intensivity = 1;

    // SOLVER   AnlgeBound
    options.SolverAngleBound.Enabled = false;
    options.SolverAngleBound.Algorithm = MeshLogicOptions_SolverAngleBound::AlgorithmType::polar;
    options.SolverAngleBound.reuse_ordering = true;
    options.SolverAngleBound.iterations_count = 1;
    options.SolverAngleBound.thetatMin = 30;
    options.SolverAngleBound.lambdaInit = 0.0001;//100;
    options.SolverAngleBound.lambdaMultFactor = 1.01;//1.5
    options.SolverAngleBound.doHardConstraints = false;
    options.SolverAngleBound.leave_x_original = false;
    options.SolverAngleBound.DebugEnabled = false;
    options.SolverAngleBound.debug_logMessages = false;
    options.SolverAngleBound.TestPerformanceCall = false;
    options.SolverAngleBound.TestPerformanceCall_repeatTimes = 10;


    // SOLVER   UV 
    options.SolverUV.Enabled = false;
    options.SolverUV.Algorithm = MeshLogicOptions_SolverUV::AlgorithmType::lscm;
    options.SolverUV.DebugEnabled = false;
    options.SolverUV.debug_show_uv = false;
    options.SolverUV.debug_show_uv__on_borders = false;
    options.SolverUV.debug_showBorderConstrainesIsoLines = false;
    options.SolverUV.debug_showBorderConstrainesIsoLines__count = 11111;
    options.SolverUV.debug_showBorderConstrainesIsoLines__intensivity = 1;
    options.SolverUV.debug_trace_solver_performance_per_mesh = false;
    options.SolverUV.TestPerformanceCall = false;
    options.SolverUV.TestPerformanceCall_repeatTimes = 10;

    // Mesh Simplification
    options.MeshSimplification.Enabled = false;
    //options.MeshSimplification.Level = -0.001;  // good values -0.001(high) and -0.0001(very high)  (about 20% and 27% reduction)     // threshold if < 0, percent if between 0 and 1, count if > 1
    options.MeshSimplification.Level = -0.01;  // work good cooperative with MinAngleBetweenEdges and MaxAngleChangeInNormal
    options.MeshSimplification.Algorithm = MeshLogicOptions_MeshSimplification::AlgorithmType::FastQuadric;
    options.MeshSimplification.RecomputeTriangleNormalsAfterEachIteration = false;
    options.MeshSimplification.MinAngleBetweenEdges_ForNewTriangles = 15; //good values [10..25]
    options.MeshSimplification.MaxAngleChangeInNormal_ForNewTriangles = 7; //good values [5..11]

    options.MeshSimplification.RemoveSharpTriangles = true;
    options.MeshSimplification.RemoveSharpTriangles_Iterations = 10;
    options.MeshSimplification.RemoveSharpTriangles_MinSharpness = 0.04;
    options.MeshSimplification.RemoveSharpTriangles_MaxFriendSharpness = 0.2;

    options.MeshSimplification.DebugEnabled = false;
    options.MeshSimplification.Debug_MaxNumOfIterations = 1; // 0 all,  otherwise only iterations less from this num
    options.MeshSimplification.Debug_HighlightSimplificationNum = 16;   // -1 show all, 0 none,  otherwise only one simplification defined by this num
    options.MeshSimplification.Debug_HighlightSharpTriangles = false;
    options.MeshSimplification.Debug_HighlightFlippedTraingles = true;
    options.MeshSimplification.Debug_HighlightTryFailEdges = true;
    options.MeshSimplification.Debug_ShowTrianglessSharpness = false;
    options.MeshSimplification.Debug_ShowEdgeErrors = false;
    options.MeshSimplification.Debug_ShowTriangleIndex = false;
    options.MeshSimplification.Debug_ShowTriangleId = false;


    // DIVIDER    
    options.Divider.Enabled = true;
    options.Divider.Algorithm = MeshLogicOptions_Divider::AlgorithmType::StreamAngles;
    options.Divider.SingularityExtendCount = 11111;
    options.Divider.fieldVectorsWeights_isrelative_to_fielddirection_norm = false;
    options.Divider.fieldVectorsWeights_is_fielddirection_norm = false;
    options.Divider.isoLines_ExtensionsCount = 5;
    options.Divider.use_always_first_vector_as_prev = true;
    options.Divider.improve_stream_angles = true;
    options.Divider.DebugEnabled = false;
    options.Divider.Debug_isoLines_show = false;
    options.Divider.Debug_point_index = -1;
    options.Divider.Debug_ri_index = -1;


    // STREAM ADJUSTER
    options.StreamAdjuster.iterate_adjustments = true;
    options.StreamAdjuster.iterate_adjustments__iterations_count = 10;
    options.StreamAdjuster.iterate_adjustments__trace_progress_in_console = false;
    options.StreamAdjuster.Algorithm = MeshLogicOptions_MeshStreamsAdjuster::AlgorithmType::Ditsts;
    options.StreamAdjuster.ShiftSmoothing = MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Linear;
    options.StreamAdjuster.testDistMethod = MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod::NewStream;
    options.StreamAdjuster.skip_first_points = 0;
    options.StreamAdjuster.tollerance = 0.001; // percent from desired distance from original-stream to adjusted-stream (0.01 == 1 percent)
    options.StreamAdjuster.max_stream_angle_diff = 40; // max angle diff for one line compare to stream, this angle should be lesser from 'max_change_angle' and not to high, since with higher angle there is a canche to go wrong direction

    options.StreamAdjuster.run_direct_calculations = true;
    options.StreamAdjuster.DirectAlgorithm = MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType::L; // good works only V and L. default V as it gives the smallest error since it works with vectors in 3D, in compare of method L what works in 2D
    options.StreamAdjuster.direct_calculations__test_streams_count = 7;
    options.StreamAdjuster.direct_calculations__replace__new_lengthTo_pointOnPoints = -1;
    options.StreamAdjuster.direct_calculations__precise_direction_translation = true; // improves precision and thus direct method have smaller error and fail less times

    options.StreamAdjuster.run_iterative_calculations = true;
    options.StreamAdjuster.iterative_calculations__max_iterations_count = 500;
    options.StreamAdjuster.iterative_calculations__max_change_angle = 90; // max angle change for one point, should be high to make possible exit wrongly taken directions
    options.StreamAdjuster.iterative_calculations__max_times_failed_in_row = 20;

    options.StreamAdjuster.run_hard_calculations = true;
    options.StreamAdjuster.hard_calculations__iterations_count_on_edge = 100;

    options.StreamAdjuster.DebugEnabled = false;
    options.StreamAdjuster.debug_singularityExtendNum = 117;
    options.StreamAdjuster.debug_debugStreamIndex = 5;
    options.StreamAdjuster.debug_debugPointNum = 0;
    options.StreamAdjuster.debug_debugIterationNum = -1;
    options.StreamAdjuster.debug_max_times_failed_in_row = 20;
    options.StreamAdjuster.debug_trace_iterations = true;
    options.StreamAdjuster.debug_trace_iterations__tests = true;
    options.StreamAdjuster.debug_draw_accepted_points = true;
    options.StreamAdjuster.debug_show_stream_to_connection_point = true;
    options.StreamAdjuster.debug_TestPerformanceCall1000Times = false;

    //DEBUG - float or double in igviewer project?
    //options.StreamAdjuster.DebugEnabled = true;
    //options.StreamAdjuster.debug_show_stream_to_connection_point = false;
    //options.StreamAdjuster.debug_singularityExtendNum = 3;
    //options.StreamAdjuster.debug_debugStreamIndex = 16;
    //options.StreamAdjuster.debug_debugPointNum = 2;
    //options.StreamAdjuster.debug_debugTryNum = 0;
    //options.StreamAdjuster.debug_max_times_failed_in_row = 3;
    //options.StreamAdjuster.debug_trace_tryis = true;
    //options.StreamAdjuster.debug_trace_tests = true;

    // STREAM JOINER
    options.Joiner.Enabled = false;
    options.Joiner.maxDist_meshSize_multipliyer = 1.1;// default 1.1 = 110%
    options.Joiner.maxDist_is_minimum_of_mesh_avg_size = true;
    options.Joiner.max_connectionAngle_Directions = 40; //default 40 - for very high meshsize - we have to connect what is possible to avoid small meshes
    options.Joiner.max_connectionAngle_Normals = 120; //default 120 - for very high meshsize - we have to connect what is possible to avoid small meshes
    options.Joiner.max_lengthsDiff_percent = 0.2; // default 0.2 = 20%. 
    options.Joiner.max_connectionArea_percent = 0.2; // default 0.2 = 20% of total mesh size
    options.Joiner.min_lengthToDist_percent = 1.3; // default 1.3 = 130% - length must be 1.3 times bigger from dist 
    options.Joiner.sort_by_length_for_very_small_distances = false; // default 'false'
    options.Joiner.optimizeConnections = true; // default 'true'

    options.Joiner.DebugEnabled = false;
    options.Joiner.debug_show_connection_points = false;
    options.Joiner.debug_debugStreamIndex = -1;
    options.Joiner.debug_meshSize_multipliyer = 1.1; // default 1.1
    options.Joiner.debug_connection = false;
    options.Joiner.debug_connection_streamIndex1 = 104;
    options.Joiner.debug_connection_streamIndex2 = 108;
    options.Joiner.debug_TestPerformanceCall1000Times = false;
    options.Joiner.debug_alternativePath_debugStreamIndex = -1;
    options.Joiner.debug_show_connection_anchor_ids = false;
    options.Joiner.debug_show_connection_indexes = false;


    // DIVIDER  ITERATOR
    options.DividerIterator.Enabled = false;
    options.DividerIterator.max_iterations_count = 1000;
    options.DividerIterator.joinStreamsAfterEachIteration = true;
    options.DividerIterator.DebugEnabled = true;
    options.DividerIterator.debug_iteration_index = -1;
    options.DividerIterator.debug_show_iteration_index_on_streams = false;
    options.DividerIterator.debug_trace_iterations_in_console = false;


    // DIVIDER  OPTIMAL CONNECTIONS SOLVER
    options.DividerOptimalConnector.Enabled = false;
    options.DividerOptimalConnector.DividingType = MeshLogicOptions_DividerOptimalConnector::LoopDividingType::Avarage;
    options.DividerOptimalConnector.max_conflicts = 20;

    options.DividerOptimalConnector.JoinConflictedLoops = false;
    options.DividerOptimalConnector.JoinConflictedLoops_ReserveSpaceForSmallInBig = true;
    options.DividerOptimalConnector.JoinConflictedLoops_ReserveSpaceForSmallInBig_acceptRation = 2.5;
    options.DividerOptimalConnector.JoinConflictedLoops_BalanceSizes = true;
    options.DividerOptimalConnector.JoinConflictedLoops_JoinBordersWithStreams = true;

    options.DividerOptimalConnector.JoinStreamsWithDividingPoints = false;

    options.DividerOptimalConnector.DebugEnabled = true;
    options.DividerOptimalConnector.debug_show_conflictedLoops = false;
    options.DividerOptimalConnector.debug_debug_conflictedLoop_num = -1;
    options.DividerOptimalConnector.debug_ReserveSpaceForSmallInBig_max_iterations_count = -1;
    options.DividerOptimalConnector.debug_show_unresolved_conflicts = true;
    options.DividerOptimalConnector.debug_show_dividingPointsForJoins = true;
    options.DividerOptimalConnector.debug_trace_iterations_in_console = false;
    options.DividerOptimalConnector.debug_debug_iteration_num = -1;

    // DIVIDER  LOGIC CONNECTIONS SOLVER
    options.DividerLogicConnector.Enabled = false;
    options.DividerLogicConnector.connectionAngle = 40;
    options.DividerLogicConnector.DebugEnabled = true;
    options.DividerLogicConnector.debug_debug_iteration_num = -1;
    options.DividerLogicConnector.debug_show_streams__intersection_points = false;
    options.DividerLogicConnector.debug_show_streams__accepted_connections = true;
    options.DividerLogicConnector.debug_streamGlobalIndex = -1;
    options.DividerLogicConnector.debug_connectionIndex = -1;


    // MESH CUTTER    
    options.MeshCutter.Enabled = false;
    options.MeshCutter.ImproveStreamLines = true;
    options.MeshCutter.ImproveStreamLines__edge_percent_tolerance = 0.2;// default 20%
    options.MeshCutter.MergeClosePointsOnEdges = true; // merge very close points to avoid tiny triangles
    options.MeshCutter.MergeClosePointOnEdges__edge_percent_tolerance = 0.05;// default 5% - very small distance
    options.MeshCutter.MergeClosePointOnEdges__connection_angle = 35;// default 35 degrees
    options.MeshCutter.MergeClosePointOnEdges__max_angle_change_belowConnectionAngle = 1;// default 1 degrees (-1 for disabled)
    options.MeshCutter.MergeClosePointOnEdges__max_angle_change_aboveConnectionAngle = 1;// default 1 degrees (-1 for disabled)
    options.MeshCutter.ImproveMeshToConnectStreamLines = true;
    options.MeshCutter.ImproveMeshToConnectStreamLines__edge_percent_tolerance = 0.33;// default 33%
    options.MeshCutter.ImproveMeshToConnectStreamLines__allow_improvements_on_borders = false; //  currently dont work well for 2 connected meshes - some of them may get improvement and othe not, and thus topology connection will be broken
    options.MeshCutter.ImproveMeshToConnectStreamLines__dissalow_move_other_streams = true;
    options.MeshCutter.FindStreamIntersections = true;
    options.MeshCutter.CutMesh = true;
    options.MeshCutter.CutMesh_useFastMethod = true;
    options.MeshCutter.DebugEnabled = false;
    options.MeshCutter.Debug_trace_removed = false;
    options.MeshCutter.Debug_show_mergedclose_points = false;
    options.MeshCutter.Debug_show_steps = false;
    options.MeshCutter.Debug_show_steps__stop_at_step = -1;
    options.MeshCutter.Debug_show_intersections = false;
    options.MeshCutter.Debug_cutmesh__show_intersecting_faces = false;
    options.MeshCutter.Debug_cutmesh__show_intersecting_faces__show_only_for_face = -1;//192, 332, 100
    options.MeshCutter.Debug_cutmesh__show_edges_that_are_recognized_as_stream_holders = false;
    options.MeshCutter.Debug_ShowModifiedMesh = true;
    options.MeshCutter.Debug_ShowModifiedMesh_onlyborders = true;

    // MESHER
    options.Mesher.Enabled = false;
    options.Mesher.Algorithm = MeshLogicOptions_Mesher::AlgorithmType::UVOnCuttedMeshes;
    options.Mesher.meshSize_absolute = 1;
    options.Mesher.meshSize_RelativeToTrianglesSize = 2;
    options.Mesher.is_meshSize_RelativeToTrianglesSize = false;
    options.Mesher.meshSize_min = 0.4; // 40% of meshsize - what is less this size we will try to remove
    options.Mesher.DebugEnabled = true;
    options.Mesher.debug_show_meshsize_on_borders = false;
    options.Mesher.debug_draw_cuttedMesh = true;
    options.Mesher.debug_debug_mesh_id = -1;
    options.Mesher.debug_show_meshingLoops_ids = false;
    options.Mesher.debug_debug_meshingLoop_id = -1;
    options.Mesher.debug_show_divisioncount_before_adjustment = false;
    options.Mesher.debug_show_divisioncount_after_adjustment = false;
    options.Mesher.debug_show_meshing_process = false;
    options.Mesher.debug_show_meshing_process_Y = false;
    options.Mesher.debug_show_produced_mesh = true;
    Update_Mesher__saved_draw_cutted_mesh();

    // FILE 
    PrecacheMeshFile = false;
    CustomTestFile = "";             // "F:\\4.q3dmesh"
    TestFile = TestFiles::circle1;
    TestFileDensity = TestFilesDensity::low;
    ShowOnlyMeshIds = "";
    ShowOnlyObjectIds = "";        // "3  44"
    LoadOnlyObjectNames = "";   // "F016 F_M_050"
    ConvertMeshToFlatMesh = false;
    TestSpeed_ReadFileFromObj = false;
    TestSpeed_AddMeshes = false;
    SeparateGroups = true;
    options.Mesher.meshSize_absolute = 1;


    //
    //  TEST OPTIONS
    //

    //window.viewport.show_wireframe = true;
    //options.MeshCutter.Enabled = true;
    //options.Solver.Enabled = false;
    //options.Mesher.Enabled = true;
    //options.Solver.Enabled = false;
    //options.SolverUV.Algorithm = MeshLogicOptions_SolverUV::AlgorithmType::lscm;
    //options.Draw.Solver.ShowDirections = true;
    //options.Draw.Solver.ShowDirectionsStyle = MeshLogicOptions_Draw_Solver::DirectionsStyle::Default;
    //options.Draw.Solver.ShowBorderConstrainesIsoLines = true;


    //AddPresetTestGroup("Testing Divider Optimal Connector");
    //options.MeshStreamsTopology.Enabled = false;
    //options.DividerIterator.Enabled = true;
    //options.DividerOptimalConnector.Enabled = false;
    //options.Joiner.Enabled = false;
    //options.DividerLogicConnector.Enabled = false;
    //options.DividerOptimalConnector.ConnectionStyle = MeshLogicOptions_DividerOptimalConnector::ConnectionType::_1to1;
    //AddPresetTestFile(TestFiles::_100750, TestFilesDensity::medium, 2, ""); 
    //AddPresetTestFile(TestFiles::circle1, TestFilesDensity::low, 1, "");
    //AddPresetTestFile(TestFiles::_2, TestFilesDensity::high, 1, "");
    //AddPresetTestFile(TestFiles::_100750, TestFilesDensity::medium, 1, "");
    //AddPresetTestFile(TestFiles::kolo, TestFilesDensity::low, 1, "");
    //AddPresetTestFile(TestFiles::krug, TestFilesDensity::high, 1, "");
    //AddPresetTestFile(TestFiles::detal22, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::charn, TestFilesDensity::low, 1, "");

    //AddPresetTestGroup("Mesh heal");
    //AddPresetTestFile("F:\\kor.obj");
    //AddPresetTestFile("K:\\Data\\obj\\Thingi10K\\1778123.stl");


    //AddPresetTestGroup("OpenGL");
    //options.MeshStreamsTopology.Enabled = false;
    //options.DividerIterator.Enabled = true;
    //options.DividerOptimalConnector.Enabled = false;
    //options.Joiner.Enabled = false;
    //options.DividerLogicConnector.Enabled = false;
    //AddPresetTestFile(TestFiles::fandisk, TestFilesDensity::high, 3, "");
    //AddPresetTestFile(TestFiles::block, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::dysk_quad, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::circle2, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::korzyna_small_3, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::korzyna_small, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::_100750, TestFilesDensity::low, 3, "");
    //AddPresetTestFile(TestFiles::korzyna, TestFilesDensity::high, 3, "");


    AddPresetTestGroup("DividerLogicConnector");
    options.MeshStreamsTopology.Enabled = false;
    options.DividerIterator.Enabled = true;
    options.DividerLogicConnector.Enabled = true;
    AddPresetTestFile(TestFiles::_2, TestFilesDensity::low, 3, "F_M_050 F016");
    AddPresetTestFile(TestFiles::circle2, TestFilesDensity::low, 1, "");
    AddPresetTestFile(TestFiles::korzyna_small_3, TestFilesDensity::medium, 1, "F021 F030 F074 F016");
    AddPresetTestFile(TestFiles::_139244, TestFilesDensity::medium, 1, "F_M_002 F120");
    AddPresetTestFile(TestFiles::_100750, TestFilesDensity::low, 0.3, "F074 F_M_007");
    AddPresetTestFile(TestFiles::dysk_quad, TestFilesDensity::low, 3, "");
    AddPresetTestFile(TestFiles::_10308589, TestFilesDensity::high, 33, "F235 F219 F117 F115 F086 F085 F087 F113");

    options.MeshStreamsTopology.Enabled = false;
    options.DividerIterator.Enabled = false;
    options.DividerLogicConnector.Enabled = false;
    options.Solver.Enabled = false;

    //AddPresetTestGroup("Fixing singularity defect");
    //window.viewport.show_wireframe = true;
    //options.Draw.Solver.ShowDirections = true;
    //options.Draw.Solver.ShowDirectionsWeights = true;
    //AddPresetTestFile(TestFiles::circle2, TestFilesDensity::low, 1, "");
    //AddPresetTestFile(TestFiles::qq2, TestFilesDensity::high, 1, "");
    //AddPresetTestFile(TestFiles::korzyna_small_3_triangle, TestFilesDensity::medium, 1, "");

    //AddPresetTestGroup("Testing MeshStreamsJoiner");
    //AddPresetTestFile(TestFiles::_152687_3, TestFilesDensity::low, 0.4, "F050");
    //AddPresetTestFile(TestFiles::_152687_3, TestFilesDensity::low, 0.2, "F050");
    //AddPresetTestFile(TestFiles::_152687_3, TestFilesDensity::low, 0.2, "");
    //AddPresetTestFile(TestFiles::_139242, TestFilesDensity::medium, 1.3, "F125 F126 F127");
    //AddPresetTestFile(TestFiles::quad_ngon, TestFilesDensity::low, 0.3, "");

    //AddPresetTestGroup("Testing SplitNakedTriangleWith2NakedEdge");
    //AddPresetTestFile(TestFiles::_139242, TestFilesDensity::medium, 1.3, "F082");
    //AddPresetTestFile(TestFiles::detal11, TestFilesDensity::low, 1.3, "");
    //AddPresetTestFile(TestFiles::_139242, TestFilesDensity::medium, 1.3, "F073  F029 F071");


    //AddPresetTestGroup("Testing Viewport zoom and translate");
    //AddPresetTestFile(TestFiles::korzyna_small_3, TestFilesDensity::low, 1.3, ""); //"F071 F104"
    //AddPresetTestFile(TestFiles::detal1, TestFilesDensity::low, 1.3, ""); //"F071 F104"
    //AddPresetTestFile(TestFiles::korzyna_small, TestFilesDensity::low, 1.3, ""); //"F071 F104"


    //AddPresetTestGroup("Testing StreamAdjuster");
    //AddPresetTestFile(TestFiles::_139242, TestFilesDensity::medium, 0.3, "F073  F029 F071");
    //AddPresetTestFile(TestFiles::coffee1, TestFilesDensity::high, 1.3, "F071 F104"); 


    //AddPresetTestGroup("Testing Joiner");
    //AddPresetTestFile(TestFiles::_152687_3, TestFilesDensity::low, 1, "");
    //AddPresetTestFile(TestFiles::detal11, TestFilesDensity::low, 1.3, "");
    //AddPresetTestFile(TestFiles::quad_ngon, TestFilesDensity::low, 0.3, "");

    //AddPresetTestFile(TestFiles::coffee2, TestFilesDensity::high, 1.3, "F104"); //"F071 F104"
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::low, 1, "F_M_002 F120");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::low, 0.7, "F_M_002 F120");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::high, 0.9, "F_M_002");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::high, 0.7, "F_M_002");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::medium, 2, "F120");
    //AddPresetTestFile(TestFiles::ex1, TestFilesDensity::medium, 1.3);
    //AddPresetTestFile(TestFiles::f3, TestFilesDensity::high, 1.3, "F026");
    //AddPresetTestFile(TestFiles::_139242, TestFilesDensity::medium, 0.3, "F122 F123 F124");
    ////AddPresetTestFile(TestFiles::f3, TestFilesDensity::medium, 1.3, "F026");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::medium, 1.3, "F120");
    ////AddPresetTestFile(TestFiles::f3, TestFilesDensity::high, 1.3, "F026");
    ////AddPresetTestFile(TestFiles::_139244, TestFilesDensity::low, 1.3, "F_M_002");
    ////AddPresetTestFile(TestFiles::_139244, TestFilesDensity::medium, 1.3, "F_M_002");
    //AddPresetTestFile(TestFiles::charn, TestFilesDensity::medium, 1.3);
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::low, 1.3, "F_M_002 F120");
    //AddPresetTestFile(TestFiles::trymach3, TestFilesDensity::medium, 1.3, "F004 F008 F010");
    ////AddPresetTestFile(TestFiles::trymach3, TestFilesDensity::high, 1.3, "F004 F008 F010");
    ////AddPresetTestFile(TestFiles::trymach3, TestFilesDensity::medium, 11.3, "F004 F008 F010");
    ////AddPresetTestFile(TestFiles::trymach3, TestFilesDensity::high, 11.3, "F004 F008 F010");
    //AddPresetTestFile(TestFiles::trymach4, TestFilesDensity::medium, 1.3, "F007 F008 F009");
    //AddPresetTestFile(TestFiles::_10308589, TestFilesDensity::high, 1.3, "F235 F219 F117 F115 F086 F085 F087 F113");
    ////AddPresetTestFile(TestFiles::f3, TestFilesDensity::low, 1.3, "F026");
    ////AddPresetTestFile(TestFiles::f3, TestFilesDensity::medium, 1.3, "F026");
    ////AddPresetTestFile(TestFiles::f3, TestFilesDensity::high, 1.3, "F026");
    //AddPresetTestFile(TestFiles::kolo, TestFilesDensity::medium, 0.3, "");
    ////AddPresetTestFile(TestFiles::korzyna_small, TestFilesDensity::high, 0.3, "F144");

    //AddPresetTestGroup("Testing Bad surfaces");
    //AddPresetTestFile(TestFiles::coffee3, TestFilesDensity::high, 1.3, "F_M_001");
    //AddPresetTestFile(TestFiles::trymach3, TestFilesDensity::medium, 1.3, "F009");


    //AddPresetTestGroup("Testing Divider Iterator");
    //AddPresetTestFile(TestFiles::_2, TestFilesDensity::low, 1.3, "F_M_050 F016");
    //AddPresetTestFile(TestFiles::_2, TestFilesDensity::low, 0.1, "F_M_050 F016");
    //AddPresetTestFile(TestFiles::_2, TestFilesDensity::high, 0.1, "F040  F_M_016");
    //AddPresetTestFile(TestFiles::kolo, TestFilesDensity::low, 1.3, "");
    //AddPresetTestFile(TestFiles::_139244, TestFilesDensity::low, 1.3, "F_M_002 F120");
    //AddPresetTestFile(TestFiles::_100750, TestFilesDensity::low, 0.3, "F074 F_M_007");
    ////AddPresetTestFile(TestFiles::_100750, TestFilesDensity::medium, 0.3, "");
    //AddPresetTestFile(TestFiles::_10308589, TestFilesDensity::high, 33, "F235 F219 F117 F115 F086 F085 F087 F113");
    ////AddPresetTestFile(TestFiles::_2, TestFilesDensity::low, 0.01, "F_M_050 F016");
    ////AddPresetTestFile(TestFiles::_119276, TestFilesDensity::low, 11.3, "F058 F_M_043 F_M_037  F034 F050");


    //AddPresetTestGroup("TEST");
    //AddPresetTestFile(TestFiles::coffee2, TestFilesDensity::high, 1.3, "F104"); //"F071 F104"


    //options.Draw.Mesh.Highlight.Enabled = true;
    //options.Draw.Mesh.Highlight.What = MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::StreamId;
    //options.Draw.Mesh.Highlight.id = "76"; 
}


















/*********************************************************/
/**** GUI ************************************************/
/*********************************************************/

void Options::AddOptions_GuiStyle(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_GuiStyle) On_OptionChanged_GuiStyle();
    };

    window.nanogui->addGroup("Gui");

    window.nanogui->addVariable<NanoguiTheme::NanoguiThemeColorStyle>("Theme", ThemeColorStyle, onOptionChanged)->setItems({ "Light blue", "Silver", "Dark" });


}

/*********************************************************/
/**** MeshSimplification ***********************************/
/*********************************************************/

void Options::AddOptions_MeshSimplification(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_MeshSimplification) On_OptionChanged_MeshSimplification();
    };
    MeshLogicOptions_MeshSimplification& _ = options.MeshSimplification;
    ngui->addGroup("Mesh simplification");
    ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
    ngui->addVariable("Level", _.Level, onOptionChanged)->setTooltip("threshold if < 0,   percent if between 0 and 1,   count if > 1");
    //ngui->addVariable<MeshLogicOptions_MeshSimplification::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)->setItems(_.AlgorithmTypeStr);
    //ngui->addVariable("Recompute every iterations", _.RecomputeTriangleNormalsAfterEachIteration, onOptionChanged);
    ngui->addVariable("Min angle between edges", _.MinAngleBetweenEdges_ForNewTriangles, onOptionChanged)->setUnitAngle();
    ngui->addVariable("Max angle change for triangles", _.MaxAngleChangeInNormal_ForNewTriangles, onOptionChanged)->setUnitAngle();

    ngui->addVariable("Remove sharp triangles", _.RemoveSharpTriangles, onOptionChanged);
    ngui->addVariable("> iterations", _.RemoveSharpTriangles_Iterations, onOptionChanged);
    ngui->addVariable("> min sharpness", _.RemoveSharpTriangles_MinSharpness, onOptionChanged);
    ngui->addVariable("> max friend sharpness", _.RemoveSharpTriangles_MaxFriendSharpness, onOptionChanged);
    ngui->addGroup(""); //delimiter
    ngui->addVariable("Debug", _.DebugEnabled, onOptionChanged);
    ngui->addVariable("> Max num of Iterations", _.Debug_MaxNumOfIterations, onOptionChanged)->setTooltip("Show only  few iteration of (0 all,  otherwise only iterations less from this num)");
    ngui->addVariable("> Highlight Simplification", _.Debug_HighlightSimplificationNum, onOptionChanged)->setTooltip("Show only single simplification inside last iteration (-1 show all, 0 none,  otherwise only one simplification defined by this num)");
    ngui->addVariable("> Highlight Sharp triangles", _.Debug_HighlightSharpTriangles, onOptionChanged);
    ngui->addVariable("> Highlight flipped triangles", _.Debug_HighlightFlippedTraingles, onOptionChanged);
    ngui->addVariable("> Highlight tryfail edges", _.Debug_HighlightTryFailEdges, onOptionChanged);
    ngui->addVariable("> show traingles sharpness", _.Debug_ShowTrianglessSharpness, onOptionChanged);
    ngui->addVariable("> show edges errors", _.Debug_ShowEdgeErrors, onOptionChanged);
    ngui->addVariable("> show traingles indexes", _.Debug_ShowTriangleIndex, onOptionChanged);
    ngui->addVariable("> show traingles Id", _.Debug_ShowTriangleId, onOptionChanged);

}


/*********************************************************/
/**** DRAW ************************************************/
/*********************************************************/
void Options::AddOptions_Viewer_Viewport(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    Viewport& viewport = window.viewport;

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };

    //ngui->addVariable("Vertex ids", viewport.show_vertid);// obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx
    //ngui->addVariable("Edge ids", viewport.show_edgeid);// obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx
    //ngui->addVariable("Face ids", viewport.show_faceid);// obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx
    //ngui->addVariable("Srf ids", viewport.show_srfid);// obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx
    //ngui->addVariable("Srf serial numbers", viewport.show_srfSerialNumber);// obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx


    //   ngui->addGroup("Workspace");
    //            ngui->addButton("Load", [&]() {this->load_scene(); });
    //ngui->addButton("Save", [&]() {this->save_scene(); });

    //   ngui->addGroup("Mesh");
    //            ngui->addButton("Load", [&]() {this->open_dialog_load_mesh(); });
    //ngui->addButton("Save", [&]() {this->open_dialog_save_mesh(); });

    /*ngui->addGroup("Viewing Options");
    ngui->addButton("Snap canonical view",[&]()
    {
    this->snap_to_canonical_quaternion();
    });
    ngui->addVariable("Zoom", viewport.camera_zoom);;*/


    //ngui->addGroup("Viewport");
    ngui->addButton("Center object", [&]()
    {
        window.align_camera_center();
    });

    //ngui->addVariable("Overlays", viewport.show_overlay, onOptionChanged);
    ngui->addVariable("Orthographic view", viewport.orthographic, onOptionChanged);
    //ngui->addVariable("Background", viewport.background_color, onOptionChanged);
    //ngui->addVariable("Wireframe color", viewport.wireframe_color, onOptionChanged);
    //ngui->addVariable("Shininess intensity", viewport.shininessIntensity, onOptionChanged);
    //ngui->addVariable("> shininess", viewport.shininess, onOptionChanged);
    //ngui->addVariable("XYZ", viewport.show_XYZ, onOptionChanged);
    //ngui->addVariable("Rotation XYZ", viewport.show_rotation_XYZ, onOptionChanged);
    ngui->addVariable("Rotation center", viewport.show_rotation_center, onOptionChanged);
    //ngui->addVariable("Rotation speed", viewport.rotation_speed, onOptionChanged);

    //ngui->addVariable("Draw surfaces separately", window.drawEverySurfaceSeparately, [&]
    //{
    //    if (On_OptionChanged_drawEverySurfaceSeparately) On_OptionChanged_drawEverySurfaceSeparately();
    //});
    //ngui->addVariable<bool>("Face-based", [&](bool checked)
    //{
    //    viewport.face_based = checked;
    //    viewport.isDirty = true;
    //}, [&]()
    //{
    //    return viewport.face_based;
    //});
    ngui->addVariable<bool>("Inverted normals", [&](bool checked)
    {    
        viewport.invert_normals = checked;   
        viewport.isObjectsDataDirty = true;
    }, [&]()
    { 
        return viewport.invert_normals;
    }); 
    ngui->addVariable<bool>("Crease normals", [&](bool checked)
    {
        viewport.crease_normals = checked;
        viewport.isObjectsDataDirty = true;
        if (On_OptionChanged_Draw_Crease) On_OptionChanged_Draw_Crease(viewport.crease_normals, viewport.crease_normals_angle);
    }, [&]()
    { 
        return viewport.crease_normals;
    });
    ngui->addVariable<D>(">   angle", [&](D angle)
    {
        viewport.crease_normals_angle = angle;
        viewport.isObjectsDataDirty = true;
        if (On_OptionChanged_Draw_Crease) On_OptionChanged_Draw_Crease(viewport.crease_normals, viewport.crease_normals_angle);
    }, [&]()
    {
        return viewport.crease_normals_angle;
    });
    ngui->addVariable("Hide menu", window.hide_menu);
    //ngui->addVariable("Hide menu on mouse move", window.hide_menu_on_mouse);
    ngui->addVariable("Trace FPS", window.showfps);
    //ngui->addVariable("Trace draw events", window.showDrawEventInConsole);
    //ngui->addVariable("Skip fast draws", window.skipFastDraws);
}

void Options::AddOptions_Common(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    Viewport& viewport = window.viewport;

    // v0 - use standard transparent/non-transparent
    // non-transparent draws as fast as possible, but looks a bit different from transparent version, and dots looks agly
    //ngui->addVariable("Transparent", openglOptions.Transparent.Enabled, [&]
    //{
    //    if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(true);
    //});
    // v1 - hide hidden layer for non-transparent 
    //  this make switching between modes nice, and additionally non-transparent will draw points better
    //  the back-side of better draw quality is performance penalty by avarage 20%, what is good trade for better quality
    ngui->addVariable("Transparent", openglOptions.Transparent.Layer1Hidden.Enabled, [&]
    {
        if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(false);
    });

    //ngui->addVariable("Simplify mesh", options.MeshSimplification.Enabled, [&]
    //{
    //    if (On_OptionChanged_MeshSimplification) On_OptionChanged_MeshSimplification();
    //});
}

void Options::AddOptions_Viewer_Show(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    Viewport& viewport = window.viewport;

    auto onOptionChanged = [&]
    {
        //if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(true);
        //viewport.isObjectsDataDirty = true;

    };
    auto onOptionChanged2 = [&]
    {
        //if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(false);
        //viewport.isObjectsDataDirty = true;
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };

    ngui->addGroup("Show");
    ngui->addVariable("Wireframe", viewport.show_wireframe, onOptionChanged);
    ngui->addVariable("Faces", viewport.show_faces, onOptionChanged);
    ngui->addVariable("Borders", options.Draw.Mesh.Borders, onOptionChanged2); 
    ngui->addVariable("Topology", options.MeshesTopology.draw.Enabled, onOptionChanged2);
    //ngui->addVariable("Texture", viewport.show_texture, onOptionChanged);
}


void Options::AddOptions_DrawDebugInfo(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };
    MeshLogicOptions_Draw_Mesh& _ = options.Draw.Mesh;

    ngui->addGroup("");
    ngui->addVariable("Vertex ids", _.Id_Vertex, onOptionChanged);
    ngui->addVariable("Edge ids", _.Id_Edge, onOptionChanged);
    ngui->addVariable("Face ids", _.Id_Face, onOptionChanged);
    ngui->addVariable("Mesh ids", _.Id_Mesh, onOptionChanged);
    ngui->addVariable("Loops ids", _.Id_MeshLoop, onOptionChanged);
    ngui->addVariable("Obj ids", _.Obj_id, onOptionChanged);
    ngui->addVariable("Obj names", _.Obj_name, onOptionChanged);

    ngui->addGroup("");
    ngui->addVariable("Border indexes", _.BorderIndexes, onOptionChanged);
    ngui->addVariable("Border edge ids", _.BorderLabels, onOptionChanged);
    ngui->addVariable("Border sharpness", _.BorderSharpnessDots, onOptionChanged);
    ngui->addVariable("Border sharpness angle", _.BorderSharpnessDotsAngle, onOptionChanged);
    ngui->addVariable("Border angles", _.BorderAngles, onOptionChanged);

    //ngui->addGroup("Normals");
    //ngui->addVariable("> faces", _.Normals_Faces, onOptionChanged);
    //ngui->addVariable("> vertixes", _.Normals_Vertixes, onOptionChanged);
    //window.ngui->addVariable("> faces caluclated", _.Normals_Faces_Calculated, onOptionChanged);

    MeshLogicOptions_Topology_Draw& __ = options.MeshesTopology.draw;
    ngui->addGroup("");
    ngui->addVariable("Topology single curves", __.drawSingleCurves, onOptionChanged);
    ngui->addVariable("Topology conn. curves", __.drawConnectedCurves, onOptionChanged);
    ngui->addVariable("Topology conn ids", __.drawConnectionsIds, onOptionChanged);
    ngui->addVariable("Topology segment ids", __.drawSegmenstIds, onOptionChanged);
    ngui->addVariable("Topology curves ids", __.drawCurvesIds, onOptionChanged);
    ngui->addVariable("Topology edge counts", __.drawSegmentsPointsCount, onOptionChanged);
    ngui->addVariable("Topology dots", __.drawDots, onOptionChanged);
    ngui->addVariable("Topology dots ids", __.drawDotsIds, onOptionChanged);
    //ngui->addVariable("Topology dots seg. ids", __.drawDotsSegmentsIds, onOptionChanged);
    //ngui->addVariable("Topology dots crv. ids", __.drawDotsCurvesIds, onOptionChanged);

}

void Options::AddOptions_Highlight(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };
    auto calbackOptionChanged2 = [&]
    {
        options.Draw.Mesh.Highlight.Enabled = true;
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };
    MeshLogicOptions_Draw_Mesh::HighligtWhat& _ = options.Draw.Mesh.Highlight;

    ngui->addGroup("Highlight");
    ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
    ngui->addVariable<MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType>(">   what", _.What, calbackOptionChanged2)->setItems(_.WhatTypeStr);
    ngui->addVariable(">   id", _.id, calbackOptionChanged2)->setTooltip("Enter one or few ids separated by any symbol");
    ngui->addGroup("Show only");
    window.nanogui->addVariable(">   mesh ids", ShowOnlyMeshIds, [&]
    {
        if (On_OptionChanged_File) On_OptionChanged_File(false, false, false);
        window.align_camera_center();
    })->setTooltip("Enter one or few ids separated by any symbol");

    window.nanogui->addVariable(">   obj ids", ShowOnlyObjectIds, [&]
    {
        if (On_OptionChanged_File) On_OptionChanged_File(false, false, false);
        window.align_camera_center();
    })->setTooltip("Enter one or few ids separated by any symbol");

    window.nanogui->addButton("select", [&]
    {
        if (On_SelectByMouse_ShowOnly) On_SelectByMouse_ShowOnly();
    });
}

void Options::AddOptions_OpenGL(OpenGL_Window& window, bool show_tech_options, bool show_visual_options)
{
    auto ngui = window.nanogui;
    auto onOptionChanged_ReinitOpenGL = [&]()
    {
        if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(true);
    };
    auto onOptionChanged = [&]()
    {
        if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(false);
    };


    //
    // Tech options
    //
    if (show_tech_options)
    {
        OpenGL_Options_WindowHint& _w = openglOptions.WindowHint;
        ngui->addGroup("WindowHint");
        ngui->addVariable("OpenGL version MAJOR", _w.OpenGL_version_MAJOR, false);
        ngui->addVariable("OpenGL version MINOR", _w.OpenGL_version_MINOR, false);
        ngui->addVariable("DEPTH_BITS", _w.DEPTH_BITS, false);
        ngui->addVariable("STENCIL_BITS", _w.STENCIL_BITS, false);

        OpenGL_Options_Performance& _p = openglOptions.Performance;
        ngui->addGroup("Performance");
        ngui->addVariable("use viewport layers", _p.use_viewport_layers, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("use nanogui layers", _p.use_nanogui_layers, onOptionChanged_ReinitOpenGL);
        //ngui->addVariable("unindex depth test", _p.unindex_depth_test, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("compact colors", _p.compactColors, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("compact normals", _p.compactNormals, onOptionChanged_ReinitOpenGL);

        OpenGL_Options_Offsets& _off = openglOptions.Offsets;
        ngui->addGroup("Offsets");
        ngui->addVariable("wireframe factor", _off.wireframe_factor, onOptionChanged);
        ngui->addVariable("wireframe units", _off.wireframe_units, onOptionChanged);

        OpenGL_Options_Sizes& _s = openglOptions.Sizes;
        ngui->addGroup("Sizes");
        ngui->addVariable("point size", _s.point_size, onOptionChanged);
        ngui->addVariable("point size MAX", _s.point_size_MAX, onOptionChanged);
        ngui->addVariable("line size", _s.line_size, onOptionChanged);
        ngui->addVariable("lineBold size", _s.lineBold_size, onOptionChanged);
        ngui->addVariable("wireframe size", _s.wireframe_size, onOptionChanged);
        ngui->addVariable(">   point size factor", _s.wireframe_pointsize_factor, onOptionChanged)->setTooltip("if wireframe is visible - increase size of points to make them more visible");
        ngui->addVariable(">   line size factor", _s.wireframe_linewidth_factor, onOptionChanged)->setTooltip("if wireframe is visible - increase thikness of lines to make them more visible");

        OpenGL_Options_Antialising& _a = openglOptions.Antialiasing;
        ngui->addGroup("Antialiasing");
        ngui->addVariable("Enabled", _a.Enabled, onOptionChanged);
        ngui->addVariable("smooth lines", _a.LineSmoothingEnabled, onOptionChanged);
        ngui->addVariable(">   smooth hidden", _a.LineSmoothingEnabled_inHiddenLayers, onOptionChanged);
        ngui->addVariable("smooth polygons", _a.PolygonSmoothingEnabled, onOptionChanged);
    }

    //
    // Visual options
    //
    if (show_visual_options)
    {
        OpenGL_Options_Light& _l = openglOptions.Light;
        ngui->addGroup("Light");
        ngui->addVariable("shininess", _l.shininess, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("shininess intensity", _l.shininess_intensity, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("lighting factor", _l.lighting_factor, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("alpha BackSideCoeff", _l.alpha_BackSideCoeff, onOptionChanged_ReinitOpenGL);
        //ngui->addVariable("lighting.x", , onOptionChanged_ReinitOpenGL);
        //ngui->addVariable("lighting.y", , onOptionChanged_ReinitOpenGL);
        //ngui->addVariable("lighting.z", , onOptionChanged_ReinitOpenGL);

        OpenGL_Options_Colors& _c = openglOptions.Colors;
        ngui->addGroup("Colors");
        ngui->addVariable("wireframe blend", _c.wireframe_blend, onOptionChanged);
        ngui->addVariable("back side color factor", _c.backSideColorFactor, onOptionChanged_ReinitOpenGL)->setTooltip("how upfront face color will differ from front face color. \nfor example value of 0.5 will half color produced for front face, thus make it more dark");;


        OpenGL_Options_Transparent& _ = openglOptions.Transparent;
        ngui->addGroup("Transparent");
        ngui->addVariable("Enabled", _.Enabled, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("sort triangles", _.sortTriangles, onOptionChanged);
        //ngui->addVariable<OpenGL_Options_Transparent::SortAlgorithm>("> algorithm", _.sortTriangles_sort_algorithm, onOptionChanged)->setItems(_.SortAlgorithmStr);
        //ngui->addVariable("> show time", _.sortTriangles_showtime_inconsole, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("alpha BackSideCoeff", _.alpha_BackSideCoeff, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("shininess", _.shininess, onOptionChanged_ReinitOpenGL);
        ngui->addVariable("lighting factor", _.lighting_factor, onOptionChanged_ReinitOpenGL);

        ngui->addVariable("Layer.Depth", _.Layer0Depth.Enabled, onOptionChanged);

        ngui->addVariable("Layer.Hidden", _.Layer1Hidden.Enabled, onOptionChanged);
        ngui->addVariable(">   alpha lines", _.Layer1Hidden.alpha_lines, onOptionChanged_ReinitOpenGL);
        ngui->addVariable(">   alpha back", _.Layer1Hidden.alpha_back, onOptionChanged_ReinitOpenGL);
        ngui->addVariable(">   alpha front", _.Layer1Hidden.alpha_front, onOptionChanged_ReinitOpenGL);
        ngui->addVariable(">   point size factor", _.Layer1Hidden.pointsize_factor, onOptionChanged_ReinitOpenGL);
        ngui->addVariable(">   smooth lines", openglOptions.Antialiasing.LineSmoothingEnabled_inHiddenLayers, onOptionChanged);

        ngui->addVariable("Layer.Visible (back)", _.Layer2Visible.Enabled_Back, onOptionChanged);
        ngui->addVariable(">   alpha", _.Layer2Visible.alpha_back, onOptionChanged_ReinitOpenGL);

        ngui->addVariable("Layer.Visible (front)", _.Layer2Visible.Enabled_Front, onOptionChanged);
        ngui->addVariable(">   alpha", _.Layer2Visible.alpha_front, onOptionChanged_ReinitOpenGL);

        //OpenGL_Options_Texture& _t = openglOptions.Texture;
        //ngui->addGroup("Texture");
        //ngui->addVariable("Enabled", _t.Enabled, onOptionChanged);
    }
    //ngui->addGroup("");
    window.nanogui->addButton("reset changes", [&]
    {
        SetOpenGLOptions();
        window.nanogui->refresh();
        if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(true);
        if (On_OptionChanged_OpenGL) On_OptionChanged_OpenGL(false);
    });
}


/*********************************************************/
/**** CONSTRAINS ****************************************/
/*********************************************************/

void Options::AddOptions_Constrain(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    ngui->addGroup("Constrain");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Constrain) On_OptionChanged_Constrain();
    };
    auto calbackOptionChanged2 = [&]
    {
        if (On_OptionChanged_Divider) On_OptionChanged_Divider();
    };
    MeshLogicOptions_Constrains& _ = options.Constrains;

    ngui->addVariable("Weights are relative to length", _.WeightIsRelativeToEdgeLength, onOptionChanged);
    ngui->addVariable("Correct in angles", _.CorrectInAngles, onOptionChanged);
    ngui->addVariable(">   percent", _.CorrectInAngles_Percent, onOptionChanged);
    ngui->addVariable(">   percent is dynamic", _.CorrectInAngles_Percent_IsDynamic, onOptionChanged);
    ngui->addVariable(">   percent is progressive", _.CorrectInAngles_Percent_isProgressive, onOptionChanged);
    ngui->addVariable(">   straightTo90_angleX2", _.CorrectInAngles_straightTo90_angleX2, onOptionChanged);
    ngui->addVariable(">   force_4side_to_straightTo90", _.CorrectInAngles_force_4side_to_straightTo90, onOptionChanged);
    ngui->addVariable(">   skipp_last_edges_count", _.CorrectInAngles_skipp_last_edges_count, onOptionChanged);
    ngui->addVariable(">   forceTo0  90to0", _.CorrectInAngles_ForceTo0_90to0, onOptionChanged);
    ngui->addVariable(">   debug", _.CorrectInAngles_Debug, onOptionChanged);

    ngui->addVariable("Add contrains in sharp edges", _.AddContrainsInSharpEdges, onOptionChanged);
    ngui->addVariable(">   sharp edge angle", _.AddContrainsInSharpEdges_sharpAngle, onOptionChanged)->setUnitAngle();



    MeshLogicOptions_Draw_Constrains& __ = options.Draw.Cons;

    ngui->addVariable("Debug", _.DebugEnabled, onOptionChanged);
    ngui->addVariable(">   show constrains", __.ShowBorderConstrains, calbackOptionChanged2);
    ngui->addVariable(">   highlight constrained faces", __.HighlightConstrainedFaces, calbackOptionChanged2);
    ngui->addVariable(">   show edges sharpness", __.ShowEdgesSharpness, calbackOptionChanged2);
}

void Options::AddOptions_ConstrainUV(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    ngui->addGroup("Constrain UV");

    auto on_resolve = [&]
    {
        if (On_OptionChanged_Constrain) On_OptionChanged_Constrain();
    };
    auto on_redraw = [&]
    {
        if (On_OptionChanged_Divider) On_OptionChanged_Divider();
    };
    MeshLogicOptions_ConstrainsUV& _ = options.ConstrainsUV;
    //ngui->addVariable("Correct in angles", _.CorrectInAngles, onOptionChanged);

    ngui->addVariable("Debug", _.DebugEnabled, on_resolve);
    ngui->addVariable(">   show constrains", _.debug_showConstrains, on_redraw);
    ngui->addVariable(">   highlight constrained faces", _.debug_HighlightConstrainedFaces, on_redraw);
    ngui->addVariable(">   show sharpness", _.debug_showSharpness, on_redraw);
}

/*********************************************************/
/**** NROSY  FIELD ****************************************/
/*********************************************************/

void Options::AddOptions_SolverNrosy__short(OpenGL_Window& window, bool add_performance_options)
{
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    auto ngui = window.nanogui;
    ngui->addGroup("Solver NRosy");
    MeshLogicOptions_SolverNrosy& _ = options.Solver;
    ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
    if (add_performance_options)
    {
        ngui->addVariable("Test performance", _.TestPerformanceCall, onOptionChanged);
        ngui->addVariable("   > call times", _.TestPerformanceCall_repeatTimes, onOptionChanged);
    }
}

void Options::AddOptions_SolverNrosy(OpenGL_Window& window, bool showNrosyDrawOptions, bool showNrosyOptions)
{
    auto ngui = window.nanogui;
    // Add as separate window
    //ngui->addWindow(Vector2i(320, 10), "Solver properties");


    //
    // draw
    //
    if (showNrosyDrawOptions)
    {
        ngui->addGroup("Solver NRosy Visual");
        auto calbackOptionChanged_Draw = [&]
        {
            if (On_OptionChanged_Draw) On_OptionChanged_Draw();
        };
        MeshLogicOptions_Draw_Solver& __ = options.Draw.Solver;
        //ngui->addVariable("Highlight fixed p", nRosy_HighlightFixedP, calbackOptionChanged2);
        ngui->addVariable("Show directions", __.ShowDirections, calbackOptionChanged_Draw);
        ngui->addVariable<MeshLogicOptions_Draw_Solver::DirectionsStyle>(">   style", __.ShowDirectionsStyle, calbackOptionChanged_Draw)->setItems(__.DirectionsStyleStr);
        ngui->addVariable("Show singularities", __.ShowSingularities, calbackOptionChanged_Draw);
        ngui->addVariable("Show directions weights", __.ShowDirectionsWeights, calbackOptionChanged_Draw);
        ngui->addVariable("Show border Iso-lines", __.ShowBorderConstrainesIsoLines, calbackOptionChanged_Draw);
        ngui->addVariable(">   count", __.ShowBorderConstrainesIsoLinesCount, calbackOptionChanged_Draw);
        ngui->addVariable(">   intensivity", __.ShowBorderConstrainesIsoLinesCount_intensivity, calbackOptionChanged_Draw);
    }


    //
    // logic
    //
    if (showNrosyOptions)
    {
        auto onOptionChanged = [&]
        {
            if (On_OptionChanged_Solver) On_OptionChanged_Solver();
        };
        ngui->addGroup("Solver NRosy");
        MeshLogicOptions_SolverNrosy& _ = options.Solver;
        ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
        if (_.MeshSolverTypeStr.size() > 1)
        {
            ngui->addVariable<MeshLogicOptions_SolverNrosy::MeshSolverType>("Solver", _.Solver, onOptionChanged)->setItems(_.MeshSolverTypeStr);
        }
        #if alternative_solvers_SUPPORTED
        //ngui->addVariable<NRosyNum>("N", nRosy_N, onOptionChanged)->setItems(_.NRosyNumStr);
        //ngui->addVariable("Smoothness / Edges", _.SoftPercent, onOptionChanged);
        ngui->addVariable("Tolerance", _.Tolerance, onOptionChanged);
        ngui->addVariable<MeshLogicOptions_Solver::SolveForVectorsXYEnum>("solve for vectors", _.SolveForVectorsXY, onOptionChanged)->setItems(_.SolveForVectorsXYEnumStr);
        #endif

        #if EROW
        ngui->addVariable("fast generate Result Field", _.fast_Generate_Result_F, onOptionChanged); // this options is valid only if we can walk our data lineary by pointer
        #endif
        ngui->addVariable("fast sparce matrix creation", _.fast_sparce_matrix_creation, onOptionChanged);
        ngui->addVariable("sse sin cos", _.use_sse_cin_cos, onOptionChanged)->setTooltip("use sse for calculating sin() and cos()");
        //ngui->addVariable("adjust K", _.adjustK, onOptionChanged)->setTooltip("[RESEARCH]  decrease influence on edges with some degree");
        //ngui->addVariable(">   percent", _.adjustK_Percent, onOptionChanged)->setUnitPercent()->setTooltip("[RESEARCH]  percent of changing K base on surface curvature");
        //ngui->addVariable(">   use cos", _.adjustK_useCos, onOptionChanged)->setTooltip("[RESEARCH]  use cos (true) or use sin (false)");
        //ngui->addVariable(">   quadric", _.adjustK_quadric, onOptionChanged)->setTooltip("[RESEARCH]  quadrify adjustment (without this it will be linear change)");

        #if NPolyVectorFieldSolverGeneral_SUPPORTED
        ngui->addVariable("compute only once", _.compute_only_once, onOptionChanged);
        ngui->addVariable("ignore_x when possible", _.ignore_x, onOptionChanged);
        //ngui->addVariable(">   max correction angle", _.ignore_x__max_correction_angle, onOptionChanged)->setUnitAngle();
        //ngui->addVariable("ignore_x always", _.ignore_x_always, onOptionChanged); 
        #endif

        ngui->addVariable("sort_complex_vectors", _.sort_complex_vectors, onOptionChanged);
        ngui->addVariable("NormalizeNrosyField fix field directions", _.NormalizeNrosyField_sort_field_directions, onOptionChanged);
        ngui->addVariable("check angle correction validity", _.check_angle_correction_validity, onOptionChanged);

        ngui->addVariable("DebugEnabled", _.DebugEnabled, onOptionChanged);
        ngui->addVariable("   > trace solver performance per mesh", _.debug_trace_solver_performance_per_mesh, onOptionChanged);
        ngui->addVariable("Test performance", _.TestPerformanceCall, onOptionChanged);
        ngui->addVariable("   > call times", _.TestPerformanceCall_repeatTimes, onOptionChanged);

    }

    // call to generate menu
    //window.screen->performLayout();

    int i = 2;
};



void Options::AddOptions_LDLTFloatSolver(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    ngui->addGroup("LDLTFloatSolver");
    MeshLogicOptions_LDLTFloatSolver& ___ = options.LDLTFloatSolver;
    ngui->addVariable("fast factorize_preordered", ___.fast_factorize_preordered, onOptionChanged);
    ngui->addVariable(">   multithreading", ___.fast_factorize_preordered__multithreading, onOptionChanged);
    ngui->addVariable(">   use AVX", ___.fast_factorize_preordered_use_AVX, onOptionChanged);
    ngui->addVariable("fast permutation", ___.fast_permutation, onOptionChanged);
    ngui->addVariable("DebugEnabled", ___.DebugEnabled, onOptionChanged);
    ngui->addVariable(">   trace factorize_preordered", ___.debug_trace_factorize_preordered, onOptionChanged);
    ngui->addVariable(">   skip_permuting", ___.debug_skip_permuting, onOptionChanged);
    ngui->addVariable(">   trace iterations count", ___.debug_trace_iterationsCount, onOptionChanged);
    ngui->addVariable(">   trace compute iterations count", ___.debug_trace_compute_iterationsCount, onOptionChanged);
}

void Options::AddOptions_MeshLinearSolver(OpenGL_Window& window)
{
    #if MeshLinearSolver_SUPPORTED
    auto ngui = window.ngui;
    ngui->addGroup("Mesh Linear Solver");
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    MeshLogicOptions_MeshLinearSolver& _ = options.MeshLinearSolver;
    ngui->addVariable("Debug", _.DebugEnabled, onOptionChanged);
    ngui->addVariable(">   debugStepNum", _.debug_debugStepNum, onOptionChanged)->setSpinnable(true);
    #endif
}

void Options::AddOptions_LinearEquationSolver(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    MeshLogicOptions_LinearEquationSolver& _ = options.LinearEquationSolver;
    #if alternative_solvers_SUPPORTED
    ngui->addGroup("LinearEquationSolver");
    ngui->addVariable<MeshLogicOptions_LinearEquationSolver::LinearEquationSolverType>("LinearEquationSolver", _.Solver, onOptionChanged)->setItems(_.LinearEquationSolverTypeStr);
    #endif
}
/*********************************************************/
/**** AngleBound  ************************************************/
/*********************************************************/

void Options::AddOptions_SolverAngleBound__short(OpenGL_Window& window, bool add_performance_options)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    ngui->addGroup("Solver AngleBound");
    MeshLogicOptions_SolverAngleBound& _ = options.SolverAngleBound;
    ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
    if (add_performance_options)
    {
        ngui->addVariable("Test performance", _.TestPerformanceCall, onOptionChanged);
        ngui->addVariable("   > call times", _.TestPerformanceCall_repeatTimes, onOptionChanged);
    }
}

void Options::AddOptions_SolverAngleBound(OpenGL_Window& window)
{
    auto ngui = window.nanogui;
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    ngui->addGroup("Solver AngleBound");
    MeshLogicOptions_SolverAngleBound& _ = options.SolverAngleBound;
    ngui->addVariable("Enabled", _.Enabled, onOptionChanged);
    ngui->addVariable<MeshLogicOptions_SolverAngleBound::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)->setItems(_.AlgorithmTypeStr);
    ngui->addVariable("reuse ordering", _.reuse_ordering, onOptionChanged)->setTooltip("reuse prdering from NSolver to avoid same calculations - speed optimization");
    ngui->addVariable("iterations", _.iterations_count, onOptionChanged);
    ngui->addVariable("thetaMin", _.thetatMin, onOptionChanged);
    ngui->addVariable("lambdaInit", _.lambdaInit, onOptionChanged);
    ngui->addVariable("lambdaMultFactor", _.lambdaMultFactor, onOptionChanged);
    ngui->addVariable("doHardConstraints", _.doHardConstraints, onOptionChanged);
    ngui->addVariable(">leave x original", _.leave_x_original, onOptionChanged);

    ngui->addVariable("DebugEnabled", _.DebugEnabled, onOptionChanged);
    ngui->addVariable(">   logMessages", _.debug_logMessages, onOptionChanged);
    ngui->addVariable("Test performance", _.TestPerformanceCall, onOptionChanged);
    ngui->addVariable(">   call times", _.TestPerformanceCall_repeatTimes, onOptionChanged);

};


/*********************************************************/
/**** UV  ************************************************/
/*********************************************************/
void Options::AddOptions_SolverUV__short(OpenGL_Window& window, bool add_performance_options)
{
    auto or_resolve = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    auto ngui = window.nanogui;
    MeshLogicOptions_SolverUV& _ = options.SolverUV;
    ngui->addGroup("Solver UV");
    ngui->addVariable("Enabled", _.Enabled, or_resolve);
    if (add_performance_options)
    {
        ngui->addVariable("Test performance", _.TestPerformanceCall, or_resolve);
        ngui->addVariable(">   call times", _.TestPerformanceCall_repeatTimes, or_resolve);
    }
}

void Options::AddOptions_SolverUV(OpenGL_Window& window)
{
    auto ngui = window.nanogui;

    auto or_resolve = [&]
    {
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    auto on_redraw = [&]
    {
        if (On_OptionChanged_Draw) On_OptionChanged_Draw();
    };
    MeshLogicOptions_SolverUV& _ = options.SolverUV;

    ngui->addGroup("Solver UV");
    ngui->addVariable("Enabled", _.Enabled, or_resolve);
    #if UVmiqSolver_SUPPORTED
    window.nanogui->addVariable<MeshLogicOptions_SolverUV::AlgorithmType>("Algorithm", _.Algorithm, or_resolve)->setItems(_.AlgorithmTypeStr);
    #endif

    ngui->addVariable("DebugEnabled", _.DebugEnabled, on_redraw);
    ngui->addVariable(">   show uv", _.debug_show_uv, on_redraw);
    ngui->addVariable(">   >   on borders", _.debug_show_uv__on_borders, on_redraw);
    ngui->addVariable(">   show border Iso-lines", _.debug_showBorderConstrainesIsoLines, on_redraw);
    ngui->addVariable(">   >   count", _.debug_showBorderConstrainesIsoLines__count, on_redraw);
    ngui->addVariable(">   >   intensivity", _.debug_showBorderConstrainesIsoLines__intensivity, on_redraw);
    ngui->addVariable(">   trace solver performance per mesh", _.debug_trace_solver_performance_per_mesh, or_resolve);
    ngui->addVariable("Test performance", _.TestPerformanceCall, or_resolve);
    ngui->addVariable(">   call times", _.TestPerformanceCall_repeatTimes, or_resolve);
}


/*********************************************************/
/**** MESH ***********************************************/
/*********************************************************/

void Options::AddOptions_Mesh(OpenGL_Window& window)
{
    window.nanogui->addGroup("Mesh");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Mesh) On_OptionChanged_Mesh();
    };
    MeshLogicOptions_Mesh& _ = options.Mesh;

    window.nanogui->addVariable<MeshLogicOptions_Mesh::computeKType>("computeK", _.computeK, onOptionChanged)
        ->setItems(_.computeKTypeStr)
        ->setTooltip("Type of computing K (angle between triangles)");
    window.nanogui->addVariable("SSEEnabled", _.SSEEnabled, onOptionChanged);
    window.nanogui->addVariable(">   triangle_triangle_adjacency", _.sse__triangle_triangle_adjacency_preprocess, onOptionChanged);
    window.nanogui->addVariable("Split doubled border triangles", _.SplitNakedTriangleWith2NakedEdge, onOptionChanged)->setTooltip("Split triangles that have 2 border edges  \nThis will clarify building constrains for solvers  ");
    window.nanogui->addVariable("Optimize vertex indexes for GPU", _.OptimizeVertexIndexesForGPU, onOptionChanged);
    window.nanogui->addVariable("Heal", _.Heal, onOptionChanged);
}

void Options::AddOptions_MeshHeal(OpenGL_Window& window)
{
    window.nanogui->addGroup("Mesh heal");

    MeshLogicOptions_MeshHeal& _ = options.MeshHeal;

    window.nanogui->addVariable("Delete zero triangles", _.PolygonMesh_DeleteZeroTriangles);
    window.nanogui->addVariable("Delete unattached vertexes", _.PolygonMesh_DeleteUnattachedVertexes);

    window.nanogui->addVariable("Merge vertexes", _.MergeVertexes);
    window.nanogui->addVariable(">   tol edge percent", _.MergeVertexes_tol_edge_percent);
    window.nanogui->addVariable("Flip normals", _.FlipNormals);
    window.nanogui->addVariable("Normalize normals", _.NormalizeNormals);
    window.nanogui->addVariable("Fill holes", _.FillHoles);
    window.nanogui->addVariable(">   max edges count", _.FillHoles_maxEdgesCount);
    window.nanogui->addVariable("Delete unattached triangles", _.DeleteUnattachedTriangles);
    window.nanogui->addVariable(">   max conected count", _.DeleteUnattachedTriangles_maxConectedCount);
    window.nanogui->addButton("Heal", [&]
    {
        if (On_MeshHeal_Executed)
        {
            On_MeshHeal_Executed();
        }
    });
}

/*********************************************************/
/**** MESH  TOPOLOGY ************************************/
/*********************************************************/



void Options::AddOptions_MeshTopology(OpenGL_Window& window)
{
    window.nanogui->addGroup("Meshes Topology");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_MeshTopology) On_OptionChanged_MeshTopology();
    };
    auto calbackOptionChanged_testPerfromance = [&]
    {
        if (On_OptionChanged_MeshTopology_TestPerformanceCall) On_OptionChanged_MeshTopology_TestPerformanceCall();
    };

    MeshLogicOptions_MeshesTopology& _ = options.MeshesTopology;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("check_PerfectConnections", _.check_PerfectConnections, onOptionChanged);
    window.nanogui->addVariable(">   fast", _.check_PerfectConnections_fast, onOptionChanged);
    window.nanogui->addVariable(">   tolerance", _.tol_PerfectConnections, onOptionChanged)->setUnitPercent()->setTooltip("percent of edge length");
    window.nanogui->addVariable("check_PartialConnections", _.check_PartialConnections, onOptionChanged);
    window.nanogui->addVariable(">   fast", _.check_PartialConnections_fast, onOptionChanged);
    window.nanogui->addVariable(">   tolerance", _.tol_PartialConnections, onOptionChanged)->setUnitPercent()->setTooltip("percent of edge length");
    window.nanogui->addVariable("check_InsideConnections", _.check_InsideConnections, onOptionChanged);
    window.nanogui->addVariable(">   tolerance", _.tol_InsideConnections, onOptionChanged)->setUnitPercent()->setTooltip("percent of edge length");
    window.nanogui->addVariable(">   tolerance dist", _.tol_InsideConnectionsDist, onOptionChanged)->setUnitPercent()->setTooltip("percent of relation start/end dist to segment length");
    window.nanogui->addVariable(">   max angle", _.max_angle_between_edges_InsideConnections, onOptionChanged)->setUnitAngle()->setTooltip("max angle between edges (in degrees)");
    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   trace pairs", _.debug_tracePairsInConsole, onOptionChanged);
    window.nanogui->addVariable(">   highligh point id", _.debug_highlighPoint_id, onOptionChanged);
    window.nanogui->addVariable(">   highligh connection id", _.debug_highlighConnection_id, onOptionChanged);
    window.nanogui->addVariable(">   highligh curve id", _.debug_highlighCurve_id, onOptionChanged);
    window.nanogui->addVariable("Test performance", _.debug_TestPerformanceCall, calbackOptionChanged_testPerfromance);
    window.nanogui->addVariable("   > call times", _.debug_TestPerformanceCall_repeatTimes, calbackOptionChanged_testPerfromance);
}


/*********************************************************/
/**** STREAM *********************************************/
/*********************************************************/

void Options::AddOptions_Stream(OpenGL_Window& window)
{
    window.nanogui->addGroup("Stream");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Stream) On_OptionChanged_Stream();
    };
    auto onOptionChanged_joiner = [&]
    {
        if (On_OptionChanged_Joiner) On_OptionChanged_Joiner();
    };
    MeshLogicOptions_MeshStream& _ = options.MeshStream;


    window.nanogui->addVariable("Draw streams", _.show_dividing_streams, onOptionChanged);
    window.nanogui->addVariable(">   show vid_eid_fid", _.show_dividing_streams__show_vid_eid_fid, onOptionChanged);
    window.nanogui->addVariable(">   show indexes", _.show_streamIndexes_atStart, onOptionChanged);
    window.nanogui->addVariable(">   show indexes in all points", _.show_streamIndexes_InAllPoints, onOptionChanged);
    window.nanogui->addVariable(">   show indexes every 10 points", _.show_streamIndexes_Every10Points, onOptionChanged);
    window.nanogui->addVariable(">   show iterations", _.show_stream_iterations, onOptionChanged);
    window.nanogui->addVariable("trace ExtendStream info in console", _.show_ExtendStream_info_in_console, onOptionChanged);
    //window.nanogui->addVariable("snap to vertex tol", _.snap_to_vertex_tol, onOptionChanged_joiner)->setUnitPercent()->setTooltip("percent of edge length. snap stream to vertex if distance to vertex is less than EdgeLength*tol");;
    //window.nanogui->addVariable("use sse41", _.use_sse41, onOptionChanged_joiner);
}

/*********************************************************/
/**** STREAM  TOPOLOGY **********************************/
/*********************************************************/

void Options::AddOptions_StreamTopology(OpenGL_Window& window)
{
    window.nanogui->addGroup("Streams Topology");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_StreamTopology) On_OptionChanged_StreamTopology();
    };
    auto calbackOptionChanged_testPerfromance = [&]
    {
        if (On_OptionChanged_StreamTopology_TestPerformanceCall) On_OptionChanged_StreamTopology_TestPerformanceCall();
    };

    MeshLogicOptions_MeshStreamsTopology& _ = options.MeshStreamsTopology;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("cut streams at cross", _.cut_streams_at_cross, onOptionChanged);
    window.nanogui->addVariable("   > show cutted streams", _.cut_streams_at_cross__show, onOptionChanged);
    window.nanogui->addVariable("show streams intersections", _.debug_show_streams_intersections, onOptionChanged);
    window.nanogui->addVariable("show streams segments", _.debug_show_streams_segments, onOptionChanged);
    window.nanogui->addVariable("show quads", _.debug_show_quads, onOptionChanged);
    window.nanogui->addVariable("show loops", _.debug_show_loops, onOptionChanged);
    window.nanogui->addVariable("show connections", _.debug_show_connections, onOptionChanged);

    window.nanogui->addVariable("Test performance", _.debug_TestPerformanceCall, calbackOptionChanged_testPerfromance);
    window.nanogui->addVariable("   > call times", _.debug_TestPerformanceCall_repeatTimes, calbackOptionChanged_testPerfromance);
}

/*********************************************************/
/**** DIVIDER *********************************************/
/*********************************************************/
void Options::AddOptions_Divider__short(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider");
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Divider) On_OptionChanged_Divider();
    };
    MeshLogicOptions_Divider& _ = options.Divider;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
}

void Options::AddOptions_Divider(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Divider) On_OptionChanged_Divider();
    };
    MeshLogicOptions_Divider& _ = options.Divider;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable<MeshLogicOptions_Divider::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)->setItems(_.AlgorithmTypeStr);
    window.nanogui->addVariable("Singularity extend count", _.SingularityExtendCount, onOptionChanged);
    window.nanogui->addVariable("fieldVectorsWeights is relative", _.fieldVectorsWeights_isrelative_to_fielddirection_norm, onOptionChanged);
    //window.ngui->addVariable("fieldVectorsWeights is norm", _.fieldVectorsWeights_is_fielddirection_norm, onOptionChanged);
    window.nanogui->addVariable("isoLines_ExtensionsCount", _.isoLines_ExtensionsCount, onOptionChanged);
    window.nanogui->addVariable("use_always_first_vector_as_prev", _.use_always_first_vector_as_prev, onOptionChanged);
    window.nanogui->addVariable("improve_stream_angles", _.improve_stream_angles, onOptionChanged);


    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   isoLines_show", _.Debug_isoLines_show, onOptionChanged);
    window.nanogui->addVariable(">   debug_point_index", _.Debug_point_index, onOptionChanged);
    window.nanogui->addVariable(">   debug_ri_index", _.Debug_ri_index, onOptionChanged);
}


void Options::AddOptions_StreamAdjuster(OpenGL_Window& window)
{
    window.nanogui->addGroup("StreamAdjuster");
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Joiner) On_OptionChanged_Joiner(); // same as joiner
    };
    MeshLogicOptions_MeshStreamsAdjuster& _ = options.StreamAdjuster;
    window.nanogui->addVariable("Adjust few times", _.iterate_adjustments, onOptionChanged)->setTooltip("Readjust adjustedStream few times to improve distance to desired connection point");
    window.nanogui->addVariable(">   adjustments count", _.iterate_adjustments__iterations_count, onOptionChanged)->setTooltip("how many times readjust adjustedStream to  improve accuracy");
    window.nanogui->addVariable(">   trace progress in console", _.iterate_adjustments__trace_progress_in_console, onOptionChanged)->setTooltip("trace progress of improving adjustedStream in console");
    //window.ngui->addVariable<MeshLogicOptions_MeshStreamsAdjuster::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)->setItems(_.AlgorithmTypeStr);
    window.nanogui->addVariable<MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType>("shift smoothing", _.ShiftSmoothing, onOptionChanged)
        ->setItems(_.ShiftSmoothingTypeStr)
        ->setTooltip("how smooth original stream will be shifted to connection point  \n\n  Linear - proportional along all length  \nSmooth - quadric equation like x^2  \nTriple - triple equation like x^3)");
    window.nanogui->addVariable<MeshLogicOptions_MeshStreamsAdjuster::TestDistMethod>("test dist method", _.testDistMethod, onOptionChanged)
        ->setItems(_.TestDistMethodStr)
        ->setTooltip("how to test desired dist from original stream to new adjusted stream");
    window.nanogui->addVariable("skip first points", _.skip_first_points, onOptionChanged)->setTooltip("skip some points at beginning of stream to preserve original direction at start");
    window.nanogui->addVariable("tollerance", _.tollerance, onOptionChanged)->setUnitPercent()->setTooltip("percent from desired distance from original-stream to adjusted-stream");
    window.nanogui->addVariable("max stream angle diff", _.max_stream_angle_diff, onOptionChanged)->setUnitAngle()->setTooltip("max angle difference for one line compare to stream, this angle should be lesser from 'max_change_angle' and not to high, since with higher angle there is a chance to go wrong direction");

    window.nanogui->addVariable("run direct calculations", _.run_direct_calculations, onOptionChanged)->setTooltip("run direct calculations before iterative when possible. \nsupports only Linear smooting method.");
    window.nanogui->addVariable<MeshLogicOptions_MeshStreamsAdjuster::DirectAlgorithmType>(">   algorithm", _.DirectAlgorithm, onOptionChanged)
        ->setItems(_.DirectAlgorithmTypeStr)
        ->setTooltip("how point should be calculated  \n\n  H - using ortogonal distance to stream (precise, sometimes fail) \n\nL - using intersection distance to stream (precise in 2D space only) \n\nX - using intersection distance to stream (high tolerance, stable)\n\nV - using intersection distance to stream (precise in 3D) - advanced and the best method");
    window.nanogui->addVariable(">   test streams count", _.direct_calculations__test_streams_count, onOptionChanged)->setTooltip("how many stream lines should be tested for each edge");
    window.nanogui->addVariable(">   replace new_lengthTo_pointOnPoints", _.direct_calculations__replace__new_lengthTo_pointOnPoints, onOptionChanged)->setTooltip("replace variable 'new_lengthTo_pointOnPoints' to this value");
    window.nanogui->addVariable(">   precise direction translation", _.direct_calculations__precise_direction_translation, onOptionChanged)->setTooltip("improves precision and thus direct method have smaller error and fail less times");


    window.nanogui->addVariable("run iterative calculations", _.run_iterative_calculations, onOptionChanged)->setTooltip("run iterative calculations if direct not enought good");
    window.nanogui->addVariable(">   max iterations", _.iterative_calculations__max_iterations_count, onOptionChanged)->setTooltip("maximum iterations allowed");
    window.nanogui->addVariable(">   max change angle", _.iterative_calculations__max_change_angle, onOptionChanged)->setUnitAngle()->setTooltip("max angle change for one point");
    window.nanogui->addVariable(">   max times failed in row", _.iterative_calculations__max_times_failed_in_row, onOptionChanged)->setTooltip("how many iterations in a row can be not better from last iteration");

    window.nanogui->addVariable("run hard calculations", _.run_hard_calculations, onOptionChanged)->setTooltip("run hard calculations if both direct and iterative methods not enought good");
    window.nanogui->addVariable(" >   edge iterations", _.hard_calculations__iterations_count_on_edge, onOptionChanged)->setTooltip("how many iterations on edges should be made if direction is going wrong direction");


    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   max debuging points", _.debug_singularityExtendNum, onOptionChanged)->setTooltip("maximum point to debug");
    window.nanogui->addVariable(">   debug stream index", _.debug_debugStreamIndex, onOptionChanged)->setUnitNum()->setTooltip("debug only one stream");
    window.nanogui->addVariable(">   debug point num", _.debug_debugPointNum, onOptionChanged)->setUnitNum()->setTooltip("debug only one point");
    window.nanogui->addVariable(">   debug iteration num", _.debug_debugIterationNum, onOptionChanged)->setUnitNum()->setTooltip("debug only one iteration");
    window.nanogui->addVariable(">   max times failed in row", _.debug_max_times_failed_in_row, onOptionChanged)->setTooltip("how many iterations in a row can be not better from last iteration (overrides normal value)");
    window.nanogui->addVariable(">   trace iterations", _.debug_trace_iterations, onOptionChanged)->setTooltip("trace debug info into console for all iterations");
    window.nanogui->addVariable(">   >   trace tests", _.debug_trace_iterations__tests, onOptionChanged)->setTooltip("trace debug info into console for all tests in iteration");
    window.nanogui->addVariable(">   draw accepted points", _.debug_draw_accepted_points, onOptionChanged)->setTooltip("draw accepted point with information: num, err, angle to stream");
    window.nanogui->addVariable(">   show original stream", _.debug_show_stream_to_connection_point, onOptionChanged)->setTooltip("show original stream until connection point in blue");;
    window.nanogui->addVariable(">   TestPerformanceCall1000Times", _.debug_TestPerformanceCall1000Times, onOptionChanged)->setTooltip("call algorithm 1000 times for profiling perfromance");

}

void Options::AddOptions_Joiner___short(OpenGL_Window& window)
{
    window.nanogui->addGroup("Joiner");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Joiner) On_OptionChanged_Joiner();
    };
    MeshLogicOptions_MeshStreamsJoiner& _ = options.Joiner;
    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("optimize connections", _.optimizeConnections, onOptionChanged)->setTooltip("replace longest connections with set of shortest connections. \ndeafult 'true'");
}

void Options::AddOptions_Joiner(OpenGL_Window& window)
{

    window.nanogui->addGroup("Joiner");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_Joiner) On_OptionChanged_Joiner();
    };
    MeshLogicOptions_MeshStreamsJoiner& _ = options.Joiner;



    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("max connection distance", _.maxDist_meshSize_multipliyer, onOptionChanged)->setUnitPercent()->setTooltip("percent of meshsize - maximum distance between streams to join.  \ndefault 1.1 = 110%");
    window.nanogui->addVariable("> is minimum of mesh avg size", _.maxDist_is_minimum_of_mesh_avg_size, onOptionChanged)->setTooltip("for very small meshsize proper connections could be lost, so to avoid problems with connections at singularities lets fix smaller possible connection distance to avg mesh size");
    window.nanogui->addVariable("max directions connection angle", _.max_connectionAngle_Directions, onOptionChanged)->setUnitAngle()->setTooltip("maximum angle between streams to join.  \ndefault 40 - for very high meshsize - we have to connect what is possible to avoid small meshes");
    window.nanogui->addVariable("max normals connection angle", _.max_connectionAngle_Normals, onOptionChanged)->setUnitAngle()->setTooltip("maximum angle between streams normals to join.  \ndefault 120 - for very sharp connectoins that can happend at bends - we have to connect what is possible");
    window.nanogui->addVariable("max connection length diff", _.max_lengthsDiff_percent, onOptionChanged)->setUnitPercent()->setTooltip("maximum percent between 2 connection length.  \ndefault 0.2 = 20%");
    window.nanogui->addVariable("max connection area", _.max_connectionArea_percent, onOptionChanged)->setUnitPercent()->setTooltip("maximum connection area compare to mesh area.  \ndefault 0.2 = 20% of total mesh size");
    window.nanogui->addVariable("min length/dist", _.min_lengthToDist_percent, onOptionChanged)->setUnitPercent()->setTooltip("maximum relation length to dist.  \ndefault 1.3 = 130% - length must be 1.3 times bigger from dist ");
    window.nanogui->addVariable("sort by length for small dists", _.sort_by_length_for_very_small_distances, onOptionChanged)->setTooltip("sort by travel length for cases when distance between streams are smaller from avg mesh size. \ndefault 'false'");
    window.nanogui->addVariable("optimize connections", _.optimizeConnections, onOptionChanged)->setTooltip("replace longest connections with set of shortest connections. \ndeafult 'true'");


    window.nanogui->addGroup("");
    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   debug stream index", _.debug_debugStreamIndex, onOptionChanged)->setUnitNum();;
    window.nanogui->addVariable(">   debug connection", _.debug_connection, onOptionChanged);
    window.nanogui->addVariable(">   >   streamIndex1", _.debug_connection_streamIndex1, onOptionChanged)->setUnitNum();;
    window.nanogui->addVariable(">   >   streamIndex2", _.debug_connection_streamIndex2, onOptionChanged)->setUnitNum();;
    window.nanogui->addVariable(">   debug every point of stream", _.debug_everyPointOfStream, onOptionChanged)->setTooltip("call algorithm 1000 times for profiling perfromance");
    window.nanogui->addGroup("");
    window.nanogui->addVariable(">   max connection distance", _.debug_meshSize_multipliyer, onOptionChanged)->setUnitPercent();
    window.nanogui->addVariable(">   TestPerformanceCall1000Times", _.debug_TestPerformanceCall1000Times, onOptionChanged)->setTooltip("call algorithm 1000 times for profiling perfromance");
    window.nanogui->addVariable(">   debug alternative path streamIndex", _.debug_alternativePath_debugStreamIndex, onOptionChanged)->setUnitNum();
    window.nanogui->addVariable(">   show connection points", _.debug_show_connection_points, onOptionChanged);
    window.nanogui->addVariable(">   show anchor ids", _.debug_show_connection_anchor_ids, onOptionChanged);
    window.nanogui->addVariable(">   show connnection indexes", _.debug_show_connection_indexes, onOptionChanged);

}

/*********************************************************/
/**** DIVIDER ITERATOR ************************************/
/*********************************************************/
void Options::AddOptions_DividerIterator__short(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider iterator");
    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_DividerIterator) On_OptionChanged_DividerIterator();
    };
    MeshLogicOptions_DividerIterator& _ = options.DividerIterator;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
}

void Options::AddOptions_DividerIterator(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider iterator");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_DividerIterator) On_OptionChanged_DividerIterator();
    };
    MeshLogicOptions_DividerIterator& _ = options.DividerIterator;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("max iterations count", _.max_iterations_count, onOptionChanged);
    window.nanogui->addVariable("join streams after each iteration", _.joinStreamsAfterEachIteration, onOptionChanged);


    window.nanogui->addGroup("");
    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   debug iteration index", _.debug_iteration_index, onOptionChanged)->setUnitNum();
    window.nanogui->addVariable(">   show iteration index on streams", _.debug_show_iteration_index_on_streams, onOptionChanged);
    window.nanogui->addVariable(">   trace iterations in console", _.debug_trace_iterations_in_console, onOptionChanged);
}


void Options::AddOptions_DividerOptimalConnector(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider optimal connections solver");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_DividerOptimalConnector) On_OptionChanged_DividerOptimalConnector();
    };
    MeshLogicOptions_DividerOptimalConnector& _ = options.DividerOptimalConnector;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable<MeshLogicOptions_DividerOptimalConnector::LoopDividingType>(">   connection type", _.DividingType, onOptionChanged)
        ->setItems(_.LoopDividingTypeStr)
        ->setTooltip("How to calculate connection between loops");
    window.nanogui->addVariable(">   max conflicts", _.max_conflicts, onOptionChanged);

    window.nanogui->addGroup("");
    window.nanogui->addVariable("Join conflicted loops", _.JoinConflictedLoops, onOptionChanged);
    window.nanogui->addVariable(">   reserve space for smaller", _.JoinConflictedLoops_ReserveSpaceForSmallInBig, onOptionChanged);
    window.nanogui->addVariable(">   >   accept ration", _.JoinConflictedLoops_ReserveSpaceForSmallInBig_acceptRation, onOptionChanged);
    window.nanogui->addVariable(">   balance sizes", _.JoinConflictedLoops_BalanceSizes, onOptionChanged);
    window.nanogui->addVariable(">   join borders", _.JoinConflictedLoops_JoinBordersWithStreams, onOptionChanged);

    window.nanogui->addGroup("");
    window.nanogui->addVariable("Join streams with dividing points", _.JoinStreamsWithDividingPoints, onOptionChanged);

    window.nanogui->addGroup("");
    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   show conflicted loops", _.debug_show_conflictedLoops, onOptionChanged);
    window.nanogui->addVariable(">   debug conflicted loop index", _.debug_debug_conflictedLoop_num, onOptionChanged)->setUnitNum()->setTooltip("debug conflicted loop index. if -1 then this options is disabled");
    window.nanogui->addVariable(">   max ReserveSpace iterations count", _.debug_ReserveSpaceForSmallInBig_max_iterations_count, onOptionChanged)->setUnitNum()->setTooltip("max conflicts process in ReserveSpaceForSmallInBig. if -1 then this options is disabled");
    window.nanogui->addVariable(">   show unresolved conflicts", _.debug_show_unresolved_conflicts, onOptionChanged);
    window.nanogui->addVariable(">   trace iterations in console", _.debug_trace_iterations_in_console, onOptionChanged);
    window.nanogui->addVariable(">   debug iteration", _.debug_debug_iteration_num, onOptionChanged)->setUnitNum()->setTooltip("debug iteration. if -1 then this options is disabled");
    window.nanogui->addVariable(">   show dividing points for joins", _.debug_show_dividingPointsForJoins, onOptionChanged);
}

void Options::AddOptions_DividerLogicConnector(OpenGL_Window& window)
{
    window.nanogui->addGroup("Divider logic connector");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_DividerLogicConnector) On_OptionChanged_DividerLogicConnector();
    };
    MeshLogicOptions_DividerLogicConnector& _ = options.DividerLogicConnector;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("connection angle", _.connectionAngle, onOptionChanged)->setUnitAngle();
    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   debug iteration", _.debug_debug_iteration_num, onOptionChanged)->setUnitNum()->setTooltip("debug iteration. if -1 then this options is disabled");
    window.nanogui->addVariable(">   show intersection points", _.debug_show_streams__intersection_points, onOptionChanged);
    window.nanogui->addVariable(">   show intersection connections", _.debug_show_streams__accepted_connections, onOptionChanged);
    window.nanogui->addVariable(">   stream global index", _.debug_streamGlobalIndex, onOptionChanged)->setUnitNum()->setTooltip("debug iteration. if -1 then this options is disabled");
    window.nanogui->addVariable(">   connection index", _.debug_connectionIndex, onOptionChanged)->setUnitNum()->setTooltip("debug iteration. if -1 then this options is disabled");

}

void Options::AddOptions_MeshCutter(OpenGL_Window& window)
{
    window.nanogui->addGroup("Mesh Cutter");

    auto onOptionChanged = [&]
    {
        if (On_OptionChanged_MeshCutter) On_OptionChanged_MeshCutter();
    };
    MeshLogicOptions_MeshCutter& _ = options.MeshCutter;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable("Improve stream lines", _.ImproveStreamLines, onOptionChanged);
    window.nanogui->addVariable(">   edge percent tolerance", _.ImproveStreamLines__edge_percent_tolerance, onOptionChanged)->setUnitPercent();
    window.nanogui->addVariable("Merge close points on edges", _.MergeClosePointsOnEdges, onOptionChanged);
    window.nanogui->addVariable(">   edge percent tolerance", _.MergeClosePointOnEdges__edge_percent_tolerance, onOptionChanged)->setUnitPercent();
    window.nanogui->addVariable(">   connection angle", _.MergeClosePointOnEdges__connection_angle, onOptionChanged)->setUnitAngle();
    window.nanogui->addVariable(">   max angle change below con-angle", _.MergeClosePointOnEdges__max_angle_change_belowConnectionAngle, onOptionChanged)->setUnitAngle();
    window.nanogui->addVariable(">   max angle change above con-angle", _.MergeClosePointOnEdges__max_angle_change_aboveConnectionAngle, onOptionChanged)->setUnitAngle();
    window.nanogui->addVariable("Improve mesh", _.ImproveMeshToConnectStreamLines, onOptionChanged);
    window.nanogui->addVariable(">   edge percent tolerance", _.ImproveMeshToConnectStreamLines__edge_percent_tolerance, onOptionChanged)->setUnitPercent();
    window.nanogui->addVariable(">   allow improvements on borders", _.ImproveMeshToConnectStreamLines__allow_improvements_on_borders, onOptionChanged);
    window.nanogui->addVariable(">   dissalow move other streams", _.ImproveMeshToConnectStreamLines__dissalow_move_other_streams, onOptionChanged);
    window.nanogui->addVariable("Find stream intersections", _.FindStreamIntersections, onOptionChanged);
    window.nanogui->addVariable("Cut mesh", _.CutMesh, onOptionChanged);
    window.nanogui->addVariable(">   use fast method", _.CutMesh_useFastMethod, onOptionChanged);


    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   trace removed vertexes", _.Debug_trace_removed, onOptionChanged);
    window.nanogui->addVariable(">   show merged close points", _.Debug_show_mergedclose_points, onOptionChanged);
    window.nanogui->addVariable(">   show move vertexes steps", _.Debug_show_steps, onOptionChanged);
    window.nanogui->addVariable(">   >   stop at step", _.Debug_show_steps__stop_at_step, onOptionChanged);
    window.nanogui->addVariable(">   show intersections", _.Debug_show_intersections, onOptionChanged);
    window.nanogui->addVariable(">   cutmesh show faces", _.Debug_cutmesh__show_intersecting_faces, onOptionChanged);
    window.nanogui->addVariable(">   >   only for face", _.Debug_cutmesh__show_intersecting_faces__show_only_for_face, onOptionChanged);
    window.nanogui->addVariable(">   cutmesh show edges stream holders", _.Debug_cutmesh__show_edges_that_are_recognized_as_stream_holders, onOptionChanged);
    window.nanogui->addVariable(">   show modified mesh", _.Debug_ShowModifiedMesh, onOptionChanged);
    window.nanogui->addVariable(">   >   only borders", _.Debug_ShowModifiedMesh_onlyborders, onOptionChanged);
}

/*********************************************************/
/**** MESHER *********************************************/
/*********************************************************/
void Options::Update_Mesher__saved_draw_cutted_mesh()
{

    Mesher__debug_draw_cuttedMesh = meshLogicOptions.Solver.Enabled
        && meshLogicOptions.Mesher.Enabled
        && meshLogicOptions.Mesher.DebugEnabled
        && meshLogicOptions.Mesher.debug_draw_cuttedMesh;

}
void Options::AddOptions_Mesher__short(OpenGL_Window& window)
{
    window.nanogui->addGroup("Mesher");

    auto onOptionChanged = [&]
    {
        bool savedvalue = Mesher__debug_draw_cuttedMesh;
        Update_Mesher__saved_draw_cutted_mesh();
        if (On_OptionChanged_Mesher) On_OptionChanged_Mesher(savedvalue != Mesher__debug_draw_cuttedMesh);
    };
    MeshLogicOptions_Mesher& _ = options.Mesher;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable<MeshLogicOptions_Mesher::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)
        ->setItems(_.AlgorithmTypeStr)
        ->setTooltip("  Streams - stream lines on cutted meshes (used for testing steams)\n\n  LSCM - create UV on cutted meshes\n\n  Laplacian - create UV on sub-mesh limited by loop");
    window.nanogui->addVariable("mesh size (absolute)", _.meshSize_absolute, onOptionChanged);
}
void Options::AddOptions_Mesher(OpenGL_Window& window)
{
    window.nanogui->addGroup("Mesher");

    auto onOptionChanged = [&]
    {
        bool savedvalue = Mesher__debug_draw_cuttedMesh;
        Update_Mesher__saved_draw_cutted_mesh();
        if (On_OptionChanged_Mesher) On_OptionChanged_Mesher(savedvalue != Mesher__debug_draw_cuttedMesh);
    };
    MeshLogicOptions_Mesher& _ = options.Mesher;

    window.nanogui->addVariable("Enabled", _.Enabled, onOptionChanged);
    window.nanogui->addVariable<MeshLogicOptions_Mesher::AlgorithmType>("Algorithm", _.Algorithm, onOptionChanged)
        ->setItems(_.AlgorithmTypeStr)
        ->setTooltip("  Streams - stream lines on cutted meshes (used for testing steams)\n\n  LSCM - create UV on cutted meshes\n\n  Laplacian - create UV on sub-mesh limited by loop");
    window.nanogui->addVariable("mesh size (absolute)", _.meshSize_absolute, onOptionChanged);
    window.nanogui->addVariable("mesh size (relative to triangles size)", _.meshSize_RelativeToTrianglesSize, onOptionChanged)->setUnitPercent()->setTooltip("percent of triangle edge length");;
    window.nanogui->addVariable(">   use relative mesh size", _.is_meshSize_RelativeToTrianglesSize, onOptionChanged);
    window.nanogui->addVariable("mesh size (min)", _.meshSize_min, onOptionChanged)->setUnitPercent()->setTooltip("percent of mesh size (absolute or relative)");;

    window.nanogui->addVariable("Debug Enabled", _.DebugEnabled, onOptionChanged);
    window.nanogui->addVariable(">   show mesh size", _.debug_show_meshsize_on_borders, onOptionChanged)->setTooltip("show mesh size on borders as dots");
    window.nanogui->addVariable(">   show cutted meshes", _.debug_draw_cuttedMesh, onOptionChanged)->setTooltip("show cutted mesh that was obtained by dividing original mesh into quadric peaces.\n suppresses drawing of original mesh in favor of cutted mesh - for easeir debuging.");
    window.nanogui->addVariable(">   debug mesh id", _.debug_debug_mesh_id, onOptionChanged)->setUnitNum()->setTooltip("make mesh only on specific mesh. if -1 then this options is disabled");
    window.nanogui->addButton("save debuged mesh to file            ", [&]
    {
        string filename = igl::file_dialog_save(_TEXT("*.obj\0*.obj"));//off\0*.off\0obj\0*.obj\0mp\0*.mp\0"
        if (utils::file::ExtractExtension(filename) != "obj") filename += ".obj";
        if (filename.length() == 0)
            return;
        if (On_Mesher_SaveDebugedMeshToFile)
        {
            On_Mesher_SaveDebugedMeshToFile(filename);
        }
    });


    window.nanogui->addVariable(">   show loop ids", _.debug_show_meshingLoops_ids, onOptionChanged)->setTooltip("show loopd ids on topology connections");
    window.nanogui->addVariable(">   debug loop id", _.debug_debug_meshingLoop_id, onOptionChanged)->setUnitNum()->setTooltip("debug specific loop");
    window.nanogui->addVariable(">   show divisioncount before", _.debug_show_divisioncount_before_adjustment, onOptionChanged)->setTooltip("show divisioncount before adjustment");
    window.nanogui->addVariable(">   show divisioncount after", _.debug_show_divisioncount_after_adjustment, onOptionChanged)->setTooltip("show divisioncount after adjustment");
    window.nanogui->addVariable(">   show meshing process", _.debug_show_meshing_process, onOptionChanged)->setTooltip("show all additional information that was produced during meshing");
    window.nanogui->addVariable(">   > show Y", _.debug_show_meshing_process_Y, onOptionChanged)->setTooltip("additionaly show Y axis what is not used for meshing - just for testing streams");
    window.nanogui->addVariable(">   show produced mesh", _.debug_show_produced_mesh, onOptionChanged)->setTooltip("show final mesh");
    window.nanogui->addButton("save quad mesh to file", [&]
    {
        string filename = igl::file_dialog_save(_TEXT("*.obj\0*.obj"));//off\0*.off\0obj\0*.obj\0mp\0*.mp\0"
        if (utils::file::ExtractExtension(filename) != "obj") filename += ".obj";
        if (filename.length() == 0)
            return;
        if (On_Mesher_SaveQuadMeshToFile)
        {
            On_Mesher_SaveQuadMeshToFile(filename);
        }
    });
}

/*********************************************************/
/**** APP ************************************************/
/*********************************************************/

void Options::AddOptions_App(OpenGL_Window& window)
{
    // lets use solver callback
    auto onOptionChanged = [&]
    {
        Update_OmpCpuCores();
        if (On_OptionChanged_Solver) On_OptionChanged_Solver();
    };
    window.nanogui->addVariable("Multithreading", IsOmpEnabled, onOptionChanged);
    //window.nanogui->addVariable("> omp hyperthreading", IsOmpHyperthreadingEnabled, onOptionChanged)->setTooltip("Your CPU has " + to_string(utils::cpu::coresCount) + " hardware threads. But if CPU has hyperthreading it is better to use only half of CPU threads to limit execution 1 thread per 1 core. It may improve performance by 10%.");
    //window.nanogui->addVariable("> omp dynamic thread adjustment", IsOmpDynamicThreadAdjustmentsEnabled, onOptionChanged)->setTooltip("OpenMP implementation may adjust the number of threads to use for executing parallel regions in order to optimize the use of system resources. It may improve performance by 5%.");
}


/*********************************************************/
/**** FILE ************************************************/
/*********************************************************/


void Options::AddPresetTestGroup(string presetGroupName)
{
    CurrentPresetFileIndex = 0;
    PresetFiles.push_back({ static_cast<int>(PresetFiles.size()), "", TestFiles::quad , TestFilesDensity::low , -1, presetGroupName });
}

void Options::AddPresetTestFile(TestFiles testFile, TestFilesDensity density, double meshSize, string loadOnlyObjectNames)
{
    PresetFiles.push_back({ static_cast<int>(PresetFiles.size()), "", testFile , density , meshSize ,loadOnlyObjectNames });
    // set first file as start file
    for (auto& file : PresetFiles)
    {
        if (file.meshSize < 0) continue; // skip group
        SetPresetFileIndex(file.Index);
        break;
    }
}
void Options::AddPresetTestFile(string testFile, double meshSize, string loadOnlyObjectNames)
{
    PresetFiles.push_back({ static_cast<int>(PresetFiles.size()), testFile, TestFiles::quad , TestFilesDensity::low , meshSize ,loadOnlyObjectNames });
    // set first file as start file
    for (auto& file : PresetFiles)
    {
        if (file.meshSize < 0) continue; // skip group
        SetPresetFileIndex(file.Index);
        break;
    }
}

void Options::SetPresetFileIndex(int index)
{
    CurrentPresetFileIndex = index;
    auto& file = PresetFiles[CurrentPresetFileIndex];
    if (file.customTestFile.empty())
    {
        TestFile = file.testFile;
        TestFileDensity = file.density;
    }
    else
    {
        CustomTestFile = file.customTestFile;
    }
    options.Mesher.meshSize_absolute = file.meshSize;
    LoadOnlyObjectNames = file.loadOnlyObjectNames;
}

string Options::GetTestFileName()
{
    if (!std::empty(CustomTestFile))
    {
        if (utils::file::Exists(CustomTestFile))
        {
            cout << "Loading custom test file: " << CustomTestFile << endl;
            string res = CustomTestFile;
            CustomTestFile = ""; // clear this custom file name to allow later selection of test files
            return res;
        }
        else
        {
            cout << "failed to load custom test file since it is no exists. filename: " << CustomTestFile << endl;
        }
    }
    return "K:\\Data\\obj_testQuadifier3d\\" + TestFilesStr[(int)TestFile] + "_" + TestFilesDensityStr[(int)TestFileDensity] + ".obj";
}
string Options::GetOptionFile(string filename)
{
    // expect file with template: "file_low.obj"
    // so first let get all before "_"
    size_t lastindex = filename.find_last_of("_");
    // if symbol "_" no found - lets get all before "."
    if (lastindex == std::string::npos)
    {
        lastindex = filename.find_last_of(".");
    }
    // if none of the symbols found - lets copy full filename
    if (lastindex == std::string::npos)
    {
        lastindex = filename.size();
    }
    string filename_withoutExtension = filename.substr(0, lastindex);
    return filename_withoutExtension + ".opt";
}

void Options::AddOptions_File(OpenGL_Window& window)
{
    // Add as separate window
    //window.ngui->addWindow(Vector2i(320, 10), "File properties");
    // Add new group
    //window.nanogui->addGroup("File");

    // File name
    window.nanogui->addVariable<TestFiles>("Obj", TestFile, [&]()
    {
        LoadOnlyObjectNames = "";
        if (On_OptionChanged_File) On_OptionChanged_File(true, false, true);
    })->setItems(TestFilesStr);

    // remove groups from 'TestFilesStr'
    vector<string>  TestFilesStrWithoutGroups;
    for (int i = 0; i < TestFilesStr.size(); i++)
    {
        const std::string &str = TestFilesStr[i];
        if (str.find("#") == 0) continue;
        TestFilesStrWithoutGroups.push_back(str);
    }
    TestFilesStr = TestFilesStrWithoutGroups;

    // File density
    window.nanogui->addVariable<TestFilesDensity>("Density", TestFileDensity, [&]
    {
        if (On_OptionChanged_File) On_OptionChanged_File(false, false, true);
    })->setItems(TestFilesDensityStr);
    window.nanogui->addVariable("Load only objects", LoadOnlyObjectNames, [&]
    {
        if (On_OptionChanged_File) On_OptionChanged_File(false, true, false);
    });
    //window.nanogui->addVariable("Test load file speed", TestSpeed_ReadFileFromObj, [&]
    //{
    //    if (On_OptionChanged_File) On_OptionChanged_File(false, false, false);
    //});
    //window.ngui->addVariable("Test load file speed", TestSpeed_AddMeshes, [&]
    //{
    //    if (On_OptionChanged_File) On_OptionChanged_File(false, true, false);
    //});

    //window.ngui->addVariable("Convert mesh to flat", ConvertMeshToFlatMesh, [&]
    //{
    //    if (On_OptionChanged_File) On_OptionChanged_File(false, true, false);
    //});
    window.nanogui->addVariable("Separate groups", SeparateGroups, [&]
    {
        if (On_OptionChanged_Mesh) On_OptionChanged_Mesh();
    });



    // obsolete - we dont use cahcing anymore
    //window.ngui->addVariable("Cache mesh on disk", PrecacheMeshFile, [&]
    //{
    //    if (On_OptionChanged_Mesh) On_OptionChanged_Mesh();
    //});


    // Expose a variable directly ...
    //window.ngui->addVariable("Elapsed ms: ", Elapsed);

    window.nanogui->addButton("Remember model location", [&]
    {
        if (On_RememberModelLocation_File)
        {
            On_RememberModelLocation_File();
        }
    });

    window.nanogui->addButton("Load obj file", [&]
    {
        //string filename = igl::file_dialog_open(_TEXT("*.obj\0*.obj"));//off\0*.off\0obj\0*.obj\0mp\0*.mp\0"
        string ext = "";
        for (auto supportedExt : MeshFile::SupportedFileExtensions())
        {
            if (!ext.empty()) ext += ";";
            ext += "*." + supportedExt;
        }
        ext = "All supported|" + ext;
        for (auto supportedExt : MeshFile::SupportedFileExtensions())
        {
            ext += "|*." + supportedExt + "|*." + supportedExt;
        }

        char text[2048];
        for (int i = 0; i < ext.size(); i++)
        {
            text[i] = ext[i];
            if (ext[i] == '|') text[i] = 0;
        }
        text[ext.size()] = 0;
        text[ext.size() + 1] = 0;

        //string filename = igl::file_dialog_open((char*)ext.c_str());
        string filename = igl::file_dialog_open(text);

        if (filename.length() == 0)
            return;
        if (On_OpenObjFile_File)
        {
            LoadOnlyObjectNames = "";
            On_OpenObjFile_File(filename);
        }
    });

};

void Options::AddOptions_TestFilesPreset(OpenGL_Window& window)
{
    //window.ngui->addGroup("Preset test files");

    for (int i = 0; i < PresetFiles.size(); i++)
    {
        if (PresetFiles[i].meshSize < 0)
        {
            window.nanogui->addGroup(PresetFiles[i].loadOnlyObjectNames);
        }
        else
        {
            window.nanogui->addButton(PresetFiles[i].ToString(*this), [&, i]
            {
                SetPresetFileIndex(PresetFiles[i].Index);
                if (On_OptionChanged_File) On_OptionChanged_File(true, false, true);
            });
        }
    }
}


/*********************************************************/
/**** TEST ************************************************/
/*********************************************************/
//
//bool Options::AddOptions_Test(OpenGL_Window& window)
//{
//    // AddSolverProperties
//    using nanogui::Alignment;
//    using nanogui::Arcball;
//    using nanogui::BoxLayout;
//    using nanogui::Button;
//    using nanogui::CheckBox;
//    using nanogui::Color;
//    using nanogui::ComboBox;
//    using nanogui::GLFramebuffer;
//    using nanogui::GroupLayout;
//    using nanogui::ImagePanel;
//    using nanogui::Label;
//    using nanogui::MessageDialog;
//    using nanogui::Orientation;
//    using nanogui::Popup;
//    using nanogui::PopupButton;
//    using nanogui::ProgressBar;
//    using nanogui::Screen;
//    using nanogui::Slider;
//    using nanogui::TextBox;
//    using nanogui::ToolButton;
//    using nanogui::VScrollPanel;
//    using nanogui::Widget;
//    using nanogui::Window;
//
//    //window.screen->size().
//    /* Initialize user interface */
//    //nanogui::Window *window = window.ngui->addWindow(Vector2i(0, 0), "Solver properties");
//    //window->setLayout(new GroupLayout());
//    //window->setId("SolverProps");
//    //window->setPosition(Vector2i(15, 15));
//    window.ngui->addGroup("Some group");
//    nanogui::Window *wnd = window.ngui->addWindow(Vector2i(0, 0), "Some window");
//    wnd->setLayout(new GroupLayout());
//    wnd->setPosition(Vector2i(1500, 0));
//    window.screen->performLayout();
//    return false;
//}



/*********************************************************/
/**** SHORTCUTS *****************************************/
/*********************************************************/

// It allows to change the degree of the field when a number is pressed
bool Options::callback_key_down(OpenGL_Window& window, int code, int modifier)
{
    unsigned char key = static_cast<unsigned char>(code);
    bool isModifier = modifier != 0;
    bool isModifierShift = modifier & 1;
    bool isModifierControl = modifier & 2;
    bool isModifierAlt = modifier & 4;

    //cout << "pressed   modifier=" << modifier << "   code = " << code << "   key = " << key << endl;

    if (!isModifier)
    {
        // change nRosy_N
        //switch (key)
        //{
            //case '1':
            //case '2':
            //case '3':
            //case '4':
            //case '5':
            //case '6':
            //case '7':
            //case '8':
            //case '9':
            //case '0':
            //    if (key == '1') nRosy_N = NRosyNum::N1;
            //    if (key == '2') nRosy_N = NRosyNum::N2;
            //    if (key == '3') nRosy_N = NRosyNum::N3;
            //    if (key == '4') nRosy_N = NRosyNum::N4;
            //    if (key == '5') nRosy_N = NRosyNum::N5;
            //    if (key == '6') nRosy_N = NRosyNum::N6;
            //    if (key == '7') nRosy_N = NRosyNum::N7;
            //    if (key == '8') nRosy_N = NRosyNum::N8;
            //    if (key == '9') nRosy_N = NRosyNum::N9;
            //    if (key == '0') nRosy_N = NRosyNum::N10;
            //    window.ngui->refresh();
            //    On_OptionChanged_Solver();
            //    return true;
            //case 'S':
            //    if (options.Solver.Solver == MeshLogicOptions_Solver::MeshSolverType::multiple_rounding)
            //        options.Solver.Solver = MeshLogicOptions_Solver::MeshSolverType::NPolyVectorFieldGeneral;
            //    else
            //        options.Solver.Solver = MeshLogicOptions_Solver::MeshSolverType::multiple_rounding;
            //    window.ngui->refresh();
            //    On_OptionChanged_Solver();
            //    return true;
        //}

        // change TestFileDensity
        switch (key)
        {
            case 'Q':
            case 'W':
            case 'E':
                if (key == 'Q') TestFileDensity = TestFilesDensity::low;
                if (key == 'W') TestFileDensity = TestFilesDensity::medium;
                if (key == 'E') TestFileDensity = TestFilesDensity::high;
                window.nanogui->refresh();
                if (On_OptionChanged_File) On_OptionChanged_File(false, false, true);
                return true;
        }


        // change mouse rotation type
        switch (key)
        {
            case 'M':
            {
                if (window.viewport.rotation_type == Viewport::RotationType::TRACKBALL)
                    window.viewport.set_rotation_type(Viewport::RotationType::ScreenXY);
                //else if (window.viewport.rotation_type == Viewport::RotationType::TWO_AXIS_VALUATOR_FIXED_UP)
                    //window.viewport.set_rotation_type(Viewport::RotationType::TRACKBALL_XY);
                else
                    window.viewport.set_rotation_type(Viewport::RotationType::TRACKBALL);
                window.nanogui->refresh();
                string rotations[3] = { "TRACKBALL", "TWO_AXIS_VALUATOR_FIXED_UP",  "ScreenXY" };
                cout << "Mouse rotation type changed to " << rotations[(int)window.viewport.rotation_type] << endl;
                break;
            }
        }

        // load next preset test file
        switch (key)
        {
            case 'T':
                if (PresetFiles.size() != 0)
                {
                    CurrentPresetFileIndex++; // increase index to next
                    if (CurrentPresetFileIndex >= PresetFiles.size()) CurrentPresetFileIndex = 0; // cycl index if it out of size
                    while (CurrentPresetFileIndex < PresetFiles.size() && PresetFiles[CurrentPresetFileIndex].meshSize < 0) CurrentPresetFileIndex++; // skip groups
                    if (CurrentPresetFileIndex < PresetFiles.size()) // if index is valid
                    {
                        SetPresetFileIndex(CurrentPresetFileIndex);
                        if (On_OptionChanged_File) On_OptionChanged_File(true, false, true);
                        window.nanogui->refresh();
                    }
                }
                return true;
        }

        //switch (key)
        //{
        //    case 'A':
        //    case 'S':
        //    case 'D':
        //    case 'I':
        //        if (key == 'A') meshLogicOptions.Divider.Algorithm = MeshLogicOptions_Divider::AlgorithmType::StreamAngles;
        //        if (key == 'S') meshLogicOptions.Divider.Algorithm = MeshLogicOptions_Divider::AlgorithmType::AvgStream;
        //        if (key == 'D') meshLogicOptions.Divider.Algorithm = MeshLogicOptions_Divider::AlgorithmType::BestDir;
        //        if (key == 'I') meshLogicOptions.Divider.improve_stream_angles = !meshLogicOptions.Divider.improve_stream_angles;
        //        window.ngui->refresh();
        //        On_OptionChanged_Divider();
        //        return true;
        //}

        //switch (key)
        //{
        //    case '1':
        //    case '2':
        //    case '3':
        //    case '4':
        //        if (key == '1') meshLogicOptions.StreamAdjuster.ShiftSmoothing = MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Linear;
        //        if (key == '2') meshLogicOptions.StreamAdjuster.ShiftSmoothing = MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Smooth;
        //        if (key == '3') meshLogicOptions.StreamAdjuster.ShiftSmoothing = MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Triple;
        //        if (key == '4') meshLogicOptions.StreamAdjuster.ShiftSmoothing = MeshLogicOptions_MeshStreamsAdjuster::ShiftSmoothingType::Quad;
        //        window.ngui->refresh();
        //        On_OptionChanged_Divider();
        //        return true;
        //}

        // next/prev test file
        switch (key)
        {
            case ',': // pressed '<'
            case '.': // pressed '>'
                int ti = static_cast<int>(TestFile);
                if (key == ',') ti--;// pressed '<'
                if (key == '.') ti++;// pressed '>'
                if (ti < 0) ti = TestFilesStr.size() - 1;
                if (ti > TestFilesStr.size() - 1) ti = 0;
                TestFile = static_cast<TestFiles>(ti);
                ShowOnlyMeshIds = "";
                ShowOnlyObjectIds = "";
                LoadOnlyObjectNames = "";
                window.nanogui->refresh();
                if (On_OptionChanged_File) On_OptionChanged_File(true, false, true);
                return true;
        }

        // show/hide all menus
        switch (code)
        {
            case 299: //F10
                window.hide_menu = !window.hide_menu;
                window.nanogui->refresh();
                return true;
        }

    }

    // change mesh size incrementaly
    switch (key)
    {
        case '-': //  pressed '-'
        case '=': //  pressed '+'
        {
            //
            //options.Mesher.meshSize_absolute
            //
            double incr = 0.1;
            if (isModifierShift) incr = 0.01;
            if (isModifierControl) incr = 1;
            if (key == '-' && options.Mesher.meshSize_absolute < incr*1.001) return false; // dissalow decrement to meshsize=0
            if (key == '-') incr = -incr;
            options.Mesher.meshSize_absolute += incr;
            window.nanogui->refresh();
            if (On_OptionChanged_Mesher) On_OptionChanged_Mesher(false);
            return true;
        }
    }

    // change DividerLogicConnector.debug_connectionIndex incrementaly
    switch (key)
    {
        case '[': //  pressed '['
        case ']': //  pressed ']'
        {
            if (key == '[' && options.DividerLogicConnector.debug_connectionIndex < -0.001) return false; // dissalow decrement to meshsize=0
            int incr = (key == '[') ? -1 : 1;
            options.DividerLogicConnector.debug_connectionIndex += incr;
            window.nanogui->refresh();
            if (On_OptionChanged_DividerLogicConnector) On_OptionChanged_DividerLogicConnector();
            return true;
        }
    }

    //window.screen->performLayout();
    return false;
}

bool Options::callback_key_pressed(OpenGL_Window& window, unsigned int unicode_key, int modifiers)
{
    //switch (unicode_key)
    //{
    //case 121:
    //{
    //    return true;
    //}
    //}
    return false;
}