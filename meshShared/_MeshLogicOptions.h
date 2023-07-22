#pragma once

// DISABLED - since this lib increase size of exe by 90Mb and anyway is slower from our fast solver
// enables Intel MKL support for solvers
// also reqiures include 3 libs
// K : \libs\mkl\compilers_and_libraries\windows\mkl\lib\intel64_win\mkl_intel_lp64.lib
// K : \libs\mkl\compilers_and_libraries\windows\mkl\lib\intel64_win\mkl_sequential.lib
// K : \libs\mkl\compilers_and_libraries\windows\mkl\lib\intel64_win\mkl_core.lib
//#define Intel_MKL_SUPPORTED 0

// DISABLED - since this lib reuqired libopenblas.dll, and also is lower qualite and speed from our fast solver
// also reqiures include 1 'libopenblas.dll.a.lib' and link 2 projects 'iglcomiso', 'CoMISo'
// .\_libs\libgl\external\CoMISo\ext\OpenBLAS - v0.2.14 - Win64 - int64\lib\libopenblas.dll.a.lib
// also copy 4 dlls from '.\_libs\CoMISo\_libopnblas_dll\'  into exe folder 
//#define Comiso_SUPPORTED 1

// DISABLED - since switching on this options will result in many errors that have to be fixed! (we use unique optimized solver that was manually tuned for performance)
// enables different linear equation solvers: SimplicialLLT, SimplicialLDLT, SparseLU, BiCGSTAB, ConjugateGradient, IncompleteLUT
//#define alternative_solvers_SUPPORTED 1

// enables NPolyVectorFieldSolver and NPolyVectorFieldSolverGeneral solve approach to get nrosy field (out fast solver is based on this method, and is 3-5x times faster due to many optimizations)
// required to inlcude NPolyVectorFieldSolverGeneral.h into project 'meshSolver'
#define NPolyVectorFieldSolverGeneral_SUPPORTED 1

// DISABLED - since this method doenst work properly and need to be additionaly researched
#define UVmiqSolver_SUPPORTED 0
// also reqiures include 1 'libopenblas.dll.a.lib' and link 2 projects 'iglcomiso', 'CoMISo'


// DISABLED - since it is not fully implemented, and therefore doenst work
// enables experemental 'MeshLinearSolver' - works only for very small systems
// required to inlcude MeshLinearSolver.h into project 'meshSolver'
//#define MeshLinearSolver_SUPPORTED 0



struct MeshLogicOptions_OpenglShader
{

};


struct MeshLogicOptions_Draw_Mesh
{
    bool Id_Vertex;
    bool Id_Edge;
    bool Id_Face;
    bool Id_Mesh;
    bool Id_MeshLoop;
    bool Obj_id;
    bool Obj_name;

    bool Borders;
    bool BorderIndexes;
    bool BorderLabels;
    bool BorderSharpnessDots;
    bool BorderSharpnessDotsAngle;
    bool BorderAngles;

    bool Normals_Faces;
    bool Normals_Vertixes;
    bool Normals_Faces_Calculated;

    Color4d Id_Vertex_Color;
    Color4d Id_Edge_Color;
    Color4d Id_Face_Color;
    Color4d Id_Mesh_Color;

    Color4d Obj_Id_Color;
    Color4d Obj_Name_Color;

    Color3d DefaultColor; // default color for faces
    Color3d TopologyConnection_Color_Outher;
    Color3d TopologyConnection_Color_Inner;
    Color3d TopologyConnection_Color_Connected;
    Color3d Points_Color;


    struct HighligtWhat
    {
        enum class WhatType
        {
            Vertex, Edge, Face, Mesh, ObjName, StreamIndex, StreamId, StreamIterativeIndex, StreamIterativeId, TopologyConnection, TopologySegment, TopologyPoint
        };
        vector<string> WhatTypeStr = { "Vertex", "Edge", "Face", "Mesh", "Object", "Stream (index)", "Stream (id)", "Stream (iterative index)", "Stream (iterative id)", "Topology Connection", "Topology Segment", "Topology Point" };
        bool Enabled = false;
        WhatType What = HighligtWhat::WhatType::Mesh;
        string id = "";
    };
    HighligtWhat Highlight;
};

struct MeshLogicOptions_Draw_Constrains
{
    bool ShowBorderConstrains;
    bool HighlightConstrainedFaces;
    bool ShowEdgesSharpness;
};

struct MeshLogicOptions_Draw_Solver
{
    enum class DirectionsStyle
    {
        Default, FirstVectors, SecondVectors, TwoColoredVectors, TwoColoredVectorsBordersOnly, TwoColoredWithBisectors, TwoColoredWithXY, XY, TwoBisectors, Weights, SolverQuuQuk
    };
    vector<string> DirectionsStyleStr = { "Default", "first vector", "second vector","2 colored vectors", "2 colored vectors (borders only)", "2 colored vectors + bisectors", "2 colored vectors + XY", "XY", "2 bisectors", "weights", "SolverQuuQuk" };
    bool HighlightFixedP;
    bool ShowDirections;
    DirectionsStyle ShowDirectionsStyle;
    bool ShowSingularities;
    bool ShowDirectionsWeights;
    bool ShowBorderConstrainesIsoLines;
    int ShowBorderConstrainesIsoLinesCount;
    int ShowBorderConstrainesIsoLinesCount_intensivity;
};


struct MeshLogicOptions_Draw
{
    MeshLogicOptions_Draw_Mesh Mesh;
    MeshLogicOptions_Draw_Constrains Cons;
    MeshLogicOptions_Draw_Solver Solver;
};

struct MeshLogicOptions_Topology_Draw
{
    bool Enabled;
    bool drawSingleCurves;
    bool drawConnectedCurves;
    bool drawConnectionsIds;
    bool drawSegmenstIds;
    bool drawCurvesIds;
    bool drawSegmentsPointsCount;
    bool drawDots;
    bool drawDotsIds;
    bool drawDotsSegmentsIds;
    bool drawDotsCurvesIds;
};

struct MeshLogicOptions_Mesh
{
    enum class computeKType
    {
        Default, Fast, Direct, CosSin
    };
    vector<string> computeKTypeStr = { "Default", "Fast", "Direct" , "CosSin" };
    computeKType computeK;
    bool SSEEnabled;
    bool sse__triangle_triangle_adjacency_preprocess;
    bool SplitNakedTriangleWith2NakedEdge;
    bool OptimizeVertexIndexesForGPU;
    bool Heal;
};

struct MeshLogicOptions_MeshHeal
{
    bool PolygonMesh_DeleteZeroTriangles;
    bool PolygonMesh_DeleteUnattachedVertexes;

    bool MergeVertexes;
    D MergeVertexes_tol_edge_percent;
    bool FlipNormals;
    bool NormalizeNormals;
    bool FillHoles;
    D FillHoles_maxEdgesCount;
    bool DeleteUnattachedTriangles;
    int DeleteUnattachedTriangles_maxConectedCount;
};

struct MeshLogicOptions_MeshesTopology
{
    MeshLogicOptions_Topology_Draw draw;
    bool Enabled;
    bool check_PerfectConnections;
    bool check_PerfectConnections_fast;
    D tol_PerfectConnections;
    bool check_PartialConnections;
    bool check_PartialConnections_fast;
    D tol_PartialConnections;
    bool check_InsideConnections;
    D tol_InsideConnections;
    D tol_InsideConnectionsDist;
    D max_angle_between_edges_InsideConnections;
    bool DebugEnabled;
    bool debug_tracePairsInConsole;
    int debug_highlighPoint_id;
    int debug_highlighConnection_id;
    int debug_highlighCurve_id;
    bool debug_TestPerformanceCall;
    int debug_TestPerformanceCall_repeatTimes;
};

struct MeshLogicOptions_MeshStream
{
    bool show_dividing_streams;
    bool show_dividing_streams__show_vid_eid_fid;
    bool show_streamIndexes_atStart;
    bool show_streamIndexes_InAllPoints;
    bool show_streamIndexes_Every10Points;
    bool show_stream_iterations;
    bool show_ExtendStream_info_in_console;
    D snap_to_vertex_tol;
    bool use_sse41; 
};

struct MeshLogicOptions_MeshStreamsTopology
{
    bool Enabled;
    bool cut_streams_at_cross;
    bool cut_streams_at_cross__show;
    bool debug_show_streams_intersections;
    bool debug_show_streams_segments;
    bool debug_show_quads;
    bool debug_show_loops;
    bool debug_show_connections;
    bool debug_trace_iterations_in_console;
    bool debug_TestPerformanceCall;
    int debug_TestPerformanceCall_repeatTimes;
};

struct MeshLogicOptions_MeshSimplification
{
    enum class AlgorithmType
    {
        FastQuadric, MyFastQuadric//, Boundingmesh
    };
    vector<string> AlgorithmTypeStr = { "Quadric        ", "My Quadric" };
    bool Enabled;
    D Level; // threshold if < 0, percent if between 0 and 1, count if > 1
    AlgorithmType Algorithm;
    bool RecomputeTriangleNormalsAfterEachIteration;
    D MinAngleBetweenEdges_ForNewTriangles;
    D MaxAngleChangeInNormal_ForNewTriangles;

    bool RemoveSharpTriangles;
    int RemoveSharpTriangles_Iterations;
    D RemoveSharpTriangles_MinSharpness; // percent [0..1] - lesser value - smaller triangle
    D RemoveSharpTriangles_MaxFriendSharpness; // percent [0..1] - skip if friend is also sharp 

    bool DebugEnabled;
    int Debug_MaxNumOfIterations;  // 0 all,  otherwise only iterations less from this num
    int Debug_HighlightSimplificationNum; // -1 show all, 0 none,  otherwise only one simplification defined by this num
    bool Debug_HighlightSharpTriangles;
    bool Debug_HighlightFlippedTraingles;
    bool Debug_HighlightTryFailEdges;
    bool Debug_ShowTrianglessSharpness;
    bool Debug_ShowEdgeErrors;
    bool Debug_ShowTriangleIndex;
    bool Debug_ShowTriangleId;

    inline bool operator == (const MeshLogicOptions_MeshSimplification& a)
    {
        return //Enabled == a.Enabled  -skip comparing Enabled - let it do manully
            abs(Level - a.Level) < 0.000000000001
            && Algorithm == a.Algorithm
            && RecomputeTriangleNormalsAfterEachIteration == a.RecomputeTriangleNormalsAfterEachIteration
            && MinAngleBetweenEdges_ForNewTriangles == a.MinAngleBetweenEdges_ForNewTriangles
            && MaxAngleChangeInNormal_ForNewTriangles == a.MaxAngleChangeInNormal_ForNewTriangles


            && RemoveSharpTriangles == a.RemoveSharpTriangles
            && RemoveSharpTriangles_Iterations == a.RemoveSharpTriangles_Iterations
            && RemoveSharpTriangles_MinSharpness == a.RemoveSharpTriangles_MinSharpness
            && RemoveSharpTriangles_MaxFriendSharpness == a.RemoveSharpTriangles_MaxFriendSharpness

            && DebugEnabled == a.DebugEnabled
            && Debug_MaxNumOfIterations == a.Debug_MaxNumOfIterations
            && Debug_HighlightSimplificationNum == a.Debug_HighlightSimplificationNum
            && Debug_HighlightSharpTriangles == a.Debug_HighlightSharpTriangles
            && Debug_HighlightFlippedTraingles == a.Debug_HighlightFlippedTraingles
            && Debug_HighlightTryFailEdges == a.Debug_HighlightTryFailEdges
            && Debug_ShowTrianglessSharpness == a.Debug_ShowTrianglessSharpness
            && Debug_ShowEdgeErrors == a.Debug_ShowEdgeErrors
            && Debug_ShowTriangleIndex == a.Debug_ShowTriangleIndex
            && Debug_ShowTriangleId == a.Debug_ShowTriangleId

            ;
    }
};








struct MeshLogicOptions_Constrains
{
    enum class MeshConstrainsType
    {
        AllEdgesWithCorrectionInCorners, QuadOnly
    };
    vector<string> MeshConstrainsTypeStr = { "Simple", "Quad" };

    MeshConstrainsType Type;
    bool WeightIsRelativeToEdgeLength;
    bool CorrectInAngles;
    D CorrectInAngles_Percent;
    bool CorrectInAngles_Percent_IsDynamic;
    bool CorrectInAngles_Percent_isProgressive;
    bool CorrectInAngles_straightTo90_angleX2;
    bool CorrectInAngles_force_4side_to_straightTo90;
    int CorrectInAngles_skipp_last_edges_count;
    bool CorrectInAngles_ForceTo0_90to0;
    bool CorrectInAngles_Debug;
    bool AddContrainsInSharpEdges;
    D AddContrainsInSharpEdges_sharpAngle;
    bool DebugEnabled;
};

struct MeshLogicOptions_ConstrainsUV
{
    bool DebugEnabled;
    bool debug_showConstrains;
    bool debug_HighlightConstrainedFaces;
    bool debug_showSharpness;
};

struct MeshLogicOptions_MeshLinearSolver
{

    bool DebugEnabled;
    int debug_debugStepNum;
};


struct MeshLogicOptions_LDLTFloatSolver
{
    bool fast_factorize_preordered;
    bool fast_factorize_preordered__multithreading;
    bool fast_factorize_preordered_use_AVX;
    bool fast_permutation;
    bool DebugEnabled;
    bool debug_trace_factorize_preordered;
    bool debug_skip_permuting;
    bool debug_trace_iterationsCount;
    int debug_trace_compute_iterationsCount; 
};

struct MeshLogicOptions_LinearEquationSolver
{
    enum  class LinearEquationSolverType
    {
        Default, LDLTFloat
        #if alternative_solvers_SUPPORTED
        , SimplicialLLT, SimplicialLDLT, SparseLU, BiCGSTAB, ConjugateGradient, IncompleteLUT
        #endif
        #if Intel_MKL_SUPPORTED
        , PardisoLDLT, PardisoLU
        #endif
    };
    vector<string> LinearEquationSolverTypeStr = { "Default", "LDLTFloat"
        #if alternative_solvers_SUPPORTED
        , "SimplicialLLT", "SimplicialLDLT", "SparseLU", "BiCGSTAB", "ConjugateGradient", "IncompleteLUT"
        #endif
        #if Intel_MKL_SUPPORTED
        ,  "PardisoLDLT", "PardisoLU"
        #endif
    };

    LinearEquationSolverType Solver;
};

struct MeshLogicOptions_SolverNrosy
{
    enum class NRosyNum
    {
        N1, N2, N3, N4, N5, N6, N7, N8, N9, N10, N11, N12, N13, N14, N15, N16
    };
    vector<string> NRosyNumStr = { "1","2","Triple","Quad", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16" };
    // must be same as 'NRosySolver' enum class from 'nrosy.h' (since we convert from this enum type in MeshSolverNrosy to 'NRosySolver' enum class)
    enum  class MeshSolverType
    {
        NPolyVectorField_Complex2x,
        NPolyVectorField_Polar2x
        #if MeshLinearSolver_SUPPORTED
        , MeshLinearSolver
        #endif
        #if NPolyVectorFieldSolverGeneral_SUPPORTED
        , NPolyVectorField
        , NPolyVectorFieldGeneral
        #endif
        #if Comiso_SUPPORTED
        , FrameField,
        SimplicialLDLT,
        gurobi_rounding,  //solve_gurobi(_A, _x, _rhs, _to_round);
        cplex_rounding,    //solve_cplex(_A, _x, _rhs, _to_round);
        no_rounding,       //solve_no_rounding(_A, _x, _rhs);
        direct_rounding,    //solve_direct_rounding( _A, _x, _rhs, _to_round);
        multiple_rounding, //solve_multiple_rounding(_A, _x, _rhs, _to_round);
        iterative,                 //solve_iterative(_A, _x, _rhs, _to_round, _fixed_order);
        #endif
    };
    vector<string> MeshSolverTypeStr = { "Fast 2x rosy field", "Polar 2x"
        #if MeshLinearSolver_SUPPORTED
        , "MeshLinearSolver"
        #endif
        #if NPolyVectorFieldSolverGeneral_SUPPORTED
        , "NPolyVectorField", "NPolyVectorFieldGeneral"
        #endif
        #if Comiso_SUPPORTED
        , "Frame Field" ,"SimplicialLDLT", "gurobi", "cplex", "no rounding", "direct rounding", "multiple rounding", "iterative"
        #endif
    };
    enum  class SolveForVectorsXYEnum
    {
        XY, xx, yy
    };
    vector<string> SolveForVectorsXYEnumStr = { "XY", "xx", "yy" };

    bool Enabled;
    NRosyNum N;
    MeshSolverType Solver;
    #if alternative_solvers_SUPPORTED
    D SoftPercent;
    D Tolerance;
    SolveForVectorsXYEnum SolveForVectorsXY;
    #endif
    bool fast_Generate_Result_F;
    bool fast_sparce_matrix_creation;
    bool use_sse_cin_cos;
    bool adjustK;
    D adjustK_Percent;
    bool adjustK_useCos;
    bool adjustK_quadric;
    #if NPolyVectorFieldSolverGeneral_SUPPORTED
    bool compute_only_once;
    bool ignore_x;
    D ignore_x__max_correction_angle;
    bool ignore_x_always;
    #endif
    bool sort_complex_vectors;
    bool NormalizeNrosyField_sort_field_directions;
    bool check_angle_correction_validity;

    bool DebugEnabled;
    bool debug_trace_solver_performance_per_mesh;
    bool TestPerformanceCall;
    int TestPerformanceCall_repeatTimes;
};


struct MeshLogicOptions_SolverAngleBound
{
    enum  class AlgorithmType
    {
        eigen, v3, polar
    };
    vector<string> AlgorithmTypeStr = { "eigen", "v3", "polar" };

    bool Enabled;
    AlgorithmType Algorithm;
    bool reuse_ordering;
    int iterations_count;
    int thetatMin;
    D lambdaInit;
    D lambdaMultFactor;
    bool doHardConstraints;
    bool leave_x_original;

    bool DebugEnabled;
    bool debug_logMessages;
    bool TestPerformanceCall;
    int TestPerformanceCall_repeatTimes;
};


struct MeshLogicOptions_SolverUV
{
    bool Enabled;
    enum class AlgorithmType
    {
        lscm, miq
    };
    vector<string> AlgorithmTypeStr = { "LSCM", "MIQ" };
    AlgorithmType Algorithm;
    bool DebugEnabled;
    bool debug_show_uv;
    bool debug_show_uv__on_borders;
    bool debug_showBorderConstrainesIsoLines;
    int debug_showBorderConstrainesIsoLines__count;
    int debug_showBorderConstrainesIsoLines__intensivity;
    bool debug_trace_solver_performance_per_mesh;
    bool TestPerformanceCall;
    int TestPerformanceCall_repeatTimes;
};


struct MeshLogicOptions_Divider
{
    enum class AlgorithmType
    {
        AvgDir, BestDir, BestDirFast, AvgStream, StreamAngles
    };
    vector<string> AlgorithmTypeStr = { "AvgDir", "BestDir", "BestDirFast", "AvgStream" , "StreamAngles" };
    bool Enabled;
    AlgorithmType Algorithm;
    int SingularityExtendCount;
    bool fieldVectorsWeights_isrelative_to_fielddirection_norm;
    bool fieldVectorsWeights_is_fielddirection_norm;
    int isoLines_ExtensionsCount;
    bool use_always_first_vector_as_prev;
    bool improve_stream_angles;
    bool DebugEnabled;
    bool Debug_isoLines_show;
    int Debug_point_index;
    int Debug_ri_index;
};

struct MeshLogicOptions_DividerIterator
{
    bool Enabled;
    int max_iterations_count;
    bool joinStreamsAfterEachIteration;
    bool DebugEnabled;
    int debug_iteration_index;
    bool debug_show_iteration_index_on_streams;
    bool debug_trace_iterations_in_console;
};

struct MeshLogicOptions_DividerOptimalConnector
{
    bool Enabled;

    enum class LoopDividingType
    {
        Min, Avarage, Max, _1to1, Independed
    };
    vector<string> LoopDividingTypeStr = { "Min", "Avarage", "Max", "1x1", "Independed" };
    LoopDividingType DividingType;
    int max_conflicts;

    bool JoinConflictedLoops;
    bool JoinConflictedLoops_ReserveSpaceForSmallInBig;
    D JoinConflictedLoops_ReserveSpaceForSmallInBig_acceptRation;
    bool JoinConflictedLoops_BalanceSizes;
    bool JoinConflictedLoops_JoinBordersWithStreams;

    bool JoinStreamsWithDividingPoints;

    bool DebugEnabled;
    bool debug_show_conflictedLoops;
    int debug_debug_conflictedLoop_num;
    int debug_ReserveSpaceForSmallInBig_max_iterations_count;
    bool debug_show_unresolved_conflicts;
    bool debug_show_dividingPointsForJoins;
    bool debug_trace_iterations_in_console;
    int debug_debug_iteration_num;
};

struct MeshLogicOptions_DividerLogicConnector
{
    bool Enabled;
    D connectionAngle;
    bool DebugEnabled;
    int debug_debug_iteration_num;
    bool debug_show_streams__intersection_points;
    bool debug_show_streams__accepted_connections;
    int debug_streamGlobalIndex;
    int debug_connectionIndex;
};

struct MeshLogicOptions_MeshStreamsAdjuster
{
    bool iterate_adjustments;
    int iterate_adjustments__iterations_count;
    bool iterate_adjustments__trace_progress_in_console;

    enum class AlgorithmType
    {
        StartAngle, Ditsts
    };
    vector<string> AlgorithmTypeStr = { "StartAngle", "Ditsts" };
    AlgorithmType Algorithm;
    enum class ShiftSmoothingType
    {
        Linear, Smooth, Triple, Quad
    };
    vector<string> ShiftSmoothingTypeStr = { "Linear", "Smooth", "Triple", "Quad" };
    ShiftSmoothingType ShiftSmoothing;
    enum class TestDistMethod
    {
        NewStream, OriginStream, NewAndOrigin
    };
    vector<string> TestDistMethodStr = { "NewStream", "OriginStream", "NewAndOrigin" };
    TestDistMethod testDistMethod;
    int skip_first_points; // how many points skip from adjusting from start
    D tollerance;  // what distance is enought to join streams - percent of avarage edge
    D max_stream_angle_diff; // max angle diff for one line compare to stream, this angle should be lesser from 'max_change_angle' and not to high, since with higher angle there is a canche to go wrong direction

    bool run_direct_calculations;
    enum class DirectAlgorithmType
    {
        H, // have weakness: ortogonal edges to stream
        L,  // works ideal if faces are in same space, otherwise have some small error. have weakness: designed to work in 2D, in curved surfaces will have error
        X,  // have weakness: big tollerance
        V,  // works with vectors in 3D without errors, but a bit slower from 'L'
    };
    vector<string> DirectAlgorithmTypeStr = { "H", "L", "X","V" };
    DirectAlgorithmType DirectAlgorithm;
    int direct_calculations__test_streams_count;
    D direct_calculations__replace__new_lengthTo_pointOnPoints;
    bool direct_calculations__precise_direction_translation;

    bool run_iterative_calculations;
    int iterative_calculations__max_iterations_count;// max steps to join streams
    D iterative_calculations__max_change_angle;  // max angle change for one point, should be high to make possible exit wrongly taken directions
    int iterative_calculations__max_times_failed_in_row;

    bool run_hard_calculations;
    int hard_calculations__iterations_count_on_edge;


    bool DebugEnabled;
    bool debug_show_stream_to_connection_point;
    int debug_singularityExtendNum;
    int debug_debugStreamIndex;
    int debug_debugPointNum;
    int debug_debugIterationNum;
    int debug_max_times_failed_in_row;
    bool debug_trace_iterations;
    bool debug_trace_iterations__tests;
    bool debug_draw_accepted_points;
    bool debug_TestPerformanceCall1000Times;
};

struct MeshLogicOptions_MeshStreamsJoiner
{
    bool Enabled;
    D maxDist_meshSize_multipliyer;
    bool maxDist_is_minimum_of_mesh_avg_size;
    D max_connectionAngle_Directions;
    D max_connectionAngle_Normals;
    D max_lengthsDiff_percent;
    D max_connectionArea_percent;
    D min_lengthToDist_percent;
    bool sort_by_length_for_very_small_distances;
    bool optimizeConnections;

    bool DebugEnabled;   
    D debug_meshSize_multipliyer;
    int debug_debugStreamIndex;
    bool debug_connection;
    int debug_connection_streamIndex1;
    int debug_connection_streamIndex2;
    bool debug_everyPointOfStream;
    bool debug_TestPerformanceCall1000Times;
    int debug_alternativePath_debugStreamIndex;
    bool debug_show_connection_points;
    bool debug_show_connection_anchor_ids;
    bool debug_show_connection_indexes;
};


struct MeshLogicOptions_MeshCutter
{
    bool Enabled;
    bool ImproveStreamLines;
    D ImproveStreamLines__edge_percent_tolerance;
    bool MergeClosePointsOnEdges;
    D MergeClosePointOnEdges__edge_percent_tolerance;
    D MergeClosePointOnEdges__connection_angle;
    D MergeClosePointOnEdges__max_angle_change_belowConnectionAngle;
    D MergeClosePointOnEdges__max_angle_change_aboveConnectionAngle;
    bool ImproveMeshToConnectStreamLines;
    D ImproveMeshToConnectStreamLines__edge_percent_tolerance;
    bool ImproveMeshToConnectStreamLines__allow_improvements_on_borders;
    bool ImproveMeshToConnectStreamLines__dissalow_move_other_streams;
    bool FindStreamIntersections;
    bool CutMesh;
    bool CutMesh_useFastMethod;
    bool DebugEnabled;
    bool Debug_trace_removed;
    bool Debug_show_mergedclose_points;
    bool Debug_show_steps;
    int Debug_show_steps__stop_at_step;
    bool Debug_show_intersections;
    bool Debug_cutmesh__show_intersecting_faces;
    int Debug_cutmesh__show_intersecting_faces__show_only_for_face;
    bool Debug_cutmesh__show_edges_that_are_recognized_as_stream_holders;
    bool Debug_ShowModifiedMesh;
    bool Debug_ShowModifiedMesh_onlyborders;
};





struct MeshLogicOptions_Mesher
{
    bool Enabled;
    enum class AlgorithmType
    {
        StreamsOnCuttedMeshes, UVOnCuttedMeshes, LaplacianOnLoopStreams
    };
    vector<string> AlgorithmTypeStr = { "Streams", "UV", "Laplacian" };
    AlgorithmType Algorithm;
    D meshSize_absolute;
    D meshSize_RelativeToTrianglesSize;
    bool is_meshSize_RelativeToTrianglesSize;
    D meshSize_min;
    bool DebugEnabled;
    bool debug_show_meshsize_on_borders;
    bool debug_draw_cuttedMesh; //  suppresses drawing of original mesh in favor of cutted mesh - for easeir debuging
    int debug_debug_mesh_id;
    bool debug_show_meshingLoops_ids;
    int debug_debug_meshingLoop_id;
    bool debug_show_divisioncount_before_adjustment;
    bool debug_show_divisioncount_after_adjustment;
    bool debug_show_meshing_process;
    bool debug_show_meshing_process_Y;
    bool debug_show_produced_mesh;
    bool temp_temp_temp;
    bool temp_temp_temp2;
    bool temp_temp_temp3;
    bool temp_temp_temp4;
}; 

 
   
struct MeshLogicOptions
{ 
    bool showTimeElapsed;
    MeshLogicOptions_OpenglShader OpenglShader;
    MeshLogicOptions_Draw Draw;
    MeshLogicOptions_Mesh Mesh;
    MeshLogicOptions_MeshHeal MeshHeal;
    MeshLogicOptions_MeshesTopology MeshesTopology;
    MeshLogicOptions_MeshStream MeshStream;
    MeshLogicOptions_MeshStreamsTopology MeshStreamsTopology;    
    MeshLogicOptions_MeshSimplification MeshSimplification;
    MeshLogicOptions_Constrains Constrains;
    MeshLogicOptions_ConstrainsUV ConstrainsUV;
    #if MeshLinearSolver_SUPPORTED
    MeshLogicOptions_MeshLinearSolver MeshLinearSolver;
    #endif
    MeshLogicOptions_LDLTFloatSolver LDLTFloatSolver;
    MeshLogicOptions_LinearEquationSolver LinearEquationSolver;
    MeshLogicOptions_SolverNrosy Solver;
    MeshLogicOptions_SolverAngleBound SolverAngleBound;
    MeshLogicOptions_SolverUV SolverUV;
    MeshLogicOptions_Divider Divider;
    MeshLogicOptions_DividerIterator DividerIterator;
    MeshLogicOptions_DividerOptimalConnector DividerOptimalConnector;
    MeshLogicOptions_DividerLogicConnector DividerLogicConnector;
    MeshLogicOptions_MeshStreamsAdjuster StreamAdjuster;
    MeshLogicOptions_MeshStreamsJoiner Joiner;
    MeshLogicOptions_MeshCutter MeshCutter;
    MeshLogicOptions_Mesher Mesher;

    MeshLogicOptions();
}; 
 
extern MeshLogicOptions meshLogicOptions;
