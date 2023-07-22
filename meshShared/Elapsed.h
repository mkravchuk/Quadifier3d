#pragma once
#include "Timer.h"
#include <vector>

extern std::vector<TimeElapsed*> allRegisteredTimeElapsed;

class TimeElapsed
{
private:
public:
    enum Type
    {
        Default, Title, Total
    };
    string caption;
    Type type;
    atomic_llong elapsed_ms;
    int goupId;
    int addedTimersCount; // how many times 'elapsed_ms' was updated. used for test if 'elapsed_ms' was initialzed before use
    TimeElapsed(int goupId, const string& caption, Type type = Type::Default);
    TimeElapsed(const string& caption, Type type = Type::Default);
    string ElapsedSecondsStr() const;
    string toString() const;
    friend ostream& operator<<(ostream& os, const TimeElapsed& t);
};

class AllTimeElapsed
{
public:
    static void coutAllTimers();
    static void coutAllTimersInGroup(int groupid);
    static void resetAllTimers();
    static void resetAllTimersInGroup(int groupid);
    static void Register(TimeElapsed* timeElapsed);
};

struct Elapsed_App
{
    int id = 1;
    TimeElapsed cout;
    TimeElapsed setCursor;
    TimeElapsed AllocateConsoleWindow;
    TimeElapsed ReadCommandLineArguments;
    TimeElapsed CreateNanoguiOptions;
    TimeElapsed InitPlugins;
    TimeElapsed SetViewerOptions;
    TimeElapsed CreateOpenGLWindow;
    TimeElapsed UpdateSimplifiedMeshes;
    TimeElapsed AddMeshToViewport;
    TimeElapsed CopyDrawData_FromModel_to_openglDataDrawers;
    TimeElapsed ClearDrawObjects;
    TimeElapsed DrawSurfacesData;
    TimeElapsed RefreshGUI;
    TimeElapsed DrawWindowAfterLoad;
    TimeElapsed DrawSolver;
    TimeElapsed DrawStreams;
    TimeElapsed DrawMesher;
    TimeElapsed DrawTopology;
    TimeElapsed AlignCameraCenter;
    TimeElapsed LoadCameraRotation;
    TimeElapsed ReadFileFromDisk;
    TimeElapsed ShowStatistic;
    TimeElapsed Total;

    Elapsed_App();
};

struct Elapsed_Mesh
{
    int id = 2;
    TimeElapsed ReadFromFile_deserialize;
    TimeElapsed ReadFromFile_serialize;
    TimeElapsed LoadFromFile;
    TimeElapsed Load_UpdateMeshTopology;
    TimeElapsed Load_SplitNakedTriangleWith2NakedEdge;
    TimeElapsed Load_CorrectFacesEdgeIndexes;
    TimeElapsed Load_OptimizeVertexIndexesForGPU;
    TimeElapsed AddMeshToModel_ExtractMesh;
    TimeElapsed Topology;
    TimeElapsed RefreshNonCachableData_Calculating_edge_avg_size;
    TimeElapsed RefreshNonCachableData_Detecting_topological_relations_for_faces;
    TimeElapsed RefreshNonCachableData_Detecting_normals;
    TimeElapsed RefreshNonCachableData_Gathering_mesh_adjacency;
    TimeElapsed RefreshNonCachableData_computek_fast;
    TimeElapsed RefreshNonCachableData_GetLoops;
    TimeElapsed RefreshNonCachableData_GetTopology;
    TimeElapsed RefreshNonCachableData_V_Info_Update;
    TimeElapsed RefreshNonCachableData_ComputeParts;
    TimeElapsed Total;

    Elapsed_Mesh();
};

struct Elapsed_LinearEquationSolver
{
    int id = 3;
    TimeElapsed compute;
    TimeElapsed solve;
    TimeElapsed Total;

    Elapsed_LinearEquationSolver();
};

struct Elapsed_Solver
{
    int id = 4;
    TimeElapsed ConstrainsInit;
    TimeElapsed initSolver;
    TimeElapsed getGeneralCoeffConstraints;
    TimeElapsed computeFacesLaplacianCoefficient;
    TimeElapsed minQuadWithKnownMini;
    TimeElapsed minQuadWithKnownMini_compute;
    TimeElapsed minQuadWithKnownMini_solve;
    TimeElapsed setFieldFromGeneralCoefficients;
    TimeElapsed GenerateResult;
    TimeElapsed NormalizeNrosyField;
    TimeElapsed findCrossFieldSingularities;
    TimeElapsed Total;

    Elapsed_Solver();
};

struct Elapsed_SolverAngleBound
{
    int id = 5;
    TimeElapsed ConstrainsInit;
    TimeElapsed initSolver;
    TimeElapsed getGeneralCoeffConstraints;
    TimeElapsed computeFacesLaplacianCoefficient;
    TimeElapsed minQuadWithKnownMini;
    TimeElapsed minQuadWithKnownMini_compute;
    TimeElapsed minQuadWithKnownMini_solve;
    TimeElapsed setFieldFromGeneralCoefficients;
    TimeElapsed GenerateResult;
    TimeElapsed NormalizeNrosyField;
    TimeElapsed findCrossFieldSingularities;
    TimeElapsed Total;

    Elapsed_SolverAngleBound();
};

struct Elapsed_SolverUV
{
    int id = 6;
    TimeElapsed ConstrainsInit;
    TimeElapsed initSolver;
    TimeElapsed getGeneralCoeffConstraints;
    TimeElapsed computeFacesLaplacianCoefficient;
    TimeElapsed minQuadWithKnownMini;
    TimeElapsed minQuadWithKnownMini_compute;
    TimeElapsed minQuadWithKnownMini_solve;
    TimeElapsed setFieldFromGeneralCoefficients;
    TimeElapsed GenerateResult;
    TimeElapsed NormalizeNrosyField;
    TimeElapsed findCrossFieldSingularities;
    TimeElapsed Total;

    Elapsed_SolverUV();
};

struct Elapsed_Mesher_DivideComplexMeshes
{
    int id = 7;
    TimeElapsed Dividing;
    TimeElapsed JoiningStreams;
    TimeElapsed StreamTopology;
    TimeElapsed DividingIterator;
    TimeElapsed DividerOptimalConnector;
    TimeElapsed DividerLogicConnector;
    TimeElapsed DividingSolver;
    TimeElapsed CuttingMesh;
    TimeElapsed Total;

    Elapsed_Mesher_DivideComplexMeshes();
};

struct Elapsed_Mesher_GenerateMesh
{
    int id = 8;
    TimeElapsed Topology;
    TimeElapsed BuildLoops;
    TimeElapsed Solver;
    TimeElapsed DrawSolver;
    TimeElapsed Dividing;
    TimeElapsed JoiningStreams;
    TimeElapsed GeneratePolygonMeshes;
    TimeElapsed Total;

    Elapsed_Mesher_GenerateMesh();
};

struct Elapsed_Viewer
{
    int id = 9;
    TimeElapsed TransparentSort;
    TimeElapsed SetData;
    TimeElapsed OpenGLDraw;
    TimeElapsed Total;

    Elapsed_Viewer();
    void Reset();
};

struct Elapsed_GUI
{
    int id = 10;
    TimeElapsed CallbackDrawEvents;
    TimeElapsed PluginDrawEvents;
    TimeElapsed Nanogui;
    TimeElapsed glfwSwapBuffers;
    TimeElapsed Total;

    Elapsed_GUI();
    void Reset();
};

struct Elapsed
{
    Elapsed_App App;
    Elapsed_Mesh Mesh;
    Elapsed_LinearEquationSolver LinearEquationSolver;
    Elapsed_Solver Solver;
    Elapsed_SolverAngleBound SolverAngleBound;
    Elapsed_SolverUV SolverUV;
    Elapsed_Mesher_DivideComplexMeshes Mesher_DivideComplexMeshes;
    Elapsed_Mesher_GenerateMesh Mesher_GenerateMesh;
    Elapsed_Viewer Viewer;
    Elapsed_GUI GUI;

    Elapsed();
};

extern Elapsed elapsedTimers;