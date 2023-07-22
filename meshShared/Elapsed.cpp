#include "stdafx.h"
#include "Timer.h"
#include "Elapsed.h"
#include <vector>

std::vector<TimeElapsed*> allRegisteredTimeElapsed;

TimeElapsed::TimeElapsed(const string& _caption, Type _type)
    : caption(_caption), type(_type), elapsed_ms(0), goupId(0), addedTimersCount(0)
{
    AllTimeElapsed::Register(this);
}

TimeElapsed::TimeElapsed(int _goupId, const string& _caption, Type _type)
    : caption(_caption), type(_type), elapsed_ms(0), goupId(_goupId), addedTimersCount(0)
{
    AllTimeElapsed::Register(this);
} 

string TimeElapsed::ElapsedSecondsStr() const
{
    auto res = to_string(static_cast<D>(elapsed_ms) / 1000.0);
    return res;
}


string TimeElapsed::toString() const
{
    auto timestr = [&]()
    {
        return to_string((1.0*elapsed_ms) / 1000.0) + "  " + caption;
    };

    switch (type)
    {
        case Type::Title:
            return "\n" + caption + "\n";
            break;
        case Type::Total:
            if (elapsed_ms > 10)
            {
                return "            " + timestr() + "\n\n";
            }
            break;
        default:
            if (elapsed_ms > 10)
            {
                return "time taken  " + timestr() + "\n";
            }
            break;
    }
    return "";
}

ostream& operator<<(ostream& os, const TimeElapsed& t)
{
    if (t.addedTimersCount == 0)
    {
        cout << "!  warning   Timer '" << t.caption << "' is not initialized. use 'Timer.stop()' method to initialize it" << endl;
    }
    os << t.elapsed_ms;
    return os;
}




void AllTimeElapsed::coutAllTimers()
{
    //cout << "------------------------------------------------------" << endl;
    string allTimers = "";
    for (TimeElapsed* timer : allRegisteredTimeElapsed)
    {
        allTimers += timer->toString();
    }
    cout << allTimers;

    // cout total time
    long long total = 0;
    for (TimeElapsed* timer : allRegisteredTimeElapsed)
    {
        if (timer->type == TimeElapsed::Type::Total)
        {
            //if (timer->elapsed_ms > 3) cout << "" << timer->caption<<"    " << timer->elapsed_ms << endl;
            total += timer->elapsed_ms;
        }
    }
    string timestr = to_string((1.0*total) / 1000.0);
    cout << "TIME TOTAL  " << timestr << endl << endl;
    cout << endl;
}

void AllTimeElapsed::coutAllTimersInGroup(int groupid)
{
    cout << "------------------------------------------------------" << endl;
    string allTimers = "";
    for (TimeElapsed* timer : allRegisteredTimeElapsed)
    {
        if (timer->goupId == groupid)
        {
            allTimers += timer->toString();
        }
    }
    cout << allTimers << endl;
}

void AllTimeElapsed::resetAllTimers()
{
    for (TimeElapsed* timer : allRegisteredTimeElapsed)
    {
        timer->elapsed_ms = 0;
    }
}

void AllTimeElapsed::resetAllTimersInGroup(int groupid)
{
    for (TimeElapsed* timer : allRegisteredTimeElapsed)
    {
        if (timer->goupId == groupid)
        {
            timer->elapsed_ms = 0;
        }
    }
}

void AllTimeElapsed::Register(TimeElapsed* timeElapsed)
{
    //if (allRegisteredTimeElapsed.size() == 0) cout << "!!! AllTimeElapsed::Register" << endl;
    allRegisteredTimeElapsed.push_back(timeElapsed);
}



Elapsed_App::Elapsed_App()
    : cout(id, "cout"),
    setCursor(id, "setCursor"),
    AllocateConsoleWindow(id, "AllocateConsoleWindow"),
    ReadCommandLineArguments(id, "ReadCommandLineArguments"),
    CreateNanoguiOptions(id, "CreateNanoguiOptions"),
    InitPlugins(id, "InitPlugins"),
    SetViewerOptions(id, "SetViewerOptions"),
    CreateOpenGLWindow(id, "Create OpenGL Window"),
    UpdateSimplifiedMeshes(id, "Changing simplification level"),
    AddMeshToViewport(id, "Adding mesh to viewport"),
    CopyDrawData_FromModel_to_openglDataDrawers(id, "CopyDrawData_FromModel_to_openglDataDrawers"),
    ClearDrawObjects(id, "Clear draw objects"),
    DrawSurfacesData(id, "Draw surface data"),
    RefreshGUI(id, "Refresh GUI"),
    DrawWindowAfterLoad(id, "Draw window after file load"),
    DrawSolver(id, "Draw solver"),
    DrawStreams(id, "Draw streams"),
    DrawMesher(id, "Draw mesher"),
    DrawTopology(id, "Draw topology"),
    AlignCameraCenter(id, "Changing viewport camera to center"),
    LoadCameraRotation(id, "Loading viewport rotation state"),
    ReadFileFromDisk(id, "ReadFileFromDisk"),
    ShowStatistic(id, "ShowStatistic"),
    Total(id, "TOTAL   (Application)", TimeElapsed::Type::Total)
{
}


Elapsed_Mesh::Elapsed_Mesh()
    :ReadFromFile_deserialize(id, "ReadFromFile.deserialize"),
    ReadFromFile_serialize(id, "ReadFromFile.serialize"),
    LoadFromFile(id, "LoadFromFile"),
    Load_UpdateMeshTopology(id, "UpdateMeshTopology"),
    Load_SplitNakedTriangleWith2NakedEdge(id, "SplitNakedTriangleWith2NakedEdge"),
    Load_CorrectFacesEdgeIndexes(id, "CorrectFacesEdgeIndexes"),
    Load_OptimizeVertexIndexesForGPU(id, "OptimizeVertexIndexesForGPU"),
    AddMeshToModel_ExtractMesh(id, "AddMeshToModel_ExtractMesh"),
    Topology(id, "Topology"),
    RefreshNonCachableData_Calculating_edge_avg_size(id, "Calculating edge avg size"),
    RefreshNonCachableData_Detecting_topological_relations_for_faces(id, "Detecting topological relations for faces"),
    RefreshNonCachableData_Detecting_normals(id, "Detecting normals"),
    RefreshNonCachableData_Gathering_mesh_adjacency(id, "Gathering mesh adjacency"),
    RefreshNonCachableData_computek_fast(id, "computek"),
    RefreshNonCachableData_GetLoops(id, "GetLoops"),
    RefreshNonCachableData_GetTopology(id, "GetTopology"),
    RefreshNonCachableData_V_Info_Update(id, "V_Info Update"),
    RefreshNonCachableData_ComputeParts(id, "ComputeParts"),
    Total(id, "TOTAL   (Mesh load)", TimeElapsed::Type::Total)
{
}

Elapsed_LinearEquationSolver::Elapsed_LinearEquationSolver()
    : compute(id, "compute"),
    solve(id, "solve"),
    Total(id, "TOTAL   (LinearEquationSolver)", TimeElapsed::Type::Total)
{
}

Elapsed_Solver::Elapsed_Solver()
    : ConstrainsInit(id, "ConstrainsInit"),
    initSolver(id, "initSolver"),
    getGeneralCoeffConstraints(id, "getGeneralCoeffConstraints"),
    computeFacesLaplacianCoefficient(id, "computeFacesLaplacianCoefficient"),
    minQuadWithKnownMini(id, "minQuadWithKnownMini"),
    minQuadWithKnownMini_compute(id, "solver.compute"),
    minQuadWithKnownMini_solve(id, "solver.solve"),
    setFieldFromGeneralCoefficients(id, "setFieldFromGeneralCoefficients"),
    GenerateResult(id, "GenerateResult"),
    NormalizeNrosyField(id, "NormalizeNrosyField"),
    findCrossFieldSingularities(id, "findCrossFieldSingularities"),
    Total(id, "TOTAL   (Solver)", TimeElapsed::Type::Total)
{
}

Elapsed_SolverAngleBound::Elapsed_SolverAngleBound()
    : ConstrainsInit(id, "ConstrainsInit"),
    initSolver(id, "initSolver"),
    getGeneralCoeffConstraints(id, "getGeneralCoeffConstraints"),
    computeFacesLaplacianCoefficient(id, "computeFacesLaplacianCoefficient"),
    minQuadWithKnownMini(id, "minQuadWithKnownMini"),
    minQuadWithKnownMini_compute(id, "solver.compute"),
    minQuadWithKnownMini_solve(id, "solver.solve"),
    setFieldFromGeneralCoefficients(id, "setFieldFromGeneralCoefficients"),
    GenerateResult(id, "GenerateResult"),
    NormalizeNrosyField(id, "NormalizeNrosyField"),
    findCrossFieldSingularities(id, "findCrossFieldSingularities"),
    Total(id, "TOTAL   (SolverAngleBound)", TimeElapsed::Type::Total)
{
}

Elapsed_SolverUV::Elapsed_SolverUV()
    : ConstrainsInit(id, "ConstrainsInit"),
    initSolver(id, "initSolver"),
    getGeneralCoeffConstraints(id, "getGeneralCoeffConstraints"),
    computeFacesLaplacianCoefficient(id, "computeFacesLaplacianCoefficient"),
    minQuadWithKnownMini(id, "minQuadWithKnownMini"),
    minQuadWithKnownMini_compute(id, "solver.compute"),
    minQuadWithKnownMini_solve(id, "solver.solve"),
    setFieldFromGeneralCoefficients(id, "setFieldFromGeneralCoefficients"),
    GenerateResult(id, "GenerateResult"),
    NormalizeNrosyField(id, "NormalizeNrosyField"),
    findCrossFieldSingularities(id, "findCrossFieldSingularities"),
    Total(id, "TOTAL   (SolverUV)", TimeElapsed::Type::Total)

{
}

Elapsed_Mesher_DivideComplexMeshes::Elapsed_Mesher_DivideComplexMeshes()
    : Dividing(id, "Dividing"),
    JoiningStreams(id, "JoiningStreams"),
    StreamTopology(id, "StreamTopology"),
    DividingIterator(id, "DividingIterator"),
    DividerOptimalConnector(id, "DividerOptimalConnector"),
    DividerLogicConnector(id, "DividerLogicConnector"),
    DividingSolver(id, "DividingSolver"),
    CuttingMesh(id, "CuttingMesh"),
    Total(id, "TOTAL   (DivideComplexMeshes)", TimeElapsed::Type::Total)
{
}

Elapsed_Mesher_GenerateMesh::Elapsed_Mesher_GenerateMesh()
    : Topology(id, "Topology"),
    BuildLoops(id, "BuildLoops"),
    Solver(id, "Solver"),
    DrawSolver(id, "DrawSolver"),
    Dividing(id, "Dividing"),
    JoiningStreams(id, "JoiningStreams"),
    GeneratePolygonMeshes(id, "GeneratePolygonMeshes"),
    Total(id, "TOTAL   (GenerateMesh)", TimeElapsed::Type::Total)
{
}


Elapsed_Viewer::Elapsed_Viewer()
    : TransparentSort(id, "TransparentSort"),
    SetData(id, "SetData"),
    OpenGLDraw(id, "OpenGLDraw"),
    Total(id, "TOTAL   (Viewer)", TimeElapsed::Type::Total)
{
    Reset();
}

void Elapsed_Viewer::Reset()
{
    AllTimeElapsed::resetAllTimersInGroup(id);
}


Elapsed_GUI::Elapsed_GUI()
    : CallbackDrawEvents(id, "Callback draw events"),
    PluginDrawEvents(id, "Plugin draw events"),
    Nanogui(id, "Nanogui"),
    glfwSwapBuffers(id, "glfwSwapBuffers"),
    Total(id, "TOTAL   (GUI)", TimeElapsed::Type::Total)
{
    Reset();
}

void Elapsed_GUI::Reset()
{
    AllTimeElapsed::resetAllTimersInGroup(id);
}

Elapsed::Elapsed()
{

}
Elapsed elapsedTimers;
