#include "stdafx.h"
#include <stdio.h>
#include "_MeshLogicOptions.h" //  have to come first, to avoid rebuilding project 'meshLogic'
#include "Options.h"
#include "OpenGL_SpeedImprovements.h"
#include "Model.h"
#include "ModelFile.h"
#include "ModelObject.h"
#include "Mesh.h"
#include "Utils.h"
#include "testEigen.h"
#include "_MeshLogicDraw.h"
#include "NanoguiThemes.h"
#include <shellapi.h>
#include "Divider.h" 
#include "../iglviewer/ViewportObject.h"
#include "MeshSimplifier.h"
#include "DividerIterator.h"
#include "Mesher.h"
#include "PolygonMesh.h"
#include "MeshFile.h"
#include <igl/unproject_onto_mesh.h>


string Loaded_filename = "";
MeshLogicOptions_MeshSimplification current_mesh_simplification;

OpenGL_Window window;
ViewportObjectCollection opengl_objects;
MeshSimpleCollection simpleMeshes_heap;
ViewportObject* model_opengl_object = nullptr;
ViewportObject* surfaces_opengl_object = nullptr;
ViewportObject* mesher_opengl_object = nullptr;
vector<string> commandLineArguments;
Options options;
Model model;
NanoguiTheme* theme;
chrono::high_resolution_clock::time_point timeStartApp; // used once to log Start app time
Elapsed_App& elapsed = elapsedTimers.App;
string LoadFile_LastLoadedFile = "";

/*********************************************************/
/*********************************************************/
/*********************************************************/
void setCursor(utils::cursor::CursorType cursorType, bool _on)
{
    Timer time;
    utils::cursor::SetCursor(cursorType, _on);
    time.stop(elapsed.setCursor, elapsed.Total);
}


void ClearConsole()
{
    Timer time;
    //cout << string(100, '\n');
    string emptyLines = string(60, '\n');
    printf(emptyLines.c_str());
    time.stop(elapsed.cout, elapsed.Total);
}

void RefreshGUI()
{
    Timer timer;
    window.nanogui->refresh(); // refresh gui checkbox Facebased - since changes in colors can automatically change this flag
    timer.stop(elapsed.RefreshGUI, elapsed.Total);
}


ViewportData_Mesh Obj_to_ViewportData_Mesh(ModelObject* obj)
{
    // if simplification level defined as different from 100% quality - return simplified mesh for opengl
    if (meshLogicOptions.MeshSimplification.Enabled
        && abs(current_mesh_simplification.Level - 1) > 0.0001)
    {
        if (obj->SimplifiedMesh != nullptr)
        {
            auto simpleMesh = static_cast<MeshSimple*>(obj->SimplifiedMesh);
            return ViewportData_Mesh(obj->Id,
                simpleMesh->V, simpleMesh->F, simpleMesh->F_normals, simpleMesh->V_normals,
                simpleMesh->V_Info.Vmin, simpleMesh->V_Info.Vmax, simpleMesh->V_Info.Vcenter, simpleMesh->V_Info.VcenterProjected);
        }
    }

    // return full quality mesh for opengl
    const Mesh& mesh = obj->srf.mesh;
    return ViewportData_Mesh(obj->Id,
        mesh.V, mesh.F, mesh.F_normals, mesh.V_normals,
        obj->srf.NormalCorrections.V, obj->srf.NormalCorrections.V_normals, obj->srf.NormalCorrections.V_correcteIndexes, obj->srf.NormalCorrections.Fcorrections,
        mesh.V_Info.Vmin, mesh.V_Info.Vmax, mesh.V_Info.Vcenter, mesh.V_Info.VcenterProjected);
}

MeshSimple* CreateSimplifiedMesh(ViewerDrawObjects& draw, const Mesh& mesh, D mesh_simplification_level)
{
    MeshSimple* res = new MeshSimple();

    // Simplify mesh
    if (!SimplifyMesh(draw, Loaded_filename, mesh, res->V, res->F, mesh_simplification_level, meshLogicOptions.MeshSimplification.Algorithm, meshLogicOptions.MeshSimplification.DebugEnabled))
    {
        delete res;
        return nullptr;
    }

    res->InitNormalsAndV();
    return res;
}

void OnOptionChanged_MeshSimplification();
void UpdateSimplifiedMeshes()
{
    auto new_mesh_simplification = meshLogicOptions.MeshSimplification;
    bool hasOptionsChanged = !(current_mesh_simplification == new_mesh_simplification) //if mesh simplification options has changed
        || new_mesh_simplification.DebugEnabled; //or is debug mode
    bool hasEnabledChanged = new_mesh_simplification.Enabled != current_mesh_simplification.Enabled;
    current_mesh_simplification = new_mesh_simplification;

    Timer timer;

    //
    // clear all previous versions: if mesh simplification level or algorithm have changed
    //
    if (hasOptionsChanged || hasEnabledChanged)
    {
        //
        // Clear old simplified meshes
        //
        if (hasOptionsChanged)
        {
            simpleMeshes_heap.Clear(); // deallocate all simplified meshes
            for (auto obj : model.AllObjects)
            {
                obj->SimplifiedMesh = nullptr; // clear pointer to dellocated object
            }
        }

        //
        // reset all opengl, so in 'UpdateViewerMeshFromModel' they will be recreated with newer simplification level
        //
        opengl_objects.Clear(); // clear all opengl objects
        model_opengl_object = nullptr;// null global opengl
        surfaces_opengl_object = nullptr;//null shared opengl for surfaces
        mesher_opengl_object = nullptr;// null mesh opengl
    }

    //
    // Create new simplified meshes for objects that not have yet
    //
    atomic_int needAddSimplifiedMeshToHeap_addedCount = 0; //  how many new simpl meshes was created
    vector<bool> needAddSimplifiedMeshToHeap(model.ActiveObjects.size(), false); //srore flag of newly created simplified mesh - later we will use it to add simplified mesh to heap
    if (current_mesh_simplification.Enabled && abs(current_mesh_simplification.Level - 1) > 0.0001) // if simplification level defined as different from 100% quality
    {
        cout << "Changing simplification level to " << new_mesh_simplification.Level << " ...";
        extern bool IsOmpEnabled;
        #pragma omp parallel for  if(IsOmpEnabled)  //must be non static since only few objects will need simplifications
        for (int i = 0; i < model.ActiveObjects.size(); i++)
        {
            ModelObject* obj = model.ActiveObjects[i];
            if (obj->SimplifiedMesh == nullptr) // if object dont have yet simplified version - create it
            {
                MeshSimple* sm = CreateSimplifiedMesh(obj->srf.draw, obj->srf.mesh, current_mesh_simplification.Level);
                if (sm != nullptr) // if simplified version is successfully created
                {
                    obj->SimplifiedMesh = sm;
                    needAddSimplifiedMeshToHeap[i] = true;
                    ++needAddSimplifiedMeshToHeap_addedCount;
                    // just setting value - not adding to heap since we work here in multithreading
                }
            }
        }
    }

    //
    // Add to heap all allocated simplified meshes that was created in multithreading above
    //
    int meshTrianglesInput = 0;
    int meshTrianglesOutput = 0;
    if (needAddSimplifiedMeshToHeap_addedCount > 0)
    {
        for (int i = 0; i < model.ActiveObjects.size(); i++)
        {
            ModelObject* obj = model.ActiveObjects[i];
            meshTrianglesInput += obj->srf.mesh.F.rows();
            auto sm = static_cast<MeshSimple*>(obj->SimplifiedMesh);
            if (sm != nullptr)
            {
                if (needAddSimplifiedMeshToHeap[i]) // if simplified mesh was just created in multithreading - we need to add it to heap
                {
                    simpleMeshes_heap.Add(sm); //add to heap
                }
                meshTrianglesOutput += sm->F.rows();
            }
            else
            {
                meshTrianglesOutput += obj->srf.mesh.F.rows(); // if no simplified version provided - add original mesh size - to provide correct debug message in console
            }
        }
        int percent = static_cast<int>((meshTrianglesOutput * 100) / meshTrianglesInput);
        cout << "  input triangles " << meshTrianglesInput << ",  output triangles " << meshTrianglesOutput << " (" << percent << "%)     ";
    }

    timer.stop(elapsed.UpdateSimplifiedMeshes, elapsed.Total);
    if (meshLogicOptions.showTimeElapsed && needAddSimplifiedMeshToHeap_addedCount > 0) cout << " done in " << timer << endl; else cout << endl;

}

void ForceRecreateSurfaces()
{
    if (surfaces_opengl_object != nullptr && surfaces_opengl_object->meshes != nullptr)
    {
        surfaces_opengl_object->meshes->Ids_sorted.clear();
    }
}

void AddMeshToViewport()
{
    cout << "Adding mesh to viewport ...";
    Timer timer;

    // create opengl data drawer for model - global draw for debug purpose, that is not related to any mesh
    if (model_opengl_object == nullptr)
    {
        model_opengl_object = opengl_objects.Add();
    }

    // create opengl data drawer for mesher - global draw for all produced quad meshes
    if (mesher_opengl_object == nullptr)
    {
        mesher_opengl_object = opengl_objects.Add();
    }



    // draw surfaces
    vector<int> IdsSorted;
    if (options.Mesher__debug_draw_cuttedMesh)
    {
        for (auto f : model.Files)
        {
            if (f->IsActive)
            {
                auto cuttedMeshes = f->mesher.GetCuttedMeshes();
                for (auto& pmesh : cuttedMeshes)
                {
                    const Mesh& mesh = *pmesh;
                    IdsSorted.push_back(mesh.id);
                }
            }
        }
    }
    else
    {
        utils::stdvector::append(IdsSorted, model.ActiveObjects_Ids);
    }
    utils::stdvector::sort(IdsSorted);
    // single objects created only once, but shared created each time model changed
    bool recreate = surfaces_opengl_object == nullptr // not created yet
        || !utils::stdvector::same(IdsSorted, surfaces_opengl_object->meshes->Ids_sorted) // or collection of surfaces has changed
        || options.Mesher__debug_draw_cuttedMesh;


    if (recreate)
    {
        // free previous data from memory
        if (surfaces_opengl_object != nullptr)
        {
            opengl_objects.Remove(surfaces_opengl_object);
            surfaces_opengl_object = nullptr;
        }
        //create opengl data drawer for all surfaces - all surfaces will merge theirs data to draw at once - this will improve performance
        vector<ViewportData_Mesh> meshDatas;

        if (options.Mesher__debug_draw_cuttedMesh)
        {
            for (auto f : model.Files)
            {
                if (f->IsActive)
                {
                    auto cuttedMeshes = f->mesher.GetCuttedMeshes();
                    for (auto& pmesh : cuttedMeshes)
                    {
                        const Mesh& mesh = *pmesh;
                        meshDatas.push_back(ViewportData_Mesh(mesh.id,
                            mesh.V, mesh.F, mesh.F_normals, mesh.V_normals,
                            mesh.V_Info.Vmin, mesh.V_Info.Vmax, mesh.V_Info.Vcenter, mesh.V_Info.VcenterProjected));
                    }
                }
            }
        }
        if (!options.Mesher__debug_draw_cuttedMesh || meshDatas.size() == 0)
        {
            extern bool IsOmpEnabled;
            #pragma omp parallel for if(IsOmpEnabled)
            for (int i = 0; i < model.ActiveObjects.size(); i++)
            {
                auto obj = model.ActiveObjects[i];
                obj->srf.InitNormalCorrections(window.viewport.crease_normals_angle);
            }
            meshDatas.reserve(model.ActiveObjects.size());
            for (auto obj : model.ActiveObjects)
            {
                meshDatas.push_back(Obj_to_ViewportData_Mesh(obj));
            }
        }
        surfaces_opengl_object = opengl_objects.Add(meshDatas);
    }



    // update active flag
    if (model_opengl_object) model_opengl_object->isActive = true;
    if (surfaces_opengl_object) surfaces_opengl_object->isActive = true;
    if (mesher_opengl_object) mesher_opengl_object->isActive = true;
    timer.stop(elapsed.AddMeshToViewport, elapsed.Total);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer << endl; else cout << endl;
}

void UpdateViewerMeshFromModel()
{
    UpdateSimplifiedMeshes();
    AddMeshToViewport();
}

void CopyDrawData_FromModel_to_openglDataDrawer(const ViewerDrawObjects& draw, ViewportObject* gl, int colors_linked_to_objectId = -1)
{
    if (gl == nullptr) return;
    ViewportData& data = gl->data;
    data.draws.push_back({ colors_linked_to_objectId, reference_wrapper<const ViewerDrawObjects>(draw) });
}

void CopyDrawData_FromModel_to_openglDataDrawers(bool drawOriginalMesh)
{
    Timer timer;

    //
    // Model
    //
    CopyDrawData_FromModel_to_openglDataDrawer(model.draw, model_opengl_object);
    CopyDrawData_FromModel_to_openglDataDrawer(window.viewport.draw, model_opengl_object);
    for (auto& p : window.plugins)
    {
        CopyDrawData_FromModel_to_openglDataDrawer(p->draw, model_opengl_object);
    }

    //
    // Mesher
    //
    for (auto f : model.Files)
    {
        if (f->IsActive)
        {
            CopyDrawData_FromModel_to_openglDataDrawer(f->mesher.meshGenerator.topology.draw, surfaces_opengl_object);// must be placed in 'surfaces_opengl_object' to get proper repaint
            CopyDrawData_FromModel_to_openglDataDrawer(f->mesher.draw, surfaces_opengl_object);// must be placed in 'surfaces_opengl_object' to get proper repaint
        }
    }

    //
    // Mesh
    //
    if (drawOriginalMesh)
    {
        surfaces_opengl_object->data.draws.reserve(model.ActiveObjects.size());
        for (auto obj : model.ActiveObjects)
        {
            int colors_linked_to_objectId = obj->Id;
            CopyDrawData_FromModel_to_openglDataDrawer(obj->srf.draw, surfaces_opengl_object, colors_linked_to_objectId);
        }
        for (auto f : model.Files)
        {
            if (f->IsActive)
            {
                CopyDrawData_FromModel_to_openglDataDrawer(f->Topology.draw, surfaces_opengl_object);
            }
        }
    }

    timer.stop(elapsed.CopyDrawData_FromModel_to_openglDataDrawers, elapsed.Total);
}


// does all calculations
void SolveObjects(bool resolve, bool resolveOnlyEmpty, ModelFile* file, vector<ModelObject*> objects, bool drawOriginalMesh)
{
    if (!meshLogicOptions.Solver.Enabled)
    {
        cout << "Solver is disabled in options. " << endl;
        return;
    }

    extern bool IsOmpEnabled;
    bool logSolverMessages = !IsOmpEnabled && (objects.size() <= 10 && meshLogicOptions.Solver.DebugEnabled && meshLogicOptions.Solver.debug_trace_solver_performance_per_mesh);

    if (resolve)
    {
        cout << endl << "----- Solving -----" << endl << endl;

        Timer timer_SolveTotal;
        // for(int k = 0; k < 10; k++)
        if (meshLogicOptions.Solver.TestPerformanceCall)
        {
            cout << "Testing performance of Solver - calling solver " << meshLogicOptions.Solver.TestPerformanceCall_repeatTimes << " times..." << endl;
            for (int perfi = 0; perfi < meshLogicOptions.Solver.TestPerformanceCall_repeatTimes - 1; perfi++) // exluding 1 call to leave it to non debug call
            {
                #pragma omp parallel for  if(IsOmpEnabled)
                for (int i = 0; i < objects.size(); i++)
                {
                    auto obj = objects[i];
                    obj->srf.solver.Constrains.Clear();
                    obj->srf.solver.Solve(false);
                }
                //cout << ".";
            }
        }
        else
        {
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < objects.size(); i++)
            {
                auto obj = objects[i];
                if (resolveOnlyEmpty && !obj->srf.solver.Result.isEmpty()) continue;
                if (!logSolverMessages) cout << ".";
                obj->srf.solver.Solve(logSolverMessages);
            }
        }
        if (!logSolverMessages)cout << endl;
        timer_SolveTotal.stop(elapsedTimers.Solver.Total);
        cout << endl;
        if (meshLogicOptions.showTimeElapsed) cout << "Resolve total time:   " << timer_SolveTotal << std::endl;
    }

    //
    // draw Solver
    //
    if (drawOriginalMesh)
    {
        cout << endl << "----- Draw solver -----" << endl << endl;
        Timer timer_DrawSolver;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < objects.size(); i++)
        {
            if (!logSolverMessages) cout << ".";
            MeshSurface& srf = objects[i]->srf;
            MeshLogicDraw draw(srf);
            draw.draw_mesh_nrosy(srf.solver, logSolverMessages);
        }
        if (!logSolverMessages)cout << endl;
        timer_DrawSolver.stop(elapsed.DrawSolver, elapsed.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DrawSolver << endl; else cout << endl;
    }
}

void SolveObjectsUV(bool resolve, bool resolveOnlyEmpty, ModelFile* file, vector<ModelObject*> objects, bool drawOriginalMesh)
{
    if (!meshLogicOptions.SolverUV.Enabled)
    {
        cout << "SolverUV is disabled in options. " << endl;
        return;
    }

    extern bool IsOmpEnabled;
    bool logSolverMessages = !IsOmpEnabled && (objects.size() <= 10 && meshLogicOptions.SolverUV.DebugEnabled && meshLogicOptions.SolverUV.debug_trace_solver_performance_per_mesh);

    if (resolve)
    {
        cout << endl << "----- Solving UV -----" << endl << endl;

        Timer timer_SolveTotal;
        // for(int k = 0; k < 10; k++)
        if (meshLogicOptions.SolverUV.TestPerformanceCall)
        {
            cout << "Testing performance of SolverUV - calling solver UV " << meshLogicOptions.SolverUV.TestPerformanceCall_repeatTimes << " times..." << endl;
            for (int perfi = 0; perfi < meshLogicOptions.SolverUV.TestPerformanceCall_repeatTimes - 1; perfi++) // exluding 1 call to leave it to non debug call
            {
                #pragma omp parallel for  if(IsOmpEnabled)
                for (int i = 0; i < objects.size(); i++)
                {
                    auto obj = objects[i];
                    obj->srf.solverUV.Constrains.Clear();
                    obj->srf.solverUV.Solve(false);
                }
                //cout << ".";
            }
        }
        else
        {
            #pragma omp parallel for  if(IsOmpEnabled)
            for (int i = 0; i < objects.size(); i++)
            {
                auto obj = objects[i];
                if (resolveOnlyEmpty && !obj->srf.solverUV.Result.isEmpty()) continue;
                if (!logSolverMessages) cout << ".";
                obj->srf.solverUV.Solve(logSolverMessages);
            }
        }
        if (!logSolverMessages)cout << endl;
        timer_SolveTotal.stop(elapsedTimers.SolverUV.Total);
        cout << endl;
        if (meshLogicOptions.showTimeElapsed) cout << "Resolve UV total time:   " << timer_SolveTotal << std::endl;
    }

    //
    // draw Solver
    //
    if (drawOriginalMesh)
    {
        cout << endl << "----- Draw solver UV -----" << endl << endl;
        Timer timer_DrawSolver;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < objects.size(); i++)
        {
            if (!logSolverMessages) cout << ".";
            MeshSurface& srf = objects[i]->srf;
            MeshLogicDraw draw(srf);
            draw.draw_mesh_UV(srf.solverUV, logSolverMessages);
        }
        if (!logSolverMessages)cout << endl;
        timer_DrawSolver.stop(elapsed.DrawSolver, elapsed.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DrawSolver << endl; else cout << endl;
    }
}

void SolveAll(bool resolve, bool resolveOnlyEmpty, bool drawOriginalMesh)
{
    for (auto file : model.Files)
    {
        if (file->IsActive && file->ActiveObjects.size() > 0)
        {
            SolveObjects(resolve, resolveOnlyEmpty, file, file->ActiveObjects, drawOriginalMesh);
            SolveObjectsUV(resolve, resolveOnlyEmpty, file, file->ActiveObjects, drawOriginalMesh);
        }
    }
}



void MeshObjects(bool redivide, bool remesh, ModelFile* file, vector<ModelObject*> objects)
{
    if (!meshLogicOptions.Solver.Enabled) return;

    Mesher& mesher = file->mesher;

    //
    // Divide
    //
    if (redivide)
    {
        cout << endl << "----- DivideComplexMeshesIntoQuadMeshes -----" << endl << endl;
        Timer timer_DivideComplexMeshes;
        mesher.DivideComplexMeshesIntoQuadMeshes(objects);
        timer_DivideComplexMeshes.stop(elapsedTimers.Mesher_DivideComplexMeshes.Total);
        if (meshLogicOptions.showTimeElapsed) cout << "DivideComplexMeshesIntoQuadMeshes time:   " << timer_DivideComplexMeshes << std::endl;
    }

    //
    // DEBUG show dividing streams 
    //    
    if (meshLogicOptions.Mesher.DebugEnabled && (meshLogicOptions.MeshStream.show_dividing_streams || meshLogicOptions.MeshStream.show_streamIndexes_atStart || meshLogicOptions.MeshStream.show_streamIndexes_InAllPoints || meshLogicOptions.MeshStream.show_streamIndexes_Every10Points || meshLogicOptions.MeshStream.show_stream_iterations))
    {
        cout << "Showing dividing streams... ";
        Timer timer_ShowingDividingStreams;
        for (auto& d : mesher.dividers)
        {
            d.DebugShowStreams();
        }
        timer_ShowingDividingStreams.stop(elapsed.DrawStreams, elapsed.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_ShowingDividingStreams << endl; else cout << endl;
    }


    //
    // Meshing
    //
    if (remesh)
    {
        if (meshLogicOptions.Mesher.Enabled)
        {
            cout << endl << "----- GenerateMesh -----" << endl << endl;
            mesher.GenerateMesh();
            if (meshLogicOptions.showTimeElapsed) cout << "GenerateMesh time:   " << elapsedTimers.Mesher_GenerateMesh.Total.ElapsedSecondsStr() << std::endl;
        }
        else
        {
            cout << "Mesher is disabled in options. " << endl;
        }
    }

    //
    // Draw mesh produced by mesher (draw quad meshes)
    //
    cout << "Draw mesher... ";
    Timer timer_DrawMesher;
    mesher.DrawAll();
    timer_DrawMesher.stop(elapsed.DrawMesher, elapsed.Total);
    if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_DrawMesher << endl; else cout << endl;
}

void MeshAll(bool redivide, bool remesh)
{
    for (auto file : model.Files)
    {
        if (file->IsActive && file->ActiveObjects.size() > 0)
        {
            MeshObjects(redivide, remesh, file, file->ActiveObjects);
        }
    }
}


void Refresh(bool resolve = true, bool redivide = true, bool remesh = true)
{
    setCursor(utils::cursor::CursorType::Wait, true);


    //cout << endl << endl;
    extern bool IsOmpEnabled;
    const Viewport& v = window.viewport;

    //
    // Clear draw data
    //
    Timer timer_ClearDrawObjects;
    if (model_opengl_object) model_opengl_object->data.clearDraws();
    if (surfaces_opengl_object) surfaces_opengl_object->data.clearDraws();
    if (mesher_opengl_object) mesher_opengl_object->data.clearDraws();
    model.ClearDrawObjects();
    window.viewport.ClearDrawObjects();
    timer_ClearDrawObjects.stop(elapsed.ClearDrawObjects, elapsed.Total);
    //cout << "Reset existing draw data time:   " << utils::time::ElapsedSecondsStr(time) << std::endl;

    //
    // draw Mesh and Topology
    //
    Timer timer_DrawSurfacesData;
    bool drawOriginalMesh = true;
    if (options.Mesher__debug_draw_cuttedMesh) drawOriginalMesh = false;
    if (drawOriginalMesh)
    {
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < model.ActiveObjects.size(); i++)
        {
            ModelObject* obj = model.ActiveObjects[i];
            obj->srf.InitMesh_VcenterProjectedIndside_IfRequiredByDraw();
            MeshLogicDraw draw(obj->srf);
            draw.DrawAll();
            draw.DrawObject(obj->Id, obj->Name);
        }
        for (auto f : model.Files)
        {
            if (f->IsActive)
            {
                f->Topology.DrawAll();
            }
        }
    }
    else
    {
        cout << "supressing draw of original mesh, since we want to see cutted mesh (see option 'meshLogicOptions.Mesher.debug_draw_cuttedMesh')" << endl;
    }
    window.DrawAll();
    timer_DrawSurfacesData.stop(elapsed.DrawSurfacesData, elapsed.Total);
    //cout << "Draw surfaces time:   " << timer_Draw_SurfacesData << std::endl;



    //
    // Solve
    //
    bool resolveOnlyEmpty = false;
    if (!resolve) // force 'resolve' if there are surfaces without solver results
    {
        for (auto& o : model.ActiveObjects)
        {
            if ((meshLogicOptions.Solver.Enabled && o->srf.solver.Result.isEmpty())
                || (meshLogicOptions.SolverUV.Enabled && o->srf.solverUV.Result.isEmpty()))
            {
                resolve = true;
                resolveOnlyEmpty = true;
                break;
            }
        }
    }
    SolveAll(resolve, resolveOnlyEmpty, drawOriginalMesh);

    //
    // Mesh
    //
    if (resolve) redivide = true; // force redivide if solver updated
    if (resolve || redivide) remesh = true; // force remesh if solver updated
    MeshAll(redivide, remesh);
    if (options.Mesher__debug_draw_cuttedMesh) UpdateViewerMeshFromModel(); // always update opengl since it links to Mesher, so every time Mesher changed - we need to update opengl

    //
    // Add draw objects to viewport
    //
    CopyDrawData_FromModel_to_openglDataDrawers(drawOriginalMesh);


    // refresh GUI since some options could be changed
    RefreshGUI();

    setCursor(utils::cursor::CursorType::Wait, false);
}

void ShowStatistic(Model& model, OpenGL_Window& window)
{
    cout << endl << "----- Statistic -----" << endl << endl;
    Timer timer_ShowStatistic;
    model.RefreshStats();
    string stat = "";
    stat += "Surfaces " + to_string(model.ActiveObjects.size());
    stat += "  Faces " + to_string(model.Stats.FacesCount);
    stat += "  Edges " + to_string(model.Stats.EdgesCount);
    stat += "  Vertexes " + to_string(model.Stats.VertexesCount);
    stat += "  Edge avarage length " + to_string(model.Stats.avg_edge_length) + "\n";

    long long memory_usage_total = 0;
    auto show_memory_usage = [&](string caption, II size_in_bytes)
    {
        if (size_in_bytes < 100) return;
        string text = caption + ": ";
        if (size_in_bytes < 1024 * 1024)
            //text += to_string(size_in_bytes) + " bytes";
            text += to_string((1.0*size_in_bytes) / (1.0*1024 * 1024)) + " Mb";
        else
            text += to_string((size_in_bytes) / (1024 * 1024)) + " Mb";
        text += "\n";
        stat += text;
        memory_usage_total += size_in_bytes;
    };

    if (meshLogicOptions.showTimeElapsed)
    {
        // show memory statistic    
        show_memory_usage("Memory allocated for mesh", model.Stats.MeshSizeOf);
        show_memory_usage("Memory allocated for draw", model.Stats.DrawSizeOf);
        show_memory_usage("Memory allocated for solver", model.Stats.SolverSizeOf);
        show_memory_usage("Memory allocated for ViewportObject mergedMeshes", opengl_objects.SizeOF(true, false, false, false));
        show_memory_usage("Memory allocated for ViewportObject data", opengl_objects.SizeOF(false, true, false, false));
        show_memory_usage("Memory allocated for ViewportObject opengl", opengl_objects.SizeOF(false, false, true, false));
        show_memory_usage("Memory allocated for Simplified meshes", simpleMeshes_heap.SizeOF());
        show_memory_usage("Memory allocated ", memory_usage_total);
        show_memory_usage("TOTAL", model.SizeOF() + opengl_objects.SizeOF(true, true, true, true));

        timer_ShowStatistic.stop(elapsed.ShowStatistic, elapsed.Total);
        AllTimeElapsed::coutAllTimers(); // counts also time of drawing, that way it is just after 'window.draw()' call
    }
    cout << stat << endl;


    // show fps stats
    //window.draw(); // first draw to avoid initialization times
    //glfwSwapBuffers(window.windowHandle);
    //auto time = utils::time::Now();
    //for (int i = 0; i < 10; i++)
    //{
    //    window.draw();
    //    glfwSwapBuffers(window.windowHandle);
    //}
    //auto draw1timeTime = utils::time::ElapsedMilliseconds(time) / (1000.0 * 10);
    //cout << "Draw time:   " << draw1timeTime << " ms  (" << 1 / draw1timeTime << " fps)" << endl;
}

void ReadFileFromDiskToMeshes(string filename, vector<Mesh>& meshes)
{
    meshes.clear();

    //
    // v0 - read obj file in 1 mesh
    //
    //meshes.push_back(Mesh());
    //Mesh::CreateFromFile(filename, meshes.back(), options.PrecacheMeshFile, options.SeparateGroups); // dont compute K when reading file - anyway data will be ignored since we will split mesh to parts and then will compute K for parts

    //
    // v1 - read obj file in multiple meshes
    //
    cout << "Reading file '" << filename << "' ...";
    Timer timer_ReadFileFromDisk;
    vector<PolygonMesh> polygonMeshes;
    vector<string> onlyObjectNames;
    int onlyGroupsCount = utils::strings::split(options.LoadOnlyObjectNames, onlyObjectNames, ";, ");
    if (MeshFile::LoadFrom(filename, polygonMeshes, onlyObjectNames, options.SeparateGroups))
    {
        vector<I3s> Fs;
        Fs.resize(polygonMeshes.size());
        extern bool IsOmpEnabled;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < polygonMeshes.size(); i++)
        {
            PolygonMesh& polyMesh = polygonMeshes[i];
            if (polyMesh.V.size() == 0 || polyMesh.ngons.size() == 0) continue;
            auto& F = Fs[i];
            polyMesh.ConvertNgonsToF(F);
        }
        timer_ReadFileFromDisk.stop(elapsedTimers.App.ReadFileFromDisk, elapsedTimers.App.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_ReadFileFromDisk << endl; else cout << endl;


        cout << "Converting ngons into meshes ...";
        Timer timer_ConvertingObj;
        meshes.resize(polygonMeshes.size());
        extern bool IsOmpEnabled;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < polygonMeshes.size(); i++)
        {
            PolygonMesh& polyMesh = polygonMeshes[i];
            if (polyMesh.V.size() == 0 || polyMesh.ngons.size() == 0) continue;
            Mesh& mesh = meshes[i];
            if (onlyGroupsCount != 0 && !utils::stdvector::exists(onlyObjectNames, polyMesh.groupname)) continue;
            //v0
            //polyMesh.ConvertToMesh(mesh);
            //v1
            mesh.Name = polyMesh.groupname;
            mesh.CreateFromVF(polyMesh.V, Fs[i], mesh, false, true, true, true, false);
            mesh.Name = polyMesh.groupname;
            mesh.FileName = filename;
        }
        polygonMeshes.clear();
        Fs.clear();
        timer_ConvertingObj.stop(elapsedTimers.Mesh.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_ConvertingObj << endl; else cout << endl;
    }
    else
    {
        cout << " Unsupported file extension!" << endl;
    }
}


// read file from disk and return fileIndex
int ReadFileFromDisk(string filename, bool alignCamera, bool loadSavedRotationState, bool& wasAlreadyLoaded)
{
    cout << endl << "----- Reading -----" << endl << endl;

    //
    // Change application title
    //
    string title = options.APPName + " - " + filename;
    glfwSetWindowTitle(window.windowHandle, title.c_str());

    //
    // Load mesh
    //
    int fileIndex = model.FindFile(filename); // try to reuse mesh loaded into memory from last load (this will be the case when same file is loaded thwice or more times)

    // clear file if we testing speed
    if ((options.TestSpeed_AddMeshes || options.TestSpeed_ReadFileFromObj) && fileIndex != -1)
    {
        model.RemoveFile(fileIndex);
        fileIndex = -1;
    }

    // clear file if we testing just few objects from file
    if (fileIndex != -1 && model.Files[fileIndex]->LoadedOnlyObjectNames != options.LoadOnlyObjectNames)
    {
        model.RemoveFile(fileIndex);
        fileIndex = -1;
    }

    wasAlreadyLoaded = (fileIndex != -1);

    //if file was never loaded into memory - load it from hard disk
    if (fileIndex == -1)
    {
        // Load obj file into mesh
        vector<Mesh> meshes;
        ReadFileFromDiskToMeshes(filename, meshes);
        if (options.TestSpeed_ReadFileFromObj)
        {
            cout << endl << " testing load speed - executing 'ReadFileFromDiskToMeshes' 10x times ..." << endl;
            cout << " disabling Solver to make test more clear ..." << endl;
            meshLogicOptions.Solver.Enabled = false;
            Timer timer_TestSpeed_ReadFileFromObj;
            for (int temp = 0; temp < 9; temp++)
            {
                ReadFileFromDiskToMeshes(filename, meshes);
            }
            timer_TestSpeed_ReadFileFromObj.stop();
            cout << " done test speed in " << timer_TestSpeed_ReadFileFromObj << endl << endl;
        }


        // Create model from mesh
        cout << "Adding mesh to model ...";
        Timer timer_AddMeshToModel;
        fileIndex = model.AddMeshes(filename, options.LoadOnlyObjectNames, meshes, false);// breaks apart big mesh to multiple meshes that has closed outher loop (to multiple surfaces) , (skip refreshing since we will do it later in this method )

        // DEBUG - test load speed - load 10x times
        if (options.TestSpeed_AddMeshes)
        {
            cout << " testing load speed - executing 'model.AddMesh' 10x times ...";
            for (int temp = 0; temp < 9; temp++)
            {
                model.RemoveFile(fileIndex);
                fileIndex = model.AddMeshes(filename, options.LoadOnlyObjectNames, meshes, false); // (skip refreshing since we will do it later in this method )
            }
            //meshLogicOptions.Solver.Enabled = false; // disable solver when we testing load speed - since we dont it it at all
        }
        timer_AddMeshToModel.stop(elapsedTimers.Mesh.Total);
        if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer_AddMeshToModel << endl; else cout << endl;
    }

    // set file active if it was loaded
    if (fileIndex != -1)
    {
        model.SetFileActive(*model.Files[fileIndex], true, true, false); // last parameter 'false' - (skip refreshing since we will do it later in this method)
    }

    //
    // activate only some surfaces if user have highlighten them
    //
    vector<int> showOnlyMeshIds = utils::strings::extractInts(options.ShowOnlyMeshIds);
    int showOnlyMeshIds_size = showOnlyMeshIds.size();
    if (showOnlyMeshIds_size > 10) utils::stdvector::sort(showOnlyMeshIds);
    vector<int> showOnlyObjectIds = utils::strings::extractInts(options.ShowOnlyObjectIds);
    int showOnlyObjectIds_size = showOnlyObjectIds.size();
    if (showOnlyObjectIds_size > 10) utils::stdvector::sort(showOnlyObjectIds);
    for (auto&o : model.AllObjects)
    {
        o->IsActive = (showOnlyMeshIds_size == 0 && showOnlyObjectIds_size == 0); // if user highlighted some then make by default unactive

        if (showOnlyMeshIds_size > 0)
        {
            o->IsActive = showOnlyMeshIds_size > 10
                ? utils::stdvector::exists_in_sorted_list(showOnlyMeshIds, o->srf.mesh.id)
                : utils::stdvector::exists(showOnlyMeshIds, o->srf.mesh.id);
        }

        if (showOnlyObjectIds_size > 0)
        {
            o->IsActive = showOnlyObjectIds_size > 10
                ? utils::stdvector::exists_in_sorted_list(showOnlyObjectIds, o->Id)
                : utils::stdvector::exists(showOnlyObjectIds, o->Id);
        }
    }
    model.RefreshActiveObjects();
    Loaded_filename = filename;

    //
    // Set mesh to viewport
    //
    UpdateViewerMeshFromModel();

    // set viewport camera
    //cout << "Changing viewport camera to center ...";
    Timer timer_alignCamera;
    if (alignCamera)
    {
        window.align_camera_center();
    }
    auto save_camera_dnear = window.viewport.camera_dnear;
    timer_alignCamera.stop(elapsed.AlignCameraCenter, elapsed.Total);

    //
    // Load saved rotation state
    //
    Timer timer_LoadRotation;
    if (loadSavedRotationState)
    {
        string optionfilename = options.GetOptionFile(Loaded_filename);
        if (utils::file::Exists(optionfilename))
        {
            // load saved model location
            window.viewport.serializeCamera = true;
            window.viewport.serializeVisualizationOptions = false;
            window.load_scene(optionfilename);
            std::cout << "Viewport Position loaded from file " << optionfilename << endl;
        }
    }
    window.viewport.camera_dnear = save_camera_dnear; // revert value from saved settings - better to remove from serialize but i dont want to refresh all options files
    timer_LoadRotation.stop(elapsed.LoadCameraRotation, elapsed.Total);

    //DEBUG show viewport serialize properties
    //const Viewport& v = window.viewport;
    //cout << endl << endl << endl << endl;
    ////cout << "trackball_angle " << v.trackball_angle << endl;
    //cout << "rotation_type " << v.rotation_type << endl;
    //cout << "model_zoom " << v.model_zoom << endl;
    //cout << "model_translation " << v.model_translation << endl;
    //cout << "model_zoom_uv " << v.model_zoom_uv << endl;
    //cout << "model_translation_uv " << v.model_translation_uv << endl;
    //cout << "camera_zoom " << v.camera_zoom << endl;
    //cout << "orthographic " << v.orthographic << endl;
    //cout << "camera_view_angle " << v.camera_view_angle << endl;
    //cout << "camera_dnear " << v.camera_dnear << endl;
    //cout << "camera_dfar " << v.camera_dfar << endl;
    //cout << "camera_eye " << v.camera_eye << endl;
    //cout << "camera_center " << v.camera_center << endl;
    //cout << "camera_up " << v.camera_up << endl;
    //cout << "object_scale " << v.object_scale << endl;
    //cout << "view " << v.view << endl;
    //cout << "model " << v.model << endl;
    //cout << "proj " << v.proj << endl;
    //cout << endl << endl << endl << endl;

    //timer_Total.stop(elapsed.ReadFileFromDisk);
    //cout << "Total read time:   " << timer_Total << endl;


    return fileIndex;
}


void LoadFile(string filename, bool alignCamera, bool loadSavedRotationState, bool resolve, bool restartAllTimers = true)
{
    LoadFile_LastLoadedFile = filename;

    Timer time;
    setCursor(utils::cursor::CursorType::Wait, true);

    ClearConsole();


    if (restartAllTimers)
    {
        AllTimeElapsed::resetAllTimers();
    }

    //Read
    bool wasAlreadyLoaded;
    model.ClearDrawObjects();
    int fileIndex = ReadFileFromDisk(filename, alignCamera, loadSavedRotationState, wasAlreadyLoaded);

    // Solve    
    if (fileIndex != -1)
    {
        if (!wasAlreadyLoaded) resolve = true;
        Refresh(resolve);
    }

    // draw
    window.drawViewportAndGui();

    // Show stats
    ShowStatistic(model, window);

    //DEBUG testing MeshSimplification
    if (meshLogicOptions.MeshSimplification.DebugEnabled) OnOptionChanged_MeshSimplification();

    setCursor(utils::cursor::CursorType::Wait, false);
    time.stop();
    if (meshLogicOptions.showTimeElapsed) cout << "LoadFile time: " << time << endl;
}
void OnOpenObjFile_File(string filename)
{
    LoadFile(filename, true, true, true);
}
void LoadFileAtAppStart()
{
    string objfilename = options.GetTestFileName();

    if (commandLineArguments.size() > 0 && utils::file::Exists(commandLineArguments[0]))
    {
        string commandLineFilename = commandLineArguments[0];
        if (MeshFile::IsFileTypeSupported(commandLineFilename))
        {
            options.ShowOnlyMeshIds = "";
            options.ShowOnlyObjectIds = "";
            options.LoadOnlyObjectNames = "";
            objfilename = commandLineFilename;
            options.PrecacheMeshFile = false;
            extern bool IsOmpEnabled;
            IsOmpEnabled = true;
        }
    }

    LoadFile(objfilename, true, true, false, false);
}

void OnOptionChanged_GuiStyle();
bool viewer_callback_createGUI(OpenGL_Window& window)
{
    options.AddOptions(window);

    theme = new NanoguiTheme(window.screen->nvgContext());
    OnOptionChanged_GuiStyle();

    return false;
};

void callback_shutdown(OpenGL_Window& window)
{
    //TODO: shutdown all opengl handles (before opengl window is destroyed)
};

bool callback_drop(OpenGL_Window& window, vector<string> filenames)
{
    cout << "droped files: ";
    for (auto f : filenames) cout << endl << f;
    cout << endl;
    if (filenames.size() == 0) return false;

    // read all files except last one
    //for (auto filename : filenames)
    //{
    //    if (filename == filenames.back()) break; // skip last file because we will load it after this loop
    //    auto alignCamera = true;
    //    auto loadSavedRotationState = true;  
    //    bool wasAlreadyLoaded;
    //    ReadFileFromDisk(filename, alignCamera, loadSavedRotationState, wasAlreadyLoaded);
    //}
    OnOpenObjFile_File(filenames.back());
    return true;
};


bool viewer_callback_init_loadFileIntoViewport(OpenGL_Window& window)
{

    // load statup mesh file
    Timer time;
    LoadFileAtAppStart();
    time.stop();

    //cout << "Start app time:   " << utils::time::ElapsedSecondsStr(timeStartApp) << "     where LoadFileAtAppStart time: " << time << endl;
    string startup_time = "Startup time:   " + utils::time::ElapsedSecondsStr(timeStartApp);
    if (meshLogicOptions.showTimeElapsed) cout << startup_time << endl;

    // DEBUG - show startup time and faces count in title
    //startup_time += "      Faces count=" + to_string(model.Stats.FacesCount);
    //glfwSetWindowTitle(window.windowHandle, startup_time.c_str());

    return false;
};

bool callback_key_down(OpenGL_Window& window, int key, int modifier)
{
    if (options.callback_key_down(window, key, modifier))
    {
        return true;
    }
    return false;
}
bool callback_key_pressed(OpenGL_Window& window, unsigned int unicode_key, int modifiers)
{
    if (options.callback_key_pressed(window, unicode_key, modifiers))
    {
        return true;
    }
    return false;
}



void OnOptionChanged_Constrain()
{
    AllTimeElapsed::resetAllTimers();
    ClearConsole();
    for (auto obj : model.AllObjects) // clear all Contrains and Results
    {
        obj->srf.solver.Clear();
        obj->srf.solverUV.Clear();
    }
    Refresh(true);
    window.drawViewportAndGui();
    if (meshLogicOptions.showTimeElapsed) AllTimeElapsed::coutAllTimers();
}

void OnOptionChanged_Solver()
{
    AllTimeElapsed::resetAllTimers();
    ClearConsole();
    for (auto obj : model.AllObjects) // clear all Results
    {
        obj->srf.solver.Result.Clear();
        obj->srf.solverUV.Result.Clear();
    }
    Refresh(true);
    window.drawViewportAndGui();
    if (meshLogicOptions.showTimeElapsed) AllTimeElapsed::coutAllTimers();
}

void OnOptionChanged_MeshTopology()
{
    AllTimeElapsed::resetAllTimers();
    ClearConsole();
    for (auto f : model.Files)
    {
        f->UpdateMeshTopology(false);
    }
    Refresh(false);
    window.drawViewportAndGui();
    if (meshLogicOptions.showTimeElapsed) AllTimeElapsed::coutAllTimers();
}

void OnOptionChanged_MeshTopology_TestPerformanceCall()
{
    //AllTimeElapsed::resetAllTimers();
    //ClearConsole();
    if (meshLogicOptions.MeshesTopology.debug_TestPerformanceCall)
    {
        bool debugEnables_save = meshLogicOptions.MeshesTopology.DebugEnabled;
        meshLogicOptions.MeshesTopology.DebugEnabled = false;
        Timer timer;
        cout << endl << "Testing performance of Mesh Topology - calling update Topology " << meshLogicOptions.MeshesTopology.debug_TestPerformanceCall_repeatTimes << " times..." << endl;
        for (int i = 0; i < meshLogicOptions.MeshesTopology.debug_TestPerformanceCall_repeatTimes; i++)
        {
            for (auto f : model.Files)
            {
                f->UpdateMeshTopology(false);
            }
        }
        timer.stop();
        cout << "total time:   " << timer << std::endl;
        meshLogicOptions.MeshesTopology.DebugEnabled = debugEnables_save;
    }
    //Resolve(false, false);
    //window.draw();
    //AllTimeElapsed::coutAllTimers();
}

void OnOptionChanged_MeshStreamTopology()
{
    AllTimeElapsed::resetAllTimers();
    ClearConsole();
    for (auto f : model.Files)
    {
        //TODO caching for StreamTopology can be done here 
        //f->UpdateStreamTopology(false);
    }
    Refresh(false);
    window.drawViewportAndGui();
    if (meshLogicOptions.showTimeElapsed) AllTimeElapsed::coutAllTimers();
}

void OnOptionChanged_MeshStreamTopology_TestPerformanceCall()
{
    if (meshLogicOptions.MeshStreamsTopology.debug_TestPerformanceCall)
    {
        //bool debugEnables_save = meshLogicOptions.MeshStreamsTopology.DebugEnabled;
        //meshLogicOptions.MeshStreamsTopology.DebugEnabled = false;
        Timer timer;
        cout << endl << "Testing performance of Stream Topology - calling update Topology " << meshLogicOptions.MeshStreamsTopology.debug_TestPerformanceCall_repeatTimes << " times..." << endl;
        for (int i = 0; i < meshLogicOptions.MeshStreamsTopology.debug_TestPerformanceCall_repeatTimes; i++)
        {
            for (auto f : model.Files)
            {
                //TODO testing speed of StreamTopology can be done here 
                //f->UpdateStreamTopology(false);
            }
        }
        timer.stop();
        cout << "total time:   " << timer << std::endl;
        //meshLogicOptions.MeshStreamsTopology.DebugEnabled = debugEnables_save;
    }
}

void OnOptionChanged_RedrawWithoutResolve()
{
    ClearConsole();
    Refresh(false);
}

void On_OptionChanged_Mesher(bool call_UpdateViewerMeshFromModel)
{
    AllTimeElapsed::resetAllTimers();
    ClearConsole();
    if (call_UpdateViewerMeshFromModel)
    {
        UpdateViewerMeshFromModel();
    }
    Refresh(false);
    window.drawViewportAndGui();
    if (meshLogicOptions.showTimeElapsed) AllTimeElapsed::coutAllTimers();
}
void OnOptionChanged_File(bool alignCamera, bool resolve, bool testFileChanged)
{
    string filename = (testFileChanged || LoadFile_LastLoadedFile.empty())
        ? options.GetTestFileName()
        : LoadFile_LastLoadedFile;
    LoadFile(filename, alignCamera, alignCamera, resolve);
}
void OnOptionChanged_Mesh()
{
    model.RemoveAllFiles();
    OnOptionChanged_File(false, true, false);
}
void On_MeshHeal_Executed()
{
    bool save = meshLogicOptions.Mesh.Heal;
    meshLogicOptions.Mesh.Heal = true;
    OnOptionChanged_Mesh();
    meshLogicOptions.Mesh.Heal = save;
    window.nanogui->refresh(); // relect change to option 'meshLogicOptions.Mesh.Heal'
}
void OnRememberModelLocation_File()
{
    string optionfilename = options.GetOptionFile(Loaded_filename);
    window.viewport.serializeCamera = true;
    window.viewport.serializeVisualizationOptions = false;
    window.save_scene(optionfilename);
    std::cout << "Viewport Position saved into file " << optionfilename << endl;
}

void On_Mesher_SaveDebugedMeshToFile(string filename)
{
    for (auto file : model.Files)
    {
        if (file->IsActive && file->ActiveObjects.size() > 0)
        {
            vector<const Mesh*> meshes = file->mesher.GetCuttedMeshes();
            for (const auto& m : meshes)
            {
                if (m->id == meshLogicOptions.Mesher.debug_debug_mesh_id)
                {
                    m->SaveToFile_Obj(filename);
                    return;
                }
            }
        }
    }
}

void On_Mesher_SaveQuadMeshToFile(string filename)
{
    int vertextesStartIndexShift = 0; // first PolygonMeshes will write from beginning of output file 
    for (auto file : model.Files) // for each file 
    {
        if (file->IsActive && file->ActiveObjects.size() > 0) // write to obj only visible files
        {
            PolygonMeshes polygons = file->mesher.GetPolygonMeshes();
            cout << "saving quad mesh to obj file: " << filename << " ...";
            Timer timeSaveToObj;
            if (MeshFile::SaveToObj(filename, polygons, vertextesStartIndexShift))
            {
                vertextesStartIndexShift += polygons.GetVertexesCount(); // next file will be append to output obj file and vertex indexes will be shifted by value 'vertextesStartIndex'
                timeSaveToObj.stop();
                if (meshLogicOptions.showTimeElapsed) cout << " done in " << timeSaveToObj << endl; else cout << endl;
            }
            else
            {
                cout << " failed to write !!! " << endl;
            }
        }
    }
}

void OnOptionChanged_GuiStyle()
{
    window.screen->setTheme(theme);
    theme->SetColorStyle(options.ThemeColorStyle);
    window.viewport.background_color = theme->background_color;
    //window.viewport.wireframe_color = theme->wireframe_color;
    //window.ngui->refresh();
    //window.screen->performLayout();
}

void OnOptionChanged_Draw()
{
    Refresh(false, false, false);
}

void OnOptionChanged_Draw_Crease(bool crease_normals, D crease_normals_angle)
{
    ForceRecreateSurfaces();
    for (auto obj : model.AllObjects)
    {
        obj->srf.NormalCorrections.Clear();
    }
    UpdateViewerMeshFromModel();
}


void OnOptionChanged_OpenGL(bool reinit_opengl)
{
    //v0 - hot refresh
    //opengl_objects.Remove(model_opengl_object);
    //opengl_objects.Remove(surfaces_opengl_object);
    //opengl_objects.Remove(mesher_opengl_object);
    //UpdateViewerMeshFromModel();// we need to reassign surfaces to another opengl
    //OnOptionChanged_Draw(); //redraw lines and labels to new opengl, but leave current solver

    //v1 - cold refresh
    if (reinit_opengl)
    {
        if (model_opengl_object) model_opengl_object->opengl.reinit();
        if (surfaces_opengl_object) surfaces_opengl_object->opengl.reinit();
        if (mesher_opengl_object) mesher_opengl_object->opengl.reinit();
    }
}


#include <igl/unproject_onto_mesh.h>

void On_SelectByMouse_ShowOnly()
{
    extern bool IsOmpEnabled;
    cout << endl << "Please pick a mesh ..." << endl;

    window.callback_mouse_down = [&](OpenGL_Window& window, int button, int modifier)->bool
    {
        double x = window.current_mouse_x;
        double y = window.viewport.size(3) - window.current_mouse_y;
        Vector2f xy_pos = Vector2d(x, y).cast<float>();
        Matrix4f viewport_view = window.viewport.view.cast<float>();
        Matrix4f viewport_model = window.viewport.model.cast<float>();
        Matrix4f viewport_proj = window.viewport.proj.cast<float>();
        Vector4f viewport_size = window.viewport.size.cast<float>();
        model.draw.Clear();
        for (auto file : model.Files)
        {
            if (file->IsActive && file->ActiveObjects.size() > 0)
            {
                auto objects = file->ActiveObjects;
                #pragma omp parallel for  if(IsOmpEnabled)
                for (int i = 0; i < objects.size(); i++)
                {
                    auto& obj = objects[i];
                    //if (!obj->srf.solver.Result.isEmpty()) continue;
                    const Mesh& mesh = obj->srf.mesh;
                    int fid = -1;
                    PointXd bc;
                    if (igl::unproject_onto_mesh(xy_pos, viewport_view*viewport_model, viewport_proj, viewport_size, convertP3sToEigenDouble(mesh.V), convertI3sToEigenInt(mesh.F), fid, bc))
                    {
                        model.draw.AddPoint(convertEigenToP3(bc), Color3d(1, 0, 0), "hit in mesh=" + to_string(mesh.id) + "   fid=" + to_string(fid));
                        //meshIds = to_string(mesh.id);
                        cout << "meshId = " << mesh.id << endl;
                        OnOptionChanged_Draw();
                        break;
                    }
                }
            }
        }
        return true;
    };
}

void OnOptionChanged_MeshSimplification()
{
    //DEBUG MeshSimplification
    if (meshLogicOptions.MeshSimplification.DebugEnabled)
    {
        // allow to draw debug data
        if (surfaces_opengl_object) surfaces_opengl_object->data.clearDraws();
        model.ClearDrawObjects(); // clear prev lines
        UpdateViewerMeshFromModel();
        window.viewport.show_faces = true;
        CopyDrawData_FromModel_to_openglDataDrawers(true);
        return; // skip normal execution - we in debug mode
    }

    OnOptionChanged_Draw(); //redraw lines and labels to new opengl, but leave current solver    
}

int main(int argc, char *argv[])
{
    current_mesh_simplification.Level = 1;

    // Set callbacks
    options.On_MeshHeal_Executed = &On_MeshHeal_Executed;
    options.On_OptionChanged_Mesh = &OnOptionChanged_Mesh;
    options.On_OptionChanged_Constrain = &OnOptionChanged_Constrain;
    options.On_OptionChanged_MeshTopology = &OnOptionChanged_MeshTopology;
    options.On_OptionChanged_MeshTopology_TestPerformanceCall = &OnOptionChanged_MeshTopology_TestPerformanceCall;
    options.On_OptionChanged_Solver = &OnOptionChanged_Solver;
    options.On_OptionChanged_Stream = &OnOptionChanged_Draw;
    options.On_OptionChanged_StreamTopology = &OnOptionChanged_MeshStreamTopology;
    options.On_OptionChanged_StreamTopology_TestPerformanceCall = &OnOptionChanged_MeshStreamTopology_TestPerformanceCall;
    options.On_OptionChanged_Divider = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_Joiner = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_DividerIterator = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_DividerOptimalConnector = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_DividerLogicConnector = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_MeshCutter = &OnOptionChanged_RedrawWithoutResolve;
    options.On_OptionChanged_Mesher = &On_OptionChanged_Mesher;
    options.On_Mesher_SaveDebugedMeshToFile = &On_Mesher_SaveDebugedMeshToFile;
    options.On_Mesher_SaveQuadMeshToFile = &On_Mesher_SaveQuadMeshToFile;
    options.On_OptionChanged_File = &OnOptionChanged_File;
    options.On_OptionChanged_GuiStyle = &OnOptionChanged_GuiStyle;
    options.On_OptionChanged_Draw = &OnOptionChanged_Draw;
    options.On_OptionChanged_Draw_Crease = &OnOptionChanged_Draw_Crease;
    options.On_OptionChanged_OpenGL = &OnOptionChanged_OpenGL;
    options.On_SelectByMouse_ShowOnly = &On_SelectByMouse_ShowOnly;
    options.On_OptionChanged_MeshSimplification = &OnOptionChanged_MeshSimplification;
    options.On_RememberModelLocation_File = &OnRememberModelLocation_File;
    options.On_OpenObjFile_File = &OnOpenObjFile_File;
    window.callback_key_down = &callback_key_down;
    window.callback_key_pressed = &callback_key_pressed;
    window.callback_createGUI = &viewer_callback_createGUI;
    window.callback_shutdown = &callback_shutdown;
    window.callback_init_loadFileIntoViewport = &viewer_callback_init_loadFileIntoViewport;
    window.callback_drop = &callback_drop;
    //window.callback_key_pressed = &callback_key_pressed;

    Timer time;
    options.SetOpenGLOptions();
    options.SetViewerOptions(window);
    options.SetMeshLogicOptions(window);
    time.stop(elapsed.SetViewerOptions, elapsed.Total);

    // Interpolate the field and plot
    //viewer_callback_key_down(window, '1', 0);

    // Launch the viewer
    window.drawer = &opengl_objects; //set opengl drawer
    window.launch(true, false, true, options.APPName);
}


std::wstring s2ws(const std::string& s)
{
    int len;
    int slength = static_cast<int>(s.length()) + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    std::wstring r(len, L'\0');
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, &r[0], len);
    return r;
}

std::string ws2s(const std::wstring& s)
{
    int len;
    int slength = static_cast<int>(s.length());
    len = WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, 0, 0, 0, 0);
    std::string r(len, '\0');
    WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, &r[0], len, 0, 0);
    return r;
}

void ReadCommandLineArguments(LPWSTR    lpCmdLine)
{
    Timer time;
    LPWSTR *szArgList;
    int argCount;
    szArgList = CommandLineToArgvW(lpCmdLine, &argCount);
    if (szArgList == NULL)
    {
        //MessageBox(NULL, "Unable to parse command line", "Error", MB_OK);
        //return 10;
        return;
    }
    for (int i = 0; i < argCount; i++)
    {
        wstring ws(szArgList[i]);
        string s = ws2s(ws);
        cout << "Command line argument #" << i << ":     " << s << endl;
        commandLineArguments.push_back(s);
        //MessageBox(NULL, szArgList[i], "Arglist contents", MB_OK);
    }
    LocalFree(szArgList);
    time.stop(elapsed.ReadCommandLineArguments, elapsed.Total);
}

void AllocateConsoleWindow()
{
    Timer time;
    AllocConsole();
    HWND consoleWindow = GetConsoleWindow();

    // hide console window from taskbar
    //long style = GetWindowLong(consoleWindow, GWL_EXSTYLE);
    //style &= ~(WS_VISIBLE);    // this works - window become invisible 
    //style |= WS_EX_TOOLWINDOW;   // flags don't work - windows remains in taskbar
    //style &= ~(WS_EX_APPWINDOW);
    //ShowWindow(consoleWindow, SW_HIDE); // hide the window
    //SetWindowLong(consoleWindow, GWL_EXSTYLE, style); // set the style
    //ShowWindow(consoleWindow, SW_SHOW); // show the window for the new style to come into effect

    // move console window to second monitor
    //SetWindowPos(consoleWindow, 0, 0, 35, 1900, 1165, SWP_NOZORDER); // current monitor 
    SetWindowPos(consoleWindow, 0, -1920, 7, 1920, 1200, SWP_NOZORDER); // second monitor is on left side
    //SetWindowPos(consoleWindow, 0, 1920, 0, 1920, 1200, SWP_NOZORDER);// second monitor is on right side


    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);

    // enable mouse wheel on console
    DWORD lpMode = 0;
    HANDLE hConsole = CreateFile("CONIN$", GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL); //  get console handle
    if (!GetConsoleMode(hConsole, &lpMode)) // get mode of console 
    {
        DWORD err = GetLastError();
    }
    SetConsoleMode(hConsole, lpMode & ~ENABLE_MOUSE_INPUT | ENABLE_PROCESSED_INPUT); // update mode with mouse wheel enabled mode
    time.stop(elapsed.AllocateConsoleWindow, elapsed.Total);
}


// using window we can catch assertions and disable console in release mode.
int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    timeStartApp = utils::time::Now();
    ios_base::sync_with_stdio(false);
    glPreinit_PixelFormat_calls();// preinit long call to opengl - this speed ups application start on my computer by 0.1 seconds

    AllocateConsoleWindow();
    //int vertexCacheColor = (178 << 16) + (0 << 8) + (0 << 0);
    ////int vertexCacheColor = 178 << 16 + 0 << 8 + 0 << 0;
    //float x = float((vertexCacheColor >> 16) & 255) / 255;
    //float y = float((vertexCacheColor >> 8) & 255) / 255;
    //float z = float((vertexCacheColor >> 0) & 255) / 255;
    //int r = int(x * 255);
    //int g = int(y * 255);
    //int b = int(z * 255);

    //float _178float = 178;
    //int* _178intP = (int*)&_178float;
    //int _178int = *_178intP;

    //#if DEBUG

    //#endif
    utils::cpu::readCPUinfo();
    Options::Update_OmpCpuCores();
    ReadCommandLineArguments(lpCmdLine);
    //testEigenMatrix();

    char* args[] = { "-c" };
    main(1, args);
    return 0;
}











