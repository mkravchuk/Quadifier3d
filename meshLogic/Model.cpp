#include "stdafx.h"
#include "Model.h"
#include "ModelFile.h"
#include "ModelObject.h"
#include "Mesh.h"
#include "ViewerDrawObjects.h"

Model::Model()
{
    
}


Model::~Model()
{
    RemoveAllFiles();
}

II Model::SizeOF() const
{    
    II r = sizeof(Model);

    r += AllObjects.size()*(sizeof(ModelObject*) + sizeof(ModelObject));
    r += ActiveObjects.size() * sizeof(ModelObject*);
    r += Files.size() * sizeof(ModelFile*);
    for (const auto file : Files)
    {
        r += file->SizeOF();
    }
    for (auto obj : AllObjects)
    {
        r += obj->SizeOF();
    }
    r += draw.SizeOF();
    return r;
}

void Model::RefreshActiveObjects()
{
    ActiveObjects.clear();
    ActiveObjects_Ids.clear();

    for (auto file : Files)
    {
        file->RefreshActiveObjects();
    }

    ActiveObjects.reserve(AllObjects.size());
    ActiveObjects_Ids.reserve(AllObjects.size());
    // v1 - using global array
    //for (auto obj : AllObjects)
    //{
    //    if (obj->IsActive && obj->file.IsActive)
    //    {
    //        ActiveObjects.push_back(obj);
    //        ActiveObjects_Ids.push_back(obj->Id);
    //    }
    //}
    // v2 - using files array
    for (auto file : Files)
    {
        for (auto obj : file->ActiveObjects)
        {
            ActiveObjects.push_back(obj);
            ActiveObjects_Ids.push_back(obj->Id);
        }
    }
    RefreshStats();
}

void Model::RefreshStats()
{
    Stats.VertexesCount = 0;
    Stats.EdgesCount = 0;
    Stats.FacesCount = 0;
    Stats.avg_edge_length = 0;
    Stats.MeshSizeOf = 0;
    Stats.DrawSizeOf = 0;
    Stats.SolverSizeOf = 0;
    for (auto o : ActiveObjects)
    {
        Stats.VertexesCount += o->srf.mesh.VertexCount;
        Stats.EdgesCount += o->srf.mesh.EdgesCount;
        Stats.FacesCount += o->srf.mesh.FacesCount;
        Stats.avg_edge_length += o->srf.mesh.avg_edge_length;
        Stats.MeshSizeOf += o->srf.mesh.SizeOF();
        Stats.DrawSizeOf += o->srf.draw.SizeOF();
        Stats.SolverSizeOf += o->srf.solver.SizeOF();
        Stats.SolverSizeOf += o->srf.solverUV.SizeOF();
    }
    for (auto f : Files)
    {
        Stats.DrawSizeOf += f->Topology.draw.SizeOF();
    }
    if (ActiveObjects.size() != 0)
    {
        Stats.avg_edge_length /= ActiveObjects.size();
    }
}

int Model::AddMeshes(string filename, string loadedOnlyObjectNames, vector<Mesh>& meshes, bool refreshActiveObjects)
{
    int fileIndex = FindFile(filename);
    if (fileIndex == -1)
    {
        // add file
        ModelFile* f = new ModelFile(*this, filename, loadedOnlyObjectNames);  // NOLINT
        Files.push_back(f);
        fileIndex = Files.size() - 1;

        // break meshes apart
        vector<Mesh> temp_part_meshes;
        vector<Mesh*> breaked_meshes;
        MeshPart::ExtractMeshes(meshes, temp_part_meshes, breaked_meshes);

        // create ModelObject from MeshPart in multithreading
        vector<ModelObject*> newobjects(breaked_meshes.size(), nullptr);
        extern bool IsOmpEnabled;
        #pragma omp parallel for  if(IsOmpEnabled)
        for (int i = 0; i < breaked_meshes.size(); i++)
        {
            Mesh* mesh = breaked_meshes[i];
            newobjects[i] = new ModelObject(*this, *Files[fileIndex], mesh->Name, *mesh);
        }

        // add new objects to model objects pull
        f->AllObjects.reserve(newobjects.size());
        AllObjects.reserve(AllObjects.size() + newobjects.size());
        for (auto o : newobjects)
        {
            f->AllObjects.push_back(o);
            AllObjects.push_back(o);
        }
    }
    else
    {
        cout << "" << endl;
    }
    if (refreshActiveObjects)
    {
        RefreshActiveObjects();
    }
    return fileIndex;
}

int Model::FindFile(const string& filename)
{
    for (int i = 0; i < Files.size(); i++)
    {
        if (filename.length() > 0
            && Files[i]->Filename == filename)
        {
            return i;
        }
    }
    return -1;
}

void Model::RemoveFile(int fileIndex)
{
    if (fileIndex == -1) return;
    if (fileIndex > Files.size() - 1) return;
    ModelFile* file = Files[fileIndex];
    
    // delete and remove objects
    vector<ModelObject*> restObject; //  will be stored not deleted objects
    restObject.reserve(AllObjects.size());
    for (ModelObject*& obj : AllObjects)
    {
        if (obj->file.Id == file->Id)
        {
            delete(obj);
        }
        else
        {
            restObject.push_back(obj);
        }
    }
    AllObjects = restObject;
    utils::stdvector::remove_at(Files, fileIndex);
    RefreshActiveObjects();
}

void Model::RemoveAllFiles()
{
    for (auto obj : AllObjects)
    {
        delete obj;
    }
    AllObjects.clear();
    for (auto file : Files)
    {
        delete file;
    }
    Files.clear();
    RefreshActiveObjects();
}

void Model::SetObjectActive(ModelObject& obj, bool active, bool inactivateOthersFromSameFile)
{
    if (inactivateOthersFromSameFile)
    {
        for (auto o : ActiveObjects)
        {
            if (o->file.Id == obj.file.Id)// if objects from same file
            {
                o->IsActive = false;
            }
        }
    }
    obj.IsActive = active;
    RefreshActiveObjects();
}

void Model::SetFileActive(ModelFile& file, bool active, bool inactivateOthers, bool refreshActiveObjects)
{
    if (inactivateOthers)
    {
        for (auto f : Files)
        {
            f->IsActive = false;
        }
    }
    file.IsActive = active;
    if (refreshActiveObjects)
    {
        RefreshActiveObjects();
    }
}

void Model::ClearDrawObjects()
{
    draw.Clear();
    for (auto o : ActiveObjects)
    {
        o->srf.draw.Clear();
    }
    for (auto f : Files)
    {
        f->ClearDrawObjects();
    }
}