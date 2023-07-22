#pragma once
#include "MeshTopology.h"
#include "Mesher.h"

class Model;
class ModelObject;

// should be created as pointer, because this class will be in the changable list 
class ModelFile
{
private:
    atomic_int nextSrfId;
    int readonly_Id;
public:
    const int& Id;
    const Model& model;
    string Filename;//should be readonly by how to make string readonly
    string LoadedOnlyObjectNames;//what objects was loaded from file. Empty string means all objects was loaded   (should be readonly by how to make string readonly)
    bool IsActive;//if file is active. model can have multiple files, and some of them can be active and visible and some inactive and unvisible. for active object we solve and show them for user.
    vector<ModelObject*> AllObjects; // all  objects in file - populated in method Model::AddMesh();
    vector<ModelObject*> ActiveObjects; // all active objects in file - populated in method Model::RefreshActiveObjects();
    vector<int> ActiveObjects_Ids; // all active objects serial numbers in file (sorted array)
    Topology Topology;
    Mesher mesher;

    ModelFile(const Model& model, const string& filename, const string& loadedOnlyObjectNames);
    int GetNextSrfUniqueId();
    II SizeOF() const;
    void UpdateMeshTopology(bool updateOnlyIfObjectsAreChanged);
    void RefreshActiveObjects(); // refreshes 'ActiveObjects' and 'ActiveObjects_SerialNumbers' after changing 'AllObjects' or objects 'IsActive' property
    void ClearDrawObjects();
};
