#pragma once
#include "ViewerDrawObjects.h"

class ModelObject;
class ModelFile;
class Mesh;

struct ModelStats
{
    II VertexesCount;
    II EdgesCount;
    II FacesCount;
    DD avg_edge_length;
    II MeshSizeOf;
    II DrawSizeOf;
    II SolverSizeOf;
};

class Model
{
public:
    vector<ModelObject*> AllObjects; // all objects - this vector contains pointer to allocated memory 
    vector<ModelObject*> ActiveObjects; // only visible objects
    vector<int> ActiveObjects_Ids; // is needed for surface shared drawing
    vector<ModelFile*> Files;
    ModelStats Stats;
    ViewerDrawObjects draw;

    Model();
    ~Model();
    II SizeOF() const;
    int AddMeshes(string filename, string loadedOnlyObjectNames, vector<Mesh>& meshes, bool refreshActiveObjects = true); // return fileIndex
    void SetObjectActive(ModelObject& obj, bool active, bool inactivateOthersFromSameFile);
    void SetFileActive(ModelFile& file, bool active, bool inactivateOthers, bool refreshActiveObjects = true);
    int FindFile(const string& filename); // return fileIndex for property 'vector<ModelFile*> Files'
    void RemoveFile(int fileIndex);
    void RemoveAllFiles();
    void ClearDrawObjects();
    void RefreshStats();
    void RefreshActiveObjects();
private:
};
