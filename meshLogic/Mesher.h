#pragma once
#include "Mesh.h"
#include "MeshTopology.h"
#include "Divider.h"
#include "MeshCutter.h"
#include "PolygonMesh.h"
#include <vector>
#include "MeshGenerator.h"

class ModelFile;
class Mesh;
class MeshSurface;
class ViewerDrawObjects;



 
class Mesher  
{
private:
    bool isDividedIntoQuadMeshes;
public:
    const ModelFile& file;
    ViewerDrawObjects draw;

    D meshSize;
    vector<Divider> dividers; //  caches dividers for original meshes
    vector<MeshCutter> meshCutters; // divides original meshes and store as a cache cuted meshes (that was made base on dividers)
    MeshGenerator meshGenerator;

    Mesher(const ModelFile& file);
    II SizeOF() const;

    void ClearCache(); // clear all information that was produced by methods 'DivideComplexMeshesIntoQuadMeshes' and 'GenerateMesh'
    vector<const Mesh*> GetCuttedMeshes(); // extract all meshes from property 'meshCutters', where all cutted meshes are cached by algorithm 'MeshCutter'
    void DivideComplexMeshesIntoQuadMeshes(vector<ModelObject*> objects);
    void GenerateMesh();
    PolygonMeshes GetPolygonMeshes();

    void ClearDrawObjects();
    void DrawAll();
private:
    D GetMeshSize(const vector<ModelObject*>& objects);

};