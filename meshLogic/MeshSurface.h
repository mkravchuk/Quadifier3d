#pragma once
#include "Mesh.h"
#include "MeshSolverNrosy.h"
#include "MeshSolverUV.h"
#include "ViewerDrawObjects.h"

class Mesh;
class ModelFile;

// Holds:
// - surface mesh, 
// - solver for mesh-quadifiyng
// - draw for drawing debug data
// 'mesh_moved' will be moved to class - after constructor reference variable 'mesh_moved' will be empty
class MeshSurface
{ 
    Mesh original_mesh;
    int readonly_id;
public:
    const int& Id; // surface Id in file. Id must be unique in file, but is not unique in model.
    const Mesh& mesh; // mesh of this surface - subset of owner mesh. Surface can have only 1 outher loop (can have 0 outher loop). Outher loop come first at index 0
    const ModelFile& file; // file to wich belongs this surface
    MeshSolverNrosy solver; // solver for this surface-mesh
    MeshSolverUV solverUV; // solver for this surface-mesh
    ViewerDrawObjects draw;
    MeshNormalCorrections NormalCorrections;

    //MeshSurface(ModelFile& file, const MeshPart& part);
    MeshSurface(ModelFile& file, Mesh& mesh_moved); // 'mesh_moved' will be moved to class - after this constructor reference variable 'mesh_moved' will be empty
    void InitMesh_VcenterProjectedIndside();
    void InitMesh_VcenterProjectedIndside_IfRequiredByDraw();
    II SizeOF() const;
    void InitNormalCorrections(D creaseAngle);
private:
};
