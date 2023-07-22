#pragma once
#include "MeshSurface.h"

class Model;
class ModelFile;

// Holds
// - surface
// - opengl for drawing it (if all surfaces drawing separatelly - controlled in viewer options)
//
// should be created as pointer, because this class will be in the changable list 
// 'mesh_moved' will be moved to class - after constructor reference variable 'mesh_moved' will be empty
class ModelObject
{
private:
    int readonly_Id;
public:
    const int& Id; // unique Id in model
    string Name;
    const Model& model;
    const ModelFile& file; //to wich file belongs this object. in model can be added few files - this is good for caching - to make fast loading previously loaded file - we just change the filter to show file to user
    bool IsActive; // if object is active - in same file some object are active and visible and some are inactive and unvisible. Active objects solved, inactive stay untached.

    MeshSurface srf;
    void* SimplifiedMesh; // simplified mesh cache - used for passing it to opengl

    //ModelObject(const Model& model, ModelFile& file, string name, const MeshPart& part);
    ModelObject(const Model& model, ModelFile& file, string name, Mesh& mesh_moved); // mesh will be moved to class - after this constructor mesh will be empty
    II SizeOF() const;
private:
};
