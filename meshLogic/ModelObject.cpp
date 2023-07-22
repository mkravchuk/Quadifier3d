#include "stdafx.h"
#include "ModelObject.h"
#include "Model.h"
#include "ModelFile.h"
#include "MeshSurface.h"

std::atomic_int ModelObject_nextId = 1;


//ModelObject::ModelObject(const Model& _model, ModelFile& _file, string _name, const MeshPart& part)
//    : readonly_Id(ModelObject_nextId++), 
//      Id(readonly_Id), Name(_name), model(_model), file(_file), IsActive(true), 
//      srf(MeshSurface(_file, part)), OpenGLDataDrawer(nullptr), SimplifiedMesh(nullptr)
//{
//
//}

ModelObject::ModelObject(const Model& _model, ModelFile& _file, string _name, Mesh& mesh_moved)
    : readonly_Id(ModelObject_nextId++),
    Id(readonly_Id), Name(_name), model(_model), file(_file), IsActive(true),
    srf(MeshSurface(_file, mesh_moved)), SimplifiedMesh(nullptr)
{
    if (empty(Name))
    {
        Name = "Obj" + to_string(Id);
    }
}

II ModelObject::SizeOF() const
{
    return srf.SizeOF();
}

