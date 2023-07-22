#include "stdafx.h"
#include "MeshSurface.h"
#include "Mesh.h"
#include "ModelFile.h"
#include "MeshLoop.h"

//MeshSurface::MeshSurface(ModelFile& _file, const MeshPart& part)
//    : ID(readonly_id), mesh(original_mesh), file(_file), solver(MeshSolverNrosy(original_mesh, draw))
//{
//    part.ExtractMesh(original_mesh, false, false); // 2 last parameters is false since we create exactly what we take without any cosmetic modifications. make all modifications for mesh before this constructor.
//    readonly_id = _file.GetNextSrfUniqueId();
//}

MeshSurface::MeshSurface(ModelFile& _file, Mesh& _mesh_moved)
    : /*original_mesh(_mesh), - instead lets do swap */ Id(readonly_id), mesh(original_mesh), file(_file), solver(MeshSolverNrosy(original_mesh, draw)), solverUV(MeshSolverUV(original_mesh, draw))
{
    swap(original_mesh, _mesh_moved); // swap meshes, since much faster from copy
    readonly_id = _file.GetNextSrfUniqueId();
    original_mesh.Loops.clear();// we must clear Loos, since Loops has reference to old mesh, and references are not copied properly to original_mesh
    original_mesh.Parts.clear();// we must clear parts, since Parts has reference to old mesh, and references are not copied properly to original_mesh
    assert(original_mesh.E_Length.size() == 0 && "RefreshNonCachableData should be called only once to avoid performance penalty");
    original_mesh.RefreshNonCachableData(false);

    //DEBUG show surface info
    //cout << "surface #" << "?" << "   facesCount = " << _mesh.F.rows() << "   loopsCount = " << _mesh.Loops.size() << "";
    //for (auto loop : _mesh.Loops)
    //{
    //    cout << (loop.Type == MeshLoopType::Outher ? ", outher" : ", inner");
    //}
    //cout << endl;
}


void MeshSurface::InitMesh_VcenterProjectedIndside()
{
    original_mesh.V_Info.Init_VcenterProjectedIndside(original_mesh);
}

void MeshSurface::InitMesh_VcenterProjectedIndside_IfRequiredByDraw()
{
    original_mesh.V_Info.InitMesh_VcenterProjectedIndside_IfRequiredByDraw(original_mesh);
}

II MeshSurface::SizeOF() const
{
    II r = sizeof(MeshSurface);
    r += mesh.SizeOF();
    r += solver.SizeOF();
    r += solverUV.SizeOF();
    r += draw.SizeOF();
    return r;
}

void MeshSurface::InitNormalCorrections(D creaseAngle)
{
    if (NormalCorrections.isInited) return;
    original_mesh.GetMeshNormalCorrections(NormalCorrections, creaseAngle);
}











