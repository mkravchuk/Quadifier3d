#include "stdafx.h"
#include "MeshPart.h"
#include "Mesh.h"


MeshPart::MeshPart(const Mesh& _mesh, const int& _partIndex)
    : Index(_partIndex), mesh(_mesh)
{

}

II MeshPart::SizeOF() const
{
    return sizeof(MeshPart)
        + FaceIds.size() * sizeof(int);
}

void MeshPart::GetParts_BaseOnLoops(const Mesh& mesh, vector<MeshPart>& parts)
{
    parts.clear();

    vector<vector<int>> faceIds;
    mesh.GetConnectedFaces_BaseOnLoops(faceIds); // return parts if there will be more than 1 part
    if (faceIds.size() == 0) return; // populate parts only if we have at least 2 parts

    parts.reserve(faceIds.size());
    for (int i = 0; i < faceIds.size(); i++)
    {
        parts.push_back(MeshPart(mesh, i));
        parts.back().FaceIds.swap(faceIds[i]); // move vector 
    }
}

void MeshPart::GetParts_BaseOnEdgeToFaceConnections(const Mesh& mesh, vector<MeshPart>& parts)
{
    vector<vector<int>> mesh_surface_faceIds;
    mesh.GetConnectedFaces_BaseOnEdgeToFaceConnections(mesh_surface_faceIds);

    parts.clear();
    parts.reserve(mesh_surface_faceIds.size());
    for (int i = 0; i < mesh_surface_faceIds.size(); i++)
    {
        parts.push_back(MeshPart(mesh, i));
        parts.back().FaceIds.swap(mesh_surface_faceIds[i]); // move vector 
    }
}

void MeshPart::ExtractMesh(Mesh& newmesh, bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes, bool optimizeVertexIndexesForGPU, bool refreshNonCachableData) const
{
    mesh.ExtractMesh(FaceIds, newmesh, splitNakedTriangleWith2NakedEdge, correctFacesEdgeIndexes, optimizeVertexIndexesForGPU, refreshNonCachableData);
    newmesh.Name = mesh.Name + "." + to_string(Index + 1);
    newmesh.FileName = mesh.FileName + "; part=" + to_string(Index + 1);
}

void MeshPart::ExtractMeshes(vector<Mesh>& meshes, vector<Mesh>& temp_part_meshes, vector<Mesh*>& breaked_meshes)
{
    // ComputeParts for all meshes
    atomic_int parts_count = 0;
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        Mesh& mesh = meshes[i];
        if (mesh.Parts.size() == 0) mesh.ComputeParts_BaseOnLoops(); // compute parts only if they was not computed before
        if (mesh.Parts.size() == 0) // all faces belong to the single part
        {
            parts_count++;
        }
        else
        {
            parts_count += mesh.Parts.size();
        }
    }

    // merge parts from all meshes into one list to be able use multithreading
    vector<pair<MeshPart*, Mesh*>> parts; // part and its original mesh
    parts.clear();
    parts.reserve(parts_count);
    for (auto& mesh : meshes)
    {
        if (mesh.Parts.size() == 0)
        {
            parts.push_back({ nullptr, &mesh }); // mesh has sinlge part
        }
        else
        {
            for (auto& part : mesh.Parts)
            {
                parts.push_back({ &part, &mesh }); // part and its original mesh
            }
        }
    }

    // create ModelObject from MeshPart in multithreading
    temp_part_meshes.resize(parts.size());
    breaked_meshes.resize(parts.size());
    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < parts.size(); i++)
    {
        MeshPart* part = parts[i].first;
        Mesh* partmesh = parts[i].second; // assume mesh has only 1 part, so we will use same mesh
        if (part!= nullptr && part->FaceIds.size() != part->mesh.F.rows()) // if there are few parts in mesh
        {
            part->ExtractMesh(temp_part_meshes[i], false, false, false, false); // extract part to new mesh
            partmesh = &temp_part_meshes[i]; // repoint mesh pointer to newly extracted mesh part
        }
        breaked_meshes[i] = partmesh;
    }
}

