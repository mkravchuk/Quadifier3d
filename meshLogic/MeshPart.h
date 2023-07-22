#pragma once

class Mesh;

class MeshPart
{
public:
    int Index; //part index in mesh.Parts
    const Mesh& mesh; // mesh of this surface - subset of owner mesh. Surface can have only 1 outher loop (can have 0 outher loop). Outher loop come first at index 0
    vector<int> FaceIds;

    MeshPart(const Mesh& _mesh, const int& _partIndex);
    II SizeOF() const;
    static void GetParts_BaseOnLoops(const Mesh& mesh, vector<MeshPart>& parts);
    static void GetParts_BaseOnEdgeToFaceConnections(const Mesh& mesh, vector<MeshPart>& parts);

    void ExtractMesh(Mesh& newmesh, bool splitNakedTriangleWith2NakedEdge, bool correctFacesEdgeIndexes, bool optimizeVertexIndexesForGPU, bool refreshNonCachableData) const;
    static void ExtractMeshes(vector<Mesh>& meshes, vector<Mesh>& temp_part_meshes, vector<Mesh*>& breaked_meshes);
private:
};