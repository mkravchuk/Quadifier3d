#pragma once
#include "ViewerDrawObjects.h"

class Mesh; 
class PolygonMesh;
class PolygonMeshes;


class MeshFile
{
private:
    static bool LoadFromObj(const string& filename, vector<PolygonMesh>& meshes, const vector<string>& loadOnlyGroups, bool separateGroups = true);
    static bool LoadFromStl(const string& filename, vector<PolygonMesh>& meshes);
public:
    static bool LoadFrom(const string& filename, vector<PolygonMesh>& meshes, const vector<string>& loadOnlyGroups, bool separateGroups = true);
    static bool SaveToObj(const string& filename, const PolygonMeshes& polygonMeshes, int vertextesStartIndexShift);
    static bool IsFileTypeSupported(const string& filename);
    static vector<string> SupportedFileExtensions();
};