#pragma once

class Topology;
class Mesh;
class MeshSolverNrosy;
class MeshSolverUV;
class MeshSurface;
class ViewerDrawObjects;


class MeshLogicDraw
{
public:
    const Mesh& mesh;
    ViewerDrawObjects& draw; 

    MeshLogicDraw(MeshSurface& srf);
    MeshLogicDraw(const Mesh& mesh, ViewerDrawObjects& draw);

    void DrawAll();

    void DrawObject(int id, string name);
    void draw_mesh_nrosy(const MeshSolverNrosy& solver, bool logMessages);
    void draw_mesh_UV(const MeshSolverUV& solverUV, bool logMessages);

    void DrawId_Vertex();
    void DrawId_Edge();
    void DrawId_Face();
    void Draw_Object_Id(int id);
    void Draw_Object_Name(string name);
    void DrawId_Mesh();
    void DrawId_MeshLoop();

    void DrawNormals(bool faces, bool vertixes, bool faces_calculated);

    void DrawBorders();
    void DrawBorderIndexes();
    void DrawBorderLabels();
    void DrawBorderSharpness();
    void DrawBorderAngles();
    void DrawHighlights();
private:    void Draw_BorderConstrainesIsoLines(const MeshSolverNrosy& solver, int borderConstrainesIsoLinesCount, int borderConstrainesIsoLinesCount_intensivity, bool drawLines, bool logMessages);
    void Draw_BorderConstrainesIsoLinesUV(const MeshSolverUV& solverUV, int borderConstrainesIsoLinesCount, int borderConstrainesIsoLinesCount_intensivity, bool drawLines, bool logMessages);
};



