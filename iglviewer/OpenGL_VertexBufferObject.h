#pragma once
#include "OpenGL_Shared.h"
#include "OpenGL_Options.h"


//Vertex Buffer Object,  for help http://wiki.lwjgl.org/wiki/The_Quad_with_DrawArrays.html
class OpenGL_VertexBufferObject
{
private:
    bool isCreated;
    int RefreshedForBufferSize;
public:
    struct RelationToProgram
    {
        bool isEnabled;
        GLint name_attr_id;
    };
    GLuint id; // vbo id
    string name; // vbo name
    const MatrixXf& Buffer; // vbo
    uint32_t& dirtyFlags; // all dirty flags
    uint32_t dirtyFlag;    // dirty flag for current vbo

    OpenGL_VertexBufferObject(const std::string &name, const MatrixXf& Buffer, uint32_t& dirtyFlags, uint32_t dirtyFlag);
    void Create();
    void Delete();
    bool isDirty();
    void RefreshBufferIfDirty(RelationToProgram& relationToProgram);
};

