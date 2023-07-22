#pragma once
#include "OpenGL_Shared.h"
#include "OpenGL_Options.h"
#include "OpenGL_VertexBufferObject.h"

//Vertex Array Object,  for help http://wiki.lwjgl.org/wiki/The_Quad_with_DrawArrays.html
class OpenGL_VertexArrayObject
{
private:
    bool isCreated;
public:
    GLuint id; // vao id
    vector<OpenGL_VertexBufferObject> buffers;
    vector<OpenGL_VertexBufferObject*> buffers_shared;
    vector <OpenGL_VertexBufferObject::RelationToProgram> buffers__RelationToProgram;
    vector <OpenGL_VertexBufferObject::RelationToProgram> buffers_shared__RelationToProgram;

    OpenGL_VertexArrayObject();
    void Create();
    void Delete();
    void Bind();
};

