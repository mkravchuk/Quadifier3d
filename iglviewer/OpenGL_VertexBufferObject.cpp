#include "stdafx.h"
#include "OpenGL_VertexBufferObject.h"

/*
A buffer object in OpenGL is a big blob of bits.Think of the "active" buffer as just a global variable,
and there are a bunch of functions which use the active buffer instead of using a parameter.
These global state variables are the ugly side of OpenGL(prior to direct state access, which is covered below).

GLuint buffer;

// Generate a name for a new buffer.
// e.g. buffer = 2
glGenBuffers(1, &buffer);

// Make the new buffer active, creating it if necessary.
// Kind of like:
// if (opengl->buffers[buffer] == null)
//     opengl->buffers[buffer] = new Buffer()
// opengl->current_array_buffer = opengl->buffers[buffer]
glBindBuffer(GL_ARRAY_BUFFER, buffer);

// Upload a bunch of data into the active array buffer
// Kind of like:
// opengl->current_array_buffer->data = new byte[sizeof(points)]
// memcpy(opengl->current_array_buffer->data, points, sizeof(points))
glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);

Now, your typical vertex shader takes vertexes as input, not a big blob of bits.So you need to specify how the blob of bits(the buffer) is decoded into vertexes.That is the job of the array.Likewise, there is an "active" array which you can think of as just a global variable :

GLuint array;
// Generate a name for a new array.
glGenVertexArrays(1, &array);
// Make the new array active, creating it if necessary.
glBindVertexArray(array);

// Make the buffer the active array buffer.
glBindBuffer(GL_ARRAY_BUFFER, buffer);
// Attach the active buffer to the active array,
// as an array of vectors with 4 floats each.
// Kind of like:
// opengl->current_vertex_array->attributes[attr] = {
//     type = GL_FLOAT,
//     size = 4,
//     data = opengl->current_array_buffer
// }
glVertexAttribPointer(attr, 4, GL_FLOAT, GL_FALSE, 0, 0);
// Enable the vertex attribute
glEnableVertexAttribArray(attr);

*/

OpenGL_VertexBufferObject::OpenGL_VertexBufferObject(const std::string &_name, const MatrixXf& _Buffer, uint32_t& _dirtyFlags, uint32_t _dirtyFlag)
    : isCreated(false), RefreshedForBufferSize(-1), id(0), name(_name), Buffer(_Buffer), dirtyFlags(_dirtyFlags), dirtyFlag(_dirtyFlag)
{
}


void OpenGL_VertexBufferObject::Create()
{
    if (!isCreated)
    {
        glGenBuffers(1, &id);
        isCreated = true;
    }
}

void OpenGL_VertexBufferObject::Delete()
{
    if (isCreated)
    {
        glDeleteBuffers(1, &id);
        id = 0;
        isCreated = false;
    }
}

bool OpenGL_VertexBufferObject::isDirty()
{
    return dirtyFlags & dirtyFlag;
}

void OpenGL_VertexBufferObject::RefreshBufferIfDirty(RelationToProgram& p)
{
    if (p.isEnabled 
        && RefreshedForBufferSize == Buffer.size()
        && !isDirty())
    {
        return;
    }

    if (p.name_attr_id < 0)
    {
        return;
    }
    RefreshedForBufferSize = Buffer.size();

    glBindBuffer(GL_ARRAY_BUFFER, id);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*Buffer.size(), Buffer.data(), GL_STATIC_DRAW);
    //glBufferStorage(GL_ARRAY_BUFFER, sizeof(float)*Buffer.size(), Buffer.data(), 0); //doenst help at all
    glVertexAttribPointer(p.name_attr_id, Buffer.cols(), GL_FLOAT, GL_FALSE, 0, 0);

    if (Buffer.size() == 0)
    {
        if (p.isEnabled)
        {
            glDisableVertexAttribArray(p.name_attr_id);
            p.isEnabled = false;
        }
    }
    else
    {
        if (!p.isEnabled)
        {
            glEnableVertexAttribArray(p.name_attr_id);
            p.isEnabled = true;
        }
    }

    //dirtyFlags &= ~dirtyFlag;
}
