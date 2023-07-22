#include "stdafx.h"
#include "OpenGL_VertexArrayObject.h"
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

OpenGL_VertexArrayObject::OpenGL_VertexArrayObject()
    : isCreated(false), id(0)
{
}


void OpenGL_VertexArrayObject::Create()
{
    if (!isCreated)
    {
        glGenVertexArrays(1, &id);
        isCreated = true;
    }
}

void OpenGL_VertexArrayObject::Delete()
{
    if (isCreated)
    {
        glDeleteVertexArrays(1, &id);
        id = 0;
        for (auto& buffer : buffers)
        {
            buffer.Delete();
        }
        buffers.clear();
        buffers__RelationToProgram.clear();
        buffers_shared.clear();
        buffers_shared__RelationToProgram.clear();
        isCreated = false;
    }
}

void OpenGL_VertexArrayObject::Bind()
{
    glBindVertexArray(id);

    /*
    In 4.3, or if you have the ARB_vertex_attrib_binding extension, you can specify the attribute format and the attribute data separately.
    This is nice because it lets you easily switch one vertex array between different buffers.

    GLuint array;
    // Generate a name for a new array array.
    glGenVertexArrays(1, &array);
    // Make the new array active, creating it if necessary.
    glBindVertexArray(array);

    // Enable my attributes
    glEnableVertexAttribArray(loc_attrib);
    glEnableVertexAttribArray(normal_attrib);
    glEnableVertexAttribArray(texcoord_attrib);
    // Set up the formats for my attributes
    glVertexAttribFormat(loc_attrib, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexAttribFormat(normal_attrib, 3, GL_FLOAT, GL_FALSE, 12);
    glVertexAttribFormat(texcoord_attrib, 2, GL_FLOAT, GL_FALSE, 24);
    // Make my attributes all use binding 0
    glVertexAttribBinding(loc_attrib, 0);
    glVertexAttribBinding(normal_attrib, 0);
    glVertexAttribBinding(texcoord_attrib, 0);

    // Quickly bind all attributes to use "buffer"
    // This replaces several calls to glVertexAttribPointer()
    // Note: you don't need to bind the buffer first!  Nice!
    glBindVertexBuffer(0, buffer, 0, 32);

    // Quickly bind all attributes to use "buffer2"
    glBindVertexBuffer(0, buffer2, 0, 32);
    */
}
