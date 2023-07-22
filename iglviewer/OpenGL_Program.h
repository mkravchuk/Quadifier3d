#pragma once
#include "OpenGL_VertexArrayObject.h"
#include "OpenGL_Shader.h"

class OpenGL_VertexBufferObject;
//#include <igl/igl_inline.h>


// This class wraps an OpenGL program composed of three shaders
class OpenGL_Program
{
public:
    map<string, GLint> map_name_to_id;
    GLuint program;
    OpenGL_Shader vertex_shader;
    OpenGL_Shader geometry_shader;
    OpenGL_Shader fragment_shader;
    OpenGL_VertexArrayObject vao;

    OpenGL_Program();
    ~OpenGL_Program();


    // Create a new program from the specified shaders source strings
    bool Create(const string &vertex_shader_string,
        const string &fragment_shader_string,
        const string &fragment_data_name,
        const vector<string> feedbackVaryings,
        const string &geometry_shader_string = "",
        int geometry_shader_max_vertices = 3);

    // Create a new program from the specified files on disk
    bool Create_FromFiles(const string &vertex_shader_filename,
        const string &fragment_shader_filename,
        const string &fragment_data_name,
        const vector<string> feedbackVaryings,
        const string &geometry_shader_filename = "",
        int geometry_shader_max_vertices = 3);

    // Select this shader for subsequent draw calls
    void Use();

    // Release all OpenGL objects
    void Delete();

    // Return the OpenGL handle of a named shader attribute (-1 if it does not exist)
    GLint getAttribId(const string &name);

    // Return the OpenGL handle of a uniform attribute (-1 if it does not exist)
    GLint getUniformId(const string &name);

    // Bind a per-vertex array attribute and refresh its contents from an Eigen matrix
    GLint bindVertexAttribArray(const string &name, GLuint vbo, const MatrixXf &M, bool refresh);
    GLint bindVertexAttribArray(const string &name, GLuint vbo, const MatrixXi &M, bool refresh);

    void AddBuffer(const string &name, const MatrixXf& Buffer, uint32_t& dirtyFlags, uint32_t dirtyFlag);
    void AddSharedBuffer(const string& name, OpenGL_VertexBufferObject* vbo_shared);
};


