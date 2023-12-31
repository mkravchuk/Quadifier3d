#include "stdafx.h"
#include "OpenGL_Program.h"
#include <string>

OpenGL_Program::OpenGL_Program()
    : program(0)
{

}

OpenGL_Program::~OpenGL_Program()
{
    Delete();
}

bool OpenGL_Program::Create_FromFiles(const string &vertex_shader_filename,
    const string &fragment_shader_filename,
    const string &fragment_data_name,
    const vector<string> feedbackVaryings,
    const string &geometry_shader_filename,
    int geometry_shader_max_vertices)
{
    auto file_to_string = [](const string &filename)->string
    {
        ifstream t(filename);
        return string((istreambuf_iterator<char>(t)),
            istreambuf_iterator<char>());
    };

    return Create(
        file_to_string(vertex_shader_filename),
        file_to_string(fragment_shader_filename),
        fragment_data_name,
        feedbackVaryings,
        file_to_string(geometry_shader_filename),
        geometry_shader_max_vertices
    );
}

bool OpenGL_Program::Create(
    const string &vertex_shader_string,
    const string &fragment_shader_string,
    const string &fragment_data_name,
    const vector<string> feedbackVaryings,
    const string &geometry_shader_string,
    int geometry_shader_max_vertices)
{
    vertex_shader.Create(GL_VERTEX_SHADER, vertex_shader_string);
    geometry_shader.Create(GL_GEOMETRY_SHADER, geometry_shader_string);
    fragment_shader.Create(GL_FRAGMENT_SHADER, fragment_shader_string);

    if (vertex_shader.id == 0)
        return false;

    program = glCreateProgram();

    glAttachShader(program, vertex_shader.id);

    if (fragment_shader.id)
    {
        glAttachShader(program, fragment_shader.id);
    }

    if (geometry_shader.id)
    {
        glAttachShader(program, geometry_shader.id);

        /* This covers only basic cases and may need to be modified */
        glProgramParameteri(program, GL_GEOMETRY_INPUT_TYPE, GL_TRIANGLES);
        glProgramParameteri(program, GL_GEOMETRY_OUTPUT_TYPE, GL_TRIANGLES);
        glProgramParameteri(program, GL_GEOMETRY_VERTICES_OUT, geometry_shader_max_vertices);
    }

    // not needed - fragment shader automatically choise default location as 0, and out data is also limited from 1 float to 4 floats - so we will defined location in shader
    //if (!fragment_data_name.empty())
    //{
    //    glBindFragDataLocation(program, 0, fragment_data_name.c_str());
    //}

    //https://open.gl/feedback
    //http://steps3d.narod.ru/tutorials/tf3-tutorial.html
    if (feedbackVaryings.size() > 0)
    {
        vector<const GLchar*> feedbackVaryings_cstr;
        for (const string& var : feedbackVaryings)
        {
            const GLchar* var_c_str = var.c_str();
            feedbackVaryings_cstr.push_back(var_c_str);
        }
        // v0 - write to multiple buffers - dont works and will be slower from 'v1 writing to single buffer'
        //glTransformFeedbackVaryings(program, feedbackVaryings_cstr.size(), feedbackVaryings_cstr.data(), GL_SEPARATE_ATTRIBS);
        // v1 writing to single buffer
        glTransformFeedbackVaryings(program, feedbackVaryings_cstr.size(), feedbackVaryings_cstr.data(), GL_INTERLEAVED_ATTRIBS);
        // GL_INTERLEAVED_ATTRIBS - varying variables for the vertices of each primitive generated by the GL are written to a SINGLE buffer
        // GL_SEPARATE_ATTRIBS      - varying variables for the vertices of each primitive generated by the GL are written to a MULTIPLE buffers
    }

    glLinkProgram(program);

    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if (status != GL_TRUE)
    {
        char buffer[512];
        glGetProgramInfoLog(program, 512, NULL, buffer);
        cerr << "Linker error: " << endl << buffer << endl;
        program = 0;
        return false;
    }

    glUseProgram(program);
    vao.Create();
    vao.Bind();

    return true;
}

void OpenGL_Program::Use()
{
    glUseProgram(program);
    vao.Bind(); 
    for (int i = 0; i < vao.buffers.size(); i++)
    {
        vao.buffers[i].RefreshBufferIfDirty(vao.buffers__RelationToProgram[i]);
    }
    for (int i = 0; i < vao.buffers_shared.size(); i++)
    {
        vao.buffers_shared[i]->RefreshBufferIfDirty(vao.buffers_shared__RelationToProgram[i]);
    }
}

GLint OpenGL_Program::getAttribId(const string &name)
{
    auto pid = map_name_to_id.find(name);
    if (pid != map_name_to_id.end())
    {
        return pid->second;
    }
    GLint id  = glGetAttribLocation(program, name.c_str());
    if (id == -1)
    {
        std::cerr << "! warning: did not find attrib '" << name << "'" << std::endl;
    }
    map_name_to_id[name] = id;
    return id;
}

GLint OpenGL_Program::getUniformId(const string &name)
{
    auto pid = map_name_to_id.find(name);
    if (pid != map_name_to_id.end())
    {
        return pid->second;
    }
    GLint id = glGetUniformLocation(program, name.c_str());
    if (id == -1)
    {
        std::cerr << "! warning: did not find uniform '" << name <<"'"<< std::endl;
    }
    map_name_to_id[name] = id;
    return id;
}

GLint OpenGL_Program::bindVertexAttribArray(const string &name, GLuint vbo, const MatrixXf &M, bool refresh)
{
    GLint id = getAttribId(name);
    if (id < 0)
    {
        return id;
    }

    if (refresh && M.size() == 0)
    {
        glDisableVertexAttribArray(id);
        return id;
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (refresh)
    {
        //using namespace std;
        //MatrixXf MGL = M.transpose();
        //cout << name << "    ";
        //for (int i = 0; i < 5; i++) cout << MGL.data()[i] << "   ";
        //cout << endl;
        //glBufferData(GL_ARRAY_BUFFER, sizeof(float)*M.size(), M.data(), GL_DYNAMIC_DRAW);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*M.size(), M.data(), GL_STATIC_DRAW); // Mupoc - doesnt change often - so lets use static
    }
    glVertexAttribPointer(id, M.cols(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(id);
    return id;
}

GLint OpenGL_Program::bindVertexAttribArray(const string &name, GLuint vbo, const MatrixXi &M, bool refresh) 
{
    GLint id = getAttribId(name);
    if (id < 0)
    {
        return id;
    }

    if (refresh && M.size() == 0)
    {
        glDisableVertexAttribArray(id);
        return id;
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (refresh)
    {
        //using namespace std;
        //MatrixXf MGL = M.transpose();
        //cout << name << "    ";
        //for (int i = 0; i < 5; i++) cout << MGL.data()[i] << "   ";
        //cout << endl;
        //glBufferData(GL_ARRAY_BUFFER, sizeof(float)*M.size(), M.data(), GL_DYNAMIC_DRAW);
        glBufferData(GL_ARRAY_BUFFER, sizeof(int)*M.size(), M.data(), GL_STATIC_DRAW); // Mupoc - doesnt change often - so lets use static
    }
    glVertexAttribPointer(id, M.cols(), GL_INT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(id);
    return id;
}

void OpenGL_Program::AddBuffer(const string& name, const MatrixXf& Buffer, uint32_t& dirtyFlags, uint32_t dirtyFlag)
{
    vao.buffers.push_back(OpenGL_VertexBufferObject(name, Buffer, dirtyFlags, dirtyFlag));
    vao.buffers.back().Create();

    glUseProgram(program);
    vao.Bind();
    vao.buffers__RelationToProgram.push_back({ false, glGetAttribLocation(program, name.c_str()) });
}

void OpenGL_Program::AddSharedBuffer(const string& name, OpenGL_VertexBufferObject* vbo_shared)
{
    vao.buffers_shared.push_back(vbo_shared);
    vao.buffers_shared.back()->Create();

    glUseProgram(program);
    vao.Bind();
    vao.buffers_shared__RelationToProgram.push_back({ false, glGetAttribLocation(program, name.c_str()) });
}

void OpenGL_Program::Delete()
{
    map_name_to_id.clear();
    if (program)
    {
        glDeleteProgram(program);
        program = 0;
    }
    vertex_shader.Delete();
    geometry_shader.Delete();
    fragment_shader.Delete();
    vao.Delete();
}

