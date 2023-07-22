#include "stdafx.h"
#include "OpenGL_Shader.h"
#include "OpenGL_Options.h"


OpenGL_Shader::OpenGL_Shader()
    :id(0), type(GL_VERTEX_SHADER), text("")
{
}

void OpenGL_Shader::Create(GLint _type, string _text)
{
    Delete();
    type = _type;
    text = _text;
    id = CreateShader(type, text);
}

void OpenGL_Shader::Delete()
{
    if (id)
    {
        glDeleteShader(id);
        id = 0;
        text = "";
    }
}


GLuint OpenGL_Shader::CreateShader(GLint type, const std::string &shader_string)
{
    if (shader_string.empty())
        return (GLuint)0;

    string version = "#version " + openglOptions.WindowHint.slhl_version + "\n";
    string shader_string_with_version = version + shader_string;
    const char *shader_string_const = shader_string_with_version.c_str();
    return CreateShader(type, shader_string_const);
}

GLuint OpenGL_Shader::CreateShader(GLint type, const char* shader_string)
{
    if (shader_string == nullptr)
        return (GLuint)0;

    GLuint id = glCreateShader(type);
    glShaderSource(id, 1, &shader_string, NULL);
    glCompileShader(id);

    GLint status;
    glGetShaderiv(id, GL_COMPILE_STATUS, &status);

    if (status != GL_TRUE)
    {
        char buffer[512];
        if (type == GL_VERTEX_SHADER)
            cerr << "Vertex shader:" << endl;
        else if (type == GL_FRAGMENT_SHADER)
            cerr << "Fragment shader:" << endl;
        else if (type == GL_GEOMETRY_SHADER)
            cerr << "Geometry shader:" << endl;
        cerr << shader_string << endl << endl;
        glGetShaderInfoLog(id, 512, NULL, buffer);
        cerr << "Error: " << endl << buffer << endl;
        return (GLuint)0;
    }

    return id;
}