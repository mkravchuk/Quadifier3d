#pragma once
#include "OpenGL_Shared.h"


// This class wraps an OpenGL shader
class OpenGL_Shader
{
private:
public:
    GLuint id;
    GLint type;
    string text;
    OpenGL_Shader();
    void Create(GLint type, string text);
    void Delete();
private:
    static GLuint CreateShader(GLint type, const std::string &shader_string);
    static GLuint CreateShader(GLint type, const char* shader_string);
};