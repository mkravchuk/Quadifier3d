#pragma once
#include "stdafx.h"
typedef unsigned int GLuint;
typedef int GLint;


namespace gl
{
    bool isEnabled(GLenum flag);
    void Enable(GLenum flag);
    void Disable(GLenum flag);

    void DrawColorBuffer(GLuint colorBufferID, bool blend);    
    void set_layers_ColorBuffer_1(GLuint id);
    GLuint get_layers_ColorBuffer_1();
}