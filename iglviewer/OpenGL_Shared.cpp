#include "stdafx.h"
#include "OpenGL_Shared.h"
#include "OpenGL_Program.h"

map<GLenum, bool> map_flag_isEnabled;

namespace gl
{
    bool& getFlagValue(GLenum flag, bool& wasDefined)
    {
        wasDefined = true;
        auto pid = map_flag_isEnabled.find(flag);
        if (pid == map_flag_isEnabled.end())
        {
            wasDefined = false;
            map_flag_isEnabled[flag] = false;
            pid = map_flag_isEnabled.find(flag);
        }
        return pid->second;
    }

    bool isEnabled(GLenum flag)
    {
        bool wasDefined;
        return getFlagValue(flag, wasDefined);
    }

    void Enable(GLenum flag)
    {
        bool wasDefined;
        auto& val = getFlagValue(flag, wasDefined);
        if (!val || !wasDefined)
        {
            val = true;
            glEnable(flag);
        }
    }

    void Disable(GLenum flag)
    {
        bool wasDefined;
        auto& val = getFlagValue(flag, wasDefined);
        if (val || !wasDefined)
        {
            val = false;
            glDisable(flag);
        }
    }

    OpenGL_Program program_draw_texture;
    bool is_program_draw_texture_inited = false;
    bool init_program_draw_texture()
    {
        if (is_program_draw_texture_inited)
        {
            return true;
        }

        //
    // Create shader for blend texture with main colorbuffer
    //

        string vertexShader = R"(
        const vec2 Position[4] = vec2[]
        (
            vec2(-1,  1),
            vec2(-1, -1),
            vec2( 1,  1),
            vec2( 1, -1) 
        );
        uniform float offset;
        out vec2 texcoord;
        void main()
        {
            vec2 pos = Position[ gl_VertexID ];
            pos.x += offset * -sign(pos.x);
            gl_Position = vec4(pos, 0.0, 1.0);
            vec2 texcoord_flipped = pos * vec2(0.5, -0.5) + 0.5;
            texcoord = vec2(texcoord_flipped.x, -texcoord_flipped.y);
        })";


        string fragmentShader = R"(
        uniform sampler2D texUnit;
        in vec2 texcoord;
        layout(location = 0) out vec4 fragColor;
        void main()
        {
            vec4 color = texture(texUnit, texcoord);
            if (color.a == 0)  //speed optimization - since most of the colors are transparent - skip them
                discard; 
            fragColor = color;
        })";



        /*  BLEND 2 TEXTURES

        gl::Enable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        The rest is easiest done in a fragment shader:

        #version 330

    uniform sampler2D texture1;
    uniform sampler2D texture2;

    // can also be uniform
    const float partOf1 = 0.5f;

    in vec2 texCoord;
    out vec4 color;

    void main()
    {
        color = texture(texture1, texCoord) * vec4(1.0f, 1.0f, 1.0f, partOf1) + texture(texture2, texCoord) * vec4(1.0f, 1.0f, 1.0f, 1.0f - partOf1);
    }

            */


            //Vertex shader (assumes a quad is drawn from (-1,-1) to (1,1) ) 
            //  with it lower-left at (-1, -1, 0)
            //  and its top-right at (1, 1, 0)
            // Note that I use texelFetch() here because linearly interpolating depth values does not give valid results.
            //string vertexShader =
            //    "#version 330 core\n"
            //    "in int width;\n"
            //    "in int height;\n"
            //    "out vec2 texcoord;\n"
            //    "void main()\n"
            //    "{\n"
            //    "    int y = gl_VertexID >> 1;\n"   // gl_VertexID div 2, each line has 2 points
            //    "    float x = -1*(0+(gl_VertexID | 1)*-1);\n" // -1 if even and 1 if odd
            //    "    float y = offset * -sign(pos.x);\n"
            //    "    gl_Position = vec4(x, y, 0.0, 1.0);\n"
            //    "    texcoord =vec2(lineIndex, lineIndex) + 0.5;\n"
            //    "}\n"
            //    ;

            //string vertexShader =
            //    "#version 330 core\n"
            //    "in int width;\n"
            //    "in int height;\n"
            //    "out ivec2 textureXY;\n"
            //    "void main()\n"
            //    "{\n"
            //    "    int ty = gl_VertexID / width;\n"  
            //    "    int tx = gl_VertexID - ty*width;\n"
            //    "    float x2 = width/2;\n"
            //    "    float x = (tx-x2)/x2;\n"
            //    "    float y2 = height/2;\n"
            //    "    float y = (ty-y2)/y2;\n"
            //    "    gl_Position = vec4(x, y, 0.0, 1.0);\n"
            //    "    textureXY = ivec2(tx, ty);\n"
            //    "}\n"
            //    ;
            ////// Fragment shader gets output color from a texture
            //string fragmentShader =
            //    "#version 330 core\n"
            //    "uniform sampler2D texUnit;\n"
            //    "in ivec2 textureXY;\n"
            //    "layout(location = 0) out vec4 fragColor;\n"
            //    "void main()\n"
            //    "{\n"
            //    "    fragColor = texelFetch(texUnit, textureXY, 0);\n"
            //    "}\n"
            //    ;

        program_draw_texture.Create(vertexShader, fragmentShader, "fragColor", {});
        //program_draw_texture.Add_VertexBufferObject("normal", V_normals_vbo, dirty, ViewportData::DIRTY_NORMAL);
        //program_draw_texture.bindVertexAttribArray("vertexCacheColor", tbo_VertexShader_POINTS.tbo, feedbackBufferOnCPU, false);
        int texUnit = 0;
        program_draw_texture.Use();
        GLint offsetI = glGetUniformLocation(program_draw_texture.program, "offset");
        GLint texUnitI = glGetUniformLocation(program_draw_texture.program, "texUnit");
        //GLint pixelsCountI = glGetUniformLocation(program_draw_texture.program, "pixelsCount");
        //GLint widthI = glGetUniformLocation(program_draw_texture.program, "width");
        //GLint heightI = glGetUniformLocation(program_draw_texture.program, "height");
        glUniform1i(texUnitI, texUnit);
        glUniform1f(offsetI, 0.f);
        //glUniform1i(pixelsCountI, width*height);
        //glUniform1i(widthI, width);
        //glUniform1i(heightI, height);

        is_program_draw_texture_inited = true;
        return true;
    };

    void DrawColorBuffer(GLuint colorBufferID, bool blend)
    {
        if (!init_program_draw_texture())
        {
            cout << "! warning:  cannot draw texture since shader is not inited" << endl;
            return;
        }

        //glActiveTexture(GL_TEXTURE0 + texUnit); //since texUnit=0 - this is default value
        glBindTexture(GL_TEXTURE_2D, colorBufferID);

        if (blend)
        {
            gl::Enable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        else
        {
            gl::Disable(GL_BLEND);
        }
        //glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ZERO);
        gl::Disable(GL_DEPTH_TEST);
        gl::Disable(GL_STENCIL_TEST);
        //glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ZERO);
        program_draw_texture.Use();
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        //glDrawArrays(GL_QUADS, 0, 4);
        //glDrawArrays(GL_POINT, 0, pixelsCountI);
        //glDrawArrays(GL_LINE, 0, height * 2);//lines=height, and for each line will be 2 points
        //glDrawArrays(GL_POINTS, 0, width*height);
    }

    GLuint layers_ColorBuffer_1 = 0;
    void set_layers_ColorBuffer_1(GLuint id)
    {
        layers_ColorBuffer_1 = id;
    }
    GLuint get_layers_ColorBuffer_1()
    {
        return layers_ColorBuffer_1;
    }
}