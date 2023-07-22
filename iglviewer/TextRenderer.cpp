#include "stdafx.h"

#include "TextRenderer.h"
#include "TextRenderer_fonts.h"
#include <igl/project.h>

#include <nanogui/opengl.h>
#include <nanovg.h>

#define NANOVG_GL3
#include <nanovg_gl.h>

TextRenderer::TextRenderer()
    : ctx(nullptr), timer_BeginDraw_EndDraw__drawCount(0)
{

}

int TextRenderer::Init()
{
    #ifdef NDEBUG
    ctx = nvgCreateGL3(NVG_STENCIL_STROKES | NVG_ANTIALIAS);
    #else
    ctx = nvgCreateGL3(NVG_STENCIL_STROKES | NVG_ANTIALIAS | NVG_DEBUG);
    #endif

    nvgCreateFontMem(ctx, "sans", igl_roboto_regular_ttf,
        igl_roboto_regular_ttf_size, 0);

    return 0;
}

int TextRenderer::Shut()
{
    if (ctx)
        nvgDeleteGL3(ctx);
    return 0;
}

void TextRenderer::BeginDraw(const Matrix4d &view, const Matrix4d &model, const Matrix4d &proj, const Vector4i &_viewportSize, double _object_scale)
{
    timer_BeginDraw_EndDraw.restart();
    timer_BeginDraw_EndDraw__drawCount = 0;
    viewportSize = _viewportSize.cast<double>();
    proj_matrix = proj;
    view_matrix = view*model;
    object_scale = _object_scale;
    BeginDraw2d();
}

void TextRenderer::BeginDraw2d()
{
    Vector2i mFBSize;
    Vector2i mSize;

    GLFWwindow* mGLFWWindow = glfwGetCurrentContext();
    glfwGetFramebufferSize(mGLFWWindow, &mFBSize[0], &mFBSize[1]);
    glfwGetWindowSize(mGLFWWindow, &mSize[0], &mSize[1]);
    glViewport(0, 0, mFBSize[0], mFBSize[1]);

    //glClear(GL_STENCIL_BUFFER_BIT); - we do this in  "void OpenGL_Window::clear_framebuffers()"

    /* Calculate pixel ratio for hi-dpi devices. */
    mPixelRatio = static_cast<double>(mFBSize[0]) / static_cast<double>(mSize[0]);
    nvgBeginFrame(ctx, mSize[0], mSize[1], static_cast<float>(mPixelRatio));
}

void TextRenderer::EndDraw()
{
    nvgEndFrame(ctx);
    timer_BeginDraw_EndDraw.stop();
    if (timer_BeginDraw_EndDraw__drawCount > 0)
    {
        //cout << endl << "TextRenderer draw time:" << timer_BeginDraw_EndDraw << "      drawCount="<< timer_BeginDraw_EndDraw__drawCount << endl;
    }
}

V3 project(
    const    V3&  obj,
    const    Matrix4d& model,
    const    Matrix4d& proj,
    const    Vector4d&  viewport)
{
    Vector4d tmp(obj(0), obj(1), obj(2), 1);

    tmp = model * tmp;

    tmp = proj * tmp;

    cout << "tmp(3)=" << tmp(3) << endl;
    tmp = tmp.array() / tmp(3);
    tmp = tmp.array() * 0.5f + 0.5f;
    tmp(0) = tmp(0) * viewport(2) + viewport(0);
    tmp(1) = tmp(1) * viewport(3) + viewport(1);

    return V3(tmp(0), tmp(1), tmp(2));
}

void TextRenderer::DrawText(const P3& pos, const V3& normal, const string &text, double transparent_alpha, const Vector4d& color, int fontSizeRelativeIncr)
{
    //Vector3f posNormalized = pos.cast<float>() + normal.cast<float>() * 0.005f * object_scale;
    //Vector3f coord = igl::project(posNormalized, view_matrix, proj_matrix, viewportSize);
    //cout << "Draw text '" << text << "'" << " at {" << coord[0] << ", " << viewportSize[3] - coord[1] <<"}"<< endl;
    //DrawText2d(coord[0], viewportSize[3] - coord[1], text, transparent_alpha, color, fontSizeRelativeIncr);
    auto coutM = [](string name, Matrix4d m)
    {
        cout << "  " << name << endl;
        for (int y = 0; y < 4; y++)
        {
            cout << "           ";
            for (int x = 0; x < 4; x++)
                cout << m(x, y) << "  ";
            cout << endl;
        }
    };
    auto coutV3 = [](string name, V3 v)
    {
        cout << "  " << name << endl;
        cout << "           ";
        for (int x = 0; x < 3; x++)
            cout << v(x) << "  ";
        cout << endl;
    };
    auto coutV4 = [](string name, Vector4d v)
    {
        cout << "  " << name << endl;
        cout << "           ";
        for (int x = 0; x < 4; x++)
            cout << v(x) << "  ";
        cout << endl;
    };
    P3 posNormalized = pos + normal * 0.005f * (float)object_scale;
    Point3d coord = igl::project(convertP3ToEigenDouble(posNormalized), view_matrix, proj_matrix, viewportSize);
    //if (text == "10" || text == "39")
    //{
    //    coord = project(posNormalized, view_matrix, proj_matrix, viewportSize);
    //    cout << endl << endl << "Draw text '" << text << "'" << " at {" << coord[0] << ", " << viewportSize[3] - coord[1] << "}    scale=" << object_scale << endl;
    //    //coutV3("normal", normal);
    //    //coutV3("posNormalized", posNormalized);
    //    coutV3("coord", coord);
    //    coutM("view*model", view_matrix);
    //    //coutM("proj_matrix", proj_matrix);
    //    //coutV4("viewportSize", viewportSize);
    //}
    DrawText2d(coord[0], viewportSize[3] - coord[1], text, transparent_alpha, color, fontSizeRelativeIncr);

}
void TextRenderer::DrawText2d(double x, double y, const string &text, double transparent_alpha, const Vector4d& color, int fontSizeRelativeIncr)
{
    timer_BeginDraw_EndDraw__drawCount++;
    nvgFontSize(ctx, static_cast<float>((16 + fontSizeRelativeIncr) * mPixelRatio));
    nvgFontFace(ctx, "sans");
    nvgTextAlign(ctx, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    Vector4d c = Vector4d(255 * color(0), 255 * color(1), 255 * color(2), 255 * color(3)*transparent_alpha);
    nvgFillColor(ctx, nvgRGBA(static_cast<unsigned char>(c(0)), static_cast<unsigned char>(c(1)), static_cast<unsigned char>(c(2)), static_cast<unsigned char>(c(3))));
    nvgText(ctx, static_cast<float>(x / mPixelRatio), static_cast<float>(y / mPixelRatio), text.c_str(), NULL);
}
