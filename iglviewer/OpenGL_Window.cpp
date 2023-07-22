#include "stdafx.h"

// Must defined this before including OpenGL_Window.h
#define IGL_VIEWER_VIEWER_CPP
#include "OpenGL_Window.h"
#include "ViewportVirtualDrawer.h"
#include "OpenGL_Options.h"
#include "OpenGL_Shared.h"


//#include <thread>         // this_thread::sleep_for
//#include <chrono>         // chrono::seconds

#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#endif

//#include <thread>

#ifndef __APPLE__
#  define GLEW_STATIC
#  include <GL/glew.h>
#endif

#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   define __gl_h_ /* Prevent inclusion of the old gl.h */
#else
#   include <GL/gl.h>
#endif

#include <Eigen/LU>

//#define GLFW_INCLUDE_GLU
#if defined(__APPLE__)
#define GLFW_INCLUDE_GLCOREARB
#else
#define GL_GLEXT_PROTOTYPES
#endif

#include <GLFW/glfw3.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <igl/project.h>
#include <igl/get_seconds.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <CoMISo/Utils/StopWatch.hh>
#include "trackballXYZaxis.h"


// Internal global variables used for glfw event handling
static OpenGL_Window * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


static void glfw_mouse_press(GLFWwindow* windowHandle, int button, int action, int modifier)
{
    //__viewer->userIteractionEventDetected = true;
    bool tw_used = __viewer->screen->mouseButtonCallbackEvent(button, action, modifier);

    OpenGL_Window::MouseButton mb;

    if (button == GLFW_MOUSE_BUTTON_1)
        mb = OpenGL_Window::MouseButton::Left;
    else if (button == GLFW_MOUSE_BUTTON_2)
        mb = OpenGL_Window::MouseButton::Right;
    else //if (button == GLFW_MOUSE_BUTTON_3)
        mb = OpenGL_Window::MouseButton::Middle;

    if (action == GLFW_PRESS)
    {
        if (!tw_used)
        {
            __viewer->mouse_down(mb, modifier);
        }
    }
    else
    {
        // Always call mouse_up on up
        __viewer->mouse_up(mb, modifier);
    }

}

static void glfw_error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

static void glfw_char_mods_callback(GLFWwindow* windowHandle, unsigned int codepoint, int modifier)
{
    __viewer->userIteractionEventDetected = true;
    // TODO: pass to nanogui (although it's also using physical key down/up
    // rather than character codes...
    if (!__viewer->screen->charCallbackEvent(codepoint))
    {
        __viewer->key_pressed(codepoint, modifier);
    }
}

static void glfw_key_callback(GLFWwindow* windowHandle, int key, int scancode, int action, int modifier)
{
    __viewer->userIteractionEventDetected = true;
    //Mupoc - very dangerous shortcut - disable it
//  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    //glfwSetWindowShouldClose(windowHandle, GL_TRUE);

    if (__viewer->screen->keyCallbackEvent(key, scancode, action, modifier) == false)
    {
        if (action == GLFW_PRESS)
            __viewer->key_down(key, modifier);
        else if (action == GLFW_RELEASE)
            __viewer->key_up(key, modifier);
    }
}

static void glfw_window_size(GLFWwindow* windowHandle, int width, int height)
{
    __viewer->userIteractionEventDetected = true;
    double w = width * highdpi;
    double h = height * highdpi;

    __viewer->resize(w, h); // truncate double to int

    // TODO: repositioning of the nanogui
    //cout << "* glfw_window_size" << endl;
}

static void glfw_mouse_move(GLFWwindow* windowHandle, double x, double y)
{
    //cout << " __viewer->userIteractionEventDetected = " << __viewer->userIteractionEventDetected << endl;
    if (__viewer->screen->cursorPosCallbackEvent(x, y))
    {
        //cout << "* glfw_mouse_move - nanogui" << endl;
    }
    else
    {
        //cout << "* glfw_mouse_move - viewport" << endl;
        __viewer->mouse_move(x*highdpi, y*highdpi);
    }
}

static void glfw_mouse_scroll(GLFWwindow* windowHandle, double x, double y)
{
    __viewer->userIteractionEventDetected = true;
    scroll_x += x;
    scroll_y += y;

    if (__viewer->screen->scrollCallbackEvent(x, y) == false)
    {
        __viewer->mouse_scroll(y);
    }
}

static void glfw_drop_callback(GLFWwindow *windowHandle, int count, const char **arg)
{
    __viewer->userIteractionEventDetected = true;
    vector<string> filenames(count);
    for (int i = 0; i < count; ++i)
        filenames[i] = arg[i];

    if (__viewer->screen->dropCallbackEvent(count, arg) == false)
    {
        __viewer->drop(filenames);
    }
}

void OpenGL_Window::createNanogui()
{
    Timer time;

    viewport.init();

    if (callback_createGUI)
        if (callback_createGUI(*this))
            return;

    //cout << "screen->performLayout();" << endl;
    //this_thread::sleep_for(chrono::seconds(2));
    nanogui->refresh();
    screen->performLayout();
    needToShowWindowFirstTime = true;

    time.stop(elapsedTimers.App.CreateNanoguiOptions, elapsedTimers.App.Total);



}

void OpenGL_Window::DrawAll()
{
    viewport.DrawAll();
    for (auto& p : plugins)
    {
        p->DrawAll();
    }
}

OpenGL_Window::OpenGL_Window()
{
    nanogui = nullptr;
    screen = nullptr;

    // Temporary variables initialization
    down = false;
    hack_never_moved = true;
    userIteractionEventDetected = true;
    scroll_position = 0.0;

    mouse_mode = MouseMode::None;
    mouse_down_modifier_isShift = false;
    mouse_down_modifier_isCtrl = false;
    mouse_down_modifier_isAlt = false;


    // C-style callbacks
    callback_createGUI = nullptr;
    callback_init_loadFileIntoViewport = nullptr;
    callback_shutdown = nullptr;
    callback_pre_draw = nullptr;
    callback_post_draw = nullptr;
    callback_mouse_down = nullptr;
    callback_mouse_up = nullptr;
    callback_mouse_move = nullptr;
    callback_mouse_scroll = nullptr;
    callback_key_down = nullptr;
    callback_key_up = nullptr;
    callback_drop = nullptr;

    callback_init_data = nullptr;
    callback_pre_draw_data = nullptr;
    callback_post_draw_data = nullptr;
    callback_mouse_down_data = nullptr;
    callback_mouse_up_data = nullptr;
    callback_mouse_move_data = nullptr;
    callback_mouse_scroll_data = nullptr;
    callback_key_down_data = nullptr;
    callback_key_up_data = nullptr;
    callback_drop_data = nullptr;

    #ifndef IGL_VIEWER_VIEWER_QUIET
    const string usage(R"(OpenGL_Window usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  Z       Snap to canonical view
  [,]     Toggle between rotation control types (e.g. trackball, two-axis
          valuator with fixed up))"
        R"(
  ;       Toggle vertex labels
  :       Toggle face labels)"
    );
    cout << usage << endl;
    #endif
}

void OpenGL_Window::init_plugins()
{
    Timer time;

    //cout << "init_plugins();" << endl;

    // Init all plugins
    for (int i = 0; i < plugins.size(); ++i)
    {
        plugins[i]->init(this);
    }
    time.stop(elapsedTimers.App.InitPlugins, elapsedTimers.App.Total);
}

OpenGL_Window::~OpenGL_Window()
{
}

void OpenGL_Window::shutdown_plugins()
{
    for (int i = 0; i < plugins.size(); ++i)
    {
        plugins[i]->shutdown();
    }
}

bool OpenGL_Window::key_pressed(unsigned int unicode_key, int modifiers)
{
    userIteractionEventDetected = true;
    if (callback_key_pressed)
        if (callback_key_pressed(*this, unicode_key, modifiers))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
    {
        if (plugins[i]->key_pressed(unicode_key, modifiers))
        {
            return true;
        }
    }

    return false;
}

bool OpenGL_Window::key_down(int key, int modifiers)
{
    userIteractionEventDetected = true;
    if (callback_key_down)
        if (callback_key_down(*this, key, modifiers))
            return true;
    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->key_down(key, modifiers))
            return true;
    return false;
}

bool OpenGL_Window::key_up(int key, int modifiers)
{
    userIteractionEventDetected = true;
    if (callback_key_up)
        if (callback_key_up(*this, key, modifiers))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->key_up(key, modifiers))
            return true;

    return false;
}

bool OpenGL_Window::drop(vector<string> filenams)
{
    userIteractionEventDetected = true;
    if (callback_drop)
        if (callback_drop(*this, filenams))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->drop(filenams))
            return true;

    return false;
}

bool OpenGL_Window::mouse_down(MouseButton button, int modifier)
{
    //userIteractionEventDetected = true;
    // Remember mouse location at down even if used by callback/plugin
    down = true;
    down_mouse_x = current_mouse_x;
    down_mouse_y = current_mouse_y;
    mouse_down_modifier_isShift = GLFW_MOD_SHIFT & modifier;
    mouse_down_modifier_isCtrl = GLFW_MOD_CONTROL & modifier;
    mouse_down_modifier_isAlt = GLFW_MOD_ALT & modifier;

    if (callback_mouse_down)
        if (callback_mouse_down(*this, static_cast<int>(button), modifier))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->mouse_down(static_cast<int>(button), modifier))
            return true;


    // Initialization code for the trackball
    P3 center(0, 0, 0);
    if (drawer) center = drawer->GetCentroid();

    Matrix4d viewport_view_model_double = viewport.view* viewport.model;
    Matrix4d viewport_proj_double = viewport.proj;
    Vector4d viewport_size_double = viewport.size;
    Point3d coord =
        igl::project(
            convertP3ToEigenDouble(center),
            viewport_view_model_double,
            viewport_proj_double,
            viewport_size_double);
    down_mouse_z = coord[2];
    down_rotation = viewport.model_rotation;
    down_translation = viewport.model_translation;

    mouse_mode = MouseMode::Translation;
    switch (button)
    {
        case MouseButton::Left:
            mouse_mode = MouseMode::Translation;
            break;

        case MouseButton::Right:
            mouse_mode = MouseMode::Rotation;
            if (mouse_down_modifier_isAlt)
            {
                mouse_mode = MouseMode::Zoom;
            }
            if (mouse_down_modifier_isCtrl)
            {
                mouse_mode = MouseMode::RotationZAxis;
            }
            //if (mouse_down_modifier_isShift)
            //{
            //    mouse_mode = MouseMode::RotationXorYAxis;
            //}
            break;

        default:
            mouse_mode = MouseMode::None;
            break;
    }

    return true;
}

bool OpenGL_Window::mouse_up(MouseButton button, int modifier)
{
    userIteractionEventDetected = true;


    bool processed = false;

    if (callback_mouse_up)
        if (callback_mouse_up(*this, static_cast<int>(button), modifier))
        {
            processed = true;
        }


    if (!processed)
    {
        for (int i = 0; i < plugins.size(); ++i)
            if (plugins[i]->mouse_up(static_cast<int>(button), modifier))
            {
                processed = true;
                break;
            }
    }

    down = false;
    mouse_mode = MouseMode::None;
    mouse_down_modifier_isAlt = false;
    mouse_down_modifier_isAlt = false;
    mouse_down_modifier_isAlt = false;

    return true;
}

Point3d unproject(const Point3d&  win, const Matrix4d& model, const Matrix4d& proj, const Vector4d&  viewport)
{
    auto coutV3 = [](string name, Point3d v)
    {
        cout << "  " << name << endl;
        cout << "           ";
        for (int x = 0; x < 3; x++)
            cout << v(x) << "  ";
        cout << endl;
    };
    //coutV3("win", win);

    Matrix4d Inverse = (proj * model).inverse();

    Vector4d tmp;
    tmp << win, 1;
    tmp(0) = (tmp(0) - viewport(0)) / viewport(2);
    tmp(1) = (tmp(1) - viewport(1)) / viewport(3);
    tmp = tmp.array() * 2.0f - 1.0f;

    Vector4d obj = Inverse * tmp;
    obj /= obj(3);

    Point3d scene = obj.head(3);
    return scene;
}


bool OpenGL_Window::mouse_move(double mouse_x, double mouse_y)
{
    if (!hack_never_moved && !down && mouse_mode == MouseMode::None)
    {
        //skipThisRedrawSinceItJustMouseMove = true;
    }
    else
    {
        userIteractionEventDetected = true;
    }

    if (hack_never_moved)
    {
        down_mouse_x = mouse_x;
        down_mouse_y = mouse_y;
        hack_never_moved = false;
    }
    current_mouse_x = mouse_x;
    current_mouse_y = mouse_y;

    if (callback_mouse_move)
        if (callback_mouse_move(*this, mouse_x, mouse_y))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->mouse_move(mouse_x, mouse_y))
            return true;

    auto Get_trackballXYZAxis = [&](bool rotateX, bool rotateY, bool rotateZ)
    {
        Quaterniond trackball_angle;
        igl::trackballXYZAxis(
            viewport.size(2),
            viewport.size(3),
            get_dynamic_rotation_speed(),
            down_rotation,
            down_mouse_x,
            down_mouse_y,
            mouse_x,
            mouse_y,
            rotateX, rotateY, rotateZ,
            trackball_angle);
        return trackball_angle;
    };

    if (down)
    {
        switch (mouse_mode)
        {
            case MouseMode::Rotation:
            {
                switch (viewport.rotation_type)
                {
                    default:
                        assert(false && "Unknown rotation type");
                    case Viewport::RotationType::TRACKBALL:
                        igl::trackball(
                            viewport.size(2),
                            viewport.size(3),
                            get_dynamic_rotation_speed(),
                            down_rotation,
                            down_mouse_x,
                            down_mouse_y,
                            mouse_x,
                            mouse_y,
                            viewport.model_rotation);
                        //cout << "viewport.trackball_angle=" << viewport.trackball_angle.coeffs()(0) << ",  " << viewport.trackball_angle.coeffs()(1) << ",  " << viewport.trackball_angle.coeffs()(2) << ",  " << viewport.trackball_angle.coeffs()(3) << endl;
                        break;
                    case Viewport::RotationType::TWO_AXIS_VALUATOR_FIXED_UP:
                        igl::two_axis_valuator_fixed_up(
                            (int)viewport.size(2),
                            (int)viewport.size(3),
                            get_dynamic_rotation_speed(),
                            down_rotation,
                            (int)down_mouse_x,
                            (int)down_mouse_y,
                            (int)mouse_x,
                            (int)mouse_y,
                            viewport.model_rotation);
                        break;
                    case Viewport::RotationType::ScreenXY:
                        viewport.model_rotation = Get_trackballXYZAxis(true, true, false);
                        break;
                }
                //Vector4d snapq = viewport.trackball_angle;

                break;
            }
            case MouseMode::RotationXorYAxis:
            {
                double shiftX = abs(down_mouse_x - mouse_x);
                double shiftY = abs(down_mouse_y - mouse_y);
                if (abs(shiftX - shiftY) > 1)
                {
                    bool rotateX = (shiftX < shiftY);
                    bool rotateY = !rotateX;
                    viewport.model_rotation = Get_trackballXYZAxis(rotateX, rotateY, false);
                }
                break;
            }

            case MouseMode::RotationZAxis:
            {
                viewport.model_rotation = Get_trackballXYZAxis(false, false, true);
                break;
            }

            case MouseMode::Translation:
            {
                //translation
                //cout << endl << endl << endl;
                Matrix4d view_model = viewport.view * viewport.model;
                Point3d pos1 = unproject(Point3d(mouse_x, viewport.size[3] - mouse_y, down_mouse_z), view_model, viewport.proj, viewport.size);
                Point3d pos0 = unproject(Point3d(down_mouse_x, viewport.size[3] - down_mouse_y, down_mouse_z), view_model, viewport.proj, viewport.size);

                Vector3d diff = pos1 - pos0;
                viewport.model_translation = down_translation + diff;
                //cout << "down_mouse_x=" << down_mouse_x << "   mouse_x=" << mouse_x << endl;
                //cout << "down_mouse_y=" << down_mouse_y << "   mouse_y=" << mouse_y << endl;
                //cout << "down_mouse_z=" << down_mouse_z << endl;
                //cout << "camera_zoom=" << viewport.camera_zoom << "   model_zoom=" << viewport.model_zoom << endl;
                //cout << "down_translation  = " << down_translation[0] << ", " << down_translation[1] << ", " << down_translation[2] << endl;
                //cout << "pos0 = " << pos0[0] << ", " << pos0[1] << ", " << pos0[2] << endl;
                //cout << "pos1 = " << pos1[0] << ", " << pos1[1] << ", " << pos1[2] << endl;
                //cout << "diff  = " << diff[0] << ", " << diff[1] << ", " << diff[2] << endl;

                break;
            }
            case MouseMode::Zoom:
            {
                //double delta = 0.001 * (mouse_x - down_mouse_x + mouse_y - down_mouse_y);
                double delta = 0.001 * (down_mouse_y - mouse_y);
                viewport.camera_zoom *= 1 + delta;
                down_mouse_x = mouse_x;
                down_mouse_y = mouse_y;
                //cout << "viewport.camera_zoom = " << viewport.camera_zoom << endl;
                break;
            }

            default:
                break;
        }
    }
    return true;
}

bool OpenGL_Window::mouse_scroll(double delta_y)
{
    userIteractionEventDetected = true;
    scroll_position += delta_y;

    if (callback_mouse_scroll)
        if (callback_mouse_scroll(*this, delta_y))
            return true;

    for (int i = 0; i < plugins.size(); ++i)
        if (plugins[i]->mouse_scroll(delta_y))
            return true;

    // Only zoom if there's actually a change
    if (delta_y != 0)
    {
        double mult = (1.0 + ((delta_y > 0) ? 1 : -1)*0.05);
        const double min_zoom = 0.1;
        viewport.camera_zoom = viewport.camera_zoom * mult > min_zoom
            ? viewport.camera_zoom * mult
            : min_zoom;
        //cout << endl << endl << "viewport.camera_zoom = " << viewport.camera_zoom << "      scroll_position=" << scroll_position << "      delta_y=" << delta_y << endl;
    }
    return true;
}


void OpenGL_Window::drawViewport()
{
    elapsedTimers.Viewer.Reset();
    Timer timeViewportTotal;
    if (drawer)
    {
        drawer->Draw(viewport);
    }
    timeViewportTotal.stop(elapsedTimers.Viewer.Total);
}
bool OpenGL_Window::drawNanogui(bool drawOnlyGui)
{
    bool was_drawed = false;
    Timer timeNanogui;
    if (isNanoguiVisible())
    {
        screen->drawContents();
        screen->drawWidgets(drawOnlyGui);
        was_drawed = true;
    }
    timeNanogui.stop(elapsedTimers.GUI.Nanogui);
    return was_drawed;

}

bool OpenGL_Window::isNanoguiVisible()
{
    bool show_nanogui = !hide_menu;
    if (mouse_mode == MouseMode::Rotation && hide_menu_on_mouse)
    {
        show_nanogui = false;
    }
    return show_nanogui;
}


void OpenGL_Window::drawViewport_ToBuffer(Viewport& viewport,
    Matrix<unsigned char, Dynamic, Dynamic>& R,
    Matrix<unsigned char, Dynamic, Dynamic>& G,
    Matrix<unsigned char, Dynamic, Dynamic>& B,
    Matrix<unsigned char, Dynamic, Dynamic>& A,
    bool updateOpenGLTransformationMatrices)
{
    assert(R.rows() == G.rows() && G.rows() == B.rows() && B.rows() == A.rows());
    assert(R.cols() == G.cols() && G.cols() == B.cols() && B.cols() == A.cols());

    int x = R.rows();
    int y = R.cols();

    // Create frame buffer
    GLuint frameBuffer;
    glGenFramebuffers(1, &frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

    // Create texture to hold color buffer
    GLuint texColorBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); //  GL_UNSIGNED_BYTE  <-->  GLubyte
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0);

    // Create Renderbuffer Object to hold depth and stencil buffers
    GLuint rboDepthStencil;
    glGenRenderbuffers(1, &rboDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepthStencil);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, x, y);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rboDepthStencil);

    assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

    // Clear the buffer
    glClearColor(viewport.background_color(0), viewport.background_color(1), viewport.background_color(2), 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Save old viewport size
    auto viewport_size_origin = viewport.size;
    viewport.size = Vector4i(0, 0, x, y).cast<double>();

    // Draw
    if (updateOpenGLTransformationMatrices) viewport.UpdateOpenGLTransformationMatrices();
    drawViewport();

    // Restore viewport size
    viewport.size = viewport_size_origin;

    // Copy back in the given Eigen matrices
    GLubyte* pixels = (GLubyte*)calloc(x*y * sizeof(GLubyte), sizeof(GLubyte));
    glReadPixels
    (
        0, 0,
        x, y,
        GL_RGBA, GL_UNSIGNED_BYTE, pixels
    );

    int count = 0;
    for (int j = 0; j < y; ++j)
    {
        for (int i = 0; i < x; ++i)
        {
            R(i, j) = pixels[count * 4 + 0];
            G(i, j) = pixels[count * 4 + 1];
            B(i, j) = pixels[count * 4 + 2];
            A(i, j) = pixels[count * 4 + 3];
            ++count;
        }
    }

    // Clean up
    free(pixels);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &rboDepthStencil);
    glDeleteTextures(1, &texColorBuffer);
    glDeleteFramebuffers(1, &frameBuffer);
}


// https://www.gamedev.net/forums/topic/634356-problem-rendering-to-a-texture/
void GLBlit(int srcx, int srcy, int srcw, int srch, GLuint src_textureID, int dstx, int dsty, GLuint dst_textureID)
{
    GLuint fboName;
    glGenFramebuffers(1, &fboName);

    // enable this frame buffer as the current frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, fboName);// attach the textures to the frame buffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, src_textureID, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, dst_textureID, 0);
    GLenum fboStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (fboStatus != GL_FRAMEBUFFER_COMPLETE)
    {
        printf("FrameBuffer incomplete: 0x%x\n", fboStatus);
        return;
    }

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    GLenum bufferlist[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
    glDrawBuffers(1, &bufferlist[1]);

    // enable alpha blending
    gl::Enable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlitFramebuffer(srcx, srcy, srcx + srcw, srcy + srch, dstx, dsty, dstx + srcw, dsty + srch, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    //glBlitFramebuffer(srcx,srcy,srcx+srcw-1,srcy+srch- ,dstx,dsty,dstx+srcw-1,dsty+srch-1,GL_COLOR_BUFFER_BIT,GL_NEAREST);
    glDeleteFramebuffers(1, &fboName);
}


//Application created framebuffer objects(i.e.those with a non - zero name) differ from the default window-system-provided 
// framebuffer in a fewimportant ways:
// 1) they have modifiable attachment points for a color buffer, a depth buffer, and a stencil buffer to which framebuffer
//     attachable images may be attached and detached.
// 2) the size and format of the attached images are controlled entirely within the GL and are not
//     affected by window - system events, such as pixel format selection,window resizes, and display mode changes.
// 3) when rendering to or reading from an application created framebuffer object, the pixel
//      ownership test always succeeds(i.e.they own all their pixels).
//  4) there are no visible color buffer bitplanes, only a single "off-screen"
//      color image attachment, so there is no sense of front and back buffers or swapping.
//  5) there is no multisample buffer, so the value of the implementation - dependent state variables GL_SAMPLES and GL_SAMPLE_BUFFERS
//      are both zero for application created framebuffer objects.
GLuint layers_frameBuffer = 0;
constexpr int layers_count = 2;
GLuint layers_ColorBuffer[layers_count] = { 0,0 };
GLuint layers_rboDepthStencil = 0;
GLuint nanogui_frameBuffer = 0;
GLuint nanogui_ColorBuffer = 0;
GLuint nanogui_rboDepthStencil = 0;
I2 DrawLayers_created_for_viewport_size = I2(0, 0);
void OpenGL_Window::CreateDrawLayers()
{
    // Tutorial 14 : Render To Texture   http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/
    // see also: OpenGL Frame Buffer Object(FBO)  http://www.songho.ca/opengl/gl_fbo.html#example
    int width = (int)viewport.size(2);
    int height = (int)viewport.size(3);

    // exit if window is minimized
    if (width == 0 || height == 0) return;

    // exit if window size is same - no need to recreate buffers
    bool isWindowSizeChanged = (DrawLayers_created_for_viewport_size(0) != width || DrawLayers_created_for_viewport_size(1) != height);
    if (!isWindowSizeChanged) return;

    DrawLayers_created_for_viewport_size = I2(width, height);

    //
    // Signal nanogui to repaint GUI
    //
    screen->markDity();

    //
    // Delete outdated framebuffers and textures
    //
    if (layers_frameBuffer != 0)
    {
        glDeleteFramebuffers(1, &layers_frameBuffer);
        layers_frameBuffer = 0;
    }
    for (int i = 0; i < layers_count; i++)
    {
        if (layers_ColorBuffer[i] == 0) continue;
        glDeleteTextures(1, &layers_ColorBuffer[i]);
        layers_ColorBuffer[i] = 0;
    }
    if (layers_rboDepthStencil != 0)
    {
        glDeleteRenderbuffers(1, &layers_rboDepthStencil);
        layers_rboDepthStencil = 0;
    }

    if (nanogui_frameBuffer != 0)
    {
        glDeleteFramebuffers(1, &nanogui_frameBuffer);
        nanogui_frameBuffer = 0;
    }
    if (nanogui_ColorBuffer != 0)
    {
        glDeleteTextures(1, &nanogui_ColorBuffer);
        nanogui_ColorBuffer = 0;
    }
    if (nanogui_rboDepthStencil != 0)
    {
        glDeleteRenderbuffers(1, &nanogui_rboDepthStencil);
        nanogui_rboDepthStencil = 0;
    }

    //
    // Create layers
    //
    // create layers framebuffer
    glGenFramebuffers(1, &layers_frameBuffer);
    // activate layers framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, layers_frameBuffer);
    // create layers (textures)
    for (int i = 0; i < layers_count; i++)
    {
        // The texture we're going to render to
        glGenTextures(1, &layers_ColorBuffer[i]);
        // "Bind" the newly created texture : all future texture functions will modify this texture
        glBindTexture(GL_TEXTURE_2D, layers_ColorBuffer[i]);
        // Give an empty image to OpenGL ( the last "NULL" )
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_FLOAT, NULL); //  GL_UNSIGNED_BYTE  <-->  GLubyte
        // Poor filtering. Needed !
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // prevent sampling textures at their borders
        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // Set "renderedTexture" as our colour attachement #0
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, layers_ColorBuffer[i], 0);
    }
    gl::set_layers_ColorBuffer_1(layers_ColorBuffer[1]);

    // create depth buffer
    glGenRenderbuffers(1, &layers_rboDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, layers_rboDepthStencil);
    //v0 depth+stencil
    //glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    //glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, layers_rboDepthStencil);
    //v1 depth
    if (openglOptions.WindowHint.DEPTH_BITS == 16)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height); // bad quality
    else if (openglOptions.WindowHint.DEPTH_BITS == 24)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height); // good quality
    else
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width, height); // perfect quality
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, layers_rboDepthStencil);
    assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);


    //
    // Create nanogui
    //
    // create nanogui framebuffer
    glGenFramebuffers(1, &nanogui_frameBuffer);
    // activate nanogui framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, nanogui_frameBuffer);

    // create nanogui layer (texture)
    {
        // The texture we're going to render to
        glGenTextures(1, &nanogui_ColorBuffer);
        // "Bind" the newly created texture : all future texture functions will modify this texture
        glBindTexture(GL_TEXTURE_2D, nanogui_ColorBuffer);
        // Give an empty image to OpenGL ( the last "NULL" )
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); //  GL_UNSIGNED_BYTE  <-->  GLubyte
        // Poor filtering. Needed !
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // prevent sampling textures at their borders
        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // Set 'nanogui_ColorBuffer' as our colour attachement #0
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, nanogui_ColorBuffer, 0);
    }

    // create depth buffer
    glGenRenderbuffers(1, &nanogui_rboDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, nanogui_rboDepthStencil);
    //v0 depth+stencil
    //glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    //glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, nanogui_rboDepthStencil);
    //v1 stencil
    glRenderbufferStorage(GL_RENDERBUFFER, GL_STENCIL_INDEX8, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, nanogui_rboDepthStencil);

    assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);


    //
    // activate default framebuffer
    //
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
}

void OpenGL_Window::clear_framebuffers()
{
    userIteractionEventDetected = true;

    //The Scissor Test is a Per-Sample Processing operation that discards Fragments that fall outside of a certain rectangular portion of the screen
    //https://www.khronos.org/opengl/wiki/Scissor_Test
    //gl::Enable(GL_SCISSOR_TEST);
    //glScissor(0, 0, 1000, 1000);
    //glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_FASTEST);// dont helps at all
    //glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_DONT_CARE); // dont helps at all


    //
    // Clear layers
    //
    if (openglOptions.Performance.use_viewport_layers)
    {
        //v0
        //glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
        //GLuint attachments[1 + layers_count];
        //attachments[0] = GL_BACK_LEFT;
        //for (int i = 0; i < layers_count; i++)
        //{
        //    attachments[1 + i] = GL_COLOR_ATTACHMENT1 + i;
        //}
        //glDrawBuffers(1+ layers_count, attachments); // set layers to draw

        //v1
        //glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
        //GLuint attachments[1] = { GL_COLOR_ATTACHMENT1 };
        //glDrawBuffers(1, attachments); // set layers to draw

        //v2
        //if (openglOptions.WindowHint.OpenGL_version_MAJOR >= 4 && openglOptions.WindowHint.OpenGL_version_MINOR >= 4)
        //{
        //    //SLOW - fps droped from 433 to 333
        //    //gl::Disable(GL_SCISSOR_TEST);
        //    gl::Disable(GL_STENCIL_TEST);
        //    gl::Disable(GL_BLEND);
        //    gl::Disable(GL_DITHER); // almost to nothing, but it seems redraw works 0.1% faster
        //    for (int i = 0; i < layers_count; i++)
        //    {
        //        if (layers_ColorBuffer[i] == 0) continue;
        //        byte clearColor[4] = { 255, 255, 255, 255 };
        //        glClearTexImage(layers_ColorBuffer[i], 0, GL_RGBA, GL_UNSIGNED_BYTE, clearColor);
        //    }
        //    short clearDepthValue = 0;
        //    //cout << "layers_rboDepthStencil= " << layers_rboDepthStencil << endl;
        //    //glClearTexImage(layers_rboDepthStencil, 0, GL_DEPTH_COMPONENT16, GL_UNSIGNED_SHORT, &clearDepthValue);
        //    glBindFramebuffer(GL_FRAMEBUFFER, layers_frameBuffer);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
        //    glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        //}
        //else
        {
            glBindFramebuffer(GL_FRAMEBUFFER, layers_frameBuffer);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
            GLuint attachments[layers_count];
            for (int i = 0; i < layers_count; i++)
            {
                attachments[i] = GL_COLOR_ATTACHMENT0 + i;
            }
            GLuint attachments0[1] = { GL_COLOR_ATTACHMENT0 };
            GLuint attachments1[1] = { GL_COLOR_ATTACHMENT1 };
            //glDrawBuffers(layers_count, attachments); // set layers to draw
            glDrawBuffers(1, attachments1); // activate layer#1 - here will be depth_layer
            glClearColor(0, 0, 0, 0);
            glClear(GL_COLOR_BUFFER_BIT);
            glDrawBuffers(1, attachments0); // activate layer#0
            glClearColor(viewport.background_color[0], viewport.background_color[1], viewport.background_color[2], 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        }
    }
    else
    {
        //
        // Clear main framebuffer
        //
        glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer

        //glFlush(); // dont helps at all
       //gl::Disable(GL_STENCIL_TEST);
       //gl::Disable(GL_BLEND);
       //gl::Disable(GL_DITHER); // almost to nothing, but it seems redraw works 0.1% faster
       //gl::Disable(GL_SCISSOR_TEST);
       //gl::Disable(GL_LIGHTING);
       //gl::Disable(GL_CLIP_PLANE0);
       //gl::Disable(GL_CLIP_PLANE1);
       //gl::Disable(GL_CLIP_PLANE2);
       //gl::Disable(GL_CLIP_PLANE3);
       //gl::Disable(GL_CLIP_PLANE4);
       //gl::Disable(GL_CLIP_PLANE5);
       //glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_FASTEST);
       //gl::Disable(GL_DEPTH_CLAMP);
       //gl::Disable(GL_POLYGON_SMOOTH);

        gl::Disable(GL_POLYGON_SMOOTH);
        //glEnable(GL_DEPTH_CLAMP);  The clipping behavior against the Z position of a vertex (ie: -w_c \le z_c \le w_c) can be turned off by activating depth clamping. This is done with glEnable(GL_DEPTH_CLAMP). This will cause the clip-space Z to remain unclipped by the front and rear viewing volume.

        //gl::Disable(GL_SCISSOR_TEST);
        glClearColor(viewport.background_color[0], viewport.background_color[1], viewport.background_color[2], 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    }

}

void DrawFrameBuffer(GLuint frameBufferID)
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, frameBufferID);
    ////glReadBuffer(GL_COLOR_ATTACHMENT0); // not neccessary, this is default value
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    ////GLuint attachments[1] = { GL_COLOR_ATTACHMENT0 }; glDrawBuffers(1, attachments); // not neccessary, this is default value
    int width = (int)DrawLayers_created_for_viewport_size(0);
    int height = (int)DrawLayers_created_for_viewport_size(1);
    ////gl::Enable(GL_SAMPLE_COVERAGE);
    ////glSampleCoverage(0.5, GL_TRUE);
    gl::Disable(GL_STENCIL_TEST);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST); // 5ms
}



void OpenGL_Window::drawViewport_ToLayer()
{
    int width = (int)viewport.size(2);
    int height = (int)viewport.size(3);

    // exit if window is minimized
    if (width == 0 || height == 0) return;

    bool needRedraw = true;// viewport->isDirty();
    if (needRedraw)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, layers_frameBuffer);

        // Always check that our framebuffer is ok
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
            return;
        }

        //viewport->setDirty(false);
        drawViewport();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
};

void OpenGL_Window::drawNanogui_ToLayer(bool drawOnlyGui)
{
    // exit if nanogui is hidden
    if (!isNanoguiVisible()) return;

    int width = (int)viewport.size(2);
    int height = (int)viewport.size(3);

    // exit if window is minimized
    if (width == 0 || height == 0) return;

    // draw
    bool needRedraw = screen->isDirty();
    if (needRedraw)
    {
        // Activate our framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, nanogui_frameBuffer);

        // Always check that our framebuffer is ok
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
            return;
        }

        // Clear the buffer
        //glClearColor(viewport.background_color(0), viewport.background_color(1), viewport.background_color(2), 0.f);
        //glDepthFunc(GL_EQUAL);
        gl::Enable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //gl::Enable(GL_DEPTH_TEST);
        //glDepthMask(GL_TRUE);
        glClearColor(0.85f, 0.85f, 0.85f, 0); // always 'white' color, because we will draw it transparently ontop of main viewport
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        //glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        //if (screen->isDirty()) cout << "screen->isDirty() = " << screen->isDirty() << endl;
        screen->setDirty(false);
        if (!drawNanogui(drawOnlyGui))
        {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
            return;
        }

        // Copy back in the given Eigen matrices
/*GLubyte* pixels = (GLubyte*)calloc(width*height * sizeof(GLubyte)*4, sizeof(GLubyte));
glReadPixels
(
    0, 0,
    width, height,
    GL_RGBA, GL_UNSIGNED_BYTE, pixels
);

int count = 0;
for (int y = 0; y < height; y++)
{
    for (int x = 0; x < width; x++)
    {
        byte R = pixels[count * 4 + 0];
        byte G = pixels[count * 4 + 1];
        byte B = pixels[count * 4 + 2];
        byte A = pixels[count * 4 + 3];
        count++;
        //cout << "RGBA=" << R << "," << G << "," << B << "," << A << endl;
        //Matrix<byte, 4, 1> colorRGBA(R, G, B, A);
    }
}
free(pixels);
*/


// Activate default framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, 0);  // apply GL_READ_FRAMEBUFFER and GL_DRAW_FRAMEBUFFER to default framebuffer
    }
}



void OpenGL_Window::drawViewportAndGui()
{
    double viewportDrawTime;
    double nanoguiDrawTime;
    double openglSwapBuffersTime;
    drawViewportAndGui(viewportDrawTime, nanoguiDrawTime, openglSwapBuffersTime, false);
}

void OpenGL_Window::drawViewportAndGui(double& viewportDrawTime, double& nanoguiDrawTime, double& openglSwapBuffersTime, bool drawOnlyGui)
{
    //elapsedTimers.GUI.Reset();

    Timer timeGUITotal;

    CreateDrawLayers();

    if (!drawOnlyGui)
    {
        COMISO::StopWatch swViewport; swViewport.start();

        clear_framebuffers();
        viewport.UpdateOpenGLTransformationMatrices();

        //
        // pre_draw (callback event)
        //
        Timer time_callback_pre_draw;
        if (callback_pre_draw)
            if (callback_pre_draw(*this))
            {
                time_callback_pre_draw.stop(elapsedTimers.GUI.CallbackDrawEvents);
                timeGUITotal.stop();
                return;
            }
        time_callback_pre_draw.stop(elapsedTimers.GUI.CallbackDrawEvents);

        //
        // pre_draw (plugins)
        //
        Timer time_plugins_pre_draw;
        for (int i = 0; i < plugins.size(); ++i)
            if (plugins[i]->pre_draw())
            {
                time_plugins_pre_draw.stop(elapsedTimers.GUI.PluginDrawEvents);
                timeGUITotal.stop(elapsedTimers.GUI.Total);
                return;
            }
        time_plugins_pre_draw.stop(elapsedTimers.GUI.PluginDrawEvents);


        //
        // Draw viewport
        //
        timeGUITotal.stop(); // pause timeGUITotal
        if (openglOptions.Performance.use_viewport_layers)
        {
            drawViewport_ToLayer();
        }
        else
        {
            drawViewport();
        }
        timeGUITotal.start(); // continue timeGUITotal


        //
        // post_draw (callback event)
        //
        Timer time_callback_post_draw;
        if (callback_post_draw)
            if (callback_post_draw(*this))
            {
                time_callback_post_draw.stop(elapsedTimers.GUI.CallbackDrawEvents);
                timeGUITotal.stop(elapsedTimers.GUI.Total);
                return;
            }
        time_callback_post_draw.stop(elapsedTimers.GUI.CallbackDrawEvents);


        //
        // post_draw (plugins)
        //
        Timer time_plugins_post_draw;
        for (int i = 0; i < plugins.size(); ++i)
            if (plugins[i]->post_draw())
            {
                time_plugins_post_draw.stop(elapsedTimers.GUI.PluginDrawEvents);
                timeGUITotal.stop(elapsedTimers.GUI.Total);
                return;
            }
        time_plugins_post_draw.stop(elapsedTimers.GUI.PluginDrawEvents);

        viewportDrawTime = swViewport.stop();

    }
    else
    {

    }

    //
    // Draw nanogui
    //
    COMISO::StopWatch swNanogui; swNanogui.start();
    if (openglOptions.Performance.use_nanogui_layers)
    {
        drawNanogui_ToLayer(false);
    }
    else
    {
        drawNanogui(drawOnlyGui);
    }


    //
    // Draw layers
    //
    if (openglOptions.Performance.use_viewport_layers)
    {
        gl::DrawColorBuffer(layers_ColorBuffer[0], false);
    }
    if (openglOptions.Performance.use_nanogui_layers && isNanoguiVisible())
    {
        gl::DrawColorBuffer(nanogui_ColorBuffer, true);
    }
    //nanoguiDrawTime = swNanogui.stop();

    timeGUITotal.stop(elapsedTimers.GUI.Total);
}



void OpenGL_Window::align_camera_center()
{
    if (drawer)
    {
        P3 Vmin(0, 0, 0);
        P3 Vmax(0, 0, 0);
        drawer->GetBox(Vmin, Vmax);
        //cout << "Vmin = " << Vmin(0) << ", " << Vmin(1) << ", " << Vmin(2) << endl;
        //cout << "Vmax = " << Vmax(0) << ", " << Vmax(1) << ", " << Vmax(2) << endl;
        viewport.align_camera_center(convertP3ToEigenDouble(Vmin), convertP3ToEigenDouble(Vmax));
    }
}

double OpenGL_Window::get_dynamic_rotation_speed()
{
    // v0 - doenst work well - model jumps - because we can't just slow down rotation process - current rotation angle is calculated every time from when we press mouse button
    //double rotation_speed = viewport.rotation_speed;
    ////if (glfwGetKey(windowHandle, GLFW_KEY_LEFT_SHIFT)  ==GLFW_PRESS)
    //if (glfwGetKey(windowHandle, GLFW_KEY_LEFT_SHIFT) > 0 || glfwGetKey(windowHandle, GLFW_KEY_RIGHT_SHIFT) > 0)
    //{
    //    rotation_speed *= 0.25;
    //}
    ////cout << "glfwGetKey(windowHandle, GLFW_KEY_LEFT_SHIFT)    " << glfwGetKey(windowHandle, GLFW_KEY_LEFT_SHIFT) << endl;
    ////cout << "rotation_speed=" << rotation_speed << endl;
    //return rotation_speed;

    //v1
    return viewport.rotation_speed;
}

bool OpenGL_Window::save_scene()
{
    string filename = igl::file_dialog_save();
    if (filename.length() == 0)
        return false;

    return save_scene(filename);
}

bool OpenGL_Window::save_scene(string filename)
{
    viewport.serializeOptions(filename);
    return true;
}

bool OpenGL_Window::load_scene()
{
    string filename = igl::file_dialog_open();
    if (filename.length() == 0)
        return false;

    return load_scene(filename);
}

bool OpenGL_Window::load_scene(string filename)
{
    viewport.deserializeOptions(filename);
    return true;
}

void OpenGL_Window::resize(double w, double h)
{
    userIteractionEventDetected = true;
    // since we have only 1 viewport - set same size as window
    viewport.size = Vector4d(0, 0, w, h);
}

void OpenGL_Window::snap_to_canonical_quaternion()
{
    Quaterniond snapq = this->viewport.model_rotation;
    igl::snap_to_canonical_view_quat(snapq, 1.0f, this->viewport.model_rotation);
}

int  OpenGL_Window::createOpenGLWindow(bool resizable, bool fullscreen, bool maximized, string windowTitle)
{
    Timer time_create_window;
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        time_create_window.stop(elapsedTimers.App.CreateOpenGLWindow, elapsedTimers.App.Total);
        return EXIT_FAILURE;
    }

    //TODO uncomment
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE); // Mupoc - help stop flickering on app startup
    if (maximized) glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE); // Mupoc - always start app in maximized mode

    // i have Firepro m7820:   DirectX 11.2 (11_0),  OpenGL 4.4,  OpenCL 1.2,  Shader Model 5.0
    // OpenGL 4.4 API is supported by 5th and 6th generation Intel Core processors
    // glDrawTransformFeedback (OpenGL 4.1), glClearTexImage (OpenGL 4.4)        
    // but glad supported only 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, openglOptions.WindowHint.OpenGL_version_MAJOR);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, openglOptions.WindowHint.OpenGL_version_MINOR);

    //glfwWindowHint(GLFW_RED_BITS, 4); // 16 is enoguht, could be 32 for more precise z-order test
    //glfwWindowHint(GLFW_GREEN_BITS, 4); // 16 is enoguht, could be 32 for more precise z-order test
    //glfwWindowHint(GLFW_BLUE_BITS, 4); // 16 is enoguht, could be 32 for more precise z-order test
    //v0 - supporting stencil
    //glfwWindowHint(GLFW_DEPTH_BITS, 24); // 24 is enought, and is good because 24+8 = 32  (24-bits will be used for depth, and the remaining 8-bits for stencil)
    //glfwWindowHint(GLFW_STENCIL_BITS, 8); // 8 - is good because 24+8 = 32  (24-bits will be used for depth, and the remaining 8-bits for stencil)
    //v1 - no-supporting stencil - redraw faster by 10% becuase depth buffer now 2 times smaller and since memory test is faster
    glfwWindowHint(GLFW_DEPTH_BITS, openglOptions.WindowHint.DEPTH_BITS);
    glfwWindowHint(GLFW_STENCIL_BITS, openglOptions.WindowHint.STENCIL_BITS);
    if (!openglOptions.WindowHint.recreateWindowAfterLoadExtensions)
    {
        glfwWindowHint(GLFW_RECREATE_WINDOW_AFTER_LOAD_EXTENSIONS, GLFW_FALSE);
    }
    if (openglOptions.WindowHint.SAMPLES != OpenGLAntialisingSamples::_0)
    {
        glfwWindowHint(GLFW_SAMPLES, (int)openglOptions.WindowHint.SAMPLES); // use super antialising for better look, 4 - is gold balance between quality and speed (4 - 90fpd, 8 - 70fps)
    }
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);  // use double buffer for avoiding flickering
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);  produces error in nanogui and disables lines thikness! // ignore old OpenGL version functionality from driver  
    //glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);  // create a debug OpenGL context, which may have additional error and performance issue reporting functionality


    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif


    if (fullscreen)
    {
        GLFWmonitor *monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode *mode = glfwGetVideoMode(monitor);
        windowHandle = glfwCreateWindow(mode->width, mode->height, windowTitle.c_str(), monitor, nullptr);
    }
    else
    {
        windowHandle = glfwCreateWindow(1280, 800, windowTitle.c_str(), nullptr, nullptr);
    }


    if (!windowHandle)
    {
        time_create_window.stop(elapsedTimers.App.CreateOpenGLWindow, elapsedTimers.App.Total);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(windowHandle);

    #ifndef __APPLE__
    glewExperimental = true;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
    fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    //#if defined(DEBUG) || defined(_DEBUG)
    int major, minor, rev;
    major = glfwGetWindowAttrib(windowHandle, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(windowHandle, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(windowHandle, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION)); // - take some time, so better to disable it
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
    //#endif

    glfwSetInputMode(windowHandle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // Initialize FormScreen
    screen = new nanogui::Screen();
    screen->initialize(windowHandle, false);
    nanogui = new nanogui::FormHelper(screen);

    __viewer = this;

    // Register callbacks
    glfwSetKeyCallback(windowHandle, glfw_key_callback);
    glfwSetCursorPosCallback(windowHandle, glfw_mouse_move);
    glfwSetWindowSizeCallback(windowHandle, glfw_window_size);
    glfwSetMouseButtonCallback(windowHandle, glfw_mouse_press);
    glfwSetScrollCallback(windowHandle, glfw_mouse_scroll);
    glfwSetCharModsCallback(windowHandle, glfw_char_mods_callback);
    glfwSetDropCallback(windowHandle, glfw_drop_callback);

    // Handle retina displays (windows and mac)
    int width, height;
    glfwGetFramebufferSize(windowHandle, &width, &height);

    int width_window, height_window;
    glfwGetWindowSize(windowHandle, &width_window, &height_window);

    highdpi = static_cast<double>(width) / static_cast<double>(width_window);

    glfw_window_size(windowHandle, width_window, height_window);

    time_create_window.stop(elapsedTimers.App.CreateOpenGLWindow, elapsedTimers.App.Total);
    return EXIT_SUCCESS;
}
/*
typedef struct
{
    GLuint       *counterList;
    int         numCounters;
    int         maxActiveCounters;
} CounterInfo;

void getGroupAndCounterList(GLuint **groupsList, int *numGroups,
    CounterInfo **counterInfo)
{
    GLint          n;
    GLuint        *groups;
    CounterInfo   *counters;

    glGetPerfMonitorGroupsAMD(&n, 0, NULL);
    groups = (GLuint*)malloc(n * sizeof(GLuint));
    glGetPerfMonitorGroupsAMD(NULL, n, groups);
    *numGroups = n;

    *groupsList = groups;
    counters = (CounterInfo*)malloc(sizeof(CounterInfo) * n);
    for (int i = 0; i < n; i++)
    {
        glGetPerfMonitorCountersAMD(groups[i], &counters[i].numCounters,
            &counters[i].maxActiveCounters, 0, NULL);

        counters[i].counterList = (GLuint*)malloc(counters[i].numCounters *
            sizeof(int));

        glGetPerfMonitorCountersAMD(groups[i], NULL, NULL,
            counters[i].numCounters,
            counters[i].counterList);
    }

    *counterInfo = counters;
}


int getCounterByName(char *groupName, char *counterName, GLuint *groupID,
    GLuint *counterID)
{
    static int  countersInitialized = 0;
    static int          numGroups =0;
    static GLuint       *groups = nullptr;
    static CounterInfo  *counters = nullptr;
    int          i = 0;

    if (!countersInitialized)
    {
        getGroupAndCounterList(&groups, &numGroups, &counters);
        countersInitialized = 1;
    }

    for (i = 0; i < numGroups; i++)
    {
        char curGroupName[256];
        glGetPerfMonitorGroupStringAMD(groups[i], 256, NULL, curGroupName);
        if (strcmp(groupName, curGroupName) == 0)
        {
            *groupID = groups[i];
            break;
        }
    }

    if (i == numGroups)
        return -1;           // error - could not find the group name

    for (int j = 0; j < counters[i].numCounters; j++)
    {
        char curCounterName[256];

        glGetPerfMonitorCounterStringAMD(groups[i], counters[i].counterList[j], 256, NULL, curCounterName);
        if (strcmp(counterName, curCounterName) == 0)
        {
            *counterID = counters[i].counterList[j];
            return 0;
        }
    }

    return -1;           // error - could not find the counter name
}

void drawFrameWithCounters(void)
{
    GLuint group[2];
    GLuint counter[2];
    GLuint monitor;
    GLuint *counterData;

    // Get group/counter IDs by name.  Note that normally the
    // counter and group names need to be queried for because
    // each implementation of this extension on different hardware
    // could define different names and groups.  This is just provided
    // to demonstrate the API.
    getCounterByName("HW", "Hardware Busy", &group[0], &counter[0]);
    getCounterByName("API", "Draw Calls", &group[1], &counter[1]);

    // create perf monitor ID
    glGenPerfMonitorsAMD(1, &monitor);

    // enable the counters
    glSelectPerfMonitorCountersAMD(monitor, GL_TRUE, group[0], 1, &counter[0]);
    glSelectPerfMonitorCountersAMD(monitor, GL_TRUE, group[1], 1, &counter[1]);

    glBeginPerfMonitorAMD(monitor);

    // RENDER FRAME HERE
    // ...

    glEndPerfMonitorAMD(monitor);

    // read the counters
    GLuint resultSize;
    glGetPerfMonitorCounterDataAMD(monitor, GL_PERFMON_RESULT_SIZE_AMD, sizeof(GLint), &resultSize, NULL);

    counterData = (GLuint*)malloc(resultSize);

    GLsizei bytesWritten;
    glGetPerfMonitorCounterDataAMD(monitor, GL_PERFMON_RESULT_AMD, resultSize, counterData, &bytesWritten);

    // display or log counter info
    GLsizei wordCount = 0;

    while ((4 * wordCount) < bytesWritten)
    {
        GLuint groupId = counterData[wordCount];
        GLuint counterId = counterData[wordCount + 1];

        // Determine the counter type
        GLuint counterType;
        glGetPerfMonitorCounterInfoAMD(groupId, counterId, GL_COUNTER_TYPE_AMD, &counterType);

        if (counterType == GL_UNSIGNED_INT64_AMD)
        {
            unsigned __int64 counterResult =
                *(unsigned __int64*)(&counterData[wordCount + 2]);

            // Print counter result

            wordCount += 4;
        }
        else if (counterType == GL_FLOAT)
        {
            float counterResult = *(float*)(&counterData[wordCount + 2]);

            // Print counter result

            wordCount += 3;
        }
        // else if ( ... ) check for other counter types
        //   (GL_UNSIGNED_INT and GL_PERCENTAGE_AMD)
    }
}

*/

bool OpenGL_Window::renderWindowInLoop(bool loop)
{
    COMISO::StopWatch swFps; swFps.start();
    COMISO::StopWatch timeElapsedSinceLastDraw; timeElapsedSinceLastDraw.start();
    bool swFpsStarted = false;
    int fpsCount = 0;
    string fpsString = "";
    int framesDrawCount = 0;
    double vieportDrawTimeTotal = 0;
    double nanoguiDrawTimeTotal = 0;
    double swapBuffersTimeTotal = 0;
    // glfwMakeContextCurrent(windowHandle);

    // Rendering loop
    while (!glfwWindowShouldClose(windowHandle))
    {
        bool draw_fps = false;
        if (showfps)
        {
            framesDrawCount++;
            if (!swFpsStarted)
            {
                swFpsStarted = true;
                vieportDrawTimeTotal = 0;
                nanoguiDrawTimeTotal = 0;
                swapBuffersTimeTotal = 0;
                framesDrawCount = 0;
                swFps.restart();
            }
            auto elapsed = swFps.stop() / 1000.0;
            if (elapsed > 1) // every 1 second show fps
            {
                fpsCount = framesDrawCount;
                framesDrawCount = 0;
                auto totalTime = vieportDrawTimeTotal + nanoguiDrawTimeTotal + swapBuffersTimeTotal;
                int coreDrawTimePercent = static_cast<int>((100 * vieportDrawTimeTotal) / totalTime);
                int nanoguiDrawTimePercent = static_cast<int>((100 * nanoguiDrawTimeTotal) / totalTime);
                int swapBuffersTimePercent = static_cast<int>((100 * swapBuffersTimeTotal) / totalTime);

                fpsString = "FPS: " + to_string(fpsCount);
                fpsString += "     (viewport " + to_string(coreDrawTimePercent) + "%   nanogui " + to_string(nanoguiDrawTimePercent) + "%   swapBuffers " + to_string(swapBuffersTimePercent) + "%)";
                cout << fpsString << endl;

                //cout << "     (viewport " << coreDrawTimeTotal << "   nanogui " << nanoguiDrawTimeTotal << "   swapBuffers " << swapBuffersTimeTotal << ")" << endl << endl;
                //viewport.textrenderer.BeginDraw(viewport.view*viewport.model, viewport.proj, viewport.size, viewport.object_scale);
                //viewport.textrenderer.DrawText(V3(0,0,0), V3(0.0, 0.0, 1.0), "11111");
                //viewport.textrenderer.EndDraw();
                vieportDrawTimeTotal = 0;
                nanoguiDrawTimeTotal = 0;
                swapBuffersTimeTotal = 0;
                swFps.restart();
            }
        }
        else
        {
            swFpsStarted = false;
        }

        // Set window visible just before we start drawing
        if (needToShowWindowFirstTime)
        {
            screen->setVisible(true);
        }

        // Draw         
        bool skipThisDrawSinceItToFast = false;
        auto elapsedSinceLastDraw = timeElapsedSinceLastDraw.stop() / 1000.0;
        if (skipFastDraws && elapsedSinceLastDraw < 0.001 && !showfps)// 1000 fps if enought for smooth move, other is just wasste of GPU resources
        {
            skipThisDrawSinceItToFast = true;
        }
        bool drawOnlyGui = !needToShowWindowFirstTime
            && mouse_mode == MouseMode::None
            && !userIteractionEventDetected
            && !showfps
            && !viewport.isObjectsDataDirty;

        //if (!drawOnlyGui)
        //{
        //    if (elapsedSinceLastDraw < 0.001)
        //    {
        //        skipThisSinceItToFast = true;
        //        drawOnlyGui = true;
        //    }
            //cout << "nanogui.drawWidgets()   "<<(drawOnlyGui ? "skip" : "paint") << "     elapsed=" << elapsed << endl;
            //cout << "OpenGL_Window::draw()     " << (drawOnlyGui ? "skip" : "paint") << "     elapsed=" << elapsedSinceLastDraw << endl;
        //}
        //drawOnlyGui = false; // disabled because gui is blinking - we need to have 2 layers
        if (showDrawEventInConsole)
        {
            cout << "OpenGL_Window::draw()" << "     elapsed=" << elapsedSinceLastDraw << "    " << (skipThisDrawSinceItToFast ? "skip" : (string("paint  ") + (drawOnlyGui ? "partial" : "full"))) << endl;
        }

        double viewportDrawTime = 0;
        double nanoguiDrawTime = 0;
        double openglSwapBuffersTime;
        if (!skipThisDrawSinceItToFast)
        {
            drawViewportAndGui(viewportDrawTime, nanoguiDrawTime, openglSwapBuffersTime, drawOnlyGui);
            if (showfps)
            {
                viewport.textrenderer.BeginDraw(viewport.view, viewport.model, viewport.proj, viewport.size.cast<int>(), viewport.object_scale);
                viewport.textrenderer.DrawText2d(600, viewport.size(3) - 20, fpsString, 0.5, Color4d(0, 0, 1, 1), 10);
                viewport.textrenderer.EndDraw();
            }
            timeElapsedSinceLastDraw.restart();
        }
        else
        {
            timeElapsedSinceLastDraw.start();
        }
        userIteractionEventDetected = false;
        vieportDrawTimeTotal += viewportDrawTime;
        nanoguiDrawTimeTotal += nanoguiDrawTime;

        //
        // Swap OpenGL Buffers
        //
        COMISO::StopWatch swswapBuffers; swswapBuffers.start();
        Timer timeSwapBuffers;
        glfwSwapBuffers(windowHandle);
        timeSwapBuffers.stop(elapsedTimers.GUI.glfwSwapBuffers, elapsedTimers.GUI.Total);
        //if (timeSwapBuffers.milliseconds() > 10) cout << "openglSwapBuffersTime=" << timeSwapBuffers << endl;
        openglSwapBuffersTime = swswapBuffers.stop();
        swapBuffersTimeTotal += openglSwapBuffersTime;



        // Call init
        if (needToShowWindowFirstTime)
        {
            needToShowWindowFirstTime = false;
            if (callback_init_loadFileIntoViewport)
                callback_init_loadFileIntoViewport(*this);
        }

        if (viewport.is_animating)
        {
            double tic = igl::get_seconds();
            glfwPollEvents();
            // In microseconds
            double duration = 1000000.*(igl::get_seconds() - tic);
            const double min_duration = 1000000. / viewport.animation_max_fps;
            if (duration < min_duration)
            {
                this_thread::sleep_for(chrono::microseconds((int)(min_duration - duration)));
            }
        }
        else
        {
            if (showfps)
                glfwPollEvents(); // dont wait events - just draw to measure draw performance!
            else
                glfwWaitEvents(); //  wait for events - draw only when needed
        }

        if (!loop)
            return !glfwWindowShouldClose(windowHandle);
    }
    return EXIT_SUCCESS;
}

void OpenGL_Window::launch_shut()
{
    if (callback_shutdown)
        callback_shutdown(*this);

    if (drawer) drawer->OnOpenGLWindowShutdown();

    viewport.shut();

    shutdown_plugins();
    delete nanogui;
    //delete screen;
    screen = nullptr;
    nanogui = nullptr;

    glfwDestroyWindow(windowHandle);
    glfwTerminate();
    return;
}

int OpenGL_Window::launch(bool resizable, bool fullscreen, bool maximized, string windowTitle)
{
    //
    // create opengl window
    //
    if (createOpenGLWindow(resizable, fullscreen, maximized, windowTitle) != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }


    //
    // create Nanogui
    //
    createNanogui();

    //
    // init plugins
    //
    init_plugins();


    //
    // loop user events
    //
    renderWindowInLoop(true);

    //
    // shutdown window
    //
    launch_shut();

    return EXIT_SUCCESS;

}
