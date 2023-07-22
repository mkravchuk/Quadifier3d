#pragma once
#include "OpenGL_Shared.h"

#ifndef IGL_OPENGL_4
#define IGL_OPENGL_4
#endif


#include "OpenGL_program.h"
#include "Viewport.h"
#include "OpenGL_Window_Plugin.h"
#include "ViewportVirtualDrawer.h"

#define IGL_MOD_SHIFT           0x0001
#define IGL_MOD_CONTROL         0x0002
#define IGL_MOD_ALT             0x0004
#define IGL_MOD_SUPER           0x0008

namespace nanogui { class FormHelper; class Screen; }

struct GLFWwindow;



// openGL window + NanoGUI + mouse + keyboard
class OpenGL_Window
{
  public:
    GLFWwindow* windowHandle;

    int launch(bool resizable = true,bool fullscreen = false, bool maximized = true, string windowTitle = "");
    int createOpenGLWindow(bool resizable = true,bool fullscreen = false, bool maximized = true, string windowTitle = "");
    bool renderWindowInLoop(bool loop = true);
    void launch_shut();

    void createNanogui();
    void DrawAll();

    // Stores all the viewing options
    Viewport viewport;

    // draws mesh + lines + points + labels + transparency - using viewport and window settings
    ViewportVirtualDrawer* drawer;

    // List of registered plugins
    vector<OpenGL_Window_Plugin*> plugins;
    void init_plugins();
    void shutdown_plugins();

    // Temporary data stored when the mouse button is pressed
    Quaterniond down_rotation;
    double current_mouse_x;          // float for support highdpi
    double current_mouse_y;          // float for support highdpi
    double down_mouse_x;            // float for support highdpi
    double down_mouse_y;            // float for support highdpi
    double down_mouse_z;            // float for support highdpi
    Vector3d down_translation;
    bool down;
    bool hack_never_moved;
    bool userIteractionEventDetected; // draw only gui or model as well - if user interact with model then we need to redraw model as well

    nanogui::FormHelper* nanogui;
    nanogui::Screen* screen;
    bool needToShowWindowFirstTime;

    // Keep track of the global position of the scrollwheel
    double scroll_position;

    // UI Enumerations
    enum class MouseButton {Left, Middle, Right};
    enum class MouseMode { None, Rotation, RotationXorYAxis, RotationZAxis, Zoom, Pan, Translation} mouse_mode;
    bool mouse_down_modifier_isShift;
    bool mouse_down_modifier_isCtrl;
    bool mouse_down_modifier_isAlt;

    OpenGL_Window();
    ~OpenGL_Window();


    // Callbacks
    bool key_pressed(unsigned int unicode_key,int modifier);
    bool key_down(int key,int modifier);
    bool key_up(int key,int modifier);
    bool drop(vector<string> filenams);

    bool mouse_down(MouseButton button,int modifier);
    bool mouse_up(MouseButton button,int modifier);


    bool mouse_move(double mouse_x, double mouse_y);
    bool mouse_scroll(double delta_y);

    // Scene IO
    bool load_scene();
    bool load_scene(string filename);
    bool save_scene();
    bool save_scene(string filename);

    // Draw everything
    bool showfps;
    bool showDrawEventInConsole;
    bool skipFastDraws;
    bool hide_menu;
    bool hide_menu_on_mouse;
    void clear_framebuffers();
    void drawViewport();
    bool drawNanogui(bool drawOnlyGui = false);
    bool isNanoguiVisible();

    void drawViewportAndGui();
    void drawViewportAndGui(double& viewportDrawTime, double& nanoguiDrawTime, double& openglSwapBuffersTime, bool drawOnlyGui);
    // Draw everything
    void drawViewport_ToBuffer(Viewport& viewport, 
        Matrix<unsigned char, Dynamic, Dynamic>& R,
        Matrix<unsigned char, Dynamic, Dynamic>& G,
        Matrix<unsigned char, Dynamic, Dynamic>& B,
        Matrix<unsigned char, Dynamic, Dynamic>& A,
        bool updateOpenGLTransformationMatrices = true);

    void CreateDrawLayers();
    void drawViewport_ToLayer();
    void drawNanogui_ToLayer(bool drawOnlyGui);
    void align_camera_center();
    double get_dynamic_rotation_speed();

    // OpenGL context resize
    void resize(double w, double h); // float because we supports highdpi monitors

    // Helper functions
    void snap_to_canonical_quaternion();

    // C++-style functions
    //
    // Returns **true** if action should be cancelled.
    function<bool(OpenGL_Window& viewer)> callback_createGUI;
    function<bool(OpenGL_Window& viewer)> callback_init_loadFileIntoViewport;
    function<void(OpenGL_Window& viewer)> callback_shutdown;
    function<bool(OpenGL_Window& viewer)> callback_pre_draw;
    function<bool(OpenGL_Window& viewer)> callback_post_draw;
    function<bool(OpenGL_Window& viewer, int button, int modifier)> callback_mouse_down;
    function<bool(OpenGL_Window& viewer, int button, int modifier)> callback_mouse_up;
    function<bool(OpenGL_Window& viewer, double mouse_x, double mouse_y)> callback_mouse_move;      // float to suppodrt high dpi
    function<bool(OpenGL_Window& viewer, double delta_y)> callback_mouse_scroll;
    function<bool(OpenGL_Window& viewer, unsigned int key, int modifiers)> callback_key_pressed;
    // THESE SHOULD BE DEPRECATED:
    function<bool(OpenGL_Window& viewer, int key, int modifiers)> callback_key_down;
    function<bool(OpenGL_Window& viewer, int key, int modifiers)> callback_key_up;
    function<bool(OpenGL_Window& viewer, vector<string> filenames)> callback_drop;

    // Pointers to per-callback data
    void* callback_init_data;
    void* callback_pre_draw_data;
    void* callback_post_draw_data;
    void* callback_mouse_down_data;
    void* callback_mouse_up_data;
    void* callback_mouse_move_data;
    void* callback_mouse_scroll_data;
    void* callback_key_pressed_data;
    void* callback_key_down_data;
    void* callback_key_up_data;
    void* callback_drop_data;

  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


