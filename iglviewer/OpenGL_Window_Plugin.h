#pragma once
#include "OpenGL_Shared.h"


// Abstract class for plugins
// All plugins MUST have this class as their parent and may implement any/all
// the callbacks marked `virtual` here.
//
// /////For an example of a basic plugins see plugins/skeleton.h
//
// Return value of callbacks: returning true to any of the callbacks tells
// Viewer that the event has been handled and that it should not be passed to
// other plugins or to other internal functions of Viewer

// Forward declaration of the viewer
class OpenGL_Window;

class OpenGL_Window_Plugin
{
protected:
    // Pointer to the main Viewer class
    OpenGL_Window * window;

public:
    ViewerDrawObjects draw;
    string Name; // plugin name


    OpenGL_Window_Plugin()
       :  window(nullptr)
    {
        Name = "";
    }

    virtual ~OpenGL_Window_Plugin()
    {

    }

    // This function is called when the viewer is initialized (no mesh will be loaded at this stage)
    virtual void init(OpenGL_Window *_window)
    {
        window = _window;
    }

    // This function is called before shutdown
    virtual void shutdown()
    {
    }


    // This function is called when the scene is serialized
    virtual bool serialize(std::vector<char>& buffer) const
    {
        return false;
    }

    // This function is called when the scene is deserialized
    virtual bool deserialize(const std::vector<char>& buffer)
    {
        return false;
    }

    // This function is called before the draw procedure of Preview3D
    virtual bool pre_draw()
    {
        return false;
    }

    // This function is called after the draw procedure of Preview3D
    virtual bool post_draw()
    {
        return false;
    }

    // This function is called when viewport will be redraw and 'draw' property have to be populated
    virtual void DrawAll()
    {
        draw.Clear();
    }

    // This function is called when the mouse button is pressed
    // - button can be GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON or GLUT_RIGHT_BUTTON
    // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
    virtual bool mouse_down(int button, int modifier)
    {
        return false;
    }

    // This function is called when the mouse button is released
    // - button can be GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON or GLUT_RIGHT_BUTTON
    // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
    virtual bool mouse_up(int button, int modifier)
    {
        return false;
    }

    // This function is called every time the mouse is moved
    // - mouse_x and mouse_y are the new coordinates of the mouse pointer in screen coordinates
    virtual bool mouse_move(double mouse_x, double mouse_y) // float to support high dpi
    {
        return false;
    }

    // This function is called every time the scroll wheel is moved
    // Note: this callback is not working with every glut implementation
    virtual bool mouse_scroll(double delta_y)
    {
        return false;
    }

    // This function is called when a keyboard key is pressed. Unlike key_down
    // this will reveal the actual character being sent (not just the physical
    // key)
    // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
    virtual bool key_pressed(unsigned int key, int modifiers)
    {
        return false;
    }

    // This function is called when a keyboard key is down
    // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
    virtual bool key_down(int key, int modifiers)
    {
        return false;
    }

    // This function is called when a keyboard key is release
    // - modifiers is a bitfield that might one or more of the following bits Preview3D::NO_KEY, Preview3D::SHIFT, Preview3D::CTRL, Preview3D::ALT;
    virtual bool key_up(int key, int modifiers)
    {
        return false;
    }

    // This function is called when a user drop some files
    virtual bool drop(vector<string> filenames)
    {
        return false;
    }

};

#ifdef ENABLE_SERIALIZATION
namespace serialization
{
    void serialize(const OpenGL_Window_Plugin& obj, std::vector<char>& buffer)
    {
        obj.serialize(buffer);
    }

    void deserialize(OpenGL_Window_Plugin& obj, const std::vector<char>& buffer)
    {
        obj.deserialize(buffer);
    }
}
#endif

