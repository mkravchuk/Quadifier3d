#pragma once

class Viewport;

class ViewportVirtualDrawer
{
public:
    virtual void GetBox(P3& Vmin, P3& Vmax) = 0;
    virtual P3 GetCentroid() = 0; // calls when mouse is down and need to get centroid of full mesh
    virtual void Draw(Viewport& v) = 0; //calls when redraw of viewport is needed
    virtual void OnOpenGLWindowShutdown() = 0; //calls when opengl window is about to shutdown and all handles of opengl should be closed
    virtual ~ViewportVirtualDrawer(){};
};