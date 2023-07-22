#pragma once
#include "Timer.h"

struct NVGcontext;


class TextRenderer
{
  public:
    TextRenderer();

    virtual int Init();
    virtual int Shut(); 

    void BeginDraw2d();
    void BeginDraw(const Matrix4d &view, const Matrix4d &model, const Matrix4d &proj, const Vector4i &_viewportSize,double _object_scale);

    void EndDraw();

    void DrawText(const P3& pos, const V3& normal,const string &text, double transparent_alpha, const Vector4d& color, int fontSizeRelativeIncr = 0);
    void DrawText2d(double x, double y, const string &text, double transparent_alpha, const Vector4d& color, int fontSizeRelativeIncr = 0);
  protected:
    map<string,void *> m_textObjects;
    Matrix4d view_matrix,proj_matrix;
    Vector4d viewportSize;
    double object_scale;
    double mPixelRatio;
    NVGcontext *ctx;
    Timer timer_BeginDraw_EndDraw;
    int timer_BeginDraw_EndDraw__drawCount;
};


