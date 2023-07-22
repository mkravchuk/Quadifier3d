#pragma once

class Viewport;
class ViewportData;
class ViewportData_OpenGLshaders;


void DrawViewportData(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl);

void DrawViewportData_Begin(Viewport& v);
