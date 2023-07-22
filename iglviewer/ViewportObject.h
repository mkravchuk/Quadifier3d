#pragma once
#include "ViewportData.h"
#include "ViewportData_Mesh.h"
#include "ViewportData_OpenGLshaders.h"
#include "ViewportVirtualDrawer.h"


//
// ViewportObject
//
struct ViewportObject
{
public:
    bool isActive;

    // Stores multiple 'ViewportData_Mesh' if class was created from multiple 'ViewportData_Mesh' objects, otherwise will be nullptr
    ViewportData_Meshes* meshes;

    // Stores all the data that should be visualized
    ViewportData data;

    // Stores the vbos indices and opengl related settings
    ViewportData_OpenGLshaders opengl;

    ViewportObject();
    ViewportObject(const vector<ViewportData_Mesh>& meshDatas);
    ~ViewportObject();

    void Draw(Viewport& v);
};




//
// ViewportObjectCollection
//
class ViewportObjectCollection : public ViewportVirtualDrawer
{
    vector<ViewportObject*> Draw_ActiveObjects; //buffer for storing temporaly information for sorting - needed to avoid every time Draw allocation and delocation - speed optimization
    vector<double> Draw_Z;//buffer for storing temporaly information for sorting - needed to avoid every time Draw allocation and delocation - speed optimization
    vector<int> Draw_Z_sort_indexes;//buffer for storing temporaly information for sorting - needed to avoid every time Draw allocation and delocation - speed optimization
public:
    vector<ViewportObject*> Objects;

    ViewportObject* Add();
    ViewportObject* Add(const ViewportData_Mesh& mesh);
    ViewportObject* Add(const vector<ViewportData_Mesh>& meshes);
    bool Remove(ViewportObject* &obj);
    void Clear();
    II SizeOF(bool sizeof_mergedMeshes = true, bool sizeof_data = true, bool sizeof_opengl = true, bool sizeof_array = true) const;

    void OnOpenGLWindowShutdown() override; //calls when opengl window is about to shutdown and all handles of opengl should be closed
    ~ViewportObjectCollection();

    void GetBox(P3& Vmin, P3& Vmax) override;
    P3 GetCentroid() override; // calls from openGL_window when mouse is down and need to get centroid of full mesh for rotation
    void Draw(Viewport& v) override; //calls when redraw of viewport is needed
};