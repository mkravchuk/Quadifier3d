#include "stdafx.h"
#include "Viewport.h"
#include "ViewportObject.h"
#include "ViewportData_Mesh.h"
#include "ViewportData_OpenGLDrawer.h"


//
//
//
// ViewportObject
//
//
//

ViewportData_Meshes empty_meshes;

ViewportObject::ViewportObject()
    : isActive(false), meshes(nullptr), data(empty_meshes)
{
    opengl.init();
}

ViewportObject::ViewportObject(const vector<ViewportData_Mesh>& _meshes)
    : isActive(false), meshes(new ViewportData_Meshes(_meshes)), data(*meshes)
{
    opengl.init();
}

ViewportObject::~ViewportObject()
{
    opengl.free();
    if (meshes)
    {
        delete meshes;
        meshes = nullptr;
    }
}

void ViewportObject::Draw(Viewport& v)
{
    DrawViewportData(v, data, opengl);
}




//
//
//
// ViewportObjectCollection
//
//
//

ViewportObject* ViewportObjectCollection::Add()
{
    auto obj = new ViewportObject();
    Objects.push_back(obj);
    return obj;
}

ViewportObject* ViewportObjectCollection::Add(const ViewportData_Mesh& meshData)
{
    auto obj = new ViewportObject({ meshData });
    Objects.push_back(obj);
    return obj;
}


ViewportObject* ViewportObjectCollection::Add(const vector<ViewportData_Mesh>& meshDatas)
{
    //Timer time;
    auto obj = new ViewportObject(meshDatas);
    Objects.push_back(obj);
    //time.stop();
    //cout << "ViewportObjectCollection::Add()   time="<<time << endl;
    return obj;
}

bool ViewportObjectCollection::Remove(ViewportObject* &obj)
{
    auto index = std::find(Objects.begin(), Objects.end(), obj);
    if (index != Objects.end())
    {
        delete obj;
        Objects.erase(index);
        obj = nullptr;
        return true;
    }
    return false;
}

void ViewportObjectCollection::Clear()
{
    for (auto obj : Objects)
    {
        delete obj;
    }
    Objects.clear();
}


II ViewportObjectCollection::SizeOF(bool sizeof_mergedMeshes, bool sizeof_data, bool sizeof_opengl, bool sizeof_array) const
{
    II r = 0;
    for (auto obj : Objects)
    {
        if (obj)
        {
            if (sizeof_mergedMeshes && obj->meshes) r += obj->meshes->SizeOF();
            if (sizeof_data) r += obj->data.SizeOF();
            if (sizeof_opengl) r += obj->opengl.SizeOF();
        }
    }
    if (sizeof_array) r += Objects.size() * sizeof(void*);
    return r;
}



void ViewportObjectCollection::OnOpenGLWindowShutdown() //calls when opengl window is about to shutdown and all handles of opengl should be closed
{
    Clear();
}
ViewportObjectCollection::~ViewportObjectCollection()
{
    Clear();
}

void ViewportObjectCollection::GetBox(P3& Vmin, P3& Vmax)
{
    bool setted = false;
    for (auto obj : Objects)
    {
        if (obj->isActive && obj->data.meshes.VCountTotal > 0)
        {
            if (!setted)
            {
                setted = true;
                Vmin = obj->data.meshes.Vmin;
                Vmax = obj->data.meshes.Vmax;
            }
            else
            {
                #ifdef USE_EIGEN
                for (int k = 0; k < 3; k++)
                {
                    Vmin(k) = min(Vmin(k), obj->data.Vmin(k));
                    Vmax(k) = max(Vmax(k), obj->data.Vmax(k));
                }
                #else
                Vmin = utils::sse::min(Vmin, obj->data.meshes.Vmin);
                Vmax = utils::sse::max(Vmax, obj->data.meshes.Vmax);
                #endif
            }
        }
    }
    //cout << "Vmin(" << Vmin(0) << "," << Vmin(1) << "," << Vmin(2) << ")" << endl;
    //cout << "Vmax(" << Vmax(0) << "," << Vmax(1) << "," << Vmax(2) << ")" << endl;
}

P3 ViewportObjectCollection::GetCentroid()
{
    int addedVcount = 0;
    DD sum0 = 0;
    DD sum1 = 0;
    DD sum2 = 0;
    for (auto obj : Objects)
    {
        if (obj->isActive)
        {
            for (auto& m : obj->data.meshes.meshes)
            {
                for (int i = 0; i < m.V.rows(); i++)
                {
                    P3 v = m.V.row(i);
                    sum0 += v(0);
                    sum1 += v(1);
                    sum2 += v(2);
                }
                addedVcount += m.V.rows();
            }
        }
    }
    if (addedVcount == 0)
    {
        return P3(0, 0, 0);
    }
    else
    {
        return P3(sum0 / addedVcount, sum1 / addedVcount, sum2 / addedVcount);
    }
}

void ViewportObjectCollection::Draw(Viewport& v) //calls when redraw of viewport is needed
{
    //
    // update dirty flag for all data objects 
    //
    if (v.isObjectsDataDirty)
    {
        for (auto obj : Objects)
        {
            obj->data.dirty = ViewportData::DIRTY_ALL;
        }
    }
    v.isObjectsDataDirty = false;


    //
    // sort base on z-order (closer object will draw last, to overwrap further ones)
    //

    // allocate/resize buffers for sorting
    int activeObjectsCount = 0;
    for (auto obj : Objects)
    {
        if (obj->isActive)
        {
            activeObjectsCount++;
        }
    }
    if (Draw_ActiveObjects.size() < activeObjectsCount) // realocate only if we have not enought space to store our sorting information
    {
        Draw_ActiveObjects.resize(activeObjectsCount);
        Draw_Z.resize(activeObjectsCount);
        Draw_Z_sort_indexes.resize(activeObjectsCount);
    }

    // get all z distances
    Matrix4d view_model = v.view*v.model;
    int index = 0;
    for (auto obj : Objects)
    {
        if (obj->isActive)
        {
            Draw_ActiveObjects[index] = obj;
            P3 objCenter = obj->data.meshes.VcenterProjected;
            Vector4d position_eye = view_model * Vector4d(objCenter(0), objCenter(1), objCenter(2), 1);
            Draw_Z[index] = position_eye(2); // z coordinate - distance to monitor
            index++;
        }
    }

    //sort
    // we can't use sort from utils - since our vector can have more elements than we need to sort (activeObjectsCount can be les than Draw_Z.size())
    // so lets sort manually
    iota(Draw_Z_sort_indexes.begin(), Draw_Z_sort_indexes.begin() + activeObjectsCount, 0);
    const vector<double>& Draw_Zp = Draw_Z;// required for lambda for next sort method
    sort(Draw_Z_sort_indexes.begin(), Draw_Z_sort_indexes.begin() + activeObjectsCount, [&Draw_Zp](unsigned i1, unsigned i2)
    {
        return Draw_Zp[i1] < Draw_Zp[i2];
    });

    //
    // initialize drawing
    //
    DrawViewportData_Begin(v);

    //
    // v1 - draw all objects (unsorted)
    //
    //for (auto obj : Objects)
    //{
    //    if (obj->isActive)
    //    {
    //        obj->Draw(v);
    //    }
    //}

    //
    // v2 - draw all objects (sorted)
    //
    for (int i = 0; i < activeObjectsCount; i++)
    {
        int indexSorted = Draw_Z_sort_indexes[i];
        Draw_ActiveObjects[indexSorted]->Draw(v);
        //DEBUG - draw only first few objects
        //if (i > 5) break;
    }
}


