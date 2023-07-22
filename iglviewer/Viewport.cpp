#include "stdafx.h"
#include "Viewport.h"
#include <igl/quat_to_mat.h>
#include <igl/snap_to_fixed_up.h>
#include <igl/look_at.h>
#include <igl/frustum.h>
#include <igl/ortho.h>



//Mupoc - moved from Viewport.cpp -to solve issue with redefinition of serialize and deserialize in OpenGL_Window.obj and Viewport.obj
#ifdef ENABLE_SERIALIZATION
#include <igl/serialize.h>
namespace igl
{
    namespace serialization
    {
        void serialization(bool s, Viewport& obj, std::vector<char>& buffer)
        {
            if (obj.serializeCamera)
            {
                SERIALIZE_MEMBER(model_rotation);
                SERIALIZE_MEMBER(model_zoom);
                SERIALIZE_MEMBER(model_translation);

                SERIALIZE_MEMBER(model_zoom_uv);
                SERIALIZE_MEMBER(model_translation_uv);

                SERIALIZE_MEMBER(camera_zoom);
                SERIALIZE_MEMBER(object_scale);

                SERIALIZE_MEMBER(view);
                SERIALIZE_MEMBER(model);
                SERIALIZE_MEMBER(proj);
            }

            if (obj.serializeVisualizationOptions)
            {
                SERIALIZE_MEMBER(rotation_type);
                SERIALIZE_MEMBER(camera_view_angle);
                SERIALIZE_MEMBER(camera_dnear);
                SERIALIZE_MEMBER(camera_dfar);
                SERIALIZE_MEMBER(camera_eye);
                SERIALIZE_MEMBER(camera_center);
                SERIALIZE_MEMBER(camera_up);
                SERIALIZE_MEMBER(orthographic);

                SERIALIZE_MEMBER(background_color);
                SERIALIZE_MEMBER(wireframe_color);

                SERIALIZE_MEMBER(show_faces);
                SERIALIZE_MEMBER(show_wireframe);
                SERIALIZE_MEMBER(invert_normals);
                SERIALIZE_MEMBER(crease_normals);
                SERIALIZE_MEMBER(show_overlay);
                SERIALIZE_MEMBER(show_overlay_depth);
                SERIALIZE_MEMBER(show_vertid);
                SERIALIZE_MEMBER(show_edgeid);
                SERIALIZE_MEMBER(show_faceid);
                SERIALIZE_MEMBER(show_srfid);
                SERIALIZE_MEMBER(show_texture);
            }

            //SERIALIZE_MEMBER(point_size);
            //SERIALIZE_MEMBER(line_width);
            //SERIALIZE_MEMBER(is_animating);
            //SERIALIZE_MEMBER(animation_max_fps);
        }

        template<>
        void serialize(const Viewport& obj, std::vector<char>& buffer)
        {
            serialization(true, const_cast<Viewport&>(obj), buffer);
        }

        template<>
        void deserialize(Viewport& obj, const std::vector<char>& buffer)
        {
            serialization(false, obj, const_cast<std::vector<char>&>(buffer));
        }
    }
}

//namespace igl {
//    namespace serialization {
//
//        void serialization(bool s, ViewportData& obj, std::vector<char>& buffer)
//        {
//            SERIALIZE_MEMBER(V);
//            SERIALIZE_MEMBER(F);
//
//            SERIALIZE_MEMBER(F_normals);
//            SERIALIZE_MEMBER(F_material_ambient);
//            SERIALIZE_MEMBER(F_material_diffuse);
//            SERIALIZE_MEMBER(F_material_specular);
//
//            SERIALIZE_MEMBER(V_normals);
//            SERIALIZE_MEMBER(V_material_ambient);
//            SERIALIZE_MEMBER(V_material_diffuse);
//            SERIALIZE_MEMBER(V_material_specular);
//
//            SERIALIZE_MEMBER(V_uv);
//            SERIALIZE_MEMBER(F_uv);
//
//            SERIALIZE_MEMBER(texture_R);
//            SERIALIZE_MEMBER(texture_G);
//            SERIALIZE_MEMBER(texture_B);
//
//            SERIALIZE_MEMBER(lines);
//            SERIALIZE_MEMBER(points);
//
//            SERIALIZE_MEMBER(labels_positions);
//            SERIALIZE_MEMBER(labels_strings);
//            SERIALIZE_MEMBER(labels_color);
//
//            SERIALIZE_MEMBER(dirty);
//
//            SERIALIZE_MEMBER(face_based);
//        }
//
//        template<>
//        void serialize(const ViewportData& obj, std::vector<char>& buffer)
//        {
//            serialization(true, const_cast<ViewportData&>(obj), buffer);
//        }
//
//        template<>
//        void deserialize(ViewportData& obj, const std::vector<char>& buffer)
//        {
//            serialization(false, obj, const_cast<std::vector<char>&>(buffer));
//            obj.dirty = ViewportData::DIRTY_ALL;
//        }
//    }
//}
#endif


void Viewport::serializeOptions(string filename)
{
    #ifdef ENABLE_SERIALIZATION
    igl::serialize(*this, "Core", filename.c_str(), true);
    #endif
}

void Viewport::deserializeOptions(string filename)
{
    #ifdef ENABLE_SERIALIZATION
    igl::deserialize(*this, "Core", filename.c_str());
    #endif
}

void Viewport::align_camera_center(const Point3d& Vmin, const Point3d& Vmax)
{
    //cout << endl;
    //cout << "object_scale = " << object_scale << endl;
    //cout << "camera_zoom = " << camera_zoom << endl;
    //cout << "model_zoom = " << model_zoom << endl;
    get_scale_and_shift_to_fit_mesh(Vmin, Vmax, model_zoom, model_translation);
    object_scale = (Vmax - Vmin).norm();
    camera_zoom = 1;
    //cout << "model_zoom new = " << model_zoom << endl;
}

void Viewport::get_scale_and_shift_to_fit_mesh(
    const Point3d& Vmin,
    const Point3d& Vmax,
    double& zoom, Vector3d& translation)
{
    Point3d centroid = (Vmin + Vmax) / 2;
    translation = -centroid;
    zoom = 2.0 / (Vmax - Vmin).array().abs().maxCoeff();
}


void Viewport::UpdateOpenGLTransformationMatrices()
{
    model = Matrix4d::Identity();
    view = Matrix4d::Identity();
    proj = Matrix4d::Identity();

    // Set view
    igl::look_at(camera_eye, camera_center, camera_up, view);

    double width = size(2) - size(0);
    double height = size(3) - size(1);

    // Set projection
    if (orthographic)
    {
        double length = (camera_eye - camera_center).norm();
        double h =tan(camera_view_angle / 360.0f * M_PI) * (length);
        igl::ortho(-h * width / height, h*width / height, -h, h, camera_dnear, camera_dfar, proj);
    }
    else
    {
        double fH =  tan(camera_view_angle / 360.0f * M_PI) * camera_dnear;
        double fW = fH * width / height;
        igl::frustum(-fW, fW, -fH, fH, camera_dnear, camera_dfar, proj);
    }
    // end projection

    // Set model transformation
    double mat[16];
    igl::quat_to_mat(model_rotation.coeffs().data(), mat);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            model(i, j) = mat[i + 4 * j];

    Matrix4d modelBeforeZoom = model;;

    // Why not just use Transform<double,3,Projective> for model...?
    model.topLeftCorner(3, 3) *= camera_zoom;
    model.topLeftCorner(3, 3) *= model_zoom;
    model.col(3).head(3) += model.topLeftCorner(3, 3)*model_translation;
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
    //cout << endl << endl << endl;
    //cout << "camera_zoom=" << camera_zoom << "   model_zoom=" << model_zoom<< endl;
    //coutM("view", view);
    //coutM("proj", proj);
    //coutM("modelBeforeZoom", modelBeforeZoom);
    //coutM("model", model);
}

void Viewport::DrawAll()
{
    if (show_XYZ)
    {
        Color3d colorXYZ = Color3d(1, 0, 0);
        draw.AddPoint(P3(0, 0, 0), colorXYZ);
        draw.AddEdge(P3(0, 0, 0), P3(1, 0, 0));
        draw.AddEdge(P3(0, 0, 0), P3(0, 1, 0));
        draw.AddEdge(P3(0, 0, 0), P3(0, 0, 1));
        draw.AddLabel(P3(0, 0, 0), "XYZ", colorXYZ);
        draw.AddLabel(P3(1, 0, 0), "X", colorXYZ);
        draw.AddLabel(P3(0, 1, 0), "Y", colorXYZ);
        draw.AddLabel(P3(0, 0, 1), "Z", colorXYZ);
    }
    if (show_rotation_XYZ)
    {
        Color3d colorxyz = Color3d(0, 0, 1);
        P3 c = convertEigenToP3(model_translation);
        draw.AddPoint(c + P3(0, 0, 0), colorxyz);
        draw.AddEdge(c + P3(0, 0, 0), c + V3(1, 0, 0), colorxyz);
        draw.AddEdge(c + P3(0, 0, 0), c + V3(1, 0, 0), colorxyz);
        draw.AddEdge(c + P3(0, 0, 0), c + V3(1, 0, 0), colorxyz);
        draw.AddLabel(c + P3(0, 0, 0), "xyz", colorxyz);
        draw.AddLabel(c + P3(1, 0, 0), "x", colorxyz);
        draw.AddLabel(c + P3(0, 1, 0), "y", colorxyz);
        draw.AddLabel(c + P3(0, 0, 1), "z", colorxyz);
    }
    if (show_rotation_center)
    {
        P3 c = convertEigenToP3(model_translation);
        draw.AddPoint(c, Color3d(0, 0, 1));
        draw.AddLabel(c, to_string(c) + " model_translation");
    }
}

void Viewport::ClearDrawObjects()
{
    draw.Clear();
}

void Viewport::set_rotation_type(
    const Viewport::RotationType & value)
{
    const RotationType old_rotation_type = rotation_type;
    rotation_type = value;
    if (rotation_type == RotationType::TWO_AXIS_VALUATOR_FIXED_UP &&
        old_rotation_type != RotationType::TWO_AXIS_VALUATOR_FIXED_UP)
    {
        igl::snap_to_fixed_up(Quaterniond(model_rotation), model_rotation);
    }
}




Viewport::Viewport()
{
    // Default colors
    background_color << 0.3f, 0.3f, 0.5f, 1.0f;
    //background_color << 1.0f, 1.0f, 1.0f, 1.0f;
    wireframe_color << 0.0f, 0.0f, 0.0f, 1.0f;



    // Default trackball
    rotation_speed = 4.0;
    model_rotation = Quaterniond::Identity();
    //set_rotation_type(Viewport::RotationType::TRACKBALL); // rotate on X Y Z. X and Y rotation depends on mouse position
    //set_rotation_type(Viewport::RotationType::TWO_AXIS_VALUATOR_FIXED_UP); // rotate on fixed X Y
    set_rotation_type(Viewport::RotationType::ScreenXY); // rotate on X Y. X and Y rotation NOT depends on mouse position


    // Defalut model viewing parameters
    model_zoom = 1.0f;
    model_translation << 0, 0, 0;

    // Camera parameters
    camera_zoom = 1.0f;
    orthographic = false;
    camera_view_angle = 45.0;
    camera_dnear = 1.0;
    camera_dfar = 100.0;
    camera_eye << 0, 0, 5;
    camera_center << 0, 0, 0;
    camera_up << 0, 1, 0;

    // Default visualization options
    isObjectsDataDirty = true;
    show_faces = true;
    show_wireframe = true;
    invert_normals = false;
    crease_normals = true;
    crease_normals_angle = 30;
    show_overlay = true;
    show_overlay_depth = false;
    show_vertid = false;
    show_edgeid = false;
    show_faceid = false;
    show_srfid = false;
    show_srfSerialNumber = false;
    show_texture = false;
    show_XYZ = true;
    show_rotation_XYZ = true;
    show_rotation_center = true;

    is_animating = false;
    animation_max_fps = 30.;
}

II Viewport::SizeOF() const
{
    II r = sizeof(Viewport);
    r += draw.SizeOF();

    return r;
}

void Viewport::init()
{
    textrenderer.Init();
}

void Viewport::shut()
{
    textrenderer.Shut();
}