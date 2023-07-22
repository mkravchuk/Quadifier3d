#pragma once
#include "TextRenderer.h"
#include "ViewerDrawObjects.h"

class OpenGL_Window;

// viewport for OpenGL_Window
class Viewport
{
public:
    ViewerDrawObjects draw;

    Viewport();
    II SizeOF() const;

    // Initialization
    void init();

    // Shutdown
    void shut();


    // ------------------- Camera control functions

    // Adjust the view to see the entire model
    void align_camera_center(const Point3d& Vmin, const Point3d& Vmax);

    // Determines how much to zoom and shift such that the mesh fills the unit
    // box (centered at the origin)
    void get_scale_and_shift_to_fit_mesh(
        const Point3d& Vmin,
        const Point3d& Vmax,
        double& zoom, Vector3d& translation);

    // ------------------- Drawing functions

    void UpdateOpenGLTransformationMatrices();
    void DrawAll();
    void ClearDrawObjects();

    // Trackball angle (quaternion)
    enum class RotationType
    {
        TRACKBALL = 0,
        TWO_AXIS_VALUATOR_FIXED_UP = 1,
        ScreenXY = 2
    };
    void set_rotation_type(const RotationType & value);

    // ------------------- Properties
    bool serializeCamera;
    bool serializeVisualizationOptions;
    void serializeOptions(string filename);
    void deserializeOptions(string filename);

    // Text rendering helper
    TextRenderer textrenderer;


    // Colors
    Color4f background_color;
    Color4f wireframe_color;



    // Mouse Rotation
    RotationType rotation_type;

    // Model viewing parameters
    double rotation_speed;
    Quaterniond model_rotation;
    double model_zoom;
    Vector3d model_translation;

    // Model viewing paramters (uv coordinates)
    double model_zoom_uv;
    Vector3d model_translation_uv;

    // Camera parameters
    double camera_zoom;
    bool orthographic;
    Vector3d camera_eye;
    Vector3d camera_up;
    Vector3d camera_center;
    double camera_view_angle;
    double camera_dnear;
    double camera_dfar;

    // Visualization options    
    bool isObjectsDataDirty; // caches must be refreshed - some option has changed (face_based, invert_normals)
    bool show_overlay;
    bool show_overlay_depth;
    bool show_texture;
    bool show_faces;
    bool show_wireframe;
    bool show_vertid;
    Color4d show_vertid_color;
    bool show_edgeid;
    Color4d show_edgeid_color;
    bool show_faceid;
    Color4d show_faceid_color;
    bool show_srfid;
    Color4d show_srfid_color;
    bool show_srfSerialNumber;
    Color4d show_srfSerialNumber_color;
    Color4d labels_default_color;
    bool invert_normals; // changing this options change also 'isDirty' to true
    bool crease_normals; // changing this options change also 'isDirty' to true
    D crease_normals_angle; // changing this options change also 'isDirty' to true
    bool show_XYZ;
    bool show_rotation_XYZ;
    bool show_rotation_center;

    Vector4d size; // Viewport size - double because we need support high dpi

    // Animation
    bool is_animating;
    double animation_max_fps;

    // Caches the two-norm between the min/max point of the bounding box
    double object_scale;

    // Save the OpenGL transformation matrices used for the previous rendering pass
    Matrix4d view;
    Matrix4d model;
    Matrix4d proj;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
