#pragma once

// Converts mesh data inside a ViewportData class in an OpenGL compatible format
// The class includes a shader and the opengl calls to plot the data

#include "OpenGL_Program.h"
#include "ViewportData.h"
#include "OpenGL_TransformFeedback.h"

struct TransparencySort_CachedData
{

    vector<unsigned int> F_vbo_duplicate; // used in sortFacesForTransparencyDraw to avoid memory allocation (speed optimization)
    vector<float> Vdistances; // used in sortFacesForTransparencyDraw to avoid memory allocation (speed optimization)
    vector<float> dist_to_faces; // used in sortFacesForTransparencyDraw to avoid memory allocation (speed optimization)
    vector<int> dist_to_faces_sorted_indexes; // used in sortFacesForTransparencyDraw to avoid memory allocation (speed optimization)
    vector<pair<float, int>> dist_to_faces_keyvalue; // used in sortFacesForTransparencyDraw to avoid memory allocation (speed optimization)
    chrono::high_resolution_clock::time_point nextTimeAllowedToSort = chrono::high_resolution_clock::now();

    int latcall_V_vbo_size;
    int latcall_F_vbo_size;
    Matrix4f latcall_model;
    Matrix4f latcall_view;
};


//
// TODO use class with built-in common methods: .\_libs\libgl\external\nanogui\include\nanogui\glutil.h
//
class ViewportData_OpenGLshaders
{
public:

    TransparencySort_CachedData TransparencySort_CachedData;

    OpenGL_Program program_mesh;
    OpenGL_Program program_mesh_for_cacheColor;
    OpenGL_Program program_mesh_for_dephTesting;
    OpenGL_Program program_overlay_lines;
    OpenGL_Program program_overlay_points;

    GLuint vbo_V_uv; // UV coordinates for the current mesh (#V x 2)
    OpenGL_TransformFeedback tbo_VertexShader_POINTS; // Transform feedback buffer for Vertex shader (per vertex)
    GLuint vbo_V_unindexed;

    GLuint vbo_F; // Faces of the mesh (#F x 3)
    GLuint vbo_tex; // Texture

    GLuint vbo_lines_F;         // Indices of the line overlay
    GLuint vbo_linesBold_F;         // Indices of the line overlay
    GLuint vbo_points_F;        // Indices of the point overlay

    // Temporary copy of the content of each VBO
    MatrixXf V;
    MatrixXf V_unindexed;
    OpenGL_VertexBufferObject shared_vbo__position;
    MatrixXf V_uv;
    MatrixXf V_normals;
    MatrixXf V_color;

    MatrixXf lines_V; // store normal and bold lines
    MatrixXf lines_V_colors;  // store normal and bold lines
    MatrixXf points_V;
    MatrixXf points_V_colors;

    int tex_u;
    int tex_v;
    Vector<char> tex;

    MatrixXui F;
    MatrixXui lines_F;       // store indexes for normal lines
    MatrixXui linesBold_F; // store indexes for bold lines
    MatrixXui points_F;

    // Marks dirty buffers that need to be uploaded to OpenGL
    uint32_t dirty;

    // Initialize shaders and buffers
    void init();

    // Release all resources
    void free();

    // restart shader by doing simple free() + init()
    void reinit();

    // Create a new set of OpenGL buffer objects
    void init_buffers();

    // Update contents from a 'Data' instance
    void set_data(const ViewportData &data, bool invert_normals, bool crease_normals);

    void bind_mesh_for_cacheColor();
    void bind_mesh_for_dephTesting();

    // Bind the underlying OpenGL buffer objects for subsequent mesh draw calls
    void bind_mesh(bool showTexture);

    /// Draw the currently buffered mesh (either solid or wireframe)
    void draw_mesh(bool solid, GLenum front_and_back);

    // Bind the underlying OpenGL buffer objects for subsequent line overlay draw calls
    bool bind_overlay_lines_program();
    void bind_overlay_lines_linesNormal(bool is_dirty);
    void bind_overlay_lines_linesBold(bool is_dirty);

    /// Draw the currently buffered line overlay
    void draw_overlay_lines();
    void draw_overlay_linesBold();

    // Bind the underlying OpenGL buffer objects for subsequent point overlay draw calls
    void bind_overlay_points();

    /// Draw the currently buffered point overlay
    void draw_overlay_points();

    ViewportData_OpenGLshaders();

    II SizeOF() const;

private:
    #ifdef USE_EIGEN
    #if EROW
    void set_data__RowMajor(const ViewportData &data, bool invert_normals);
    #else
    void set_data__ColMajor(const ViewportData &data, bool invert_normals);
    #endif
    #else
    void set_data__V3P3(const ViewportData &data, bool invert_normals, bool crease_normals);
    #endif
};
