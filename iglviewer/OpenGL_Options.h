#pragma once

//  MSAA samples for Antialising
// 4 - is gold balance between quality and speed(4 - 90fpd, 8 - 70fps)
// The more smaples, the slower app and the more takes memory
// 0 should be with 'GL_LINE_SMOOTH_enabled=true', to compensate smooting
enum class OpenGLAntialisingSamples : int
{
    _0 = 0,// fast  
    _4 = 4,//  
    _8 = 8 // very slow, cold be 2.5 slower from _0 with lineSmooth
};

struct OpenGL_Options_WindowHint
{
    // i have Firepro m7820:   DirectX 11.2 (11_0),  OpenGL 4.4,  OpenCL 1.2,  Shader Model 5.0
    // OpenGL 4.4 API is supported by 5th and 6th generation Intel Core processors
    // glDrawTransformFeedback (OpenGL 4.1), 
    // glClearTexImage (OpenGL 4.4)        
    //int OpenGL_version_MAJOR = 4;
    //int OpenGL_version_MINOR = 4;
    int OpenGL_version_MAJOR = 3;
    int OpenGL_version_MINOR = 3;
    string slhl_version = "330";

    // v0 - supporting stencil - default
    //int DEPTH_BITS = 24; // 24 is enought, and is good because 24+8 = 32  (24-bits will be used for depth, and the remaining 8-bits for stencil)
    //int STENCIL_BITS = 8; // 8 - is good because 24+8 = 32  (24-bits will be used for depth, and the remaining 8-bits for stencil)
    // v1 - no-supporting stencil 
    int DEPTH_BITS = 32;  // must be 32 bits to achieve good quality, otherwise there will be arifacts
    int STENCIL_BITS = 0; 
    OpenGLAntialisingSamples SAMPLES = OpenGLAntialisingSamples::_0; // samples count when creating opengl window.  The more samples, the slower app and the more takes memory

    bool recreateWindowAfterLoadExtensions = true; // recreate window, to apply better context. must be setted if SAMPLES != _0 (takes 0.05 sec on application startup)
};

struct OpenGL_Options_Performance
{
    bool use_viewport_layers; // improves rotation by 10%-30% for very large models, and decrease rotation by 10% for smaller models
    bool use_nanogui_layers;  // improves rotation by 10-30%
    bool unindex_depth_test; // test - slows down rotation by 30% because to many vertexes are calculated - indexes vertexes works faster
    bool compactColors; // pass color as 1 int instead of 3 floats //TODO implement
    bool compactNormals; // store normal as 4-th value of position (as position.w) - performance improvement by 1%-3%
};

struct OpenGL_Options_Offsets
{
    float wireframe_factor;
    float wireframe_units;
};

struct OpenGL_Options_Sizes
{
    // Sizes (point size, line width, viewport size)
    double point_size;
    double point_size_MAX;
    double line_size;
    double lineBold_size;
    double wireframe_size;
    double wireframe_pointsize_factor; //if wireframe is visible - increase size of points to make them more visible
    double wireframe_linewidth_factor; //if wireframe is visible - increase thikness of lines to make them more visible
};

enum class OpenGLAntialisingQuality
{
    low, high
};

struct OpenGL_Options_Antialising
{
    bool Enabled;
    bool LineSmoothingEnabled; // smooth lines - very good quality, compareble with MSAA samples = 8
    bool LineSmoothingEnabled_inHiddenLayers;
    bool PolygonSmoothingEnabled;
    OpenGLAntialisingQuality LineSmoothingEnabled_quality;
};


struct OpenGL_Options_Light
{
    double shininess; // how bright light emits. Transparent override this value
    double shininess_intensity; // how big difference from direct and indirect light
    Vector3d light_position;
    double lighting_factor; // Transparent override this value
    double alpha_BackSideCoeff; // Transparent override this value
};

struct OpenGL_Options_Colors
{
    bool wireframe_blend; // blend mesh color with surface color
    double backSideColorFactor; // how upfront face color will differ from front face color. for example value of 0.5 will half color produced for front face, thus make it more dark
};
struct OpenGL_Options_Transparent_Layer0Depth
{
    bool Enabled;
};
struct OpenGL_Options_Transparent_Layer1Hidden
{
    bool Enabled;
    double alpha_lines;
    double alpha_back;
    double alpha_front;
    double pointsize_factor;
};
struct OpenGL_Options_Transparent_Layer2Visible
{
    bool Enabled_Back;
    bool Enabled_Front;
    double alpha_back;
    double alpha_front;
};
struct OpenGL_Options_Transparent
{
    enum class SortAlgorithm
    {
        Simple, KeyValue, Batch
    };
    vector<string> SortAlgorithmStr = { "Simple", "KeyValue", "Batch" };
    bool Enabled;
    bool sortTriangles;
    bool sortTriangles_showtime_inconsole;
    SortAlgorithm sortTriangles_sort_algorithm;

    double alpha_BackSideCoeff; // overrides OpenGL_Options_Light.shininess
    double shininess;// overrides OpenGL_Options_Light.shininess
    double lighting_factor; // overrides OpenGL_Options_Light.lighting_factor
    OpenGL_Options_Transparent_Layer0Depth Layer0Depth;
    OpenGL_Options_Transparent_Layer1Hidden Layer1Hidden;
    OpenGL_Options_Transparent_Layer2Visible Layer2Visible;
};

struct OpenGL_Options_Texture
{
    bool Enabled;
};

struct OpenGL_Options
{
    OpenGL_Options_WindowHint WindowHint;
    OpenGL_Options_Performance Performance;
    OpenGL_Options_Offsets Offsets;
    OpenGL_Options_Sizes Sizes;
    OpenGL_Options_Antialising Antialiasing;
    OpenGL_Options_Light Light;
    OpenGL_Options_Colors Colors;
    OpenGL_Options_Transparent Transparent;
    OpenGL_Options_Texture Texture;
};


extern OpenGL_Options openglOptions;
