#pragma once

class ViewportData_Meshes;
class ViewportData_Mesh;

// Stores all the data that should be visualized
class ViewportData
{
public:

    enum DirtyFlags
    {
        DIRTY_NONE = 0x0000,
        DIRTY_POSITION = 0x0001,
        DIRTY_NORMAL = 0x0002,
        DIRTY_UV = 0x0004,
        DIRTY_COLOR_AMBIENT = 0x0008,
        DIRTY_COLOR_DIFFUSE = 0x0010,
        DIRTY_COLOR_SPECULAR = 0x0020,
        DIRTY_COLOR = DIRTY_COLOR_AMBIENT | DIRTY_COLOR_DIFFUSE | DIRTY_COLOR_SPECULAR,
        DIRTY_TEXTURE = 0x0040,
        DIRTY_FACE = 0x0080,
        DIRTY_MESH = 0x00FF,
        DIRTY_OVERLAY_LINES = 0x0100,
        DIRTY_OVERLAY_POINTS = 0x0200,
        DIRTY_ALL = 0x03FF
    };

    // Marks dirty buffers that need to be uploaded to OpenGL
    uint32_t dirty;
    const ViewportData_Meshes& meshes;
    vector<pair<int, reference_wrapper<const ViewerDrawObjects>>> draws; // list of pairs {ObjectId, ViewerDrawObjects},  ObjectId can be -1 if it is not belong to any object and thus is not linked to any mesh

    ViewportData(const ViewportData_Meshes& meshes);
    II SizeOF() const;
    bool IsEmpty() const;

    void clearDraws();

    //
    // Inputs:
    //   UV  #V by 2 list of UV coordinates (indexed by F)
    void set_uv(const MatrixXf& UV);
    // Set per-corner UV coordinates
    //
    // Inputs:
    //   UV_V  #UV by 2 list of UV coordinates
    //   UV_F  #F by 3 list of UV indices into UV_V
    void set_uv(const MatrixXf& UV_V, const MatrixXi& UV_F);
    // Set the texture associated with the mesh.
    //
    // Inputs:
    //   R  width by height image matrix of red channel
    //   G  width by height image matrix of green channel
    //   B  width by height image matrix of blue channel
    //
    void set_texture(
        const Matrix<unsigned char, Dynamic, Dynamic>& R,
        const Matrix<unsigned char, Dynamic, Dynamic>& G,
        const Matrix<unsigned char, Dynamic, Dynamic>& B);
    
    void grid_texture();// Generates a default grid texture
    MatrixXf V_uv; // UV parametrization  UV vertices
    MatrixXi F_uv; // UV parametrization  optional faces for UVs

    // Texture
    Matrix<unsigned char, Dynamic, Dynamic> texture_R;
    Matrix<unsigned char, Dynamic, Dynamic> texture_G;
    Matrix<unsigned char, Dynamic, Dynamic> texture_B;
};

