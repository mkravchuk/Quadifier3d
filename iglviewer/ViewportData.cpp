#include "stdafx.h"
#include "ViewportData.h"
#include "ViewportData_Mesh.h"
#include "OpenGL_Options.h"
#include <igl/parula.h>

#define options openglOptions


ViewportData::ViewportData(const ViewportData_Meshes& _meshes)
    : dirty(DIRTY_ALL), meshes(_meshes)
{
    clearDraws();
    grid_texture();
    dirty |= DIRTY_FACE | DIRTY_POSITION | DIRTY_NORMAL;
}


void ViewportData::set_uv(const MatrixXf& UV)
{
    using namespace std;
    if (UV.rows() == meshes.VCountTotal)
    {
        V_uv = UV;
    }
    else
        cerr << "ERROR (set_UV): Please provide uv per vertex." << endl;;
    dirty |= DIRTY_UV;
}

void ViewportData::set_uv(const MatrixXf& UV_V, const MatrixXi& UV_F)
{
    V_uv = UV_V.block(0, 0, UV_V.rows(), 2);
    F_uv = UV_F;
    dirty |= DIRTY_UV;
}


void ViewportData::set_texture(
    const Matrix<unsigned char, Dynamic, Dynamic>& R,
    const Matrix<unsigned char, Dynamic, Dynamic>& G,
    const Matrix<unsigned char, Dynamic, Dynamic>& B)
{
    texture_R = R;
    texture_G = G;
    texture_B = B;
    dirty |= DIRTY_TEXTURE;
}


void ViewportData::clearDraws()
{    
    if (options.Texture.Enabled)
    {
        V_uv = MatrixXf(0, 2);
        F_uv = MatrixXi(0, 3);
        dirty |= DIRTY_UV;
    }

    draws.clear();

    dirty |= DIRTY_COLOR;
    dirty |= DIRTY_OVERLAY_POINTS;
    dirty |= DIRTY_OVERLAY_LINES;
}


void ViewportData::grid_texture()
{
    if (!options.Texture.Enabled) return;

    // Don't do anything for an empty mesh
    if (meshes.VCountTotal == 0)
    {
        V_uv.resize(0, 2);
        return;
    }
    if (V_uv.rows() == 0)
    {
        //V_uv = V.block(0, 0, V.rows(), 2);
        V_uv.resize(meshes.VCountTotal, 2);
        int uv_index = 0;
        for (int im = 0; im < meshes.meshes.size(); im++)
        {
            const P3s& V = meshes.meshes[im].V;
            for (int i = 0; i < V.rows(); i++)
            {
                V_uv(uv_index + i, 0) = V(i, 0);
                V_uv(uv_index + i, 1) = V(i, 1);
            }
            uv_index += V.rows();
        }
        V_uv.col(0) = V_uv.col(0).array() - V_uv.col(0).minCoeff();
        V_uv.col(0) = V_uv.col(0).array() / V_uv.col(0).maxCoeff();
        V_uv.col(1) = V_uv.col(1).array() - V_uv.col(1).minCoeff();
        V_uv.col(1) = V_uv.col(1).array() / V_uv.col(1).maxCoeff();
        V_uv = V_uv.array() * 10;
        dirty |= DIRTY_TEXTURE;
    }

    int size = 128;
    int size2 = size / 2;
    texture_R.resize(size, size);
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            texture_R(i, j) = 0;
            if ((i < size2 && j < size2) || (i >= size2 && j >= size2))
                texture_R(i, j) = 255;
        }
    }

    texture_G = texture_R;
    texture_B = texture_R;
    dirty |= DIRTY_TEXTURE;
}



II ViewportData::SizeOF() const
{
    II r = sizeof(ViewportData);

    r += meshes.SizeOF();
    for (auto& draw: draws)
    {
        r += draw.second.get().SizeOF();
    }

    r += V_uv.size() * sizeof(double);
    r += F_uv.size() * sizeof(int);

    r += texture_R.size() * sizeof(unsigned);
    r += texture_G.size() * sizeof(unsigned);
    r += texture_B.size() * sizeof(unsigned);

    return r;
}


bool ViewportData::IsEmpty() const
{
    if (!meshes.IsEmpty()) return false;
    for (auto& draw : draws)
    {
        const ViewerDrawObjects& d = draw.second.get();
        if (!d.IsEmpty()) return false;
    }
    return true;
}
