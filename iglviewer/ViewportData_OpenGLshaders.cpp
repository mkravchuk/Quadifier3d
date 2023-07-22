#include "stdafx.h"
#include "ViewportData_OpenGLshaders.h"
#include "ViewportData.h"
#include "OpenGL_Options.h"
#include "ViewportData_Mesh.h"


#define options openglOptions


void ViewportData_OpenGLshaders::init_buffers()
{
    glGenBuffers(1, &vbo_V_uv);
    tbo_VertexShader_POINTS.Delete();


    glGenBuffers(1, &vbo_V_unindexed);
    glGenBuffers(1, &vbo_F);
    glGenTextures(1, &vbo_tex);

    // Line overlay
    glGenBuffers(1, &vbo_lines_F);
    glGenBuffers(1, &vbo_linesBold_F);

    // Point overlay
    glGenBuffers(1, &vbo_points_F);

    dirty = ViewportData::DIRTY_ALL;
}


#ifdef USE_EIGEN
#if EROW

template<typename sourceInternalType>
void copy3times(MatrixXf& dest, const Matrix<sourceInternalType, Dynamic, Dynamic>& source)
{
    dest.resize(source.rows() * 3, source.cols());
    float* pDest = dest.data();
    const sourceInternalType* pSource = source.data();
    for (int i = 0; i < source.rows(); ++i)
    {
        float r = static_cast<float>(*(pSource + 0));
        float g = static_cast<float>(*(pSource + 1));
        float b = static_cast<float>(*(pSource + 2));
        *(pDest + 0) = r;
        *(pDest + 1) = g;
        *(pDest + 2) = b;
        *(pDest + 3) = r;
        *(pDest + 4) = g;
        *(pDest + 5) = b;
        *(pDest + 6) = r;
        *(pDest + 7) = g;
        *(pDest + 8) = b;
        pDest += 9;
        pSource += 3;
    }
};

template<typename sourceInternalType>
void copy3times_lookup_data_F(MatrixXf& dest, const Matrix<sourceInternalType, Dynamic, Dynamic>& source, const MatrixXi& dataF)
{
    //V_ambient_vbo.resize(data.F.rows() * 3, 3);
    //for (int i = 0; i < data.F.rows(); ++i)
    //    for (int j = 0; j < 3; ++j)
    //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));

    dest.resize(dataF.rows() * 3, 3);
    float* pDest = dest.data();
    const sourceInternalType* pSource = source.data();
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int row0 = *(pF + 0);
        int row1 = *(pF + 1);
        int row2 = *(pF + 2);
        *(pDest + 0) = static_cast<float>(*(pSource + row0 * 3 + 0));
        *(pDest + 1) = static_cast<float>(*(pSource + row0 * 3 + 1));
        *(pDest + 2) = static_cast<float>(*(pSource + row0 * 3 + 2));
        *(pDest + 3) = static_cast<float>(*(pSource + row1 * 3 + 0));
        *(pDest + 4) = static_cast<float>(*(pSource + row1 * 3 + 1));
        *(pDest + 5) = static_cast<float>(*(pSource + row1 * 3 + 2));
        *(pDest + 6) = static_cast<float>(*(pSource + row2 * 3 + 0));
        *(pDest + 7) = static_cast<float>(*(pSource + row2 * 3 + 1));
        *(pDest + 8) = static_cast<float>(*(pSource + row2 * 3 + 2));
        pDest += 9;
        pF += 3;
    }
};
template<class T>
void copy3times_lookup_data_F_P3V3(MatrixXf& dest, const T& source, const MatrixXi& dataF)
{
    //V_ambient_vbo.resize(data.F.rows() * 3, 3);
    //for (int i = 0; i < data.F.rows(); ++i)
    //    for (int j = 0; j < 3; ++j)
    //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));

    dest.resize(dataF.rows() * 3, 3);
    float* pDest = dest.data();
    const float* pSource = source.data();
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int row0 = *(pF + 0);
        int row1 = *(pF + 1);
        int row2 = *(pF + 2);
        *(pDest + 0) = static_cast<float>(*(pSource + row0 * 3 + 0));
        *(pDest + 1) = static_cast<float>(*(pSource + row0 * 3 + 1));
        *(pDest + 2) = static_cast<float>(*(pSource + row0 * 3 + 2));
        *(pDest + 3) = static_cast<float>(*(pSource + row1 * 3 + 0));
        *(pDest + 4) = static_cast<float>(*(pSource + row1 * 3 + 1));
        *(pDest + 5) = static_cast<float>(*(pSource + row1 * 3 + 2));
        *(pDest + 6) = static_cast<float>(*(pSource + row2 * 3 + 0));
        *(pDest + 7) = static_cast<float>(*(pSource + row2 * 3 + 1));
        *(pDest + 8) = static_cast<float>(*(pSource + row2 * 3 + 2));
        pDest += 9;
        pF += 3;
    }
};
template<typename sourceInternalType>
void copy3times_lookup_data_F2x(MatrixXf& dest, const Matrix<sourceInternalType, Dynamic, Dynamic>& source, const MatrixXi& dataF)
{
    //V_ambient_vbo.resize(data.F.rows() * 3, 3);
    //for (int i = 0; i < data.F.rows(); ++i)
    //    for (int j = 0; j < 3; ++j)
    //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));

    dest.resize(dataF.rows() * 3, 2);
    float* pDest = dest.data();
    const sourceInternalType* pSource = source.data();
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int row0 = *(pF + 0);
        int row1 = *(pF + 1);
        int row2 = *(pF + 2);
        *(pDest + 0) = static_cast<float>(*(pSource + row0 * 2 + 0));
        *(pDest + 1) = static_cast<float>(*(pSource + row0 * 2 + 1));
        *(pDest + 2) = static_cast<float>(*(pSource + row1 * 2 + 0));
        *(pDest + 3) = static_cast<float>(*(pSource + row1 * 2 + 1));
        *(pDest + 4) = static_cast<float>(*(pSource + row2 * 2 + 0));
        *(pDest + 5) = static_cast<float>(*(pSource + row2 * 2 + 1));
        pDest += 6;
        pF += 3;
    }
};

void ViewportData_OpenGLshaders::set_data__RowMajor(const ViewportData &data, bool invert_normals)
{
    bool per_corner_uv = (data.F_uv.rows() == data.F.rows());
    bool per_corner_normals = (data.F_normals.rows() == 3 * data.F.rows());

    dirty |= data.dirty;

    //MatrixXf dataVfloat = data.V.cast<float>();



    if (data.face_based)
    {
        if (dirty & ViewportData::DIRTY_POSITION)
        {
            //V_vbo.resize(data.F.rows() * 3, 3);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_vbo.row(i * 3 + j) = data.V.row(data.F(i, j)).cast<float>();
            copy3times_lookup_data_F(V_vbo, data.V, data.F);
        }

        if (dirty & ViewportData::DIRTY_AMBIENT)
        {
            //V_ambient_vbo.resize(data.F.rows() * 3, 3);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_ambient_vbo.row(i * 3 + j) = data.F_material_ambient.row(i);
            copy3times(V_ambient_vbo, data.F_material_ambient);
        }

        if (dirty & ViewportData::DIRTY_DIFFUSE)
        {
            //V_diffuse_vbo.resize(data.F.rows() * 3, 3);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_diffuse_vbo.row(i * 3 + j) = data.F_material_diffuse.row(i);
            copy3times(V_diffuse_vbo, data.F_material_diffuse);
        }

        if (dirty & ViewportData::DIRTY_SPECULAR)
        {
            //V_specular_vbo.resize(data.F.rows() * 3, 3);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_specular_vbo.row(i * 3 + j) = data.F_material_specular.row(i);
            copy3times(V_specular_vbo, data.F_material_specular);
        }

        if (dirty & ViewportData::DIRTY_NORMAL)
        {
            //V_normals_vbo.resize(data.F.rows() * 3, 3);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_normals_vbo.row(i * 3 + j) =
            //        per_corner_normals ?
            //        data.F_normals.row(i * 3 + j).cast<float>() :
            //        data.F_normals.row(i).cast<float>();

            if (per_corner_normals)
                V_normals_vbo = data.F_normals.cast<float>();
            else
                copy3times(V_normals_vbo, data.F_normals);

            if (invert_normals)
                V_normals_vbo = -V_normals_vbo;
        }

        if (dirty & ViewportData::DIRTY_FACE)
        {
            F_vbo.resize(data.F.rows(), 3);
            for (int i = 0; i < data.F.rows(); ++i)
                F_vbo.row(i) << i * 3 + 0, i * 3 + 1, i * 3 + 2;
        }

        if (dirty & ViewportData::DIRTY_UV)
        {
            //V_uv_vbo.resize(data.F.rows() * 3, 2);
            //for (int i = 0; i < data.F.rows(); ++i)
            //    for (int j = 0; j < 3; ++j)
            //        V_uv_vbo.row(i * 3 + j) = data.V_uv.row(per_corner_uv ? data.F_uv(i, j) : data.F(i, j)).cast<float>();
            if (per_corner_uv)
                copy3times_lookup_data_F2x(V_uv_vbo, data.V_uv, data.F_uv);
            else
                copy3times_lookup_data_F2x(V_uv_vbo, data.V_uv, data.F);
        }
    }
    else
    {
        if (per_corner_uv)
        {
            // Per vertex properties with per corner UVs
            if (dirty & ViewportData::DIRTY_POSITION)
            {
                //V_vbo.resize(data.F.rows() * 3, 3);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_vbo.row(i * 3 + j) = data.V.row(data.F(i, j)).cast<float>();
                copy3times_lookup_data_F(V_vbo, data.V, data.F);
            }

            if (dirty & ViewportData::DIRTY_AMBIENT)
            {
                //V_ambient_vbo.resize(data.F.rows() * 3, 3);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));
                copy3times_lookup_data_F(V_ambient_vbo, data.V_material_ambient, data.F);
            }

            if (dirty & ViewportData::DIRTY_DIFFUSE)
            {
                //V_diffuse_vbo.resize(data.F.rows() * 3, 3);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_diffuse_vbo.row(i * 3 + j) = data.V_material_diffuse.row(data.F(i, j));
                copy3times_lookup_data_F(V_diffuse_vbo, data.V_material_diffuse, data.F);
            }

            if (dirty & ViewportData::DIRTY_SPECULAR)
            {
                //V_specular_vbo.resize(data.F.rows() * 3, 3);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_specular_vbo.row(i * 3 + j) = data.V_material_specular.row(data.F(i, j));
                copy3times_lookup_data_F(V_specular_vbo, data.V_material_specular, data.F);
            }

            if (dirty & ViewportData::DIRTY_NORMAL)
            {
                //V_normals_vbo.resize(data.F.rows() * 3, 3);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_normals_vbo.row(i * 3 + j) = data.V_normals.row(data.F(i, j)).cast<float>();
                copy3times_lookup_data_F(V_normals_vbo, data.V_normals, data.F);

                if (invert_normals)
                    V_normals_vbo = -V_normals_vbo;
            }

            if (dirty & ViewportData::DIRTY_FACE)
            {
                F_vbo.resize(data.F.rows(), 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    F_vbo.row(i) << i * 3 + 0, i * 3 + 1, i * 3 + 2;
            }

            if (dirty & ViewportData::DIRTY_UV)
            {
                //V_uv_vbo.resize(data.F.rows() * 3, 2);
                //for (int i = 0; i < data.F.rows(); ++i)
                //    for (int j = 0; j < 3; ++j)
                //        V_uv_vbo.row(i * 3 + j) = data.V_uv.row(data.F(i, j)).cast<float>();
                copy3times_lookup_data_F2x(V_uv_vbo, data.V_uv, data.F);
            }
        }
        else
        {
            // Vertex positions
            if (dirty & ViewportData::DIRTY_POSITION)
                V_vbo = data.V.cast<float>();

            // Vertex normals
            if (dirty & ViewportData::DIRTY_NORMAL)
            {
                V_normals_vbo = data.V_normals.cast<float>();
                if (invert_normals)
                    V_normals_vbo = -V_normals_vbo;
            }

            // Per-vertex material settings
            if (dirty & ViewportData::DIRTY_AMBIENT)
                V_ambient_vbo = data.V_material_ambient;
            if (dirty & ViewportData::DIRTY_DIFFUSE)
                V_diffuse_vbo = data.V_material_diffuse;
            if (dirty & ViewportData::DIRTY_SPECULAR)
                V_specular_vbo = data.V_material_specular;

            // Face indices
            if (dirty & ViewportData::DIRTY_FACE)
                F_vbo = data.F.cast<unsigned>();

            // Texture coordinates
            if (dirty & ViewportData::DIRTY_UV)
                V_uv_vbo = data.V_uv.cast<float>();


        }
    }

    if (dirty & ViewportData::DIRTY_TEXTURE)
    {
        tex_u = data.texture_R.rows();
        tex_v = data.texture_R.cols();
        tex.resize(data.texture_R.size() * 3);
        for (int i = 0; i < data.texture_R.size(); ++i)
        {
            tex(i * 3 + 0) = data.texture_R(i);
            tex(i * 3 + 1) = data.texture_G(i);
            tex(i * 3 + 2) = data.texture_B(i);
        }
    }

    if (dirty & ViewportData::DIRTY_OVERLAY_LINES)
    {
        lines_V_vbo.resize(data.lines.rows() * 2, 3);
        lines_V_colors_vbo.resize(data.lines.rows() * 2, 3);
        lines_F_vbo.resize(data.lines.rows() * 2, 1);
        for (int i = 0; i < data.lines.rows(); ++i)
        {
            lines_V_vbo.row(2 * i + 0) = data.lines.block<1, 3>(i, 0).cast<float>(); // start 
            lines_V_vbo.row(2 * i + 1) = data.lines.block<1, 3>(i, 3).cast<float>(); // end
            lines_V_colors_vbo.row(2 * i + 0) = data.lines.block<1, 3>(i, 6).cast<float>(); //color of start
            lines_V_colors_vbo.row(2 * i + 1) = data.lines.block<1, 3>(i, 6).cast<float>(); //color of end
            lines_F_vbo(2 * i + 0) = 2 * i + 0;
            lines_F_vbo(2 * i + 1) = 2 * i + 1;
        }
    }


    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS)
    {
        points_V_vbo.resize(data.points.rows(), 3);
        points_V_colors_vbo.resize(data.points.rows(), 3);
        points_F_vbo.resize(data.points.rows(), 1);
        for (int i = 0; i < data.points.rows(); ++i)
        {
            points_V_vbo.row(i) = data.points.block<1, 3>(i, 0).cast<float>();
            points_V_colors_vbo.row(i) = data.points.block<1, 3>(i, 3).cast<float>();
            points_F_vbo(i) = i;
        }
    }
}

#else
void ViewportData_OpenGLshaders::set_data__ColMajor(const ViewportData &data, bool invert_normals)
{
    bool per_corner_uv = (data.F_uv.rows() == data.F.rows());
    bool per_corner_normals = (data.F_normals.rows() == 3 * data.F.rows());

    dirty |= data.dirty;

    if (data.face_based)
    {
        if (dirty & ViewportData::DIRTY_POSITION)
        {
            V_vbo.resize(3, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_vbo.col(i * 3 + j) = data.V.row(data.F(i, j)).transpose().cast<float>();
        }

        if (dirty & ViewportData::DIRTY_AMBIENT)
        {
            V_ambient_vbo.resize(3, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_ambient_vbo.col(i * 3 + j) = data.F_material_ambient.row(i).transpose();
        }

        if (dirty & ViewportData::DIRTY_DIFFUSE)
        {
            V_diffuse_vbo.resize(3, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_diffuse_vbo.col(i * 3 + j) = data.F_material_diffuse.row(i).transpose();
        }

        if (dirty & ViewportData::DIRTY_SPECULAR)
        {
            V_specular_vbo.resize(3, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_specular_vbo.col(i * 3 + j) = data.F_material_specular.row(i).transpose();
        }

        if (dirty & ViewportData::DIRTY_NORMAL)
        {
            V_normals_vbo.resize(3, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_normals_vbo.col(i * 3 + j) =
                    per_corner_normals ?
                    data.F_normals.row(i * 3 + j).transpose().cast<float>() :
                    data.F_normals.row(i).transpose().cast<float>();

            if (invert_normals)
                V_normals_vbo = -V_normals_vbo;
        }

        if (dirty & ViewportData::DIRTY_FACE)
        {
            F_vbo.resize(3, data.F.rows());
            for (int i = 0; i < data.F.rows(); ++i)
                F_vbo.col(i) << i * 3 + 0, i * 3 + 1, i * 3 + 2;
        }

        if (dirty & ViewportData::DIRTY_UV)
        {
            V_uv_vbo.resize(2, data.F.rows() * 3);
            for (int i = 0; i < data.F.rows(); ++i)
                for (int j = 0; j < 3; ++j)
                    V_uv_vbo.col(i * 3 + j) = data.V_uv.row(per_corner_uv ? data.F_uv(i, j) : data.F(i, j)).transpose().cast<float>();
        }
    }
    else
    {
        if (per_corner_uv)
        {
            // Per vertex properties with per corner UVs
            if (dirty & ViewportData::DIRTY_POSITION)
            {
                V_vbo.resize(3, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_vbo.col(i * 3 + j) = data.V.row(data.F(i, j)).transpose().cast<float>();
            }

            if (dirty & ViewportData::DIRTY_AMBIENT)
            {
                V_ambient_vbo.resize(3, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_ambient_vbo.col(i * 3 + j) = data.V_material_ambient.row(data.F(i, j)).transpose();
            }

            if (dirty & ViewportData::DIRTY_DIFFUSE)
            {
                V_diffuse_vbo.resize(3, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_diffuse_vbo.col(i * 3 + j) = data.V_material_diffuse.row(data.F(i, j)).transpose();
            }

            if (dirty & ViewportData::DIRTY_SPECULAR)
            {
                V_specular_vbo.resize(3, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_specular_vbo.col(i * 3 + j) = data.V_material_specular.row(data.F(i, j)).transpose();
            }

            if (dirty & ViewportData::DIRTY_NORMAL)
            {
                V_normals_vbo.resize(3, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_normals_vbo.col(i * 3 + j) = data.V_normals.row(data.F(i, j)).transpose().cast<float>();

                if (invert_normals)
                    V_normals_vbo = -V_normals_vbo;
            }

            if (dirty & ViewportData::DIRTY_FACE)
            {
                F_vbo.resize(3, data.F.rows());
                for (int i = 0; i < data.F.rows(); ++i)
                    F_vbo.col(i) << i * 3 + 0, i * 3 + 1, i * 3 + 2;
            }

            if (dirty & ViewportData::DIRTY_UV)
            {
                V_uv_vbo.resize(2, data.F.rows() * 3);
                for (int i = 0; i < data.F.rows(); ++i)
                    for (int j = 0; j < 3; ++j)
                        V_uv_vbo.col(i * 3 + j) = data.V_uv.row(data.F(i, j)).transpose().cast<float>();
            }
        }
        else
        {
            // Vertex positions
            if (dirty & ViewportData::DIRTY_POSITION)
                V_vbo = (data.V.transpose()).cast<float>();

            // Vertex normals
            if (dirty & ViewportData::DIRTY_NORMAL)
            {
                V_normals_vbo = (data.V_normals.transpose()).cast<float>();
                if (invert_normals)
                    V_normals_vbo = -V_normals_vbo;
            }

            // Per-vertex material settings
            if (dirty & ViewportData::DIRTY_AMBIENT)
                V_ambient_vbo = (data.V_material_ambient.transpose()).cast<float>();
            if (dirty & ViewportData::DIRTY_DIFFUSE)
                V_diffuse_vbo = (data.V_material_diffuse.transpose()).cast<float>();
            if (dirty & ViewportData::DIRTY_SPECULAR)
                V_specular_vbo = (data.V_material_specular.transpose()).cast<float>();

            // Face indices
            if (dirty & ViewportData::DIRTY_FACE)
                F_vbo = (data.F.transpose()).cast<unsigned>();

            // Texture coordinates
            if (dirty & ViewportData::DIRTY_UV)
                V_uv_vbo = (data.V_uv.transpose()).cast<float>();


        }
    }

    if (dirty & ViewportData::DIRTY_TEXTURE)
    {
        tex_u = data.texture_R.rows();
        tex_v = data.texture_R.cols();
        tex.resize(data.texture_R.size() * 3);
        for (int i = 0; i < data.texture_R.size(); ++i)
        {
            tex(i * 3 + 0) = data.texture_R(i);
            tex(i * 3 + 1) = data.texture_G(i);
            tex(i * 3 + 2) = data.texture_B(i);
        }
    }

    if (dirty & ViewportData::DIRTY_OVERLAY_LINES)
    {
        lines_V_vbo.resize(3, data.lines.rows() * 2);
        lines_V_colors_vbo.resize(3, data.lines.rows() * 2);
        lines_F_vbo.resize(1, data.lines.rows() * 2);
        for (int i = 0; i < data.lines.rows(); ++i)
        {
            lines_V_vbo.col(2 * i + 0) = data.lines.block<1, 3>(i, 0).transpose().cast<float>(); // start 
            lines_V_vbo.col(2 * i + 1) = data.lines.block<1, 3>(i, 3).transpose().cast<float>(); // end
            lines_V_colors_vbo.col(2 * i + 0) = data.lines.block<1, 3>(i, 6).transpose().cast<float>(); //color of start
            lines_V_colors_vbo.col(2 * i + 1) = data.lines.block<1, 3>(i, 6).transpose().cast<float>(); //color of end
            lines_F_vbo(2 * i + 0) = 2 * i + 0;
            lines_F_vbo(2 * i + 1) = 2 * i + 1;
        }
    }


    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS)
    {
        points_V_vbo.resize(3, data.points.rows());
        points_V_colors_vbo.resize(3, data.points.rows());
        points_F_vbo.resize(1, data.points.rows());
        for (int i = 0; i < data.points.rows(); ++i)
        {
            points_V_vbo.col(i) = data.points.block<1, 3>(i, 0).transpose().cast<float>();
            points_V_colors_vbo.col(i) = data.points.block<1, 3>(i, 3).transpose().cast<float>();
            points_F_vbo(i) = i;
        }
    }

    #if EROW
    // Originally Eigen is colwise, but igl works like rowwise - so there was conflict in linear data aligment
    // To solve this issue this method is generating all VBO data  colwise - to be able pass vbo buffers to GL
    // Since i have overriden Eigen default aligment  to rowwise - now there no more issue with linear data aligment
    //  but since code is old - we must to transpose matrixes back 
    // so actually in this method we transpose data twice - so we have to remove these 2 transose - and code will be more readable and a bit faster
    // TODO generate matrixes rowwise and remove this transpose
    if (dirty & ViewportData::DIRTY_POSITION) V_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_NORMAL) V_normals_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_AMBIENT) V_ambient_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_DIFFUSE) V_diffuse_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_SPECULAR) V_specular_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_UV) V_uv_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_LINES) lines_V_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_LINES) lines_V_colors_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS) points_V_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS) points_V_colors_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_FACE) F_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_LINES) lines_F_vbo.transposeInPlace();
    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS) points_F_vbo.transposeInPlace();
    #endif
}
#endif
#else


template<typename T>
void copyP3V3(MatrixXf& dest, int shift_index, const T& source)
{
    float* pDest = dest.data() + shift_index;
    for (int i = 0; i < source.rows(); ++i)
    {
        auto row = source.row(i);
        *(pDest + 0) = row(0);
        *(pDest + 1) = row(1);
        *(pDest + 2) = row(2);
        pDest += 3;
    }
};

int clamp(int value, int minValue, int maxValue)
{
    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;
    return value;
}
int compactOneValue(float value)
{
    if (isnan(value)) return 0;
    assert(abs(value) >= 0 && abs(value) <= 1);
    int sign = value < 0 ? 1 : 0;
    //int valueInt = clamp((int)(500 * abs(value)), 0, 500);
    int valueInt = (int)(500 * abs(value));
    return (valueInt << 1) + sign;
}
int compact(V3 normal)
{
    //normal.normalize();
    int x = compactOneValue(normal(0));
    int y = compactOneValue(normal(1));
    int z = compactOneValue(normal(2));
    int normalCompacted = (x << 20) + (y << 10) + (z << 0);
    return normalCompacted;
}
void copyP3V3_Compact_PositionAndNormal(MatrixXf& destV, int shift_index, const P3s& sourceV, const V3s& sourceVNormal, bool invert_normals)
{
    float* pDest = destV.data() + shift_index;
    for (int i = 0; i < sourceV.rows(); ++i)
    {
        auto v = sourceV.row(i);
        *(pDest + 0) = v(0);
        *(pDest + 1) = v(1);
        *(pDest + 2) = v(2);
        auto rowNormal = sourceVNormal.row(i);
        if (invert_normals) rowNormal = -rowNormal;
        int* pDest3 = (int*)(pDest + 3);
        *pDest3 = compact(rowNormal);
        pDest += 4;
    }
};

template<typename T1, typename T2>
void copyFaces(T1& dest, int shift_index, const T2& source, int shift_value, const I4s& Fcorrections, bool useFcorrections)
{
    unsigned int* pDest = dest.data() + shift_index;
    unsigned int* pDestStart = pDest; // rememeber position to be able to correct faces
    for (int i = 0; i < source.rows(); ++i)
    {
        auto row = source.row(i);
        *(pDest + 0) = shift_value + row(0);
        *(pDest + 1) = shift_value + row(1);
        *(pDest + 2) = shift_value + row(2);
        pDest += 3;
    }
    if (Fcorrections.size() > 0 && useFcorrections)
    {
        const int* pFcorrections = Fcorrections.data();
        const int* pFcorrectionsEnd = pFcorrections + Fcorrections.size();
        while (pFcorrections < pFcorrectionsEnd)
        {
            int correctedIndex = pFcorrections[0];
            pDestStart[correctedIndex * 3 + 0] = shift_value + pFcorrections[1]; // x
            pDestStart[correctedIndex * 3 + 1] = shift_value + pFcorrections[2]; // y
            pDestStart[correctedIndex * 3 + 2] = shift_value + pFcorrections[3]; // z
            pFcorrections += 4;
        }
    }
};

template<typename T1, typename T2>
void copyFaces_faceBased(T1& dest, int shift_index, const T2& source, int shift_value)
{
    unsigned int* pDest = dest.data() + shift_index;
    for (int i = 0; i < source.rows(); ++i)
    {
        *(pDest + 0) = shift_value + i * 3 + 0;
        *(pDest + 1) = shift_value + i * 3 + 1;
        *(pDest + 2) = shift_value + i * 3 + 2;
        pDest += 3;
    }
};

template<typename T1, typename T2>
void copyCompletely(T1& dest, int shift_index, const T2& source)
{
    memcpy(dest.data() + shift_index, source.data(), source.size() * sizeof(float));
};

template<typename T1, typename T2>
void copyConstant(T1& dest, int shift_index, T2 value, int count)
{
    float* pDest = dest.data() + shift_index;
    float* pDestEnd = pDest + count;
    while (pDest < pDestEnd)
    {
        *pDest = value;
        pDest++;
    }
};

void copyConstantColor(MatrixXf& dest, int shift_index, Color3f color, int rows)
{
    float* pDest = dest.data() + shift_index;
    float* pDestEnd = pDest + rows * 3;
    float R = color(0);
    float G = color(1);
    float B = color(2);
    while (pDest < pDestEnd)
    {
        *(pDest + 0) = R;
        *(pDest + 1) = G;
        *(pDest + 2) = B;
        pDest += 3;
    }
};



void copy3times(MatrixXf& dest, int shift_index, const MatrixXf& source)
{
    float* pDest = dest.data() + shift_index;
    const float* pSource = source.data();
    for (int i = 0; i < source.rows(); ++i)
    {
        float r = *(pSource + 0);
        float g = *(pSource + 1);
        float b = *(pSource + 2);
        *(pDest + 0) = r;
        *(pDest + 1) = g;
        *(pDest + 2) = b;
        *(pDest + 3) = r;
        *(pDest + 4) = g;
        *(pDest + 5) = b;
        *(pDest + 6) = r;
        *(pDest + 7) = g;
        *(pDest + 8) = b;
        pDest += 9;
        pSource += 3;
    }
};

template<typename T>
void copyP3V3_3times(MatrixXf& dest, int shift_index, const T& source)
{
    float* pDest = dest.data() + shift_index;
    for (int i = 0; i < source.rows(); ++i)
    {
        auto row = source.row(i);
        float x = row(0);
        float y = row(1);
        float z = row(2);
        *(pDest + 0) = x;
        *(pDest + 1) = y;
        *(pDest + 2) = z;
        *(pDest + 3) = x;
        *(pDest + 4) = y;
        *(pDest + 5) = z;
        *(pDest + 6) = x;
        *(pDest + 7) = y;
        *(pDest + 8) = z;
        pDest += 9;
    }
};

template<typename T>
void copy3times_lookup_data_F(MatrixXf& dest, int shift_index, const T& source, const MatrixXi& dataF)
{
    constexpr int COLS_PER_ROW = 3;

    //V_ambient_vbo.resize(data.F.rows() * 3, 3);
    //for (int i = 0; i < data.F.rows(); ++i)
    //    for (int j = 0; j < 3; ++j)
    //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));

    float* pDest = dest.data() + shift_index;
    const float* pSource = source.data();
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int vid0 = *(pF + 0);
        int vid1 = *(pF + 1);
        int vid2 = *(pF + 2);
        *(pDest + 0) = static_cast<float>(*(pSource + vid0 * COLS_PER_ROW + 0));
        *(pDest + 1) = static_cast<float>(*(pSource + vid0 * COLS_PER_ROW + 1));
        *(pDest + 2) = static_cast<float>(*(pSource + vid0 * COLS_PER_ROW + 2));
        *(pDest + 3) = static_cast<float>(*(pSource + vid1 * COLS_PER_ROW + 0));
        *(pDest + 4) = static_cast<float>(*(pSource + vid1 * COLS_PER_ROW + 1));
        *(pDest + 5) = static_cast<float>(*(pSource + vid1 * COLS_PER_ROW + 2));
        *(pDest + 6) = static_cast<float>(*(pSource + vid2 * COLS_PER_ROW + 0));
        *(pDest + 7) = static_cast<float>(*(pSource + vid2 * COLS_PER_ROW + 1));
        *(pDest + 8) = static_cast<float>(*(pSource + vid2 * COLS_PER_ROW + 2));
        pDest += 9;
        pF += 3;
    }
};
template<class T>
void copyP3V3_3times_lookup_data_F(MatrixXf& dest, int dest_shift_index, const T& source, const I3s& dataF)
{
    float* pDest = dest.data() + dest_shift_index;
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int vid0 = *(pF + 0);
        int vid1 = *(pF + 1);
        int vid2 = *(pF + 2);
        auto v0 = source.row(vid0);
        auto v1 = source.row(vid1);
        auto v2 = source.row(vid2);
        *(pDest + 0) = v0(0);
        *(pDest + 1) = v0(1);
        *(pDest + 2) = v0(2);
        *(pDest + 3) = v1(0);
        *(pDest + 4) = v1(1);
        *(pDest + 5) = v1(2);
        *(pDest + 6) = v2(0);
        *(pDest + 7) = v2(1);
        *(pDest + 8) = v2(2);
        pDest += 9;
        pF += 3;
    }
};

template<class T>
void copyP3V3_3times_lookup_data_F_Compact_PositionAndNormal(MatrixXf& dest, int dest_shift_index, const T& sourceV, const I3s& sourceF, const V3s& sourceF_normals, bool invert_normals)
{
    float* pDest = dest.data() + dest_shift_index;
    const int* pF = sourceF.data();
    for (int i = 0; i < sourceF.rows(); ++i)
    {
        int vid0 = *(pF + 0);
        int vid1 = *(pF + 1);
        int vid2 = *(pF + 2);
        V3 normal = sourceF_normals.row(i);
        if (invert_normals) normal = -normal;
        int normalCompacted = compact(normal);
        auto v0 = sourceV.row(vid0);
        auto v1 = sourceV.row(vid1);
        auto v2 = sourceV.row(vid2);
        *(pDest + 0) = v0(0);
        *(pDest + 1) = v0(1);
        *(pDest + 2) = v0(2);
        *(int*)(pDest + 3) = normalCompacted;

        *(pDest + 4) = v1(0);
        *(pDest + 5) = v1(1);
        *(pDest + 6) = v1(2);
        *(int*)(pDest + 7) = normalCompacted;

        *(pDest + 8) = v2(0);
        *(pDest + 9) = v2(1);
        *(pDest + 10) = v2(2);
        *(int*)(pDest + 11) = normalCompacted;

        pDest += 3 * 4;
        pF += 3;
    }
};

template<typename sourceInternalType>
void copy3times_lookup_data_F2x(MatrixXf& dest, const Matrix<sourceInternalType, Dynamic, Dynamic>& source, const MatrixXi& dataF)
{
    //V_ambient_vbo.resize(data.F.rows() * 3, 3);
    //for (int i = 0; i < data.F.rows(); ++i)
    //    for (int j = 0; j < 3; ++j)
    //        V_ambient_vbo.row(i * 3 + j) = data.V_material_ambient.row(data.F(i, j));

    dest.resize(dataF.rows() * 3, 2);
    float* pDest = dest.data();
    const sourceInternalType* pSource = source.data();
    const int* pF = dataF.data();
    for (int i = 0; i < dataF.rows(); ++i)
    {
        int row0 = *(pF + 0);
        int row1 = *(pF + 1);
        int row2 = *(pF + 2);
        *(pDest + 0) = static_cast<float>(*(pSource + row0 * 2 + 0));
        *(pDest + 1) = static_cast<float>(*(pSource + row0 * 2 + 1));
        *(pDest + 2) = static_cast<float>(*(pSource + row1 * 2 + 0));
        *(pDest + 3) = static_cast<float>(*(pSource + row1 * 2 + 1));
        *(pDest + 4) = static_cast<float>(*(pSource + row2 * 2 + 0));
        *(pDest + 5) = static_cast<float>(*(pSource + row2 * 2 + 1));
        pDest += 6;
        pF += 3;
    }
};

void ViewportData_OpenGLshaders::set_data__V3P3(const ViewportData &data, bool invert_normals, bool crease_normals)
{
    dirty |= data.dirty;
    bool per_corner_uv = (data.F_uv.rows() == data.meshes.FCountTotal);


    //
    // create a map<objid, ViewerDrawObjects>
    //
    map<int, int> map_objid_drawIndex;
    for (int i = 0; i < data.draws.size(); i++)
    {
        int objId = data.draws[i].first;
        if (objId != -1) map_objid_drawIndex[objId] = i;
    }

    //
    // get new VCount
    //
    vector<pair<int, int>> VCounts;
    VCounts.reserve(data.meshes.meshes.size());
    int VCountTotal = 0;
    for (int mi = 0; mi < data.meshes.meshes.size(); mi++)
    {
        const ViewportData_Mesh& m = data.meshes.meshes[mi];
        auto find = map_objid_drawIndex.find(m.Id);
        bool meshHasDraw = (find != map_objid_drawIndex.end());
        MeshColor defaultColor;
        const MeshColor* meshColor = &defaultColor;
        if (meshHasDraw)
        {
            int drawIndex = find->second;
            const ViewerDrawObjects& draw = data.draws[drawIndex].second.get();
            meshColor = &draw.meshColor;
        }

        int VCountO = 0;
        int VCountC = 0;
        if (meshColor->isFaceBased())
        {
            VCountO = m.F.rows() * 3;
            VCountC = 0;
        }
        else
        {
            VCountO = m.V.rows();
            if (crease_normals && m.NormalCorrections_V.rows() == m.NormalCorrections_V_normals.rows())
            {
                VCountC += m.NormalCorrections_V.rows();
            }
        }
        VCountTotal += VCountO + VCountC;
        VCounts.push_back({ VCountO, VCountC });
    }

    //
    // Update dirty flag
    //
    if (VCountTotal != V.rows()) dirty |= ViewportData::DIRTY_POSITION | ViewportData::DIRTY_FACE | ViewportData::DIRTY_NORMAL | ViewportData::DIRTY_COLOR | ViewportData::DIRTY_UV;
    if (VCountTotal != V_normals.rows()) dirty |= ViewportData::DIRTY_NORMAL;
    if (VCountTotal != V_color.rows()) dirty |= ViewportData::DIRTY_COLOR;
    if (VCountTotal != V_uv.rows()) dirty |= ViewportData::DIRTY_UV;


    //
    // Resize containers
    //
    int V_COLS = 3;
    int V_shift_index = 0;
    int Vcolor_shift_index = 0;
    int Vu_shift_index = 0;
    int F_shift_index = 0;
    int F_shift_vertex_index = 0;
    if (options.Performance.compactNormals)
    {
        V_COLS = 4;
        if (dirty & ViewportData::DIRTY_POSITION || dirty & ViewportData::DIRTY_NORMAL) V.resize(VCountTotal, V_COLS);
    }
    else
    {
        V_COLS = 3;
        if (dirty & ViewportData::DIRTY_POSITION) V.resize(VCountTotal, V_COLS);
        if (dirty & ViewportData::DIRTY_NORMAL) V_normals.resize(VCountTotal, V_COLS);
    }
    if (dirty & ViewportData::DIRTY_COLOR) V_color.resize(VCountTotal, 3);
    if (dirty & ViewportData::DIRTY_UV) V_uv.resize(VCountTotal, 2);
    if (dirty & ViewportData::DIRTY_FACE) F.resize(data.meshes.FCountTotal, 3);

    for (int mi = 0; mi < data.meshes.meshes.size(); mi++)
    {
        const ViewportData_Mesh& m = data.meshes.meshes[mi];
        int objid = m.Id;
        auto find = map_objid_drawIndex.find(objid);
        bool meshHasDraw = (find != map_objid_drawIndex.end());
        MeshColor defaultColor;
        const MeshColor* meshColor = &defaultColor;
        if (meshHasDraw)
        {
            int drawIndex = find->second;
            const ViewerDrawObjects& draw = data.draws[drawIndex].second.get();
            meshColor = &draw.meshColor;
        }

        int VCountO = VCounts[mi].first;
        int VCountC = VCounts[mi].second;
        if (meshColor->isFaceBased())
        {
            if (options.Performance.compactNormals)
            {
                if (dirty & ViewportData::DIRTY_POSITION || dirty & ViewportData::DIRTY_NORMAL)
                {
                    copyP3V3_3times_lookup_data_F_Compact_PositionAndNormal(V, V_shift_index, m.V, m.F, m.F_normals, invert_normals);
                }
            }
            else
            {
                if (dirty & ViewportData::DIRTY_POSITION) copyP3V3_3times_lookup_data_F(V, V_shift_index, m.V, m.F);
                if (dirty & ViewportData::DIRTY_NORMAL) copyP3V3_3times(V_normals, V_shift_index, m.F_normals);
            }

            if (dirty & ViewportData::DIRTY_COLOR)
            {
                if (meshColor->Scheme == MeshColorSceme::PerFace && meshColor->Colors.rows() == m.F.rows())
                {
                    copy3times(V_color, Vcolor_shift_index, meshColor->Colors);
                }
                else
                {
                    Color3f singleColor = (meshColor->Colors.rows() == 1)
                        ? meshColor->Colors.row(0)
                        : MeshColor::GetDefaultColor();
                    copyConstantColor(V_color, Vcolor_shift_index, singleColor, m.F.rows() * 3);
                }
            }

            if (dirty & ViewportData::DIRTY_FACE) copyFaces_faceBased(F, F_shift_index, m.F, F_shift_vertex_index); // yes copy int to uint - we never will have -1, so int and uint will be same

            //if (dirty & ViewportData::DIRTY_UV)
            //{
            //    if (per_corner_uv)
            //        copy3times_lookup_data_F2x(V_uv, data.V_uv, data.F_uv);
            //    else
            //        copy3times_lookup_data_F2x(V_uv, data.V_uv, m.F);
            //}
        }
        else
        {
            /*if (per_corner_uv)
            {
                // Per vertex properties with per corner UVs
                if (dirty & ViewportData::DIRTY_POSITION)
                {
                    copy3times_lookup_data_F_P3V3(V, m.V, m.F);
                }


                if (dirty & ViewportData::DIRTY_COLOR)copy3times_lookup_data_F(V_color, data.V_color, m.F);


                if (dirty & ViewportData::DIRTY_NORMAL)
                {
                    copy3times_lookup_data_F_P3V3(V_normals, m.V_normals, m.F);

                    if (invert_normals)
                        V_normals = -V_normals;
                }

                if (dirty & ViewportData::DIRTY_FACE)
                {
                    F.resize(m.F.rows(), 3);
                    for (int i = 0; i < m.F.rows(); ++i)
                        F.row(i) = Vector3ui(i * 3 + 0, i * 3 + 1, i * 3 + 2);
                }

                if (dirty & ViewportData::DIRTY_UV)
                {
                    copy3times_lookup_data_F2x(V_uv, data.V_uv, m.F);
                }
            }
            else*/
            {
                int V_shift_indexC = V_shift_index + VCountO * V_COLS;
                int Vcolor_shift_indexC = Vcolor_shift_index + VCountO * 3;
                int Vu_shift_indexC = Vu_shift_index + VCountO * 2;
                int F_shift_vertex_indexC = F_shift_vertex_index + VCountO;

                if (options.Performance.compactNormals)
                {
                    if (dirty & ViewportData::DIRTY_POSITION || dirty & ViewportData::DIRTY_NORMAL)
                    {
                        copyP3V3_Compact_PositionAndNormal(V, V_shift_index, m.V, m.V_normals, invert_normals);
                        if (crease_normals) copyP3V3_Compact_PositionAndNormal(V, V_shift_indexC, m.NormalCorrections_V, m.NormalCorrections_V_normals, invert_normals);
                    }
                }
                else
                {
                    if (dirty & ViewportData::DIRTY_POSITION)
                    {
                        copyP3V3(V, V_shift_index, m.V);
                        if (crease_normals) copyP3V3(V, V_shift_indexC, m.NormalCorrections_V);
                    }
                    if (dirty & ViewportData::DIRTY_NORMAL)
                    {
                        copyP3V3(V_normals, V_shift_index, m.V_normals);
                        if (crease_normals) copyP3V3(V_normals, V_shift_indexC, m.NormalCorrections_V_normals);
                    }
                }

                if (dirty & ViewportData::DIRTY_COLOR)
                {
                    if (meshColor->Scheme == MeshColorSceme::PerVertex && meshColor->Colors.rows() == m.V.rows())
                    {
                        copyCompletely(V_color, Vcolor_shift_index, meshColor->Colors);
                        // copy colors for m.NormalCorrections_V
                        if (crease_normals)
                        {
                            MatrixXf correctedColors;
                            correctedColors.resize(m.NormalCorrections_V.rows(), 3);
                            for (int i = 0; i < m.NormalCorrections_V_correctedIndexes.size(); i++)
                            {
                                int originalIndex = m.NormalCorrections_V_correctedIndexes(i);
                                correctedColors.row(i) = meshColor->Colors.row(originalIndex);
                            }
                            copyCompletely(V_color, Vcolor_shift_indexC, correctedColors);
                        }
                    }
                    else
                    {
                        Color3f singleColor = (meshColor->Colors.rows() == 1)
                            ? meshColor->Colors.row(0)
                            : MeshColor::GetDefaultColor();
                        copyConstantColor(V_color, Vcolor_shift_index, singleColor, m.V.rows());
                        if (crease_normals) copyConstantColor(V_color, Vcolor_shift_indexC, singleColor, m.NormalCorrections_V.rows());
                    }
                }

                if (dirty & ViewportData::DIRTY_FACE)
                {
                    copyFaces(F, F_shift_index, m.F, F_shift_vertex_index, m.NormalCorrections_Fcorrections, crease_normals); // yes copy int to uint - we never will have -1, so int and uint will be same
                }

                // Texture coordinates
                //if (dirty & ViewportData::DIRTY_UV) copyCompletely(V_uv, Vu_shift_index, data.V_uv);
            }

        }

        // shift indexes
        int VCount = VCountO + VCountC;
        V_shift_index += VCount * V_COLS;
        Vcolor_shift_index += VCount * 3;
        Vu_shift_index += VCount * 2;
        F_shift_index += m.F.rows() * 3;
        F_shift_vertex_index += VCount;
    }



    if (invert_normals)
    {
        V_normals = -V_normals;
        for (int i = 0; i < F.rows(); i++)
        {
            Vector3ui row = F.row(i);
            swap(row(0), row(2));
            F.row(i) = row;
        }
    }

    if (dirty & ViewportData::DIRTY_TEXTURE)
    {
        tex_u = data.texture_R.rows();
        tex_v = data.texture_R.cols();
        tex.resize(data.texture_R.size() * 3);
        for (int i = 0; i < data.texture_R.size(); ++i)
        {
            tex(i * 3 + 0) = data.texture_R(i);
            tex(i * 3 + 1) = data.texture_G(i);
            tex(i * 3 + 2) = data.texture_B(i);
        }
    }

    if (dirty & ViewportData::DIRTY_OVERLAY_LINES)
    {
        int linesCountNormal = 0;
        int linesBoldCount = 0;
        for (auto& draw : data.draws)
        {
            linesCountNormal += draw.second.get().edges.size();
            linesBoldCount += draw.second.get().edgesBold.size();
        }

        lines_V.resize((linesCountNormal + linesBoldCount) * 2, 3);
        lines_V_colors.resize((linesCountNormal + linesBoldCount) * 2, 3);
        lines_F.resize(linesCountNormal * 2, 1);
        linesBold_F.resize(linesBoldCount * 2, 1);
        int iv = 0; // index for lines_V
        int ii = 0; // index for lines_F
        for (auto& draw : data.draws)
        {
            for (const auto& edge : draw.second.get().edges)
            {
                lines_V.row(2 * iv + 0) = convertP3ToEigen(edge.start); // start 
                lines_V.row(2 * iv + 1) = convertP3ToEigen(edge.end); // end
                lines_V_colors.row(2 * iv + 0) = edge.color; //color of start
                lines_V_colors.row(2 * iv + 1) = edge.color; //color of end
                lines_F(2 * ii + 0) = 2 * ii + 0;
                lines_F(2 * ii + 1) = 2 * ii + 1;
                iv++;
                ii++;
            }
        }
        //iv = iv; // continue to write to same lines_V buffer
        ii = 0; // clear indexes to start now writing to linesBold_F buffer
        for (auto& draw : data.draws)
        {
            for (const auto& edge : draw.second.get().edgesBold)
            {
                lines_V.row(2 * iv + 0) = convertP3ToEigen(edge.start); // start 
                lines_V.row(2 * iv + 1) = convertP3ToEigen(edge.end); // end
                lines_V_colors.row(2 * iv + 0) = edge.color; //color of start
                lines_V_colors.row(2 * iv + 1) = edge.color; //color of end
                linesBold_F(2 * ii + 0) = 2 * iv + 0;
                linesBold_F(2 * ii + 1) = 2 * iv + 1;
                iv++;
                ii++;
            }
        }
    }

    if (dirty & ViewportData::DIRTY_OVERLAY_POINTS)
    {
        int pointsCount = 0;
        for (auto& draw : data.draws)
        {
            pointsCount += draw.second.get().points.size();
        }

        points_V.resize(pointsCount, 3);
        points_V_colors.resize(pointsCount, 3);
        points_F.resize(pointsCount, 1);
        int i = 0;
        for (auto& draw : data.draws)
        {
            for (const auto& point : draw.second.get().points)
            {
                points_V.row(i) = convertP3ToEigen(point.point);
                points_V_colors.row(i) = point.color;
                points_F(i) = i;
                i++;
            }
        }
    }
}

#endif

void ViewportData_OpenGLshaders::set_data(const ViewportData &data, bool invert_normals, bool crease_normals)
{
    Timer time;
    #ifdef USE_EIGEN
    #if EROW
    set_data__RowMajor(data, invert_normals);
    #else
    set_data__ColMajor(data, invert_normals);
    #endif
    #else
    set_data__V3P3(data, invert_normals, crease_normals);
    #endif
    time.stop(elapsedTimers.Viewer.SetData);
}

void ViewportData_OpenGLshaders::bind_mesh_for_cacheColor()
{
    bool isDirtyPosition = dirty & ViewportData::DIRTY_POSITION;
    program_mesh_for_cacheColor.Use();

    if (isDirtyPosition)
    {
        tbo_VertexShader_POINTS.BindBuffer(V, 1);
    }

    //TODO need to uncomment, but there is an issue:  this stops repaint normal draw 
    //dirty &= ~ViewportData::DIRTY_POSITION;
    //dirty &= ~ViewportData::DIRTY_NORMAL;
    //dirty &= ~ViewportData::DIRTY_AMBIENT;
    //dirty &= ~ViewportData::DIRTY_DIFFUSE;
    //dirty &= ~ViewportData::DIRTY_SPECULAR;
    //dirty &= ~ViewportData::DIRTY_FACE;
};


void ViewportData_OpenGLshaders::bind_mesh_for_dephTesting()
{
    if (options.Performance.unindex_depth_test)
    {
        if (dirty & ViewportData::DIRTY_POSITION)
        {
            V_unindexed.resize(F.rows() * 3, 3);
            float* pV = V_unindexed.data();
            unsigned int *pF = F.data();
            for (int i = 0; i < F.size(); i++)
            {
                Vector3f v = V.row(*pF); pF++;
                *pV = v(0); pV++;
                *pV = v(1); pV++;
                *pV = v(2); pV++;
            }
        }
        program_mesh_for_dephTesting.Use();
    }
    else
    {
        program_mesh_for_dephTesting.Use();
        

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_F);
        if (dirty & ViewportData::DIRTY_FACE)
        {
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned)*F.size(), F.data(), GL_STATIC_DRAW);
            dirty &= ~ViewportData::DIRTY_FACE;
        }
    }
};

void ViewportData_OpenGLshaders::bind_mesh(bool showTexture)
{
    program_mesh.Use();

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_F);
    if (dirty & ViewportData::DIRTY_FACE)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned)*F.size(), F.data(), GL_STATIC_DRAW);
    }



    if (showTexture)
    {
        //TODO we need pass texture in Vertex shader and in Fragment shader since we commented input and output parameters in shader
        program_mesh.bindVertexAttribArray("texcoord", vbo_V_uv, V_uv, dirty & ViewportData::DIRTY_UV);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, vbo_tex);
        if (dirty & ViewportData::DIRTY_TEXTURE)
        {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_u, tex_v, 0, GL_RGB, GL_UNSIGNED_BYTE, tex.data());
        }
        glUniform1i(program_mesh.getUniformId("tex"), 0);
    }

    dirty &= ~ViewportData::DIRTY_MESH;
}

bool ViewportData_OpenGLshaders::bind_overlay_lines_program()
{
    bool is_dirty = dirty & ViewportData::DIRTY_OVERLAY_LINES;
    program_overlay_lines.Use();
    dirty &= ~ViewportData::DIRTY_OVERLAY_LINES;
    return is_dirty;
}


void ViewportData_OpenGLshaders::bind_overlay_lines_linesNormal(bool is_dirty)
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_lines_F);
    if (is_dirty)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned)*lines_F.size(), lines_F.data(), GL_STATIC_DRAW);
    }
}

void ViewportData_OpenGLshaders::bind_overlay_lines_linesBold(bool is_dirty)
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_linesBold_F);
    if (is_dirty)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned)*linesBold_F.size(), linesBold_F.data(), GL_STATIC_DRAW);
    }
}
void ViewportData_OpenGLshaders::bind_overlay_points()
{
    bool is_dirty = dirty & ViewportData::DIRTY_OVERLAY_POINTS;

    program_overlay_points.Use();

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_points_F);
    if (is_dirty)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned)*points_F.size(), points_F.data(), GL_STATIC_DRAW);
    }

    dirty &= ~ViewportData::DIRTY_OVERLAY_POINTS;
}

void ViewportData_OpenGLshaders::draw_mesh(bool solid, GLenum front_and_back)
{
    if (solid)
    {
        //glEnable(GL_POLYGON_OFFSET_FILL);
        //glPolygonOffset(10, 10);
        //glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        //glDisable(GL_POLYGON_OFFSET_FILL);
        //glPolygonMode(front_and_back, GL_FILL); //produces error in nanogui - Error 0005000 after convex fill
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        //glDisable(GL_POLYGON_OFFSET_FILL);
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);  
        glPolygonOffset(options.Offsets.wireframe_factor, options.Offsets.wireframe_units); // offset in z-buffer to allow lines draw over the mesh  (draw_overlay_points was fighting with wireframe and was invisible when wireframe was on)
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glDisable(GL_POLYGON_OFFSET_LINE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void ViewportData_OpenGLshaders::draw_overlay_lines()
{
    // we cant do zbuffering for lines in glDrawElements
    // so we have to make it in shader by multiplying z coordinate by 0.9999  (vec4(1,1,0.9999,1))
    glDrawElements(GL_LINES, lines_F.size(), GL_UNSIGNED_INT, 0);
}

void ViewportData_OpenGLshaders::draw_overlay_linesBold()
{
    // we cant do zbuffering for lines in glDrawElements
    // so we have to make it in shader by multiplying z coordinate by 0.9999  (vec4(1,1,0.9999,1))
    glDrawElements(GL_LINES, linesBold_F.size(), GL_UNSIGNED_INT, 0);
}

void ViewportData_OpenGLshaders::draw_overlay_points()
{
    // we cant do zbuffering for lines in glDrawElements
    // so we have to make it in shader by multiplying z coordinate by 0.9999  (vec4(1,1,0.9999,1))
    glDrawElements(GL_POINTS, points_F.size(), GL_UNSIGNED_INT, 0);
}

ViewportData_OpenGLshaders::ViewportData_OpenGLshaders()
    : shared_vbo__position(OpenGL_VertexBufferObject("position", V, dirty, ViewportData::DIRTY_POSITION))
{
}

void ViewportData_OpenGLshaders::init()
{
    TransparencySort_CachedData.latcall_V_vbo_size = 0;
    TransparencySort_CachedData.latcall_F_vbo_size = 0;

    init_buffers();

    //
    // Mesh
    //
    string  mesh_vertex__shaderstring = R"(
        in vec3 position;
        in vec3 normal;
        in vec3 vertexColor;

        uniform mat4 view_model;
        uniform mat4 proj_view_model;
        uniform float alpha_BackSideCoeff;
        uniform vec3 light_position_eye;
        uniform float specular_exponent;
        uniform float specular_intensity;
        uniform float lighting_factor;
        
        out vec3 outColorVERTEXSHADER; // flat - The value will not be interpolated.  see https://www.khronos.org/opengl/wiki/Type_Qualifier_%28GLSL%29 
        //in vec2 texcoord;
        //out vec2 texcoordi;


        float uncompactOne(int value)
        {
            float res = (0.0 + (value >> 1)) / 500.0;
            if ((value & 1) == 1) res = -res;
            return res;
        }

        vec3 uncompact(int normalCompacted)
        {
            int x = (normalCompacted >> 20) & 1023;
            int y = (normalCompacted >> 10) & 1023;
            int z = (normalCompacted >> 0) & 1023;
            return vec3(uncompactOne(x), uncompactOne(y), uncompactOne(z));
        }
    
        void main()
        {
            //vec3 normal = ;

             vec3 position_eye = vec3 (view_model * vec4 (position.xyz, 1.0));
             vec3 normal_eye = vec3 (view_model * vec4 (normal, 0.0));
             normal_eye = normalize(normal_eye);

             // set Ambient Diffuse Specular sub-colors from vertex color
             vec3 Ka = vertexColor * 0.5;
             vec3 Kd = vertexColor - Ka;
             vec3 Ks = vec3(0.5, 0.5, 0.5) + 0.1*(vertexColor - vec3(0.5, 0.5, 0.5));   // const float grey = 0.5;  grey + 0.1f*(C.array() - grey); // from ViewportData.cpp
             
    
             // Ambient intensity
             vec3 Ia = Ka;    
    
             // Diffuse intensity
             vec3 vector_to_light_eye = light_position_eye - position_eye;
             vec3 direction_to_light_eye = normalize (vector_to_light_eye);
             float dot_prod = dot (direction_to_light_eye, normal_eye);
             float clamped_dot_prod = max (dot_prod, -dot_prod*alpha_BackSideCoeff); // Mupoc  - diffuse front side a 2 times lighter compare to back side
             vec3 Id = Kd * clamped_dot_prod;    

             // Specular intensity
             vec3 reflection_eye = reflect (-direction_to_light_eye, normal_eye);
             vec3 surface_to_viewer_eye = normalize (-position_eye);
             float dot_prod_specular = dot (reflection_eye, surface_to_viewer_eye);
             dot_prod_specular = float(abs(dot_prod)==dot_prod) * max (dot_prod_specular, 0.0);
             float specular_factor = pow (dot_prod_specular, specular_exponent);
             vec3 Is = Ks * specular_factor*specular_intensity;    

             // Final color
             vec3 color =  lighting_factor * (Is + Id) + Ia + (1.0-lighting_factor) * Kd;
             outColorVERTEXSHADER = color; 

             // Position
             vec4 outPositionVERTEXSHADER = proj_view_model * vec4(position.xyz, 1.0); 
             gl_Position = outPositionVERTEXSHADER; 
             //texcoordi = texcoord;
        })";


    if (options.Performance.compactColors)
    {
        //mesh_vertex__shaderstring = utils::strings::ReplaceAll(mesh_vertex__shaderstring, "in vec3 vertexColor", "const vec3 vertexColor = vec3(0.5,0.5,0.5)");
        //mesh_vertex__shaderstring = utils::strings::ReplaceAll(mesh_vertex__shaderstring, "in vec3 normal", "const vec3 normal = vec3(0.5,0.5,0.5)");
    }

    if (options.Performance.compactNormals)
    {
        mesh_vertex__shaderstring = utils::strings::ReplaceAll(mesh_vertex__shaderstring, "in vec3 position", "in vec4 position");  //extend position by 1 float for compacted normal
        mesh_vertex__shaderstring = utils::strings::ReplaceAll(mesh_vertex__shaderstring, "in vec3 normal;", ""); // normal is compacted into position.w
        mesh_vertex__shaderstring = utils::strings::ReplaceAll(mesh_vertex__shaderstring, "//vec3 normal = ;", "vec3 normal = uncompact(floatBitsToInt(position.w));");
    }


    string mesh_fragment__shaderstring = R"(
        //layout(early_fragment_tests) in;
        uniform float transparent_alpha_front;
        uniform float transparent_alpha_back;
        //uniform float texture_factor;
        //uniform sampler2D tex;
        //in vec2 texcoordi;
        in vec3 outColorVERTEXSHADER;
        uniform vec4 fixed_color;
        out vec4 outColor;
        void main()
        {
            if (fixed_color != vec4(0.0)) {outColor = fixed_color;}else{
                float transparent_alpha = transparent_alpha_front;
                if (!gl_FrontFacing) {transparent_alpha = transparent_alpha_back;}
                //outColor = mix(vec4(1,1,1,transparent_alpha), texture(tex, texcoordi), texture_factor) * vec4(outColorVERTEXSHADER, transparent_alpha);
    vec3 color = outColorVERTEXSHADER;
    //if (!gl_FrontFacing) {color = vec3(0.4, 0.4, 0.4);}
    if (!gl_FrontFacing) {color = color*$backSideColorFactor;}
                outColor = vec4(color, transparent_alpha) ;
    //if (!gl_FrontFacing) {outColor = vec4(0.4, 0.4, 0.4, transparent_alpha);}
            }
        })";

    mesh_fragment__shaderstring = utils::strings::ReplaceAll(mesh_fragment__shaderstring, "$backSideColorFactor", to_string(options.Colors.backSideColorFactor));


    //
    // Depth test
    //
    string mesh_vertex_DEPTH_TEST__shaderstring = R"(
        uniform mat4 proj_view_model;
        in vec3 position;
        void main()
        {
            gl_Position = proj_view_model * vec4(position.xyz, 1.0); 
        })";
    if (options.Performance.compactNormals)
    {
        mesh_vertex_DEPTH_TEST__shaderstring = utils::strings::ReplaceAll(mesh_vertex_DEPTH_TEST__shaderstring, "in vec3 position", "in vec4 position");  //extend position by 1 float for compacted normal
    }

    //
    // common for line and point 
    //
    string overlay_vertex__shaderstring = R"(
        uniform mat4 proj_view_model;
        in vec3 position;
        in vec3 color;
        out vec3 color_frag;
        void main()
        {
            gl_Position = proj_view_model * vec4 (position, 1)*vec4(1,1,0.9999,1); // 0.9999 - special values - shifting lines in z buffer to draw on top of faces
            color_frag = color;
        })";

    //
    // Lines
    //
    string overlay_line_fragment__shaderstring = R"(
        uniform float transparent_alpha;
        in vec3 color_frag;
        layout(location = 0) out vec4 outColor;
        void main()
        {
            outColor = vec4(color_frag, transparent_alpha);
        })";

    //
    // Points
    //
    string overlay_point_fragment__shaderstring = R"(
        uniform float transparent_alpha;
        in vec3 color_frag;
        out vec4 outColor;
        void main()
        {
            if (length(gl_PointCoord - vec2(0.5)) > 0.5) // makes circle from quad
              discard;
            outColor = vec4(color_frag, transparent_alpha);
        })";


    //
    // depth_test
    //
    program_mesh_for_dephTesting.Create(mesh_vertex_DEPTH_TEST__shaderstring, "", "", {}); // we dont write to color buffer, since we populate only depth buffer
    if (options.Performance.unindex_depth_test)
    {
        program_mesh_for_dephTesting.AddBuffer("position", V_unindexed, dirty, ViewportData::DIRTY_POSITION); // use unindexed vertexes
    }
    else
    {
        program_mesh_for_dephTesting.AddSharedBuffer("position", &shared_vbo__position); // use shared indexed vertexes
    }


    //
    // mesh
    //
    program_mesh.Create(mesh_vertex__shaderstring, mesh_fragment__shaderstring, "outColor", {});

    program_mesh.AddSharedBuffer("position", &shared_vbo__position);
    if (options.Performance.compactNormals)
    {
        // we pass normal as 4-th value in position: 'position.w'
    }
    else
    {
        program_mesh.AddBuffer("normal", V_normals, dirty, ViewportData::DIRTY_NORMAL);
    }
    program_mesh.AddBuffer("vertexColor", V_color, dirty, ViewportData::DIRTY_COLOR);



    //
    // lines
    //
    program_overlay_lines.Create(overlay_vertex__shaderstring, overlay_line_fragment__shaderstring, "outColor", {});
    program_overlay_lines.AddBuffer("position", lines_V, dirty, ViewportData::DIRTY_OVERLAY_LINES);
    program_overlay_lines.AddBuffer("color", lines_V_colors, dirty, ViewportData::DIRTY_OVERLAY_LINES);


    //
    // points
    //
    program_overlay_points.Create(overlay_vertex__shaderstring, overlay_point_fragment__shaderstring, "outColor", {});
    program_overlay_points.AddBuffer("position", points_V, dirty, ViewportData::DIRTY_OVERLAY_POINTS);
    program_overlay_points.AddBuffer("color", points_V_colors, dirty, ViewportData::DIRTY_OVERLAY_POINTS);
}

void ViewportData_OpenGLshaders::free()
{
    program_mesh.Delete();
    program_mesh_for_cacheColor.Delete();
    program_mesh_for_dephTesting.Delete();
    program_overlay_lines.Delete();
    program_overlay_points.Delete();
    shared_vbo__position.Delete(); // we must delete shared buffer manually, becase shaders dont delete shared buffers

    glDeleteBuffers(1, &vbo_V_uv);
    tbo_VertexShader_POINTS.Delete();
    glDeleteBuffers(1, &vbo_V_unindexed);
    glDeleteBuffers(1, &vbo_F);
    glDeleteTextures(1, &vbo_tex);

    glDeleteBuffers(1, &vbo_lines_F);
    glDeleteBuffers(1, &vbo_linesBold_F);
    glDeleteBuffers(1, &vbo_points_F);
}

void ViewportData_OpenGLshaders::reinit()
{
    free();
    init();
}


II ViewportData_OpenGLshaders::SizeOF() const
{
    II r = sizeof(ViewportData_OpenGLshaders);

    r += V.size() * sizeof(float);
    r += V_normals.size() * sizeof(float);
    r += V_color.size() * sizeof(float);
    r += V_uv.size() * sizeof(float);

    r += lines_V.size() * sizeof(float);
    r += lines_V_colors.size() * sizeof(float);
    r += points_V.size() * sizeof(float);
    r += points_V_colors.size() * sizeof(float);


    r += tex.size() * sizeof(char);

    r += F.size() * sizeof(unsigned int);
    r += lines_F.size() * sizeof(unsigned int);
    r += linesBold_F.size() * sizeof(unsigned int);
    r += points_F.size() * sizeof(unsigned int);

    r += TransparencySort_CachedData.F_vbo_duplicate.size() * sizeof(unsigned int);
    r += TransparencySort_CachedData.Vdistances.size() * sizeof(float);
    r += TransparencySort_CachedData.dist_to_faces.size() * sizeof(float);
    r += TransparencySort_CachedData.dist_to_faces_sorted_indexes.size() * sizeof(unsigned int);
    r += TransparencySort_CachedData.dist_to_faces_keyvalue.size() * sizeof(pair<float, int>);

    return r;
}


