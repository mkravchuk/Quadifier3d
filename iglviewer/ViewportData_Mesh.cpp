#include "stdafx.h"
#include "ViewportData_Mesh.h"

P3 Point3d_Zero(0, 0, 0);
P3s P3s0(0, 3);
V3s V3s0(0, 3);
Is Is_0(0);
MatrixXi MatrixXi_0_2(0, 2);
I3s I3s_0_3(0, 3);
I4s I4s_0_4(0, 4);

ViewportData_Mesh::ViewportData_Mesh()
    : ViewportData_Mesh(-1, P3s0, I3s_0_3, V3s0, V3s0, P3s0, V3s0, Is_0, I4s_0_4, Point3d_Zero, Point3d_Zero, Point3d_Zero, Point3d_Zero)
{

}

ViewportData_Mesh::ViewportData_Mesh(int _obj_id, const P3s& _V, const I3s& _F, const V3s& _F_normals, const V3s& _V_normals,
    const P3& _Vmin, const P3& _Vmax, const P3& _Vcenter, const P3& _VcenterProjected)
    : Id(_obj_id), V(_V), F(_F), F_normals(_F_normals), V_normals(_V_normals),
    NormalCorrections_V(P3s0), NormalCorrections_V_normals(V3s0), NormalCorrections_V_correctedIndexes(Is_0), NormalCorrections_Fcorrections(I4s_0_4),
    Vmin(_Vmin), Vmax(_Vmax), Vcenter(_Vcenter), VcenterProjected(_VcenterProjected)
{

}

ViewportData_Mesh::ViewportData_Mesh(int _obj_id, const P3s& _V, const I3s& _F, const V3s& _F_normals, const V3s& _V_normals,
    const P3s& _NormalCorrections_V, const V3s& _NormalCorrections_V_normals, const Is& _NormalCorrections_V_correcteIndexes, const I4s& _NormalCorrections_Fcorrections,
    const P3& _Vmin, const P3& _Vmax, const P3& _Vcenter, const P3& _VcenterProjected)
    : Id(_obj_id), V(_V), F(_F), F_normals(_F_normals), V_normals(_V_normals),
    NormalCorrections_V(_NormalCorrections_V), NormalCorrections_V_normals(_NormalCorrections_V_normals), NormalCorrections_V_correctedIndexes(_NormalCorrections_V_correcteIndexes), NormalCorrections_Fcorrections(_NormalCorrections_Fcorrections),
    Vmin(_Vmin), Vmax(_Vmax), Vcenter(_Vcenter), VcenterProjected(_VcenterProjected)
{

}

bool ViewportData_Mesh::IsEmpty() const
{
    return  (V.size() == 0 || F.size() == 0);
}


ViewportData_Meshes::ViewportData_Meshes()
    : VCountTotal(0), FCountTotal(0), Vmin(0, 0, 0), Vmax(0, 0, 0), Vcenter(0, 0, 0), VcenterProjected(0, 0, 0)
{
}

ViewportData_Meshes::ViewportData_Meshes(const vector<ViewportData_Mesh>& _meshes)
    : meshes(_meshes)
{
    int count = meshes.size();
    Ids.reserve(count);
    Ids_sorted.reserve(count);
    bool vminmax_isset = false;
    Vmin = P3(0, 0, 0);
    Vmax = P3(0, 0, 0);
    if (count > 0)
    {
        Vmax = meshes[0].Vmax;
    }
    for (int i = 0; i < meshes.size(); i++)
    {
        const ViewportData_Mesh& m = meshes[i];
        Ids.push_back(m.Id);
        Ids_sorted.push_back(m.Id);

        if (!vminmax_isset)
        {
            vminmax_isset = true;
            Vmin = m.Vmin;
            Vmax = m.Vmax;
        }
        else
        {
            #ifdef USE_EIGEN
            Vmin = P3(min(Vmin(0), m.Vmin(0)), min(Vmin(1), m.Vmin(1)), min(Vmin(2), m.Vmin(2)));
            Vmax = P3(max(Vmax(0), m.Vmax(0)), max(Vmax(1), m.Vmax(1)), max(Vmax(2), m.Vmax(2)));
            #else
            Vmin = utils::sse::min(Vmin, m.Vmin);
            Vmax = utils::sse::max(Vmax, m.Vmax);
            #endif
        }
    }
    std::sort(Ids_sorted.begin(), Ids_sorted.end());

    // Calculate Vcenter
    Vcenter = P3(0, 0, 0);
    DD sum0 = 0;
    DD sum1 = 0;
    DD sum2 = 0;
    VCountTotal = 0;
    FCountTotal = 0;
    for (auto& m : meshes)
    {
        //for (int i = 0; i < m.V.rows(); i++)
        //{
        //    sum0 += m.V(i, 0);
        //    sum1 += m.V(i, 1);
        //    sum2 += m.V(i, 2);
        //}
        sum0 += 1.0*m.Vcenter(0)*m.V.rows();
        sum1 += 1.0*m.Vcenter(1)*m.V.rows();
        sum2 += 1.0*m.Vcenter(2)*m.V.rows();
        VCountTotal += m.V.rows();
        FCountTotal += m.F.rows();
    }
    if (VCountTotal > 0)
    {
        Vcenter = P3(sum0 / (DD)VCountTotal, sum1 / (DD)VCountTotal, sum2 / (DD)VCountTotal);
    }
    VcenterProjected = Vcenter;
}

bool ViewportData_Meshes::IsEmpty() const
{
    //for (auto& m : meshes)
    //    if (!m.IsEmpty()) return false;
    //return true;
    return VCountTotal == 0;
}


II ViewportData_Meshes::SizeOF() const
{
    II r = sizeof(ViewportData_Meshes);

    r += Ids.capacity() * sizeof(int);
    r += Ids_sorted.capacity() * sizeof(int);

    return r;
}

