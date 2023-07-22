#pragma once
class ViewportData_Meshes;

// requiered data for drawing mesh in viewport
class ViewportData_Mesh
{
public:
    int Id;
    const P3s& V;
    const I3s& F;
    const V3s& F_normals;
    const V3s& V_normals;
    const P3s& NormalCorrections_V;
    const V3s& NormalCorrections_V_normals;
    const Is& NormalCorrections_V_correctedIndexes;
    const I4s& NormalCorrections_Fcorrections;
    const P3& Vmin;
    const P3& Vmax;
    const P3& Vcenter;
    const P3& VcenterProjected;

    // Empty constructor - all matricies will be empty and all vectors zero
    ViewportData_Mesh();

    ViewportData_Mesh(int obj_id, const P3s& V, const I3s& F, const V3s& F_normals, const V3s& V_normals,
        const P3& Vmin, const P3& Vmax, const P3& Vcenter, const P3& VcenterProjected);


    ViewportData_Mesh(int obj_id, const P3s& V, const I3s& F,  const V3s& F_normals, const V3s& V_normals,
        const P3s& NormalCorrections_V, const V3s& NormalCorrections_V_normals, const Is& NormalCorrections_V_correctedIndexes, const I4s& NormalCorrections_Fcorrections,
        const P3& Vmin, const P3& Vmax, const P3& Vcenter, const P3& VcenterProjected);
    bool IsEmpty() const;
};


class ViewportData_Meshes
{
public:
    vector<int> Ids;
    vector<int> Ids_sorted;
    vector<ViewportData_Mesh> meshes;
    int VCountTotal;
    int FCountTotal;
    P3 Vmin;
    P3 Vmax;
    P3 Vcenter;
    P3 VcenterProjected;

    ViewportData_Meshes();
    ViewportData_Meshes(const vector<ViewportData_Mesh>& meshes);
    II SizeOF() const;
    bool IsEmpty() const;

};