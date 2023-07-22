#pragma once
#include "CompactVectorVector.h"
class ViewerDrawObjects;

// defines full methods for testing correctness of fast method - currently not supported after change to V3, P3, M3
//#define FULL_FindCrossFieldSingularities_IS_AVAILABLE

class FindCrossFieldSingularities
{
private:
    #ifdef FULL_FindCrossFieldSingularities_IS_AVAILABLE
    int find_cross_field_singularities_full(const MatrixXi& Handle_MMatch, Ds& Result_Singularities);
    inline int calculateMissmatch_MissMatchByCross(const int f0, const int f1, const V3s& PD1, const V3s& PD2);
    void calculateMissmatch_full(const V3s& BIS1, const V3s& BIS2, MatrixXi &Handle_MMatch);
    bool comb_cross_field_full(const V3s& PD1, const V3s& PD2, MatrixXd& PD1out, MatrixXd& PD2out);
    int findCrossFieldSingularities_full(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool log);
    #endif

    int find_cross_field_singularities_fast(const vector<bool>& vertexesToCheck, const MatrixXi& Handle_MMatch, Ds& Result_Singularities);
    void calculateMissmatch_fast(const vector<bool>& checkForFaces, const vector<int>& edges_Ids, const vector<D33>& edges_transformationMatrixes, const V3s& BIS1, const V3s& BIS2, MatrixXi &Handle_MMatch);
    int getFacesWithWeakWeight(const V3s& FF1, const V3s& FF2, Vector<bool>& facesWithWeakWeight, bool f1andf2_isAlmostSameLength);
    void getVertexesToCheckForSing(const V3s& FF1, const V3s& FF2, const Vector<bool>& facesWithWeakWeight, vector<bool>& vertexesToCheck);
    void cacheTransformationMatrixes(const vector<bool>& vertexesToCheck, vector<bool>& checkForFaces, vector<int>& edges_Ids, vector<D33>& edges_transformationMatrixes);
    int findCrossFieldSingularities_fast(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool log, bool f1andf2_isAlmostSameLength);
public:
    ViewerDrawObjects& draw;
    const P3s& V;
    const I3s& F;
    const V3s& F_X;
    const V3s& F_Y;
    const V3s& F_normals;
    const P3s& F_Barycenters;
    const I3s& FE;
    const I2s& EV;
    const I2s& EF;
    const I3s& FF;
    const Bs&  V_border;
    const CompactVectorVector<int>& VF;

    FindCrossFieldSingularities(ViewerDrawObjects& draw, const P3s& V, const I3s& F,
        const V3s& F_X, const V3s& F_Y, const V3s& F_normals, const P3s& F_Barycenters,
        const I3s& FE, const I2s& EV, const I2s& EF, const I3s& FF,
        const Bs&  V_border, const CompactVectorVector<int>& VF);

    int findCrossFieldSingularities(const V3s& FF1, const V3s& FF2, Ds& Result_Singularities, bool f1andf2_isAlmostSameLength);
};
