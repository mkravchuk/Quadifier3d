#pragma once

class ViewerDrawObjects;
struct FaceConstrain;


void n_polyvector_polar2x(
    ViewerDrawObjects& draw,
    int meshid, // use for debugging only
    const P3s &V,
    const I3s& F,
    const Ds &F_Areas,
    const Bs& E_isborder,
    const I2s& EV,
    const Ds& E_Length,
    const I3s& FE,
    const I2s& EF,
    const Ds& K,
    const V3s& F_X,
    const V3s& F_Y,
    const V3s& F_Z,
    const D avg_edge_length,
    const D max_edge_length,
    const vector<FaceConstrain>& Constrains,  // Constrained faces representative vector
    D softPercent, // ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    vector<V3s> &Result_F, bool& Result_F_isSorted,
    bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected);
