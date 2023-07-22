#pragma once

class ViewerDrawObjects;
struct FaceConstrain;



void n_polyvector_general_soft(
    ViewerDrawObjects& draw,
    const P3s &V,
    const I3s& F,
    const Ds &F_Areas,
    const I2s& EV,
    const I3s& FE,
    const I2s& EF,
    const Ds& K,
    const V3s& F_X,
    const V3s& F_Y,
    const V3s& F_Z,
    vector<FaceConstrain> Constrains,  // Constrained faces representative vector
    D softPercent, // ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
    vector<V3s>& Result_F, bool takeDirectionCorrected);
