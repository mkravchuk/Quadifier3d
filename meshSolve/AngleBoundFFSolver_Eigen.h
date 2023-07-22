#pragma once

bool angle_bound_frame_fields_eigen(const MatrixXd &V, const I3s& F,
    const Is& isConstrained, const MatrixXd& initialSolution,
    int maxIter,
    double thetaMin,
    double lambdaInit,
    double lambdaMultFactor,
    bool doHardConstraints,
    MatrixXd& output
    );

bool angle_bound_frame_fields_eigen(const P3s &V, const I3s& F,
    const Bs& isConstrained, const vector<V3s> &initialSolution,
    int maxIter,
    double thetaMin,
    double lambdaInit,
    double lambdaMultFactor,
    bool doHardConstraints,
    vector<V3s>& output);