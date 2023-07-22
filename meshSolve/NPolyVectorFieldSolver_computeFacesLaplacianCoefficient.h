#pragma once
#include "SolverTypes.h"


void computeFacesLaplacianCoefficient(int facesCount, const MatrixXi& EF, const VectorXd& K, const VectorXi& isBorderEdge,
    const VectorXi &isFaceConstrained, int numConstrained,
    int degree, SparceMatrixType &Quu, SparceMatrixType &Quk);