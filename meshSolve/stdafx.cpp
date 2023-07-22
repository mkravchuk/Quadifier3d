// stdafx.cpp : source file that includes just the standard includes
// Quadifier3d.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"

//// TODO: reference any additional headers you need in STDAFX.H and not in this file
//#pragma optimize( "", off )  
//SparseMatrix<complex<D>, ColMajor>  __scol;
//SparseMatrix<complex<D>, RowMajor>  __srow;
//Matrix<complex<D>, Dynamic, Dynamic, ColMajor> __mcol;
//Matrix<complex<D>, Dynamic, Dynamic, RowMajor> __mrow;
//SimplicialLDLT    < SparseMatrix<complex<D>, ColMajor>> __solvercol;
//#pragma optimize( "", on )   
// 



//// Type caching
//#undef EIGEN_DBG_SPARSE
//#undef EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE
//
//// Single
//template class SparseMatrix<complex<float>, ColMajor>;
//template class Matrix<complex<float>, Dynamic, 1>;
//template class SparseTriangularView < SparseMatrix<complex<float>, ColMajor>, UnitLower>;
////template class SparseTriangularView<typename SparseMatrix<complex<float>, ColMajor>::AdjointReturnType, UnitUpper>;
//template class Matrix<complex<float>, Dynamic, Dynamic, ColMajor>;
//template class AMDOrdering<int>;
//template class Matrix<float, Dynamic, 2>;
//
//// 2x
//
//std::ostream & operator << (std::ostream & s, const Complex4f& m)
//{
//    s << m.get_low().toComplex() << m.get_high().toComplex();
//    return s;
//}
//template class SparseMatrix<Complex4f, ColMajor>;
//template class Matrix<Complex4f, Dynamic, 1>;
//template class SparseTriangularView<SparseMatrix<Complex4f, ColMajor>, UnitLower>;
////template class SparseTriangularView<typename SparseMatrix<Complex4f, ColMajor>::AdjointReturnType, UnitUpper>;
//template class Matrix<Complex4f, Dynamic, Dynamic, ColMajor>;
//template class Matrix<Complex2f, Dynamic, 2>;