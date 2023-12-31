#include "writeBF.h"
#include <fstream>
#include <cassert>
template < 
  typename DerivedWI,
  typename DerivedP,
  typename DerivedO>
IGL_INLINE bool igl::writeBF(
  const std::string & filename,
  const Eigen::PlainObjectBase<DerivedWI> & WI,
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<DerivedO> & O)
{
  using namespace Eigen;
  using namespace std;
  const int n = WI.rows();
  assert(n == WI.rows() && "WI must have n rows");
  assert(n == P.rows()  && "P must have n rows");
  assert(n == O.rows()  && "O must have n rows");
  MatrixXd WIPO(n,1+1+3);
  for(int i = 0;i<n;i++)
  {
    WIPO(i,0) = WI(i);
    WIPO(i,1) = P(i);
    WIPO(i,2+0) = O(i,0);
    WIPO(i,2+1) = O(i,1);
    WIPO(i,2+2) = O(i,2);
  }
  ofstream s(filename);
  if(!s.is_open())
  {
    fprintf(stderr,"IOError: writeBF() could not open %s\n",filename.c_str());
    return false;
  }
  s<<
    WIPO.format(IOFormat(FullPrecision,DontAlignCols," ","\n","","","","\n"));
  return true;
}

#ifdef IGL_STATIC_LIBRARY
template bool igl::writeBF<Eigen::VectorXi, Eigen::VectorXi, Eigen::MatrixXd >(std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, Eigen::PlainObjectBase<Eigen::MatrixXd > const&);
#endif
