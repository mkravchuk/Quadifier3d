#include "stdafx.h"
#include "Utils_vector.h"
#include "Utils_num.h"
#include "Utils_angle.h"
#include "Utils_point.h"
#include "Utils_angle.h"
#include "Utils_cpu.h"

#include <igl/rotation_matrix_from_directions.h>


namespace utils
{
    namespace vector
    {

        V3 Rotate(const V3& v, V3 rotationAxis, D angleInDegrees, bool isnormalized_rotationAxis)
        {
            //https://open.gl/transformations
            // v1 - using opennurbs
            //ON_3dVector onv = ON_3dVector(v.x(), v.y(), v.z());
            //ON_3dVector onaxis = ON_3dVector(rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
            //D angleInRadians = angle::DegreesToRadians(angleInDegrees);
            //onv.Rotate(angleInRadians, onaxis);
            //V3 vRotated1 =  V3(onv.x, onv.y, onv.z);

            //v2
            double angleInRadians2 = angle::DegreesToRadians(angleInDegrees);
            if (!isnormalized_rotationAxis) rotationAxis.normalize();
            Eigen::AngleAxis<double> aa(angleInRadians2, convertV3ToEigenDouble(rotationAxis));
            auto vRotated2 = aa * convertV3ToEigenDouble(v); // rotate vector v using transformation class AngleAxis

            return convertEigenToV3(vRotated2);
            //cout << "error!!!   Rotate temporaly is disabled - please uncomment code to enable it!!!" << endl;
            //return V3(0, 0, 0); 
        }

        V3 Translate(const V3& v, const V3& normalTo, bool preserveLength)
        {
            V3 coonormal = Cross(v, normalTo);
            V3 vTranslated = Cross(normalTo, coonormal);
            if (preserveLength)
            {
                //vTranslated.normalize();
                //vTranslated *= Length(v);
                vTranslated *= sqrt(LengthPow2(v) / LengthPow2(vTranslated));
            }
            return vTranslated;
        }

        bool rotation_matrix_from_directions(const V3& v0, const V3& v1, D33& rotM, bool vectorsAreNormalized)
        {
            const D epsilon_cos = 0.0000001f;// only 1e-7 allowed, other values fails at ' if (vo_v1_cos > 1 - epsilon_cos)' condition for floats
            D vo_v1_cos = utils::vector::Cos(v0, v1, vectorsAreNormalized);

            //D vo_v1_cos_v = utils::vector::Cos(v0, v1, vectorsAreNormalized);
            //D diff = abs(vo_v1_cos - vo_v1_cos_v);
            //bool is0 = (vo_v1_cos > 1 - epsilon_cos);
            //bool is1 = vo_v1_cos < -(1 - epsilon_cos);
            //bool is0v = (vo_v1_cos_v > 1 - epsilon_cos);
            //bool is1v = vo_v1_cos_v < -(1 - epsilon_cos);
            //if (is0 != is0v || is1 != is1v)
            //{
            //    D v0norm = v0.norm();
            //    D v1norm = v1.norm();
            //    cout << "v0.norm=" << v0norm << "   v1.norm=" << v1norm << "   vo_v1_cos=" << vo_v1_cos << "   vo_v1_cos_v=" << vo_v1_cos_v << endl;
            //}

            //validate if there is no rotation
            if (vo_v1_cos > 1 - epsilon_cos)
            {
                rotM = D33::Identity();
                return true;
            }
            //validate if there is no rotation
            if (vo_v1_cos < -(1 - epsilon_cos))
            {
                rotM = -D33::Identity();
                rotM(0, 0) = 1.;
                std::cerr << "utils::vector::rotation_matrix_from_directions: rotating around x axis by 180 degree" << std::endl;
                return true;
            }

            // get axis of v0 v1
            V3 axis = Cross(v0, v1);
            D vo_v1_sin = axis.norm_sse41();
            axis /= vo_v1_sin; // normalize axis
            if (isnan(axis(0))) // sometimes condition 'if (vo_v1_cos > 1 - epsilon_cos)' doesnt recognize parallel vectors, so this is second check
            {
                if (vo_v1_cos > 0)
                {
                    rotM = D33::Identity();
                    return true;
                }
                else
                {
                    rotM = -D33::Identity();
                    rotM(0, 0) = 1.;
                    std::cerr << "utils::vector::rotation_matrix_from_directions: rotating around x axis by 180 degree" << std::endl;
                    return true;
                }
            }

            //construct rotation matrix
            D u = axis(0);
            D v = axis(1);
            D w = axis(2);
            //D dot = Dot(v0, v1);
            //D phi = acos(dot);
            //D rcos = cos(phi);
            D rcos = vo_v1_cos;
            //D rsin = sin(phi);
            D rsin = vo_v1_sin;
            D _1_minus_rcos = (1 - rcos);

            rotM(0, 0) = rcos + u * u*_1_minus_rcos;
            rotM(1, 0) = w * rsin + v * u*_1_minus_rcos;
            rotM(2, 0) = -v * rsin + w * u*_1_minus_rcos;
            rotM(0, 1) = -w * rsin + u * v*_1_minus_rcos;
            rotM(1, 1) = rcos + v * v*_1_minus_rcos;
            rotM(2, 1) = u * rsin + w * v*_1_minus_rcos;
            rotM(0, 2) = v * rsin + u * w*_1_minus_rcos;
            rotM(1, 2) = -u * rsin + v * w*_1_minus_rcos;
            rotM(2, 2) = rcos + w * w*_1_minus_rcos;
            return true;
        }

        bool rotation_matrix_from_directions_sse41(const V3& v0, const V3& v1, D33& rotM, bool vectorsAreNormalized)
        {
            const D epsilon_cos = 0.0000001f; // only 1e-7 allowed, other values fails at ' if (vo_v1_cos > 1 - epsilon_cos)' condition for floats
            D vo_v1_cos = utils::vector::Cos_sse41(v0, v1, vectorsAreNormalized);

            //D vo_v1_cos_v = utils::vector::Cos(v0, v1);
            //D diff = abs(vo_v1_cos - vo_v1_cos_v);
            //bool is0 = (vo_v1_cos > 1 - epsilon_cos);
            //bool is1 = vo_v1_cos < -(1 - epsilon_cos);
            //bool is0v = (vo_v1_cos_v > 1 - epsilon_cos);
            //bool is1v = vo_v1_cos_v < -(1 - epsilon_cos);
            //if (is0 != is0v || is1 != is1v)
            //{
            //    D v0norm = v0.norm();
            //    D v1norm = v1.norm();
            //    cout << "v0.norm=" << v0norm << "   v1.norm=" << v1norm << "   vo_v1_cos=" << vo_v1_cos << "   vo_v1_cos_v=" << vo_v1_cos_v << endl;
            //}

            //validate if there is no rotation
            if (vo_v1_cos > 1 - epsilon_cos)
            {
                rotM = D33::Identity();
                return true;
            }
            //validate if there is no rotation
            if (vo_v1_cos < -(1 - epsilon_cos))
            {
                rotM = -D33::Identity();
                rotM(0, 0) = 1.;
                std::cerr << "utils::vector::rotation_matrix_from_directions: rotating around x axis by 180 degree" << std::endl;
                return true;
            }

            // get axis of v0 v1
            V3 axis = Cross(v0, v1);
            D vo_v1_sin = axis.norm_sse41();
            axis /= vo_v1_sin; // normalize axis
            if (isnan(axis(0))) // sometimes condition 'if (vo_v1_cos > 1 - epsilon_cos)' doesnt recognize parallel vectors, so this is second check
            {
                if (vo_v1_cos > 0)
                {
                    rotM = D33::Identity();
                    return true;
                }
                else
                {
                    rotM = -D33::Identity();
                    rotM(0, 0) = 1.;
                    std::cerr << "utils::vector::rotation_matrix_from_directions: rotating around x axis by 180 degree" << std::endl;
                    return true;
                }
            }

            //construct rotation matrix
            D u = axis(0);
            D v = axis(1);
            D w = axis(2);
            //D dot = Dot(v0, v1);
            //D phi = acos(dot);
            //D rcos = cos(phi);
            //D rsin = sin(phi);
            D rcos = vo_v1_cos;
            D rsin = vo_v1_sin;
            D _1_minus_rcos = (1 - rcos);

            //v0
            //rotM(0, 0) = rcos + u * u*_1_minus_rcos;
            //rotM(1, 0) = w * rsin + v * u*_1_minus_rcos;
            //rotM(2, 0) = -v * rsin + w * u*_1_minus_rcos;
            //rotM(0, 1) = -w * rsin + u * v*_1_minus_rcos;
            //rotM(1, 1) = rcos + v * v*_1_minus_rcos;
            //rotM(2, 1) = u * rsin + w * v*_1_minus_rcos;
            //rotM(0, 2) = v * rsin + u * w*_1_minus_rcos;
            //rotM(1, 2) = -u * rsin + v * w*_1_minus_rcos;
            //rotM(2, 2) = rcos + w * w*_1_minus_rcos;

            //v1
            //V3 axis_1_minus_rcos = axis * _1_minus_rcos;
            //rotM(0, 0) = rcos + u * axis_1_minus_rcos(0);
            //rotM(1, 0) = w * rsin + v * axis_1_minus_rcos(0);
            //rotM(2, 0) = -v * rsin + w * axis_1_minus_rcos(0);
            //rotM(0, 1) = -w * rsin + u * axis_1_minus_rcos(1);
            //rotM(1, 1) = rcos + v * axis_1_minus_rcos(1);
            //rotM(2, 1) = u * rsin + w * axis_1_minus_rcos(1);
            //rotM(0, 2) = v * rsin + u * axis_1_minus_rcos(2);
            //rotM(1, 2) = -u * rsin + v * axis_1_minus_rcos(2);
            //rotM(2, 2) = rcos + w * axis_1_minus_rcos(2);

            //v2
            //V3 axis_1_minus_rcos = axis * _1_minus_rcos;
            //V3 axis_0 = axis * axis_1_minus_rcos(0);
            //V3 axis_1 = axis * axis_1_minus_rcos(1);
            //V3 axis_2 = axis * axis_1_minus_rcos(2);
            //rotM(0, 0) = rcos + axis_0(0);
            //rotM(1, 0) = w * rsin + axis_0(1);
            //rotM(2, 0) = -v * rsin + axis_0(2);
            //rotM(0, 1) = -w * rsin + axis_1(0);
            //rotM(1, 1) = rcos + axis_1(1);
            //rotM(2, 1) = u * rsin + axis_1(2);
            //rotM(0, 2) = v * rsin + axis_2(0);
            //rotM(1, 2) = -u * rsin + axis_2(1);
            //rotM(2, 2) = rcos + axis_2(2);

            //v3
            //V3 axis_1_minus_rcos = axis * _1_minus_rcos;
            //V3 axis_0 = axis * axis_1_minus_rcos(0);
            //V3 axis_1 = axis * axis_1_minus_rcos(1);
            //V3 axis_2 = axis * axis_1_minus_rcos(2);
            //rotM.row(0) = Vector34f(rcos + axis_0(0), -w * rsin + axis_1(0), v * rsin + axis_2(0));
            //rotM.row(1) = Vector34f(w * rsin + axis_0(1), rcos + axis_1(1), -u * rsin + axis_2(1));
            //rotM.row(2) = Vector34f(-v * rsin + axis_0(2), u * rsin + axis_1(2), rcos + axis_2(2));

            //v4
            V3 axis_1_minus_rcos = axis * _1_minus_rcos;
            V3 axis_0 = axis * axis_1_minus_rcos(0) + Vector34f(rcos, w * rsin, -v * rsin);
            V3 axis_1 = axis * axis_1_minus_rcos(1) + Vector34f(-w * rsin, rcos, u * rsin);
            V3 axis_2 = axis * axis_1_minus_rcos(2) + Vector34f(v * rsin, -u * rsin, rcos);
            rotM.row(0) = Vector34f(axis_0(0), axis_1(0), axis_2(0));
            rotM.row(1) = Vector34f(axis_0(1), axis_1(1), axis_2(1));
            rotM.row(2) = Vector34f(axis_0(2), axis_1(2), axis_2(2));

            return true;
        }

        V3 Translate(const V3& v, const V3& normalFrom, const V3& normalTo, bool normalsAreNormalized)
        {
            //https://open.gl/transformations
            // v1 - my method - not very strong approach
            //D facesRotationAngle = utils::vector::Angle(normalFrom, normalTo);
            //if (facesRotationAngle > 0.00001) // we can calculate cross of normals only if there is some angle between normals
            //{
            //    V3 facesRotationAxis = normalFrom.cross(normalTo);
            //    V3 vTranslated = utils::vector::Rotate(v, facesRotationAxis, facesRotationAngle);
            //    return vTranslated;
            //}
            //return v;

            // v2 - strong mathematical approach (EIGEN)
            //Matrix3f rotMEigen = igl::rotation_matrix_from_directions(convertV3ToEigen(normalFrom), convertV3ToEigen(normalTo));
            //Vector3f vTranslatedEigen = rotMEigen *convertV3ToEigen(v);
            //V3 vTranslatedIGL = convertEigenToV3(vTranslatedEigen);

            // v3 - expanded 'igl::rotation_matrix_from_directions'
            D33 rotM;
            if (utils::cpu::isSupportedSSE4)
            {
                rotation_matrix_from_directions_sse41(normalFrom, normalTo, rotM, normalsAreNormalized);
                return Vector34f(rotM.row(0).dot_sse41(v), rotM.row(1).dot_sse41(v), rotM.row(2).dot_sse41(v));
            }
            else
            {
                rotation_matrix_from_directions(normalFrom, normalTo, rotM, normalsAreNormalized);
                V3 vTranslated = rotM * v;
                return vTranslated;
            }
        }

        P3 ClosestPoint_ToLine(const P3& linePoint1, const P3& linePoint2, const P3& p, int& closestPointIsOnCornerPoint012)
        {
            // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
            V3 u = linePoint2 - linePoint1;
            D uLengthPow2 = LengthPow2(u);
            V3 PQ = p - linePoint1;

            //v1
            //V3 w2 = PQ - u * (PQ.dot(u) / uLengthPow2);
            //P3 closestPoint = p - w2;
            // if ortogonal point is out of line - set closest point to point1 or point2
            //D distP1 = utils::vector::LengthPow2(linePoint1 - closestPoint);
            //D distP2 = utils::vector::LengthPow2(linePoint2 - closestPoint);
            //if (distP1 > uLengthPow2 || distP2 > uLengthPow2)
            //{
            //    if (distP1 > distP2)
            //    {
            //        closestPoint = linePoint2;
            //        closestPointIsOnCornerPoint012 = 2;
            //    }
            //    else
            //    {
            //        closestPoint = linePoint1;
            //        closestPointIsOnCornerPoint012 = 1;
            //    }
            //}


            //v2
            D PQ_dot_u = Dot(PQ, u);
            if (PQ_dot_u < 0)
            {
                closestPointIsOnCornerPoint012 = 1;
                return linePoint1;
            }
            if (PQ_dot_u > uLengthPow2)
            {
                closestPointIsOnCornerPoint012 = 2;
                return linePoint2;
            }
            closestPointIsOnCornerPoint012 = 0;
            return linePoint1 + u * (PQ_dot_u / uLengthPow2);
        }

        D ClosestPoint_ToLine_IsOnLinePercent(const P3& linePoint1, const P3& linePoint2, const P3& p)
        {
            V3 line = linePoint2 - linePoint1;
            D lineLengthPow2 = LengthPow2(line);
            V3 PQ = p - linePoint1;
            D PQ_dot_u = Dot(PQ, line);
            return PQ_dot_u / lineLengthPow2;
        }
        int ClosestPoint_ToLine_IsOutOfLineSign(const P3& linePoint1, const P3& linePoint2, const P3& p)
        {
            D precent = ClosestPoint_ToLine_IsOnLinePercent(linePoint1, linePoint2, p);
            if (precent < 0) return -1;
            if (precent > 1) return 1;
            return 0;
        }

        bool ClosestPoint_ToLine_IsOnLine(const P3& linePoint1, const P3& linePoint2, const P3& p)
        {
            D precent = ClosestPoint_ToLine_IsOnLinePercent(linePoint1, linePoint2, p);
            return !(precent < 0 || precent > 1);
        }

        P3 ClosestPoint_ToLine(const P3& linePoint1, const P3& linePoint2, const P3& p)
        {
            int closestPointIsOnCornerPoint012 = 0;
            return ClosestPoint_ToLine(linePoint1, linePoint2, p, closestPointIsOnCornerPoint012);
        }

        bool FindIntersectionBetween_EdgeAndVectorPow2(const P3 &edgev0, const P3 &edgev1, const P3 &dirv0, const V3 &dir, D &edgePosPow2)
        {
            const D eps = 1e-6f;
            const D epsPow2 = eps * eps;
            const D epsPlus1 = 1 + eps;

            // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
            // Search intersection between two segments
            // p1 + t*v1 :  t \in [0,1]
            // p2 + u*v2 :  u \in [0,1]

            // p1 + t * v1 = p2 + u * v2  // x v2
            // t(v1 x v2) = (p2 - p1) x v2
            // t = (p2 - p1) x v2 / (v1 x v2)

            V3 edge = edgev1 - edgev0;
            V3 fromP2toP1 = dirv0 - edgev0;

            // (v1 x v2) ~ 0 --> directions are parallel, they will never cross
            V3 rxs = Cross(edge, dir);
            D rxs_normPow2 = LengthPow2(rxs);
            if (rxs_normPow2 <= epsPow2) // if there is suspicious to be directions parallel - test precise
            {
                V3 rxsNorm = edge.normalized().cross(dir.normalized());
                D rxsNorm_norm = LengthPow2(rxsNorm);
                if (rxsNorm_norm <= epsPow2)
                {
                    edgePosPow2 = 0;
                    return false;
                }
            }

            V3 t1 = Cross(fromP2toP1, dir);
            int sign = ((Dot(t1, rxs)) > 0) ? 1 : -1;
            //cout << "t sign = " << sign << endl;
            edgePosPow2 = LengthPow2(t1) / rxs_normPow2;
            edgePosPow2 = edgePosPow2 * sign;

            return true;
        }
        bool FindIntersectionBetween_EdgeAndVector(const P3 &edgev0, const P3 &edgev1, const P3 &dirv0, const V3 &dir, D &edgePos)
        {
            D edgePosPow2;
            if (!FindIntersectionBetween_EdgeAndVectorPow2(edgev0, edgev1, dirv0, dir, edgePosPow2))
            {
                edgePos = 0;
                return false;
            }
            edgePos = num::sqrtsign(edgePosPow2);
            return true;
        }

        bool FindIntersectionBetween_VectorsPow2(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos1Pow2, D &pos2Pow2)
        {
            const D eps = 1e-6f;
            const D epsPow2 = eps * eps;

            // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
            // given: p,r,t  and q,s,u
            // t = (q − p) × s / (r × s)
            // u = (q − p) × r / (r × s)     because originally 'u = (p − q) × r / (s × r)'   and  since 's × r'=='- r × s' we can simplify by moving minus from 's × r' to '(p-q)' 
            //   = 
            //
            // Search intersection between two segments
            // p1 + t*v1 :  t \in [0,1]
            // p2 + u*v2 :  u \in [0,1]

            // p1 + t * v1 = p2 + u * v2  // x v2
            // t(v1 x v2) = (p2 - p1) x v2
            // t = (p2 - p1) x v2 / (v1 x v2)

            V3 fromP2toP1 = point2 - point1; // q − p

            // (v1 x v2) ~ 0 --> directions are parallel, they will never cross
            V3 rxs = Cross(dir1, dir2);
            D rxs_normPow2 = LengthPow2(rxs);
            if (rxs_normPow2 <= epsPow2) // if there is suspicious to be directions parallel - test precise
            {
                V3 rxsNorm = dir1.normalized().cross(dir2.normalized());
                D rxsNorm_norm = LengthPow2(rxsNorm);
                if (rxsNorm_norm <= epsPow2)
                {
                    pos1Pow2 = 0;
                    pos2Pow2 = 0;
                    return false;
                }
            }

            //bool edgesAreIntersecting = true;

            // pos2 = (p2 - p1) × dir1 / (dir1 × dir2)
            V3 u = Cross(fromP2toP1, dir1);
            int u_sign = ((Dot(u, rxs)) > 0) ? 1 : -1;
            //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
            pos2Pow2 = (LengthPow2(u) / rxs_normPow2) * u_sign;

            //pos1 = (p2 − p1) × dir2 / (dir1 × dir2)
            V3 t = Cross(fromP2toP1, dir2);
            int t_sign = ((Dot(t, rxs)) > 0) ? 1 : -1;
            //cout << "t sign = " << sign << endl;
            pos1Pow2 = (LengthPow2(t) / rxs_normPow2)*t_sign;

            return true;
        }

        bool FindIntersectionBetween_VectorsPow2(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos2Pow2)
        {
            const D eps = 1e-6f;
            const D epsPow2 = eps * eps;

            // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
            // given: p,r,t  and q,s,u
            // t = (q − p) × s / (r × s)
            // u = (q − p) × r / (r × s)     because originally 'u = (p − q) × r / (s × r)'   and  since 's × r'=='- r × s' we can simplify by moving minus from 's × r' to '(p-q)' 
            //   = 
            //
            // Search intersection between two segments
            // p1 + t*v1 :  t \in [0,1]
            // p2 + u*v2 :  u \in [0,1]

            // p1 + t * v1 = p2 + u * v2  // x v2
            // t(v1 x v2) = (p2 - p1) x v2
            // t = (p2 - p1) x v2 / (v1 x v2)

            V3 fromP2toP1 = point2 - point1; // q − p

            // (v1 x v2) ~ 0 --> directions are parallel, they will never cross
            V3 rxs = Cross(dir1, dir2);
            D rxs_normPow2 = LengthPow2(rxs);
            if (rxs_normPow2 <= epsPow2) // if there is suspicious to be directions parallel - test precise
            {
                V3 rxsNorm = dir1.normalized().cross(dir2.normalized());
                D rxsNorm_norm = LengthPow2(rxsNorm);
                if (rxsNorm_norm <= epsPow2)
                {
                    pos2Pow2 = 0;
                    return false;
                }
            }

            //bool edgesAreIntersecting = true;

            // pos2 = (p2 - p1) × dir1 / (dir1 × dir2)
            V3 u = Cross(fromP2toP1, dir1);
            int u_sign = ((Dot(u, rxs)) > 0) ? 1 : -1;
            //cout << "u sign = " << sign <<   "     u = "<< u1<< endl;
            pos2Pow2 = (LengthPow2(u) / rxs_normPow2) * u_sign;

            return true;
        }

        bool FindIntersectionBetween_Vectors(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos1, D &pos2)
        {
            D pos1Pow2;
            D pos2Pow2;
            if (!FindIntersectionBetween_VectorsPow2(point1, dir1, point2, dir2, pos1Pow2, pos2Pow2))
            {
                pos1 = 0;
                pos2 = 0;
                return false;
            }
            pos1 = num::sqrtsign(pos1Pow2);
            pos2 = num::sqrtsign(pos2Pow2);
            return true;
        }

        bool FindIntersectionBetween_Vectors(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos2)
        {
            D pos2Pow2;
            if (!FindIntersectionBetween_VectorsPow2(point1, dir1, point2, dir2, pos2Pow2))
            {
                pos2 = 0;
                return false;
            }
            pos2 = num::sqrtsign(pos2Pow2);
            return true;
        }


        void ClosestPoint_BetweenLines_iterative(const P3& line1Point1, const P3& line1Point2, const P3& line2Point1, const P3& line2Point2,
            P3& closestPointOnLine1, P3& closestPointOnLine2, D& minDistBetweenLines, int& line1_ClosestPointIsOnCornerPoint012)
        {
            V3 line1 = line1Point2 - line1Point1;
            D line1LengthPow2 = utils::vector::LengthPow2(line1);
            line1_ClosestPointIsOnCornerPoint012 = 0;

            // if first line is of zero length - take fist point as closest
            if (line1LengthPow2 < 0.000000000001)
            {
                closestPointOnLine1 = line1Point1;
                closestPointOnLine2 = ClosestPoint_ToLine(line2Point1, line2Point2, closestPointOnLine1);
                V3 dist = closestPointOnLine1 - closestPointOnLine2;
                minDistBetweenLines = dist.norm();
                return;
            }

            auto closest = [&line1Point1, &line1Point2, &line1, &line1LengthPow2](const P3&p, P3& closestPoint, D& closestDistPow2, int& line1_ClosestPointIsOnCornerPoint012)
            {
                V3 PQ = p - line1Point1;
                V3 w2 = PQ - line1 * (utils::vector::Dot(PQ, line1) / line1LengthPow2);
                closestPoint = p - w2;
                D distP1 = utils::vector::LengthPow2(line1Point1 - closestPoint);
                D distP2 = utils::vector::LengthPow2(line1Point2 - closestPoint);
                line1_ClosestPointIsOnCornerPoint012 = 0;
                if (distP1 > line1LengthPow2 || distP2 > line1LengthPow2)
                {
                    if (distP1 > distP2)
                    {
                        closestPoint = line1Point2;
                        line1_ClosestPointIsOnCornerPoint012 = 2;
                    }
                    else
                    {
                        closestPoint = line1Point1;
                        line1_ClosestPointIsOnCornerPoint012 = 1;
                    }
                }
                closestDistPow2 = utils::vector::LengthPow2(closestPoint - p);
            };

            P3 p1 = line2Point1;
            P3 p2 = line2Point2;
            int iMax = 15;
            for (int i = 0; i < iMax; i++) // after 14 divides by 2 we will get 0.005% of the length what is super precise calculation
            {
                P3 pMid = (p1 + p2) / 2;
                P3 pMid_closestPoint;
                D pMid_closestDistPow2;
                closest(pMid, pMid_closestPoint, pMid_closestDistPow2, line1_ClosestPointIsOnCornerPoint012);

                P3 p1_closestPoint;
                D p1_closestDistPow2;
                closest(p1, p1_closestPoint, p1_closestDistPow2, line1_ClosestPointIsOnCornerPoint012);
                if (p1_closestDistPow2 < pMid_closestDistPow2)
                {
                    if (i == iMax - 1)
                    {
                        closestPointOnLine1 = p1_closestPoint;
                        closestPointOnLine2 = p1;
                        minDistBetweenLines = sqrt(p1_closestDistPow2);
                        return;
                    }
                    p1 = p1;
                    p2 = pMid;

                    continue;
                }

                P3 p2_closestPoint;
                D p2_closestDistPow2;
                closest(p2, p2_closestPoint, p2_closestDistPow2, line1_ClosestPointIsOnCornerPoint012);
                if (p2_closestDistPow2 < pMid_closestDistPow2)
                {
                    if (i == iMax - 1)
                    {
                        closestPointOnLine1 = p2_closestPoint;
                        closestPointOnLine2 = p2;
                        minDistBetweenLines = sqrt(p2_closestDistPow2);
                        return;
                    }
                    p1 = p2;
                    p2 = pMid;
                    continue;
                }

                if (i == iMax - 1)
                {
                    closestPointOnLine1 = pMid_closestPoint;
                    closestPointOnLine2 = pMid;
                    minDistBetweenLines = sqrt(pMid_closestDistPow2);
                    return;
                }
                p1 = (p1 + pMid) / 2;
                p2 = (p2 + pMid) / 2;
            }


        }

        void ClosestPoint_BetweenLines(const P3& line1Point1, const P3& line1Point2, const P3& line2Point1, const P3& line2Point2,
            P3& closestPointOnLine1, P3& closestPointOnLine2, D& minDistBetweenLines, int& line1_ClosestPointIsOnCornerPoint012)
        {
            V3 dir1 = line1Point2 - line1Point1;
            V3 dir2 = line2Point2 - line2Point1;
            D pos1_pow2;
            D pos2_pow2;
            FindIntersectionBetween_VectorsPow2(line1Point1, dir1, line2Point1, dir2, pos1_pow2, pos2_pow2);

            line1_ClosestPointIsOnCornerPoint012 = 0;
            if (pos1_pow2 < 0 * 0) line1_ClosestPointIsOnCornerPoint012 = 1;
            if (pos1_pow2 > 1 * 1) line1_ClosestPointIsOnCornerPoint012 = 2;

            int line2_ClosestPointIsOnCornerPoint012 = 0;
            if (pos2_pow2 < 0 * 0) line2_ClosestPointIsOnCornerPoint012 = 1;
            if (pos2_pow2 > 1 * 1) line2_ClosestPointIsOnCornerPoint012 = 2;

            closestPointOnLine1 = line1Point1 + dir1 * num::sqrtsign(pos1_pow2);
            if (line1_ClosestPointIsOnCornerPoint012 == 0 && line2_ClosestPointIsOnCornerPoint012 == 0)
            {
                closestPointOnLine2 = closestPointOnLine1;
                minDistBetweenLines = 0;
            }
            else
            {
                return ClosestPoint_BetweenLines_iterative(line1Point1, line1Point2, line2Point1, line2Point2,
                    closestPointOnLine1, closestPointOnLine2, minDistBetweenLines, line1_ClosestPointIsOnCornerPoint012);
            }
        }



    }

}

