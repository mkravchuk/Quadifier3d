#pragma once
#include "Utils_angle.h"
#include "Utils_sse.h"

namespace utils
{
    namespace vector
    {


        //A • B = |A| |B| cos(θ)
        __forceinline D Dot(const V3& v1, const V3& v2)
        {
            #ifdef USE_EIGEN
            //v0
            //return v1.dot(v2);
            //v1
            //return v1(0)*v2(0) + v1(1)*v2(1) + v1(2)*v2(2);
            //v2 - fast even in debug mode
            const D* a = v1.data();
            const D* b = v2.data();
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            #else
            return v1.dot(v2);
            #endif
        }
        __forceinline D Dot_sse41(const V3& v1, const V3& v2)
        {
            return v1.dot_sse41(v2);
        }

        //A × B = |A| |B| sin(θ) n
        __forceinline V3 Cross(const V3& v1, const V3& v2)
        {
            #ifdef USE_EIGEN
            // v0 - slow
            //return v1.cross(v2);
            // v1 - fast
            //V3 r;
            //r(0) = v1(1) * v2(2) - v1(2) * v2(1);
            //r(1) = v1(2) * v2(0) - v1(0) * v2(2);
            //r(2) = v1(0) * v2(1) - v1(1) * v2(0);
            // v2 - fast even in debug mode
            const D* a = v1.data();
            const D* b = v2.data();
            D r0 = a[1] * b[2] - a[2] * b[1];
            D r1 = a[2] * b[0] - a[0] * b[2];
            D r2 = a[0] * b[1] - a[1] * b[0];
            return V3(r0, r1, r2);
            #else
            return v1.cross(v2);
            #endif
        }

        __forceinline void Minus(V3& v, const V3& v2)
        {
            #ifdef USE_EIGEN
            D* a = v.data();
            const D* b = v.data();
            a[0] -= b[0];
            a[1] -= b[1];
            a[2] -= b[2];
            #else
            v -= v2; 
            #endif
        }

        __forceinline D LengthPow2(const V3& v)
        {
            #ifdef USE_EIGEN
            //v1
            //return v(0)*v(0) + v(1)*v(1) + v(2)*v(2);
            //v2 - fast even in debug mode
            const D* a = v.data();
            return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
            #else
            return v.normPow2();
            #endif
        }
        __forceinline D LengthPow2_sse41(const V3& v)
        {
            #ifdef USE_EIGEN
            //v1
            //return v(0)*v(0) + v(1)*v(1) + v(2)*v(2);
            //v2 - fast even in debug mode
            const D* a = v.data();
            return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
            #else
            return v.normPow2_sse41();
            #endif
        }

        __forceinline D Length(const V3& v)
        {
            //v1
            // return v.norm();
            //v2
            //return sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
            //v3
            #ifdef USE_EIGEN
            return sqrt(LengthPow2(v));
            #else
            return v.norm();
            #endif
        }

        __forceinline D Length_sse41(const V3& v)
        {
            //v1
            // return v.norm();
            //v2
            //return sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
            //v3
            #ifdef USE_EIGEN
            return sqrt(LengthPow2(v));
            #else
            return v.norm_sse41();
            #endif
        }
        __forceinline D Sin(const V3& v1, const V3& v2, bool vectorsAreNormalized = false)
        {
            //return v1.normalized().eval().cross(v2.normalized()).eval().norm();
            V3 cross = Cross(v1, v2);
            if (!vectorsAreNormalized)
            {
                //v1
                //D lenv1v2 = sqrt(LengthPow2(v1)*LengthPow2(v2));
                //cross /= lenv1v2;
                //v2 - faster from v1 since we dont use sqrt and we devide only number but not vector 
                D lenv1v2Pow2 = LengthPow2(v1)*LengthPow2(v2);
                return Length(cross) / lenv1v2Pow2; 
            }
            return Length(cross);
        }
        __forceinline D Cos(const V3& v1, const V3& v2, bool vectorsAreNormalized = false)
        {
            //return v1.normalized().eval().dot(v2.normalized());
            D dot = Dot(v1, v2);
            if (!vectorsAreNormalized)
            {
                D lenv1v2 = utils::sse::sqrt(LengthPow2(v1)*LengthPow2(v2));
                dot /= lenv1v2;
            }
            //if (dot > 1.0)
            //    dot = 1.0;
            //if (dot < -1.0)
            //    dot = -1.0;
            dot = utils::sse::clamp(dot, -1, 1);
            return dot;
        }
        __forceinline D Cos_sse41(const V3& v1, const V3& v2, bool vectorsAreNormalized = false)
        {
            //return v1.normalized().eval().dot(v2.normalized());
            D dot = Dot_sse41(v1, v2);
            if (!vectorsAreNormalized)
            {
                D lenv1v2 = utils::sse::sqrt(LengthPow2_sse41(v1)*LengthPow2_sse41(v2));
                dot /= lenv1v2;
            }
            //if (dot > 1.0)
            //    dot = 1.0;
            //if (dot < -1.0)
            //    dot = -1.0;
            dot = utils::sse::clamp(dot, -1, 1);
            return dot;
        }

        // force to be not inline since this method is slow and always should be visible for profiler - this method call 2 norm and acos what should be omitted every time it is possible
        //__declspec(noinline) D AngleInRadians(const V3& v1, const V3& v2, bool vectorsAreNormalized = false);

        // returns angle between v1 and v2 in direction of v2Ortogonal - angle can be negative
        __forceinline D AngleInRadians(const V3& v1, const V3& v2, bool vectorsAreNormalized)
        {
            // v1
            //a · b = |a| * |b| * cos(θ)
            //a · b = a.norm() * b.norm() * cos(θ)
            //a.normalized() · b.normalized() = cos(θ)
            D d = Cos(v1, v2, vectorsAreNormalized);


            //v2 - only for 90 degress triangles
            //|a|/|c| = cos(θ)    where a is a smaller side
            //D c = v1.squaredNorm();
            //D a = v2.squaredNorm();
            //if (c < a) swap(c, a);
            //D d2 = a / c;
            //D angle = acos(d);
            //D angle2 = acos(d2*d2);           
            //cout << "angle = " << angle << "     angle2 = " << angle2 << endl;

            //v3
            //vec3 a = v, b = *this;
            //D dot = v.x*x + v.y*y + v.z*z;
            //D len = a.length() * b.length();
            //if (len == 0)len = 0.00001f;
            //D input = dot / len;
            //if (input < -1) input = -1;
            //if (input > 1) input = 1;
            //return (D)acos(input);

            return acos(d);
        }

        // returns angle between v1 and v2 in direction of v2Ortogonal - angle can be negative
        __forceinline D AngleInRadiansFull(const V3& v1, const V3& v2, const V3& normal)
        {
            V3 v1n = v1.normalized();
            V3 v2n = v2.normalized();
            D angle = atan2(Dot(v1n, v2n), Dot(v1n, normal));
            return angle;
        }
        __forceinline D Angle(const V3& v1, const V3& v2, bool vectorsAreNormalized = false)
        {
            return utils::angle::RadiansToDegrees(AngleInRadians(v1, v2, vectorsAreNormalized));
        }
        // returns angle between v1 and v2 in direction of v2Ortogonal - angle can be negative
        //__forceinline D Angle(const V3& v1, const V3& v2, const V3& v2Ortogonal)
        //{
        //    return angle::RadiansToDegrees(AngleInRadians(v1, v2, v2Ortogonal));
        //}
        __forceinline D AngleFull(const V3& v1, const V3& v2, const V3& normal, bool vectorsAreNormalized = false)
        {
            D angle = utils::angle::RadiansToDegrees(AngleInRadians(v1, v2, vectorsAreNormalized));
            V3 v1_v2_cross = Cross(v1, v2);
            bool clockwise = Dot(v1_v2_cross, normal) > 0;
            if (!clockwise)
            {
                angle = 360 - angle;
            }
            return angle;
        }
        __forceinline bool Parallel(const V3& v1, const V3& v2)
        {
            //http://mathworld.wolfram.com/ParallelVectors.html
            D s = Cross(v1, v2).sum();
            return abs(s) < 0.00000001;
        }
        __forceinline bool SameDirectionIfTheyParallel(const V3& v1, const V3& v2)
        {
            //https://math.stackexchange.com/questions/646780/how-to-tell-if-two-3d-vectors-are-in-the-same-direction
            D d = Dot(v1, v2);
            return d > 0;
        }
        __forceinline bool OppositeDirectionIfTheyParallel(const V3& v1, const V3& v2)
        {
            D d = Dot(v1, v2);
            return d < 0;
        }
        __forceinline bool SameDirection(const V3& v1, const V3& v2)
        {
            return SameDirectionIfTheyParallel(v1, v2) && Parallel(v1, v2);
        }
        __forceinline bool OppositeDirection(const V3& v1, const V3& v2)
        {
            return OppositeDirectionIfTheyParallel(v1, v2) && Parallel(v1, v2);
        }
        //inline V3 ProjectOnPlane(const V3& v, const V3& planeNormal)
        //{
        //    //TODO implement
        //    return v;
        //}
        // same as igl::rotation_matrix_from_directions
        inline bool rotation_matrix_from_directions(const V3& v0, const V3& v1, D33& rotM, bool vectorsAreNormalized = false);
        inline bool rotation_matrix_from_directions_sse41(const V3& v0, const V3& v1, D33& rotM, bool vectorsAreNormalized = false);
        V3 Rotate(const V3& v, V3 rotationAxis, D angleInDegrees, bool isnormalized_rotationAxis);
        V3 Translate(const V3& v, const V3& normalTo, bool preserveLength = true);
        V3 Translate(const V3& v, const V3& normalFrom, const V3& normalTo, bool normalsAreNormalized = false);


        // find ortogonal point to line or closest point to its point1 or point2
        P3 ClosestPoint_ToLine(const P3& linePoint1, const P3& linePoint2, const P3& p, int& closestPointIsOnCornerPoint012);
        P3 ClosestPoint_ToLine(const P3& linePoint1, const P3& linePoint2, const P3& p);
        D ClosestPoint_ToLine_IsOnLinePercent(const P3& linePoint1, const P3& linePoint2, const P3& p);
        int ClosestPoint_ToLine_IsOutOfLineSign(const P3& linePoint1, const P3& linePoint2, const P3& p);//-1,0,1 - out in left, inside, out of tright
        bool ClosestPoint_ToLine_IsOnLine(const P3& linePoint1, const P3& linePoint2, const P3& p);

        // find ortogonal point to vector
        // if result is in between [0..1] then point is inside vector, otherwise outside
        __forceinline D ClosestPoint_ToVector_Position__vLengthPow2(const P3& vectorStartPoint, const V3& vector, const D& vLengthPow2, const P3& p)
        {
            // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
            V3 PQ = p - vectorStartPoint;
            D PQ_dot_v = Dot(PQ, vector);
            return (PQ_dot_v / vLengthPow2);
        }
        // if result is in between [0..1] then point is inside vector, otherwise outside
        __forceinline D ClosestPoint_ToVector_Position(const P3& vectorStartPoint, const V3& vector, const D& vLength, const P3& p)
        {
            return ClosestPoint_ToVector_Position__vLengthPow2(vectorStartPoint, vector, vLength * vLength, p);
        }
        __forceinline P3 ClosestPoint_ToVector(const P3& vectorStartPoint, const V3& vector, D vLength, const P3& p)
        {
            return vectorStartPoint + vector * ClosestPoint_ToVector_Position(vectorStartPoint, vector, vLength, p);
        }
        __forceinline P3 ClosestPoint_ToVector(const P3& vectorStartPoint, const V3& vector, const P3& p)
        {
            // get closet point to line in 3d - http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point
            return vectorStartPoint + vector * ClosestPoint_ToVector_Position__vLengthPow2(vectorStartPoint, vector, LengthPow2(vector), p);
        }

        bool FindIntersectionBetween_EdgeAndVectorPow2(const P3 &edgev0, const P3 &edgev1, const P3 &dirv0, const V3 &dir, D &edgePosPow2);
        bool FindIntersectionBetween_EdgeAndVector(const P3 &edgev0, const P3 &edgev1, const P3 &dirv0, const V3 &dir, D &edgePos);

        bool FindIntersectionBetween_VectorsPow2(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos1Pow2, D &pos2Pow2);
        bool FindIntersectionBetween_Vectors(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos1, D &pos2);
        bool FindIntersectionBetween_VectorsPow2(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos2Pow2);
        bool FindIntersectionBetween_Vectors(const P3 &point1, const V3 &dir1, const P3 &point2, const V3 &dir2, D &pos2);

        void ClosestPoint_BetweenLines(const P3& line1Point1, const P3& line1Point2, const P3& line2Point1, const P3& line2Point2,
            P3& closestPointOnLine1, P3& closestPointOnLine2, D& minDistBetweenLines, int& line1_ClosestPointIsOnCornerPoint012);

        void ClosestPoint_BetweenLines_iterative(const P3& line1Point1, const P3& line1Point2, const P3& line2Point1, const P3& line2Point2,
            P3& closestPointOnLine1, P3& closestPointOnLine2, D& minDistBetweenLines, int& line1_ClosestPointIsOnCornerPoint012);

        __forceinline D DistFromLineToPoint(const P3& linePoint1, const P3& linePoint2, const P3& p)
        {
            P3 pOnLine = ClosestPoint_ToLine(linePoint1, linePoint2, p);
            V3 distVector = p - pOnLine;
            D dist = Length(distVector);
            return dist;
        }

        __forceinline D DistFromVectorToPoint(const P3& vectorStartPoint, V3& vector, const P3& p)
        {
            P3 pOnLine = ClosestPoint_ToVector(vectorStartPoint, vector, p);
            V3 distVector = p - pOnLine;
            D dist = Length(distVector);
            return dist;
        }

        __forceinline D DistFromVectorToPoint(const P3& vectorStartPoint, V3& vector, const D& vLength, const P3& p)
        {
            P3 pOnLine = ClosestPoint_ToVector(vectorStartPoint, vector, vLength, p);
            V3 distVector = p - pOnLine;
            D dist = Length(distVector);
            return dist;
        }
    }

}

