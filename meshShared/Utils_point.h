#pragma once
#include "Utils_vector.h"


namespace utils
{
    namespace point
    {
        __forceinline D DistToPointPow2(const P3& point, const P3& toPoint)
        {
            #ifdef USE_EIGEN
            // v0 
            //return utils::vector::LengthPow2(point - toPoint);
            // v1 - fast in debug mode
            const D* a = point.data();
            const D* b = toPoint.data();
            D x = a[0] - b[0];
            D y = a[1] - b[1];
            D z = a[2] - b[2];
            return x * x + y * y + z * z;
            #else
            return (point - toPoint).normPow2();
            #endif
        }
        __forceinline D DistToPoint(const P3& point, const P3& toPoint)
        {
            return sqrt(DistToPointPow2(point, toPoint));
        }
        __forceinline D LengthPow2(const P3& point1, const P3& point2)
        {
            #ifdef USE_EIGEN
            // v0
            //return utils::vector::LengthPow2(point1 - point2);
            // v1 - fast in debug mode
            const D* a = point1.data();
            const D* b = point2.data();
            D x = a[0] - b[0];
            D y = a[1] - b[1];
            D z = a[2] - b[2];
            return x * x + y * y + z * z;
            #else
            return (point1 - point2).normPow2();
            #endif
        }
        __forceinline D Length(const P3& point1, const P3& point2)
        {
            return sqrt(LengthPow2(point1, point2));
        }

        /// Get disnace from point to plane
        /// planeNormal - must be normalized
        __forceinline D DistToPlane(const P3& point, const P3& planePoint, const V3& planeNormal)
        {
            //https://stackoverflow.com/questions/9605556/how-to-project-a-3d-point-to-a-3d-plane

            // 1) Make a vector from your orig point to the point of interest:
            // v = point - orig(in each dimension);
            V3 v = point - planePoint; // vector from point to plane point

            // 2) Take the dot product of that vector with the unit normal vector n :
            // dist = vx*nx + vy*ny + vz*nz; dist = scalar distance from point to plane along the normal
            D dist = utils::vector::Dot(v, planeNormal); // scalar distance from point to plane along the normal

            return dist;
        }

        /// Project point on plane
        /// planeNormal - must be normalized
        __forceinline P3 ProjectToPlane(const P3& point, const P3& planePoint, const V3& planeNormal)
        {

            // 3) Multiply the unit normal vector by the distance, and subtract that vector from your point.
            // projected_point = point - dist*normal;
            P3 projected_point = point - DistToPlane(point, planePoint, planeNormal) * planeNormal;

            return projected_point;
        }
    }

}

