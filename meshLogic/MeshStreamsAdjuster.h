#pragma once
#include "Divider.h"
struct Minerr;
class Mesh;
class Divider;
class MeshSolverNrosy;
class MeshSurface;
class ViewerDrawObjects;
class MeshStreams;


class MeshStreamsAdjuster
{
public:
    const Mesh& mesh;
    const MeshSolverNrosy& solver;
    ViewerDrawObjects& draw;

    MeshStreamsAdjuster(const Mesh& mesh, const MeshSolverNrosy& solver, ViewerDrawObjects& draw);

    bool AdjustStream(const P3& connectionPoint, StreamStartPoint& sp, MeshStream& origin_points, MeshStream& adjusted_points);
    int callsCount_directMethod;
    int callsCount_iterativeMethod;
private:
    //bool AdjustStream_StartAngle(const P3& connectionPoint, const StreamPoints& origin_points, StreamStartDirection& sp, StreamPoints& adjusted_points);

    bool TryRememberNewPointTo__test_min(D max_stream_angle_diff, int testid, D ri, StreamPoint& newpoint, const StreamPoint& point,
        D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
        const Minerr& try_min, Minerr& test_min, int test_fid,
        DD shift_coeff, const MeshStream& points, int points_startFromIndex, int rot_sign,
        bool isDebug, bool isDebugThisPoint, bool isDebugThisIteration, D newpointEdgePosPercent = NAN);
    bool TryRememberNewPointTo__harder(D max_stream_angle_diff, const StreamPoint& point, const V3& dir, D angleSumm360,
        D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
        const Minerr& try_min, Minerr& test_min,
        D shift_coeff, const MeshStream& points, int points_startFromIndex, int rot_sign,
        bool isDebug, bool isDebugThisPoint, bool isDebugThisIteration,
        const vector<VertexToFacesSortedInfo>& faces, D& riMiddle, int& testsMade);
    void AdjustStream_Ditsts_iterative(const P3& connectionPoint, const MeshStream& origin_points, int points_startFromIndex,
        int pointNum, const StreamPoint& point, const V3& dir,
        D shift_coeff, int rot_sign, D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
        D join_tollerance, D ri_max_change_angle,
        bool isDebug, bool isDebugThisPoint,
        Minerr& try_min, int& testsMade);

    // idea to test firstly edges that are on the way of 'dir', and if we found solution - return result, so in this way we skip other edges and save a time
    // also, firstly to test points that on same side as 'sign'(right or left) (since 'solve' method can return two points)
    void AdjustStream_Ditsts_direct(const P3& connectionPoint, const MeshStream& origin_points, int points_startFromIndex,
        int pointNum, const StreamPoint& point, const V3& dir,
        D shift_coeff, int rot_sign, D new_lengthTo_pointOnPoints, D lengthTo_pointOnPoints,
        D join_tollerance, D ri_max_change_angle,
        bool isDebug, bool isDebugThisPoint,
        Minerr& try_min, int& testsMade);
    bool AdjustStream_Ditsts(const P3& connectionPoint, const MeshStream& origin_points, const StreamStartPoint& sp, MeshStream& adjusted_points);
    bool AdjustStream_Ditsts_SingleIteration(int shiftIterationIndex, D lengthTo_pointOnPoints, const P3& connectionPoint, const MeshStream& origin_points, const StreamStartPoint& sp, MeshStream& adjusted_points);

};