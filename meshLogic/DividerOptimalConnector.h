#pragma once
#include "MeshStreamsTopology.h"


class DividerOptimalConnector
{
public:
    ViewerDrawObjects& draw;
    SSMeshes& ss;
    DividerOptimalConnector(ViewerDrawObjects& draw, SSMeshes& s);
    void Solve();
private:
    vector<SSMeshConflictedLoop> conflictedLoops; // mesh-loops joined despite conflicts
    void CalculateInitialDivs();
    void JoinConnections();
    void JoinConflictedLoops();
    void JoinStreamsWithDividingPoints();
    void ConflictedLoops_Build();
    bool ConflictedLoops_ReserveSpaceForSmallInBig(int reserveSpace_iteration_num);
    void DrawDebug();
};
