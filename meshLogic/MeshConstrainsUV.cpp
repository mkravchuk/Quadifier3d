#include "stdafx.h"
#include "Mesh.h"
#include "MeshConstrainsUV.h"

const MeshLogicOptions_ConstrainsUV& options = meshLogicOptions.ConstrainsUV;


string UV::uv_toString() const
{
    string us = to_string(uv[0]);
    string vs = to_string(uv[1]);
    string s = " " + us.substr(0, us.find(",") + 5) + "  " + vs.substr(0, vs.find(",") + 5);
    return s;
}

bool UV::isUndefined() const
{
    return (uv[0] < -99 && uv[1] < -99);
}

bool MeshUV::isUndefined() const
{
    return (vid_eid_fid == -1);
}

MeshUV MeshUV::CreateOnVertex(const Mesh& mesh, int vid, UV uv)
{
    P3 p = mesh.V.row(vid);
    return MeshUV(MeshPointType::onVertex, vid, p, uv);
}

MeshUV MeshUV::CreateOnEdge(const Mesh& mesh, int eid, UV uv)
{    
    P3 p0 = mesh.V.row(mesh.EV(eid, 0));
    P3 p1 = mesh.V.row(mesh.EV(eid, 1));
    P3 p = (p0 + p1) / 2;
    return MeshUV(MeshPointType::onEdge, eid, p, uv);
}

string MeshUV::toString() const
{
    return uv.uv_toString() + "  " + vid_eid_fid_toString();
}


MeshConstrainsUV::MeshConstrainsUV(const Mesh& _mesh, ViewerDrawObjects& _draw)
    : mesh(_mesh), draw(_draw)
{
}

void MeshConstrainsUV::InitFromMeshLoop()
{
    Constrains.clear();

    if (mesh.LoopsCount != 1) return;
    const MeshLoop& loop = mesh.Loops[0];

    // get all sharp points
    vector<const MeshLoopPoint*> sharpPoints;
    sharpPoints.reserve(10);
    for (int i = 0; i < loop.points.size(); i++)
    {
        const MeshLoopPoint& p = loop.points[i];
        if (p.IsSharp || p.probably_IsSharp)
        {
            sharpPoints.push_back(&p);
        }
    }

    // shrink sharp points to 4 points only
    if (sharpPoints.size() > 4)
    {
        sort(sharpPoints.begin(), sharpPoints.end(), [&sharpPoints](const MeshLoopPoint* p1, const MeshLoopPoint* p2)
        {
            return p1->AngleBetweenEdges > p2->AngleBetweenEdges;
        });
        sharpPoints.resize(4);
        sort(sharpPoints.begin(), sharpPoints.end(), [&sharpPoints](const MeshLoopPoint* p1, const MeshLoopPoint* p2)
        {
            return p1->Index < p2->Index;
        });
    }

    // get all paths
    vector<vector<int>> paths; // indexes of edges in a loop
    paths.resize(sharpPoints.size());
    int loop_points_size = loop.points.size();
    for (int i = 0; i < sharpPoints.size(); i++)
    {
        int inext = (i == sharpPoints.size() - 1) ? 0 : i + 1;
        vector<int>& path = paths[i];
        path.reserve(loop.points.size());
        int pointIndex = sharpPoints[i]->Index;
        int pointIndexEnd = sharpPoints[inext]->Index;
        while (pointIndex != pointIndexEnd)
        {
            path.push_back(pointIndex);
            pointIndex++;
            if (pointIndex > loop_points_size - 1) pointIndex = 0;
        }
    }


    // generate constrains
    auto getPathLength3d = [&](const vector<int>& path)
    {
        D length3d = 0;
        for (int i = 0; i < path.size(); i++)
        {
            length3d += loop.edges[path[i]].Length;
        }
        return length3d;
    };
    auto clamp01 = [](D value)
    {
        if (value < (D)0) return (D)0;
        if (value > (D)1) return (D)1;
        return value;
    };
    Constrains.reserve(loop_points_size);
    if (paths.size() == 4) // algorithm for 4 paths
    {
        for (int pi = 0; pi < paths.size(); pi++)
        {
            const auto& path = paths[pi];
            D totalLength3d = getPathLength3d(path);
            D totalLength3di = 0;
            for (int i = 0; i < path.size(); i++)
            {
                D length3d = loop.edges[path[i]].Length;
                int vid = loop.points[path[i]].VertexId;
                D percentLength = clamp01(totalLength3di / totalLength3d);
                totalLength3di += length3d;
                D u = 0;
                D v = 0;
                if (pi == 0)
                {
                    u = percentLength;
                    v = 0;
                }
                else if (pi == 1)
                {
                    u = (D)1;
                    v = percentLength;
                }
                else if (pi == 2)
                {
                    v = (D)1;
                    u = clamp01(1 - percentLength);
                }
                else if (pi == 3)
                {
                    u = 0;
                    v = clamp01(1 - percentLength);
                }
                Constrains.push_back(MeshUV::CreateOnVertex(mesh, vid, {u, v }));

                //DEBUG add text
                //P3 p = mesh.V.row(vid);
                //draw.AddLabel(p, Constrains.back().toString());
            }
        }
    }
}

void MeshConstrainsUV::Clear()
{
    Constrains.clear();
}


II MeshConstrainsUV::SizeOF() const
{
    return sizeof(MeshConstrainsUV)
        + Constrains.capacity() * sizeof(MeshUV);
}
