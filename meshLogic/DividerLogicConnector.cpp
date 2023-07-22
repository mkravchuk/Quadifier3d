#include "stdafx.h"
#include "DividerLogicConnector.h"
#include "Divider.h"
#include "Mesh.h"
#include "MeshLoop.h"
#include "MeshStreams.h"
#include "MeshStreamsJoiner.h"
#include "MeshCutter.h"

#define var auto

const MeshLogicOptions_DividerLogicConnector& options = meshLogicOptions.DividerLogicConnector;

ComplexStream::ComplexStream(int _globalStreamIndex)
    :globalStreamIndex(_globalStreamIndex)
{
}

DividerLogicConnector_Mesh::DividerLogicConnector_Mesh(int _index, Divider& _divider, ViewerDrawObjects& _draw)
    : Index(_index), mesh(_divider.mesh), divider(_divider), streams(_divider.streams.streams), draw(_draw)
{
}

void DividerLogicConnector_Mesh::SaveDividerStreamsPoints()
{
    saved_divider_streams_points.clear();
    saved_divider_streams_points.reserve(streams.size());
    for (var& s : streams)
    {
        saved_divider_streams_points.push_back(s.Points);
    }
}

void DividerLogicConnector_Mesh::InitLengths()
{
    for (var& s : streams)
    {
        s.Init_lengthToNextPoint();
    }
}

void DividerLogicConnector_Mesh::CalculateIntersectionPoints(const vector<ComplexStream>& complexStreams)
{
    MeshCutter meshCutter(draw, mesh, false, false, false);
    int IntersectionsCount = meshCutter.FindStreamIntersections(streams, false, false, false, false);
    // update lengths after interting intersection points
    InitLengths();
    // populate 'Intersections'
    Intersections.resize(IntersectionsCount);
    for (MeshStream& stream : streams)
    {
        if (stream.Points.size() < 2) continue; //skip invalid streams
        for (int i = 0; i < stream.Points.size(); i++)
        {
            var& point = stream.Points[i];
            if (point.intersectionID == -1)  continue;
            var& I = Intersections[point.intersectionID];
            if (!I.IsInited)
            {
                I.IsInited = true;
                I.meshId = mesh.id;
                I.point = MeshPoint(point.Type, point.vid_eid_fid, point.point);
            }
            if (stream.dividingIteration.globalStreamIndex != -1)
            {
                I.streamIntersectionPoints.push_back(StreamIntersectionPoint());
                var& info = I.streamIntersectionPoints.back();
                info.local.pointIndex = i;
                info.local.streamIndex = stream.Index;
                info.local.Lengt3d = point.Length3dUntilThisPoint;
                info.global.pointIndex = info.local.pointIndex; // will be finally calculated later
                info.global.streamIndex = stream.dividingIteration.globalStreamIndex;
                info.global.Lengt3d = info.local.Lengt3d; // will be finally calculated later

                for (var& cs : complexStreams[stream.dividingIteration.globalStreamIndex].streams)
                {
                    if (mesh.id == cs->mesh.id) break;
                    info.global.pointIndex += cs->Points.size();
                    info.global.Lengt3d += cs->Length3d;
                }
            }
        }
    }

}

void DividerLogicConnector_Mesh::CreateConnectionsOnIntersectionPoints(const vector<ComplexStream>& complexStreams)
{
    D connectionAngle = options.connectionAngle;
    for (var& I : Intersections)
    {
        if (!I.IsInited) continue;
        if (I.streamIntersectionPoints.size() == 0) continue;
        for (int i0 = 0; i0 < I.streamIntersectionPoints.size() - 1; i0++)
        {
            var& p0 = I.streamIntersectionPoints[i0];
            for (int i1 = i0 + 1; i1 < I.streamIntersectionPoints.size(); i1++)
            {
                var& p1 = I.streamIntersectionPoints[i1];
                if (p0.global.pointIndex == 0 && p1.global.pointIndex == 0) continue;// skip intersections that start at same point (on singularities for example)
                //if (p0.global.pointIndex == 0 || p1.global.pointIndex == 0) // if some stream goes from start point that of another then this is proper connection without a doubts
                //{
                //    I.connections.push_back(StreamsConnection(p0, p1, 0));
                //    continue;
                //}
                //if (p0.local.pointIndex == 0 || p1.local.pointIndex == 0) continue; // ignore such case for simplicity
                var&s0 = streams[p0.local.streamIndex];
                var&s1 = streams[p1.local.streamIndex];
                int pointIndex0 = (p0.local.pointIndex == 0) ? 0 : p0.local.pointIndex - 1;
                int pointIndex1 = (p1.local.pointIndex == 0) ? 0 : p1.local.pointIndex - 1;
                var v0 = s0.Points[pointIndex0].dirToNextPointNormalized;
                var v1 = s1.Points[pointIndex1].dirToNextPointNormalized;
                //draw.AddPoint(s0.Points[p0.local.pointIndex - 1].point, Color3d(0, 0, 1));
                //draw.AddPoint(s1.Points[p1.local.pointIndex - 1].point, Color3d(0, 0, 1));
                //draw.AddEdgeBold(s0.Points[p0.local.pointIndex - 1].point, v0, s0.Points[p0.local.pointIndex - 1].lengthToNextPoint, Color3d(0, 0, 1));
                //draw.AddEdgeBold(s1.Points[p1.local.pointIndex - 1].point, v1, s1.Points[p1.local.pointIndex - 1].lengthToNextPoint, Color3d(0, 0, 1));
                D angle = utils::vector::Angle(v0, v1);
                if (angle > 90) angle = 180 - angle;
                if (angle < 0) angle = 0;
                if (angle < connectionAngle) continue;
                I.connections.push_back(StreamsConnection(p0, p1, angle));
            }
        }
    }
}

DividerLogicConnector::DividerLogicConnector(const Topology& _topology, ViewerDrawObjects& _draw, vector<Divider>& _dividers)
    : topology(_topology), draw(_draw)
{
    meshes.reserve(_dividers.size());
    for (int i = 0; i < _dividers.size(); i++)
    {
        meshes.push_back(DividerLogicConnector_Mesh(i, _dividers[i], _draw));
    }
}

template<class T>
inline void sort_custom(vector<T>& v, std::function<bool(const T &a, const T &b)> predicate)
{
    sort(v.begin(), v.end(), predicate);
}


void DividerLogicConnector::Solve()
{
    cout << endl << "LogicConnection" << endl;
    /*
        0) create complex streams by grouping streams using dividingIteration.globalStreamIndex
        1) make streams copy to avoid changing original streams in dividers
        2) calculate intersection points for each mesh using our duplicated streams
        3) create intersections infos base on intersections that are valid(connected by close to 90 degree angle) for each mesh
        4) create global connections based on intersections infos DividerLogicConnector_Mesh.Intersections
    */

    CreateComplexStreams();

    extern bool IsOmpEnabled;
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        var& m = meshes[i];
        m.SaveDividerStreamsPoints();  // 1) make streams copy to avoid changing original streams in dividers
        m.CalculateIntersectionPoints(complexStreams); // 2) calculate intersection points for each mesh using our duplicated streams
        m.CreateConnectionsOnIntersectionPoints(complexStreams); // 3) create intersections infos base on intersections that are valid(connected by close to 90 degree angle) for each mesh
    }
    CreateConnections(); // 4) create global connections based on intersections infos DividerLogicConnector_Mesh.Intersections
}


void DividerLogicConnector::DrawDebug()
{
    if (!options.DebugEnabled) return;

    // Show complex stream
    if (options.debug_streamGlobalIndex != -1)
    {
        DrawComplexStream(options.debug_streamGlobalIndex, Color3d(0, 0, 1));
    }


    // Show complex streams connection
    if (options.debug_connectionIndex != -1)
    {
        if (options.debug_connectionIndex != -1 && options.debug_connectionIndex < connections.size())
        {
            var &c = connections[options.debug_connectionIndex];

            var s0 = c.connection.p0.global;
            var s1 = c.connection.p1.global;
            if (s0.Lengt3d < s1.Lengt3d)
            {
                swap(s0, s1);
            }
            DrawComplexStream(s0.streamIndex, Color3d(1, 0, 0), s0.pointIndex - 1); // draw longer
            DrawComplexStream(s1.streamIndex, Color3d(0, 1, 0), s1.pointIndex - 1); // draw shortest
        }
        else
        {
            cout << endl << "Error:    Entered  index=" << options.debug_connectionIndex << "  is out of connections.size()=" << connections.size() << endl << endl;
        }
    }


    // Show intersection Ids
    if (options.debug_show_streams__intersection_points || options.debug_show_streams__accepted_connections)
    {
        int addcount = 0;
        for (auto& m : meshes) addcount += m.Intersections.size();
        draw.ReservePoints(addcount);
        draw.ReserveLabels(addcount);
        for (auto& m : meshes)
        {
            for (int i = 0; i < m.Intersections.size(); i++)
            {
                var& I = m.Intersections[i];
                if (!options.debug_show_streams__accepted_connections || !options.debug_show_streams__intersection_points)
                {
                    B accepted_connection = (I.connections.size() != 0);
                    if (options.debug_show_streams__accepted_connections && !accepted_connection)  continue;
                    if (options.debug_show_streams__intersection_points && accepted_connection)  continue;
                }
                string text = "I#" + to_string(i);
                text += "  ";
                for (var& si : I.streamIntersectionPoints) text += " s" + to_string(si.local.streamIndex);
                text += "  ";
                for (var& si : I.streamIntersectionPoints) text += " S" + to_string(si.global.streamIndex);
                text += "  ";
                for (var& ci : I.connections)
                {
                    text += "   [S" + to_string(ci.p0.global.streamIndex) + "^S" + to_string(ci.p1.global.streamIndex) + "=" + to_string(ci.angle) + "]";
                }
                m.draw.AddPoint(I.point.point, Color3d(1, 0, 0), text);
            }
        }
    }
}

void DividerLogicConnector::DrawComplexStream(int index, Color3d color, int maximumPointsCount)
{
    if (index != -1 && index < complexStreams.size())
    {
        int drawedPointsCount = 0;
        for (var& s : complexStreams[index].streams)
        {
            if (maximumPointsCount!= -1 && drawedPointsCount > maximumPointsCount) break;
            var& p = s->Points;
            if (p.size() == 0) continue;

            //draw start point information
            draw.AddPoint(p[0].point, color);
            draw.AddLabel(p[0].point, "  S#" + to_string(index) + ", i" + to_string(s->dividingIteration.iterationNum), color, 4);

            // draw edges
            for (int i = 0; i < p.size() - 1; i++)
            {
                if (maximumPointsCount != -1 && drawedPointsCount > maximumPointsCount) break;
                drawedPointsCount++;
                int iNext = i + 1;
                draw.AddEdgeBold(p[i].point, p[iNext].point, color);
            }
        }
    }
    else
    {
        cout << endl << "Error:    Entered  index=" << index << "  is out of complexStreams.size()=" << complexStreams.size() << endl << endl;
    }
}

void DividerLogicConnector::CreateComplexStreams()
{
    complexStreams.clear();

    // get count of complex streams
    int max_globalStreamIndex = 0;
    for (var& m : meshes)
    {
        for (MeshStream& s : m.streams)
        {
            max_globalStreamIndex = max(max_globalStreamIndex, s.dividingIteration.globalStreamIndex);
        }
    }

    // allocate vector
    for (int i = 0; i <= max_globalStreamIndex; i++)
    {
        complexStreams.push_back(ComplexStream(i));
    }

    // populate vector
    for (int i = 0; i < meshes.size(); i++)
    {
        var& m = meshes[i];
        for (MeshStream& s : m.streams)
        {
            if (s.dividingIteration.globalStreamIndex == -1) continue;
            complexStreams[s.dividingIteration.globalStreamIndex].streams.push_back(&s);
        }
    }

    // sort by iterationNum
    for (ComplexStream& ss : complexStreams)
    {
        //sort_custom(ss, [&](const MeshStream& a, const MeshStream& b)
        //{
        //    return false;
        //});
        sort(ss.streams.begin(), ss.streams.end(), [](const MeshStream* s1, const MeshStream* s2)
        {
            return s1->dividingIteration.iterationNum < s2->dividingIteration.iterationNum;
        });
    }
}

void DividerLogicConnector::CreateConnections()
{
    connections.clear();
    for (var& m : meshes)
    {
        for (var&I : m.Intersections)
        {
            for (var& c : I.connections)
            {
                connections.push_back(ComplexStreamConnections(connections.size(), c, m.mesh.id, I.point, complexStreams[c.p0.global.streamIndex], complexStreams[c.p1.global.streamIndex]));
            }
        }
    }
}


