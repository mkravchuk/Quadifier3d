#pragma once
#include "MeshStreamsTopology.h"


class ComplexStream
{
public:
    int globalStreamIndex;
    vector<MeshStream*> streams;
    ComplexStream(int globalStreamIndex);
private:
};

class StreamIntersectionPointInfo
{
public:
    int pointIndex;
    int streamIndex;
    D Lengt3d;
};

class StreamIntersectionPoint
{
public:
    StreamIntersectionPointInfo local;
    StreamIntersectionPointInfo global;
};

class StreamsConnection
{
public:
    StreamIntersectionPoint& p0;
    StreamIntersectionPoint& p1;
    D angle;
    StreamsConnection(StreamIntersectionPoint& _p0, StreamIntersectionPoint& _p1, D _angle)
        : p0(_p0), p1(_p1), angle(_angle)
    {
    }
};

class StreamsIntersectionPoint
{
public:
    bool IsInited;
    int meshId;
    MeshPoint point;
    vector<StreamIntersectionPoint> streamIntersectionPoints;
    vector<StreamsConnection> connections;
    StreamsIntersectionPoint()
        : IsInited(false), meshId(-1)
    {
    }
};

class ComplexStreamConnections
{
public:
    int Index;  //index in vector<ComplexStreamConnections> connections
    StreamsConnection& connection;
    int meshId;
    MeshPoint point;
    ComplexStream& s0;
    ComplexStream& s1;
    ComplexStreamConnections(int index, StreamsConnection& connection, int mesh_id, const MeshPoint& point, ComplexStream& s0, ComplexStream& s1)
        : Index(index),
        connection(connection),
        meshId(mesh_id),
        point(point),
        s0(s0),
        s1(s1)
    {
    }
};

class DividerLogicConnector_Mesh
{
public:
    int Index; // index in 'dividers'
    const Mesh& mesh;
    Divider& divider;
    vector<MeshStream>& streams;
    ViewerDrawObjects& draw;
    vector<vector<StreamPoint>>  saved_divider_streams_points;
    vector<StreamsIntersectionPoint> Intersections;
    DividerLogicConnector_Mesh(int index, Divider& divider, ViewerDrawObjects& draw);
    void SaveDividerStreamsPoints();
    void InitLengths();
    void CalculateIntersectionPoints(const vector<ComplexStream>& complexStreams);
    void CreateConnectionsOnIntersectionPoints(const vector<ComplexStream>& complexStreams);
};

class DividerLogicConnector
{
public:
    const Topology& topology; // topology for current file
    ViewerDrawObjects& draw;
    vector<DividerLogicConnector_Mesh> meshes; // all dividers
    vector<ComplexStream> complexStreams;
    vector<ComplexStreamConnections> connections;
    DividerLogicConnector(const Topology& topology, ViewerDrawObjects& draw, vector<Divider>& dividers);
    void Solve();
    void DrawDebug();
private:
    void DrawComplexStream(int index, Color3d color, int maximumPointsCount = -1);
    void CreateComplexStreams();
    void CreateConnections();
};
