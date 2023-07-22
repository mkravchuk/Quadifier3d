#pragma once

class Mesh;
class MeshLoop;
class ViewerDrawObjects;

struct UV
{
    D uv[2];
    string uv_toString() const;
    UV()
        : uv{ -100, -100 }
    {
    }
    UV(D u, D v)
        : uv{u, v}
    {
    }
    D& operator [] (int index)
    {
        #if DEBUG
        assert(index >= 0 && index < 2 && "index is out of range");
        #endif
        return uv[index];
    }
    D operator [] (int index) const
    {
        #if DEBUG
        assert(index >= 0 && index < 2 && "index is out of range");
        #endif
        return uv[index];
    }
    friend inline UV operator + (UV const & a, UV const & b)
    {
        return UV(a[0] + b[0], a[1] + b[1]);
    }
    friend inline UV operator - (UV const & a, UV const & b)
    {
        return UV(a[0] - b[0], a[1] - b[1]);
    }
    friend inline UV operator * (UV const & a, D c)
    {
        return UV(a[0] * c, a[1] * c);
    }
    friend inline UV operator / (UV const & a, D c)
    {
        return UV(a[0] / c, a[1] / c);
    }

    bool isUndefined() const;
    static D DistPow2(UV const & a, UV const & b)
    {
        D x = a[0] - b[0];
        D y = a[1] - b[1];
        return x * x + y * y;
    }
    P3 toP3()
    {
        return P3(uv[0], uv[1], (D)0);
    }
    UV getUVEnd()
    {
        UV uvEnd;
        D minDist0 = min(uv[0], 1 - uv[0]);
        D minDist1 = min(uv[1], 1 - uv[1]);
        int minUVindex = (minDist0 < minDist1) ? 0 : 1;
        int otherUVindex = (minUVindex == 0) ? 1 : 0;
        uvEnd[minUVindex] = (uv[minUVindex] < 0.5 ? 1 : 0);
        uvEnd[otherUVindex] = uv[otherUVindex];
        return uvEnd;
    }
};

struct MeshUV : MeshPoint
{
    UV uv;
    MeshUV()
        : MeshPoint(MeshPointType::onVertex, -1, P3(0, 0, 0)), uv()
    {
    }
    MeshUV(MeshPointType _Type, int _vid_eid_fid, const P3& _point, UV _uv)
        : MeshPoint(_Type, _vid_eid_fid, _point), uv(_uv)
    {
    }
    D& operator [] (int index)
    {
        #if DEBUG
        assert(index >= 0 && index < 2 && "index is out of range");
        #endif
        return uv.uv[index];
    }
    D operator [] (int index) const
    {
        #if DEBUG
        assert(index >= 0 && index < 2 && "index is out of range");
        #endif
        return uv.uv[index];
    }
    bool isUndefined() const;
    static MeshUV CreateOnVertex(const Mesh& mesh, int vid, UV uv);
    static MeshUV CreateOnEdge(const Mesh& mesh, int eid, UV uv);
    string toString() const;
};


class MeshConstrainsUV
{
public:
    const Mesh& mesh; // for which mesh we generate contrains
    ViewerDrawObjects& draw;
    vector<MeshUV> Constrains; // constrains for this mesh

    MeshConstrainsUV(const Mesh& mesh, ViewerDrawObjects& draw);
    void InitFromMeshLoop();
    void Clear();// clears Type and Constrains
    II SizeOF() const;
};