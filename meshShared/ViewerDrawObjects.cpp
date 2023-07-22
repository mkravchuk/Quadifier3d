#include "stdafx.h"
#include "ViewerDrawObjects.h"
#include "_MeshLogicOptions.h"

II MeshColor::SizeOF() const
{
    return Colors.size() * sizeof(float);
}

void MeshColor::Clear()
{
    Scheme = MeshColorSceme::DefaultColor;
    Resize(0);
}

void MeshColor::Resize(int rows)
{
    Colors.resize(rows, 3);
}

float MeshColor::CreateShortMonochromeColor(unsigned char monochromeColor, unsigned char alpha)
{
    return CreateShortColor(monochromeColor, monochromeColor, monochromeColor, alpha);
}

float MeshColor::CreateShortColor(unsigned char r, unsigned char g, unsigned char b, unsigned char alpha)
{
    int color = (((int)r) << 24)
        + (((int)g) << 16)
        + (((int)b) << 8)
        + alpha;
    return static_cast<float>(color);
}

float MeshColor::CreateShortColor(const Color3d& color)
{
    float rf = color(0);
    float gf = color(1);
    float bf = color(2);
    unsigned char r = (unsigned char)(255 * rf);
    unsigned char g = (unsigned char)(255 * gf);
    unsigned char b = (unsigned char)(255 * bf);
    return CreateShortColor(r, g, b);
}

float MeshColor::CreateShortColor(const Color4d& color)
{
    float rf = color(0); 
    float gf = color(1);
    float bf = color(2);
    float af = color(3);
    unsigned char r = min((int)(255 * rf), 255);
    unsigned char g = min((int)(255 * gf), 255);
    unsigned char b = min((int)(255 * bf), 255);
    unsigned char a = min((int)(255 * af), 255);
    return CreateShortColor(r, g, b, a);
}

Color3f MeshColor::GetDefaultColor()
{
    return meshLogicOptions.Draw.Mesh.DefaultColor.cast<float>();
}

bool MeshColor::isFaceBased() const
{
    return (Scheme == MeshColorSceme::PerFace);
}




ViewerDrawObjects::ViewerDrawObjects()
{

}

II ViewerDrawObjects::SizeOF() const
{
    return sizeof(ViewerDrawObjects)
        + points.capacity() * sizeof(ViewerDrawObject_Point)
        + edges.capacity() * sizeof(ViewerDrawObject_Edge)
        + edgesBold.capacity() * sizeof(ViewerDrawObject_Edge)
        + labels.capacity() * sizeof(ViewerDrawObject_Label)
        + meshColor.SizeOF();
}

bool ViewerDrawObjects::IsEmpty() const
{
    return points.size() + edges.size() + edgesBold.size() + labels.size() == 0;
}

void ViewerDrawObjects::Clear()
{
    points.clear();
    edges.clear();
    edgesBold.clear();
    labels.clear();
    meshColor.Clear();
}

void ViewerDrawObjects::ReservePoints(int reserveAdditionalPointsCount)
{
    points.reserve(points.size() + reserveAdditionalPointsCount);
}
void ViewerDrawObjects::ReserveEdges(int reserveAdditionalEdgesCount)
{
    edges.reserve(edges.size() + reserveAdditionalEdgesCount);
}
void ViewerDrawObjects::ReserveEdgesBold(int reserveAdditionalEdgesCount)
{
    edgesBold.reserve(edgesBold.size() + reserveAdditionalEdgesCount);
}
void ViewerDrawObjects::ReserveLabels(int reserveAdditionalLabelsCount)
{
    labels.reserve(labels.size() + reserveAdditionalLabelsCount);
}
void ViewerDrawObjects::AddPoint(const P3& point)
{
    points.push_back(ViewerDrawObject_Point(point, Color3d(0, 0, 0)));
}
void ViewerDrawObjects::AddPoint(const P3& point, const Color3d& color, string decription)
{
    points.push_back(ViewerDrawObject_Point(point, color));
    if (!decription.empty())
    {
        AddLabel(point, "   " + decription, color);
    }
}
void ViewerDrawObjects::AddEdge(const P3& start, const P3& end)
{
    edges.push_back(ViewerDrawObject_Edge(start, end, Color3d(0, 0, 0)));
}
void ViewerDrawObjects::AddEdge(const P3& start, const P3& end, const Color3d& color, string decription)
{
    edges.push_back(ViewerDrawObject_Edge(start, end, color));
    if (!decription.empty())
    {
        AddLabel(end, " " + decription, color);
    }
}
void ViewerDrawObjects::AddEdge(const P3& start, const V3& vector, D length, const Color3d& color, string decription)
{
    P3 end = start + vector.normalized()*length;
    edges.push_back(ViewerDrawObject_Edge(start, end, color));
    if (!decription.empty())
    {
        AddLabel(end, " " + decription, color);
    }
}
void ViewerDrawObjects::AddEdgeBold(const P3& start, const P3& end)
{
    edgesBold.push_back(ViewerDrawObject_Edge(start, end, Color3d(0, 0, 0)));
}
void ViewerDrawObjects::AddEdgeBold(const P3& start, const P3& end, const Color3d& color, string decription)
{
    edgesBold.push_back(ViewerDrawObject_Edge(start, end, color));
    if (!decription.empty())
    {
        AddLabel(end, " " + decription, color, 2); // 2 - to bold font as well
    }
}
void ViewerDrawObjects::AddEdgeBold(const P3& start, const V3& vector, D length, const Color3d& color, string decription)
{
    P3 end = start + vector.normalized()*length;
    edgesBold.push_back(ViewerDrawObject_Edge(start, end, color));
    if (!decription.empty())
    {
        AddLabel(end, " " + decription, color, 2);// 2 - to bold font as well
    }
}
void ViewerDrawObjects::AddLabel(const P3& point, const string& caption)
{
    AddLabel(point, caption, Color4d(-1, -1, -1, -1)); // set default color
}
void ViewerDrawObjects::AddLabel(const P3& point, const string& caption, const Color3d& color, int _fontSizeRelativeIncr)
{
    Color4d color4 = Color4d(color(0), color(1), color(2), 1);
    labels.push_back(ViewerDrawObject_Label(point, caption, color4, _fontSizeRelativeIncr));
}
void ViewerDrawObjects::AddLabel(const P3& point, const string& caption, const Color4d& color, int _fontSizeRelativeIncr)
{
    labels.push_back(ViewerDrawObject_Label(point, caption, color, _fontSizeRelativeIncr));
}