#pragma once


struct ViewerDrawObject_Point
{
    P3 point;
    Color3f color;
    ViewerDrawObject_Point(const P3& _point, const Color3d& _color)
        :point(_point), color(_color.cast<float>())
    {
    }
};

struct ViewerDrawObject_Edge
{
    P3 start;
    P3 end;
    Color3f color;
    ViewerDrawObject_Edge(const P3& _start, const P3& _end, const Color3d& _color)
        :start(_start), end(_end), color(_color.cast<float>())
    {
    }
};

struct ViewerDrawObject_Label
{
    P3 point;
    string caption;
    Color4d color;
    int FontSizeRelativeIncrease;
    bool IsDefaultColor() const
    {
        return color(0) < -0.5;
    }
    ViewerDrawObject_Label(const P3& _point, const string& _caption, const Color4d& _color, int _fontSizeRelativeIncr = 0)
        :point(_point), caption(_caption), color(_color), FontSizeRelativeIncrease(_fontSizeRelativeIncr)
    {
    }
};

enum class MeshColorSceme
{
    DefaultColor, // all faces will have sinlge color taken from MeshColor::GetDefaultColor()
    PerVertex,     // MeshColor.Colors should have colors for each vertex, or single color for each vertex in case MeshColor.Colors.rows()==1
    PerFace         // MeshColor.Colors should have colors for each face, or single color for each face in case MeshColor.Colors.rows()==1
};


struct MeshColor
{
    MeshColorSceme Scheme;
    MatrixXf Colors; // each face/vertex can have unique color of MeshColorQuality, and for a single color it will be one row
    MeshColor()
        : Scheme(MeshColorSceme::DefaultColor)
    {
        Clear();
    }
    II SizeOF() const;
    void Clear();
    void Resize(int rows);

    static float CreateShortMonochromeColor(unsigned char monochromeColor, unsigned char alpha = 255); // creates color of quality '_0_Monochrome'
    static float CreateShortColor(unsigned char r, unsigned char g, unsigned char b, unsigned char alpha = 255); // creates color of quality '_1_Colored_Short'
    static float CreateShortColor(const Color3d& color); // creates color of quality '_1_Colored_Short' from Color3d, where rgb is in diapasons [0.f..1.f]
    static float CreateShortColor(const Color4d& color); // creates color of quality '_1_Colored_Short' from Color4d, where rgba is in diapasons [0.f..1.f]
    static Color3f GetDefaultColor(); //  color for Scheme==MeshColorSceme::DefaultColor
    bool isFaceBased() const;
};


class ViewerDrawObjects
{
public:
    vector<ViewerDrawObject_Point> points;
    vector<ViewerDrawObject_Edge> edges;
    vector<ViewerDrawObject_Edge> edgesBold;
    vector<ViewerDrawObject_Label> labels;
    MeshColor meshColor;

    ViewerDrawObjects();
    II SizeOF() const;
    bool IsEmpty() const;
    void Clear();
    void ReservePoints(int reserveAdditionalPointsCount);
    void ReserveEdges(int reserveAdditionalEdgesCount);
    void ReserveEdgesBold(int reserveAdditionalEdgesCount);
    void ReserveLabels(int reserveAdditionalLabelsCount);
    void AddPoint(const P3& point); // color == Color3d(0,0,0)
    void AddPoint(const P3& point, const Color3d& color, string decription = "");
    void AddEdge(const P3& start, const P3& end);
    void AddEdge(const P3& start, const P3& end, const Color3d& color, string decription = "");
    void AddEdge(const P3& start, const V3& vector, D length, const Color3d& color, string decription = "");
    void AddEdgeBold(const P3& start, const P3& end);
    void AddEdgeBold(const P3& start, const P3& end, const Color3d& color, string decription = "");
    void AddEdgeBold(const P3& start, const V3& vector, D length, const Color3d& color, string decription = "");
    void AddLabel(const P3& point, const string& caption);
    void AddLabel(const P3& point, const string& caption, const Color3d& color, int _fontSizeRelativeIncr = 0);
    void AddLabel(const P3& point, const string& caption, const Color4d& color, int _fontSizeRelativeIncr = 0);
};