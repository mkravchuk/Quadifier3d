#pragma once



class FastQuadricSimplifier
{
private:

    class SymetricMatrix
    {
    public:
        D m[10];

        SymetricMatrix()
        {
            m[0] = 0.0;
            m[1] = 0.0;
            m[2] = 0.0;
            m[3] = 0.0;
            m[4] = 0.0;
            m[5] = 0.0;
            m[6] = 0.0;
            m[7] = 0.0;
            m[8] = 0.0;
            m[9] = 0.0;
        }

        SymetricMatrix(D c)
        {
            m[0] = c;
            m[1] = c;
            m[2] = c;
            m[3] = c;
            m[4] = c;
            m[5] = c;
            m[6] = c;
            m[7] = c;
            m[8] = c;
            m[9] = c;
        }

        SymetricMatrix(D m11, D m12, D m13, D m14,
            D m22, D m23, D m24,
            D m33, D m34,
            D m44)
        {
            m[0] = m11;  m[1] = m12;  m[2] = m13;  m[3] = m14;
            m[4] = m22;  m[5] = m23;  m[6] = m24;
            m[7] = m33;  m[8] = m34;
            m[9] = m44;
        }

        // Make plane
        SymetricMatrix(D a, D b, D c, D d)
        {
            m[0] = a*a;  m[1] = a*b;  m[2] = a*c;  m[3] = a*d;
            m[4] = b*b;  m[5] = b*c;  m[6] = b*d;
            m[7] = c*c;   m[8] = c*d;
            m[9] = d*d;
        }

        inline void Clear()
        {
            m[0] = 0.0;
            m[1] = 0.0;
            m[2] = 0.0;
            m[3] = 0.0;
            m[4] = 0.0;
            m[5] = 0.0;
            m[6] = 0.0;
            m[7] = 0.0;
            m[8] = 0.0;
            m[9] = 0.0;
        }

        inline D operator[](int c) const
        {
            return m[c];
        }

        // Determinant
        inline D det(int a11, int a12, int a13,
            int a21, int a22, int a23,
            int a31, int a32, int a33)
        {
            //TODO simplify equation - do we have common multpliyers?
            D det = m[a11] * m[a22] * m[a33]
                + m[a13] * m[a21] * m[a32]
                + m[a12] * m[a23] * m[a31]
                - m[a13] * m[a22] * m[a31]
                - m[a11] * m[a23] * m[a32]
                - m[a12] * m[a21] * m[a33];
            return det;
        }

        const SymetricMatrix operator+(const SymetricMatrix& n) const
        {
            return SymetricMatrix(m[0] + n[0], m[1] + n[1], m[2] + n[2], m[3] + n[3],
                m[4] + n[4], m[5] + n[5], m[6] + n[6],
                m[7] + n[7], m[8] + n[8],
                m[9] + n[9]);
        }

        SymetricMatrix& operator+=(const SymetricMatrix& n)
        {
            m[0] += n[0];   m[1] += n[1];   m[2] += n[2];   m[3] += n[3];
            m[4] += n[4];   m[5] += n[5];   m[6] += n[6];   m[7] += n[7];
            m[8] += n[8];   m[9] += n[9];
            return *this;
        }

    };

    struct vec3
    {
        D x, y, z;

        inline vec3(void)
        {
        }

        inline vec3(const D X, const D Y, const D Z)
        {
            x = X; y = Y; z = Z;
        }

        inline vec3 operator + (const vec3& a) const
        {
            return vec3(x + a.x, y + a.y, z + a.z);
        }

        inline vec3 operator += (const vec3& a) const
        {
            return vec3(x + a.x, y + a.y, z + a.z);
        }

        inline vec3 operator * (const D a) const
        {
            return vec3(x * a, y * a, z * a);
        }

        inline vec3 operator * (const vec3 a) const
        {
            return vec3(x * a.x, y * a.y, z * a.z);
        }

        inline vec3 v3() const
        {
            return vec3(x, y, z);
        }

        inline vec3 operator = (const vec3 a)
        {
            x = a.x; y = a.y; z = a.z; return *this;
        }

        inline vec3 operator / (const vec3 a) const
        {
            return vec3(x / a.x, y / a.y, z / a.z);
        }

        inline vec3 operator - (const vec3& a) const
        {
            return vec3(x - a.x, y - a.y, z - a.z);
        }

        inline vec3 operator / (const D a) const
        {
            return vec3(x / a, y / a, z / a);
        }

        inline D dot(const vec3& a) const
        {
            return a.x*x + a.y*y + a.z*z;
        }

        inline vec3 cross(const vec3& a, const vec3& b)
        {
            x = a.y * b.z - a.z * b.y;
            y = a.z * b.x - a.x * b.z;
            z = a.x * b.y - a.y * b.x;
            return *this;
        }

        inline D angle(const vec3& v)
        {
            vec3 a = v, b = *this;
            D dot = v.x*x + v.y*y + v.z*z;
            D len = a.length() * b.length();
            if (len == 0)len = 0.00001f;
            D input = dot / len;
            if (input < -1) input = -1;
            if (input > 1) input = 1;
            return (D)acos(input);
        }

        inline D angle2(const vec3& v, const vec3& w)
        {
            vec3 a = v, b = *this;
            D dot = a.x*b.x + a.y*b.y + a.z*b.z;
            D len = a.length() * b.length();
            if (len == 0)len = 1;

            vec3 plane; plane.cross(b, w);

            if (plane.x * a.x + plane.y * a.y + plane.z * a.z > 0)
                return (D)-acos(dot / len);

            return (D)acos(dot / len);
        }

        inline vec3 rot_x(D a)
        {
            D yy = cos(a) * y + sin(a) * z;
            D zz = cos(a) * z - sin(a) * y;
            y = yy; z = zz;
            return *this;
        }
        inline vec3 rot_y(D a)
        {
            D xx = cos(-a) * x + sin(-a) * z;
            D zz = cos(-a) * z - sin(-a) * x;
            x = xx; z = zz;
            return *this;
        }
        inline vec3 rot_z(D a)
        {
            D yy = cos(a) * y + sin(a) * x;
            D xx = cos(a) * x - sin(a) * y;
            y = yy; x = xx;
            return *this;
        }
        inline void clamp(D min, D max)
        {
            if (x < min) x = min;
            if (y < min) y = min;
            if (z < min) z = min;
            if (x > max) x = max;
            if (y > max) y = max;
            if (z > max) z = max;
        }

        inline vec3 invert()
        {
            x = -x; y = -y; z = -z; return *this;
        }
        inline vec3 frac()
        {
            return vec3(
                x - D(int(x)),
                y - D(int(y)),
                z - D(int(z))
            );
        }

        inline vec3 integer()
        {
            return vec3(
                D(int(x)),
                D(int(y)),
                D(int(z))
            );
        }

        static inline D dot(const vec3& v1, const vec3& v2)
        {
            return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
        }

        static inline D cos_quad(const vec3& v1, const vec3& v2)
        {
            D dot = vec3::dot(v1, v2);
            D len_quad = v1.length_quad() * v2.length_quad();
            if (len_quad == 0) len_quad = static_cast<D>(0.00000000000001);
            D quad_cos = dot*dot / len_quad;
            return quad_cos;
        }

        static inline D sin_quad(const vec3& v1, const vec3& v2)
        {
            D dot = vec3::dot(v1, v2);
            D len_quad = v1.length_quad() * v2.length_quad();
            if (len_quad == 0) len_quad = static_cast<D>(0.00000000000001);
            D quad_cos = dot*dot / len_quad;
            return quad_cos;
        }

        inline D length() const
        {
            return sqrt(length_quad());
        }

        inline D length_quad() const
        {
            return x*x + y*y + z*z;
        }


        inline vec3 normalize(D desired_length = 1)
        {
            D square = length();
            /*
            if (square <= 0.00001f )
            {
            x=1;y=0;z=0;
            return *this;
            }*/
            //D len = desired_length / square;
            x /= square; y /= square; z /= square;

            return *this;
        }
        //static vec3f normalize(vec3f a);

        //static void random_init();
        //static D random_double();
        //static vec3f random();

        static int random_number;

        D random_double_01(D a)
        {
            double rnf = a*14.434252 + a*364.2343 + a*4213.45352 + a*2341.43255 + a*254341.43535 + a*223454341.3523534245 + 23453.423412;
            int rni = ((int)rnf) % 100000;
            return D(rni) / (100000.0f - 1.0f);
        }

        vec3 random01_fxyz()
        {
            x = (D)random_double_01(x);
            y = (D)random_double_01(y);
            z = (D)random_double_01(z);
            return *this;
        }
    };

    struct Vertex
    {
        int Index;
        bool isDeleted;
        vec3 p;
        int tstart;  //index in 'refs' array
        int tcount; //count of triangles connected to this vertex
        SymetricMatrix q;
        bool isBorder;
    };

    struct Edge
    {
        int Index;
        bool isDeleted;
        bool isRemovedFromLoop;
        int PrevEdgeIndex;
        int NextEdgeIndex;
        int dirtyUntilIteration;
        int v[2];
        bool isErrorInited;
        D error;
        D error_det;
        int failed_at_iteration;
        vec3 collapsePoint;
        void MakeDirty(int iteration)
        {
            isErrorInited = false;
            dirtyUntilIteration = iteration;
        }
         void ReplaceVertex(int vid_old, int vid_new, int iteration)
        {
             if (v[0] == vid_old)
            {
                v[0] = vid_new;
            }
            else if (v[1] == vid_old)
            {
                v[1] = vid_new;
            }
         };
        void UpdateError(const vector<Vertex>& vertices)
        {
            if (!isErrorInited)
            {
                error = FastQuadricSimplifier::edge_error(vertices[v[0]], vertices[v[1]], collapsePoint, error_det);
                isErrorInited = true;
            }
        }
        void RemoveFromLoop(vector<Edge>& edges, int& firstEdgeIndexInLoop, int& lastEdgeIndexInLoop)
        {
            if (isRemovedFromLoop) return;
            isRemovedFromLoop = true;

            if (PrevEdgeIndex == -1)
            {
                firstEdgeIndexInLoop = NextEdgeIndex;
            }
            else
            {
                edges[PrevEdgeIndex].NextEdgeIndex = NextEdgeIndex;
            }

            if (NextEdgeIndex == -1)
            {
                lastEdgeIndexInLoop = PrevEdgeIndex;
            }
            else
            {
                edges[NextEdgeIndex].PrevEdgeIndex = PrevEdgeIndex;
            }

            PrevEdgeIndex = -1;
            NextEdgeIndex = -1;
        }

        void Delete(vector<Edge>& edges, int& firstEdgeIndexInLoop, int& lastEdgeIndexInLoop, int& currentEdgeIndexInLoop)
        {
            if (currentEdgeIndexInLoop == Index)
            {
                currentEdgeIndexInLoop = NextEdgeIndex;
            }
            RemoveFromLoop(edges, firstEdgeIndexInLoop, lastEdgeIndexInLoop);
            isDeleted = true;
        }
    };

    struct Triangle
    {
        int id;
        bool isDeleted;
        int v[3]; //vertex indexes
        int e[3]; //edge indexes
        vec3 normal; //triangle normal (recomputes every iteration)

        D edgesLengths[3]; // lengths of edges
        D sharpness;
        inline void UpdateEdgeLengthsAndSharpness(const vector<Vertex>& vertices);
        inline vec3 Centroid(const vector<Vertex>& vertices) const;
        bool RemoveSharpTriangles__wasAFriend;
        inline void replace_edge(int eid_old, int eid_new)
        {
            if (e[0] == eid_old) e[0] = eid_new;
            if (e[1] == eid_old) e[1] = eid_new;
            if (e[2] == eid_old) e[2] = eid_new;
        }
        inline void replace_vertex(int vid_old, int vid_new)
        {
            if (v[0] == vid_old) v[0] = vid_new;
            if (v[1] == vid_old) v[1] = vid_new;
            if (v[2] == vid_old) v[2] = vid_new;
        }
    };

    struct Ref
    {
        int tid; // triangle index
        int tvertex; //triangle internal vertex index [0,1,2] (each triangle has 3 vertexes)
    };

    int TriangleNextId;
    bool debugBreakExecution; //  set it to break execution just to see current situation in view - dont forget to add some debug data to viewport before :)
    D FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS;
    D FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS;
    D FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS_quad;
    D FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS_quad;
    const bool RecomputeTriangleNormalsAfterEachIteration;
    int currentEdgeIndexInLoop;
    int firstEdgeIndexInLoop;
    int lastEdgeIndexInLoop;
public:
    ViewerDrawObjects& draw;
    FastQuadricSimplifier(ViewerDrawObjects& draw);
    vector<Vertex> vertices;
    vector<Edge> edges;
    vector<Triangle> triangles;
    vector<Ref> refs;//keeps list of vertex's connections to triangle

    void load(const P3s& V, const I3s& F, const Bs& V_isborder, const V3s& F_normals, const  I2s& EV, const I3s& FE);
    void write(P3s& V, I3s& F);

    //
    // Simplify mesh with target quality
    //
    // target_count  : target nr. of triangles
    // agressiveness : sharpness to increase the threashold.
    //                 5..8 are good numbers
    //                 more iterations yield higher quality
    //
    void simplify_mesh_count(int target_count, D agressiveness = 7, bool verbose = false);

    //
    // Simplify mesh with target quality
    //
    // threshold - All triangles with edges below the threshold will be removed.  The following numbers works well for most models.
    void simplify_mesh_lossless(D threshold = DBL_EPSILON, bool verbose = false);

private:

    // main loop of simplifications - called by 'simplify_mesh' and 'simplify_mesh_lossless'
    int simplify_loop(int iteration, D target_threshold, int target_count, bool showOnlyDebugInfo, int Debug_HighlightSimplificationNum);
    void simplify_mesh(bool isTargetingCount, D target_threshold, int target_count, D agressiveness, bool verbose);

    // Check if a triangle flips when this edge is removed
    bool flipped(vec3 collapsePoint, Vertex &v_collapsed, int vid_opposite, D det);

    void replace_edge(Vertex &v, int eid_old, int eid_new);
    void remove_triangles(Vertex &v, int vid_collapsed, int iteration, int &deleted_triangles);
    // Update triangle connections and edge error after a edge is collapsed
    void push_vertex_refs(const Vertex &v, int vid_collapsed, int iteration);

    void Build_Ref();

    // compact triangles, compute edge error and build reference list
    void update_triangleNormal_and_vertexQuadricMatrix(bool isUpdatingMeshFirstTime);

    // Finally compact mesh before exiting
    void compact_mesh();

    // Error between vertex and Quadric
    static D vertex_error(const SymetricMatrix& q, D x, D y, D z);
    static D vertex_error(const SymetricMatrix& q, const vec3& v);

    // Error for one edge
    inline D FastQuadricSimplifier::edge_error(int id_v1, int id_v2)
    {
        vec3 p_result;
        D det;
        return edge_error(id_v1, id_v2, p_result, det);
    }
    inline D FastQuadricSimplifier::edge_error(int id_v1, int id_v2, vec3 &p_result, D& det)
    {
        return edge_error(vertices[id_v1], vertices[id_v2], p_result, det);
    }

    static D edge_error(const Vertex& v1, const Vertex& v2, vec3 &p_result, D& det);


    P3 Convert(vec3 p);

    // change ref for vertex
    void ChangeVertexRef(int v_id, vector<int> skip_T_ids, vector<int> add_T_ids, vector<int> T_vertexLocalIndexes);

    // remove sharp triangles that cause surface deformation due to very sharp angles
    int FastQuadricSimplifier::RemoveSharpTriangles(int& iterationsDone);
};
