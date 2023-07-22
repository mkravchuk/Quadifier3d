#include "stdafx.h"
#include "FastQuadricMeshSimplification.h"

const MeshLogicOptions_MeshSimplification& options = meshLogicOptions.MeshSimplification;

#define loopi(start_l,end_l) for ( int i=start_l;i<end_l;++i )
#define loopi(start_l,end_l) for ( int i=start_l;i<end_l;++i )
#define loopj(start_l,end_l) for ( int j=start_l;j<end_l;++j )
#define loopk(start_l,end_l) for ( int k=start_l;k<end_l;++k )


inline D min(D v1, D v2)
{
    //return fmin(v1, v2);
    return v1 < v2 ? v1 : v2;
}

inline D max(D v1, D v2)
{
    //return fmax(v1, v2);
    return v1 > v2 ? v1 : v2;
}
  
inline D min(D v1, D v2, D v3)
{
    return  fmin(fmin(v1, v2), v3);
}

inline D max(D v1, D v2, D v3)
{
    return  fmax(fmax(v1, v2), v3);
}



void FastQuadricSimplifier::Triangle::UpdateEdgeLengthsAndSharpness(const vector<Vertex>& vertices)
{
    edgesLengths[0] = (vertices[v[0]].p - vertices[v[1]].p).length();
    edgesLengths[1] = (vertices[v[1]].p - vertices[v[2]].p).length();
    edgesLengths[2] = (vertices[v[2]].p - vertices[v[0]].p).length();

    D minLength = min(edgesLengths[0], edgesLengths[1], edgesLengths[2]);
    D maxLength = max(edgesLengths[0], edgesLengths[1], edgesLengths[2]);
    D totalLength = edgesLengths[0] + edgesLengths[1] + edgesLengths[2];
    D summOf2SmallerEdges = totalLength - maxLength;
    sharpness = (summOf2SmallerEdges - maxLength) / maxLength;
}

inline FastQuadricSimplifier::vec3 FastQuadricSimplifier::Triangle::Centroid(const vector<Vertex>& vertices) const 
{
    vec3 tCenter = (vertices[v[0]].p + vertices[v[1]].p + vertices[v[2]].p) / 3;
    return tCenter;
}




FastQuadricSimplifier::FastQuadricSimplifier(ViewerDrawObjects& _draw)
    :TriangleNextId(0), debugBreakExecution(false), draw(_draw),
    RecomputeTriangleNormalsAfterEachIteration(options.RecomputeTriangleNormalsAfterEachIteration)
{
    FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS = utils::angle::DegreesToCos(options.MinAngleBetweenEdges_ForNewTriangles);
    FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS = utils::angle::DegreesToCos(options.MaxAngleChangeInNormal_ForNewTriangles);
    FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS_quad = FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS * FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS;
    FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS_quad = FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS * FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS;
}

void FastQuadricSimplifier::load(const P3s& V, const I3s& F, const Bs& V_isborder, const V3s& F_normals,
    const  I2s& EV, const I3s& FE)
{
    Vertex vZero;
    vertices.resize(V.rows(), vZero);
    for (int i = 0; i < V.rows(); i++)
    {
        Vertex& v = vertices[i];
        v.Index = i;
        v.isDeleted = false;
        v.p.x = V(i, 0);
        v.p.y = V(i, 1);
        v.p.z = V(i, 2);
        vertices[i].isBorder = V_isborder[i];
    }

    int eCount = EV.rows();
    triangles.resize(F.rows());
    for (int i = 0; i < F.rows(); i++)
    {
        Triangle& t = triangles[i];
        t.isDeleted = false;
        t.id = TriangleNextId++;
        t.v[0] = F(i, 0);
        t.v[1] = F(i, 1);
        t.v[2] = F(i, 2);
        t.e[0] = FE(i, 0);
        t.e[1] = FE(i, 1);
        t.e[2] = FE(i, 2);
        if (t.e[0] > eCount || t.e[1] > eCount || t.e[2] > eCount)
        {
            cout << "FastQuadricSimplifier::load   error    FE has higher index than EV size" << endl;
        }
        t.normal.x = F_normals(i, 0);
        t.normal.y = F_normals(i, 1);
        t.normal.z = F_normals(i, 2);
    }


    edges.resize(EV.rows());
    for (int i = 0; i < EV.rows(); i++)
    {
        Edge& e = edges[i];
        e.Index = i;
        e.isDeleted = false;
        e.isRemovedFromLoop = false;
        e.PrevEdgeIndex = i - 1;
        e.NextEdgeIndex = i + 1;
        e.dirtyUntilIteration = 0;
        e.v[0] = EV(i, 0);
        e.v[1] = EV(i, 1);
        e.isErrorInited = false;
        e.error = 0;
        e.error_det = 0;
        e.collapsePoint = { 0,0,0 };
    }
    //set first and last
    firstEdgeIndexInLoop = 0;
    lastEdgeIndexInLoop = EV.rows() - 1;
    if (EV.rows() > 0)
    {
        edges[firstEdgeIndexInLoop].PrevEdgeIndex = -1;
        edges[lastEdgeIndexInLoop].NextEdgeIndex = -1;
    }
    // remove border edges from simplify loop
    for (Edge& e : edges)
    {
        if (vertices[e.v[0]].isBorder || vertices[e.v[1]].isBorder)
        {
            e.RemoveFromLoop(edges, firstEdgeIndexInLoop, lastEdgeIndexInLoop);
        }
    }


    Build_Ref();
}

void FastQuadricSimplifier::write(P3s& V, I3s& F)
{
    V.resize(vertices.size(), 3);
    for (int i = 0; i < V.rows(); i++)
    {
        V(i, 0) = vertices[i].p.x;
        V(i, 1) = vertices[i].p.y;
        V(i, 2) = vertices[i].p.z;
    }

    F.resize(triangles.size(), 3);
    for (int i = 0; i < F.rows(); i++)
    {
        F(i, 0) = triangles[i].v[0];
        F(i, 1) = triangles[i].v[1];
        F(i, 2) = triangles[i].v[2];
    }
}





int FastQuadricSimplifier::simplify_loop(int iteration, D target_threshold, int target_count, bool showOnlyDebugInfo, int Debug_HighlightSimplificationNum)
{
    if (options.DebugEnabled && options.Debug_HighlightTryFailEdges)
    {
        for (Edge& e : edges)
        {
            e.failed_at_iteration = 0;
        }
    }
    int deleted_triangles = 0;
    int simplificationNum = 0;
    int triangle_count = triangles.size();

    currentEdgeIndexInLoop = firstEdgeIndexInLoop;
    while (currentEdgeIndexInLoop != -1)
    {
        Edge& edge = edges[currentEdgeIndexInLoop];
        currentEdgeIndexInLoop = edge.NextEdgeIndex;

        if (iteration <= edge.dirtyUntilIteration)
        {
            edge.failed_at_iteration = 1;
            continue;
        }

        //
        // Compute vertex to collapse to
        //
        edge.UpdateError(vertices);
        if (edge.error > target_threshold)
        {
            edge.failed_at_iteration = 2;
            continue;
        }

        vec3 collapsePoint = edge.collapsePoint;
        D det = edge.error_det;

        int i0 = edge.v[0];
        Vertex &v0 = vertices[i0];

        int i1 = edge.v[1];
        Vertex &v1 = vertices[i1];

        //
        // dont remove if flipped
        //
        if (flipped(collapsePoint, v0, i1, det))
        {
            edge.failed_at_iteration = 3;
            continue;
        }
        if (debugBreakExecution) return deleted_triangles;
        if (flipped(collapsePoint, v1, i0, det))
        {
            edge.failed_at_iteration = 4;
            continue;
        }
        if (debugBreakExecution) return deleted_triangles;

        //
        // not flipped, so remove edge
        //
        simplificationNum++;

        if (showOnlyDebugInfo
            && (Debug_HighlightSimplificationNum == -1 || Debug_HighlightSimplificationNum == simplificationNum))
        {
            draw.AddEdge(Convert(v0.p), Convert(v1.p), Color3d(0, 0, 0));
            draw.AddPoint(Convert(collapsePoint), Color3d(1, 0, 0));

            if (Debug_HighlightSimplificationNum == simplificationNum)
            {
                printf("\t\t\t\t\t\t\t\t\tcurrentError = %f \n", edge.error);
                return deleted_triangles;
            }
            // if not defined - then we do only highlight - otherwise proceed with simplification and make changes visible
            if (Debug_HighlightSimplificationNum == 0)
            {
                break;
            }
        }

        // not flipped, so remove edge
        v0.p = collapsePoint;
        v0.q += v1.q;
        int tstart = refs.size();

        remove_triangles(v1, i0, iteration, deleted_triangles);
        push_vertex_refs(v0, i0, iteration);
        push_vertex_refs(v1, i0, iteration);
        edge.Delete(edges, firstEdgeIndexInLoop, lastEdgeIndexInLoop, currentEdgeIndexInLoop);

        int tcount = refs.size() - tstart;

        if (tcount <= v0.tcount)
        {
            // save ram
            //TODO avoid moving memory
            if (tcount)memcpy(&refs[v0.tstart], &refs[tstart], tcount * sizeof(Ref));
        }
        else
            // append
            v0.tstart = tstart;

        v0.tcount = tcount;

        // done?
        if (triangle_count - deleted_triangles <= target_count)
        {
            //cout << "simplify_loop  break   (triangle_count - deleted_triangles <= target_count)" << endl;
            break;
        }
    }
    return deleted_triangles;
}


void FastQuadricSimplifier::simplify_mesh(bool isTargetingCount, D target_threshold, int target_count, D agressiveness, bool verbose)
{
    // main iteration loop
    int deleted_trianglesTotal = 0;
    int triangle_count = triangles.size();

    int update_mesh_trothle = RecomputeTriangleNormalsAfterEachIteration ? 1 : 5; // once in 5 iterations if specific option not set
    int iteration_max = options.DebugEnabled && options.Debug_MaxNumOfIterations > 0 ? options.Debug_MaxNumOfIterations + 1 : 999;
    update_triangleNormal_and_vertexQuadricMatrix(true);
    for (int iteration = 1; iteration < iteration_max; iteration++)
    {
        // update mesh once in a while
        if (iteration % update_mesh_trothle == 0)
        {
            update_triangleNormal_and_vertexQuadricMatrix(false);
            if (debugBreakExecution) break;
        }

        //
        // All triangles with edges below the threshold will be removed
        // The following numbers works well for most models.
        // If it does not, try to adjust the 3 parameters
        //
        if (isTargetingCount) target_threshold = static_cast<D>(0.000000001)*pow(D(iteration + 3), agressiveness);

        if (verbose)
        {
            printf("iteration %d - triangles %d threshold %g\n", iteration, triangle_count - deleted_trianglesTotal, target_threshold);
        }

        bool showOnlyDebugInfo = options.DebugEnabled && iteration == iteration_max - 1;
        int Debug_HighlightSimplificationNum = options.DebugEnabled ? options.Debug_HighlightSimplificationNum : 0;
        int deleted_trianglesI = simplify_loop(iteration, target_threshold, target_count, showOnlyDebugInfo, Debug_HighlightSimplificationNum);
        if (debugBreakExecution) break;
        deleted_trianglesTotal += deleted_trianglesI;

        // target number of triangles reached ? Then break
        if (triangle_count - deleted_trianglesTotal <= target_count)break;
        if (deleted_trianglesI <= 0)break;
        deleted_trianglesI = 0;
    }


    // DEBUG show TriangleIndex
    if (options.DebugEnabled && options.Debug_ShowTriangleIndex)
    {
        loopi(0, triangles.size())
        {
            Triangle &t = triangles[i];
            if (t.isDeleted) continue;
            P3 c = Convert(t.Centroid(vertices));
            draw.AddLabel(c, to_string(i), Color3d(0, 0, 1));
        }
    }

    // DEBUG show TriangleIndex
    if (options.DebugEnabled && options.Debug_ShowTriangleId)
    {
        loopi(0, triangles.size())
        {
            Triangle &t = triangles[i];
            if (t.isDeleted) continue;
            P3 c = Convert(t.Centroid(vertices));
            draw.AddLabel(c, to_string(t.id), Color3d(0, 0, 1));
        }
    }

    // DEBUG show EdgeErrors
    if (options.DebugEnabled && options.Debug_ShowEdgeErrors)
    {
        for(Edge& e: edges)
        {
            if (!e.isDeleted && e.isErrorInited)
            {
                e.UpdateError(vertices);
                draw.AddLabel(Convert((vertices[e.v[0]].p + vertices[e.v[1]].p) / 2), to_string(e.error), Color3d(1, 0, 0));
            }
            
        }
    }


    if (options.DebugEnabled && options.Debug_HighlightTryFailEdges)
    {
        for (Edge& e : edges)
        {
            if (!e.isRemovedFromLoop)
            {
                draw.AddEdge(Convert(vertices[e.v[0]].p), Convert(vertices[e.v[1]].p), e.failed_at_iteration ? Color3d(1, 0, 0) : Color3d(0, 0, 0));
                if (e.failed_at_iteration) draw.AddLabel(Convert((vertices[e.v[0]].p + vertices[e.v[1]].p) / 2), to_string(e.failed_at_iteration), Color3d(1, 0, 0));
            }
        }
    }

    // DEBUG show sharpness
    if (options.DebugEnabled && options.Debug_ShowTrianglessSharpness)
    {
        loopi(0, triangles.size())
        {
            Triangle &t = triangles[i];
            if (t.isDeleted) continue;
            //if (vertices[t.v[0]].isBorder) continue;
            //if (vertices[t.v[1]].isBorder) continue;
            //if (vertices[t.v[2]].isBorder) continue;

            // Check if triangle is sharp
            t.UpdateEdgeLengthsAndSharpness(vertices);
            P3 c = Convert(t.Centroid(vertices));
            Color3d color = Color3d(0, 1, 0); // by default not sharp
            if (t.sharpness < options.RemoveSharpTriangles_MaxFriendSharpness) color = Color3d(0.4, 0, 0.2); // can be a friend for sharp triangle
            if (t.sharpness < options.RemoveSharpTriangles_MinSharpness) color = Color3d(1, 0, 0); // is sharp - candidate to remove
            draw.AddLabel(c, to_string(t.sharpness), color);
        }
    }
    int iterationsDone = 0;
    int simplifiedCountTotal = 0;
    loopi(0, options.RemoveSharpTriangles_Iterations)
    {
        //cout << "RemoveSharpTriangles  #" << i << endl;
        int simplifiedCount = RemoveSharpTriangles(iterationsDone);
        if (simplifiedCount == 0) break;//if no more sharp triangles simplifed - stop searching
        simplifiedCountTotal += simplifiedCount;
    }
    if (options.DebugEnabled)
    {
        cout << "Removed " << simplifiedCountTotal << " sharp triangles" << endl;
    }
    // clean up mesh
    compact_mesh();
}

//
// Simplify mesh with target quality
//
// target_count  : target nr. of triangles
// agressiveness : sharpness to increase the threashold.
//                 5..8 are good numbers
//                 more iterations yield higher quality
//
void FastQuadricSimplifier::simplify_mesh_count(int target_count, D agressiveness, bool verbose)
{
    simplify_mesh(true, 0, target_count, agressiveness, verbose);
}


//
// Simplify mesh with target quality
//
// threshold - All triangles with edges below the threshold will be removed.  The following numbers works well for most models.
void FastQuadricSimplifier::simplify_mesh_lossless(D threshold, bool verbose)
{
    simplify_mesh(false, threshold, 0, 0, verbose);
}



// Check if a triangle flips when this edge is removed, read more at https://classes.soe.ucsc.edu/cmps160/Spring05/finalpages/scyiu/
bool FastQuadricSimplifier::flipped(vec3 collapsePoint, Vertex &v_collapsed, int vid_opposite, D det)
{
    auto debug_showRejection = [&](int id1, int id2, vec3 d1, vec3 d2, const Triangle &t)
    {
        D dot_d1_d2 = d1.dot(d2);
        vec3 crossed_d1_d2;
        crossed_d1_d2.cross(d1, d2);
        D cross_d1_d2_Len = crossed_d1_d2.length();

        draw.AddEdge(Convert(vertices[id1].p), Convert(collapsePoint), Color3d(0, 0, 0));
        draw.AddEdge(Convert(vertices[id2].p), Convert(collapsePoint), Color3d(0, 0, 0));
        draw.AddEdge(Convert(v_collapsed.p), Convert(vertices[vid_opposite].p), Color3d(0, 1, 0));
        draw.AddPoint(Convert(t.Centroid(vertices)), Color3d(0, 0, 1), "Triangle ID " + to_string(t.id));
        draw.AddPoint(Convert(v_collapsed.p), Color3d(0, 1, 0), "collapsed Point  (fabs(d1.dot(d2)) > 0.999)");
        draw.AddPoint(Convert(collapsePoint), Color3d(1, 0, 0), "collapse Point   (det = " + to_string(det) + ")");
        vec3 n;
        n.cross(d1, d2);
        n.normalize();
        D dot_nd1d2_n = n.dot(t.normal);
        draw.AddPoint(Convert((vertices[id1].p + vertices[id2].p) / 2), Color3d(1, 0, 0), "dot_d1_d2 = " + to_string(dot_d1_d2) + "    cross_d1_d2_Len = " + to_string(cross_d1_d2_Len) + "   dot_nd1d2_n = " + to_string(dot_nd1d2_n));
        debugBreakExecution = true;
    };


    loopi(0, v_collapsed.tcount)// for each traingle in vertex
    {
        Ref &r = refs[v_collapsed.tstart + i];
        const Triangle &t = triangles[r.tid]; // take current triangle
        if (t.isDeleted) continue; //to avoid memory movement we can mark triangle as deleted - so lets ignore it

        int id0Local = refs[v_collapsed.tstart + i].tvertex; // take triangle's local vertex index of merged vertex
        int id1 = t.v[(id0Local + 1) % 3]; // take next global vertex index (not same vertex as merged one, means opposite vertex, triangle has 3 vertexes - one merged and two opposite)
        int id2 = t.v[(id0Local + 2) % 3]; // take next-next global vertex index (not same vertex as merged one, means opposite vertex, triangle has 3 vertexes - one merged and two opposite)


        //ignore deleted triangles
        if (id1 == vid_opposite || id2 == vid_opposite)
        {
            continue;
        }

        // check if triangle is not collapsed into a single line and also is not sharp
        vec3 d1 = vertices[id1].p - collapsePoint; // take vector from opposite vertex to merged vertex 
        vec3 d2 = vertices[id2].p - collapsePoint; // take vector from opposite vertex to merged vertex

        // take cos() between d1 and d2 (how close d1 and d2 to each other)
        //v1 - simple
        //d1.normalize();
        //d2.normalize();
        //D dot_d1_d2 = d1.dot(d2);
        //bool isToSharp = fabs(dot_d1_d2) > FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS;
        //v2 - fast - without using normalize
        //D d1_d2_cos_quad = vec3::cos_quad(d1, d2);
        //bool isToSharp_quad = d1_d2_cos_quad > FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS_quad;
        //if (isToSharp_quad != isToSharp)
        //{
        //    cout << "isToSharp_quad != isToSharp" << endl;
        //}
        //v3 - fast - without using normalize and devide (avoiding zero devision)
        D dot_d1_d2 = d1.dot(d2);
        D len_quad = d1.length_quad() * d2.length_quad();
        bool isToSharp = dot_d1_d2*dot_d1_d2 > len_quad*FLIPPED_MIN_ANGLE_BETWEEN_EDGES_COS_quad;

        if (isToSharp)
        {
            if (options.DebugEnabled && options.Debug_HighlightFlippedTraingles)
            {
                debug_showRejection(id1, id2, d1, d2, t);
            }
            return true;// angle between vectors should be possitive, means new traingle should'nt be flipped
        }

        // check if new triangle is not flipped
        // angle between normals of original triangle and new triangle should be not to high (plane is formed by new vertex and opposite edge of triangle), 
        // also if angle < 0 then new triangle is flipped
        //v1 - simple
        //vec3 n;
        //n.cross(d1, d2);
        //n.normalize();
        //D dot_nd1d2_n = n.dot(t.normal);
        //bool isFlipped = dot_nd1d2_n < FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS;
        //v2-fast without sqrt
        vec3 normald1d2;
        normald1d2.cross(d1, d2);
        D dot_normald1d2_t = normald1d2.dot(t.normal);
        bool isFlipped = dot_normald1d2_t<0 || dot_normald1d2_t*dot_normald1d2_t < normald1d2.length_quad()*FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS_quad;
        //if (isFlipped_quad != isFlipped)
        //{
        //    cout << "isFlipped_quad != isFlipped  " << isFlipped_quad<< " " << isFlipped << " dot="<< dot_normald1d2_t*dot_normald1d2_t << "  max = " << normald1d2.length_quad()*FLIPPED_MAX_ANGLE_CHANGE_IN_NORMAL_COS_quad << endl;
        //}

        if (isFlipped)
        {
            if (options.DebugEnabled && options.Debug_HighlightFlippedTraingles)
            {
                debug_showRejection(id1, id2, d1, d2, t);
            }
            return true;
        }
    }
    return false;
}

void FastQuadricSimplifier::replace_edge(Vertex &v, int eid_old, int eid_new)
{
    loopi(0, v.tcount)
    {
        Ref &r = refs[v.tstart + i];
        Triangle &t = triangles[r.tid];
        if (t.isDeleted) continue; //to avoid memory movement we can mark triangle as deleted - so lets ignore it
        t.replace_edge(eid_old, eid_new);
    }
}


void FastQuadricSimplifier::remove_triangles(Vertex &v, int vid_collapsed, int iteration, int &deleted_triangles)
{
    //delete vertex
    v.isDeleted = true;

    loopi(0, v.tcount)
    {
        Ref &r = refs[v.tstart + i];
        Triangle &t = triangles[r.tid];
        if (t.isDeleted) continue; //to avoid memory movement we can mark triangle as deleted - so lets ignore it

        int id0Local = refs[v.tstart + i].tvertex; // take triangle's local vertex index of merged vertex
        int id1 = t.v[(id0Local + 1) % 3]; // take next global vertex index (not same vertex as merged one, means opposite vertex, triangle has 3 vertexes - one merged and two opposite)
        int id2 = t.v[(id0Local + 2) % 3]; // take next-next global vertex index (not same vertex as merged one, means opposite vertex, triangle has 3 vertexes - one merged and two opposite)

        if (id1 == vid_collapsed || id2 == vid_collapsed)
        {
            // delete and replace edge
            if (id1 == vid_collapsed)
            {
                int eid_new = t.e[(id0Local + 1) % 3];
                int eid_old = t.e[(id0Local + 2) % 3];
                replace_edge(v, eid_old, eid_new);
                edges[eid_old].Delete(edges, firstEdgeIndexInLoop, lastEdgeIndexInLoop, currentEdgeIndexInLoop);
            }

            // delete and replace edge
            if (id2 == vid_collapsed)
            {
                int eid_new = t.e[(id0Local + 1) % 3];
                int eid_old = t.e[(id0Local + 0) % 3];
                replace_edge(v, eid_old, eid_new);
                edges[eid_old].Delete(edges, firstEdgeIndexInLoop, lastEdgeIndexInLoop, currentEdgeIndexInLoop);
            }
            
            // delete triangle            
            t.isDeleted = true;
            deleted_triangles++;
        }
    }
}

void FastQuadricSimplifier::push_vertex_refs(const Vertex &v, int vid_collapsed, int iteration)
{
    loopi(0, v.tcount)
    {
        Ref &r = refs[v.tstart + i];
        Triangle &t = triangles[r.tid];
        if (t.isDeleted)continue;

        // Replace vertex
        int ei0 = t.e[r.tvertex];
        int ei2 = t.e[(r.tvertex + 2) % 3];
        edges[ei0].MakeDirty(iteration);
        edges[ei2].MakeDirty(iteration);

        if (v.Index != vid_collapsed)
        {
            t.v[r.tvertex] = vid_collapsed;
            edges[ei0].ReplaceVertex(v.Index, vid_collapsed, iteration);
            edges[ei2].ReplaceVertex(v.Index, vid_collapsed, iteration);
        }

        refs.push_back(r);
    }
}

void FastQuadricSimplifier::Build_Ref()
{
    // Init Reference ID list
    loopi(0, vertices.size())
    {
        vertices[i].tcount = 0;
    }
    loopi(0, triangles.size())
    {
        Triangle &t = triangles[i];
        loopj(0, 3) vertices[t.v[j]].tcount++;
    }
    int tstart = 0;
    loopi(0, vertices.size())
    {
        Vertex &v = vertices[i];
        v.tstart = tstart;
        tstart += v.tcount;
        v.tcount = 0;//temporaly clear this count - we will recount it in next (below) loop 'Write References'
    }

    // Write References (triangles infos connected to vertex)
    refs.reserve(triangles.size() * 3 * 10);// this array changes often - so lets allocate additional space to avoid memory movements
    refs.resize(triangles.size() * 3);
    loopi(0, triangles.size())
    {
        Triangle &t = triangles[i];
        loopk(0, 3)
        {
            Vertex &v = vertices[t.v[k]];
            Ref& ref = refs[v.tstart + v.tcount];
            //ref.tid = i;
            //ref.tvertex = k;
            ref = { i, k };
            v.tcount++;
        }
    }
}

void FastQuadricSimplifier::update_triangleNormal_and_vertexQuadricMatrix(bool isUpdatingMeshFirstTime)
{
    // Init Quadrics by Plane & Edge Errors
    // required at the beginning ( iteration == 0 )
    // recomputing during the simplification is not required, but mostly improves the result for closed meshes
    if (isUpdatingMeshFirstTime || RecomputeTriangleNormalsAfterEachIteration)
    {
        // TODO dont recompute if friends didnt changed!!!
        loopi(0, vertices.size())
            vertices[i].q.Clear();

        loopi(0, triangles.size())
        {
            Triangle &t = triangles[i];
            if (t.isDeleted) continue;

            if (!isUpdatingMeshFirstTime)
            {
                vec3 p[3];
                loopj(0, 3) p[j] = vertices[t.v[j]].p;

                vec3 normal;
                normal.cross(p[1] - p[0], p[2] - p[0]);
                normal.normalize();
                t.normal = normal;
            }

            vec3 normal = t.normal;
            loopk(0, 3)
            {
                Vertex &v = vertices[t.v[k]];
                //D normal_dot_p = normal.dot(vertices[t.v[0]].p);
                D normal_dot_p = normal.dot(v.p);
                v.q += SymetricMatrix(normal.x, normal.y, normal.z, -normal_dot_p);
            }
        }
        // Deinit triangles edge error
        if (!isUpdatingMeshFirstTime)
        {
            int index = firstEdgeIndexInLoop;
            while (index != -1)
            {
                Edge &e = edges[index];
                e.isErrorInited = false;
                index = e.NextEdgeIndex;
            }
        }
    }
}

// Finally compact mesh before exiting
void FastQuadricSimplifier::compact_mesh()
{
    int dst = 0;
    loopi(0, vertices.size())
    {
        vertices[i].tcount = 0;
    }
    loopi(0, triangles.size())
        if (!triangles[i].isDeleted)
        {
            Triangle &t = triangles[i];
            triangles[dst++] = t;
            loopj(0, 3)vertices[t.v[j]].tcount = 1;
        }
    triangles.resize(dst);
    dst = 0;
    loopi(0, vertices.size())
        if (vertices[i].tcount)
        {
            vertices[i].tstart = dst;
            vertices[dst].p = vertices[i].p;
            dst++;
        }
    loopi(0, triangles.size())
    {
        Triangle &t = triangles[i];
        loopj(0, 3)t.v[j] = vertices[t.v[j]].tstart;
    }
    vertices.resize(dst);
}

// Error between vertex and Quadric
D FastQuadricSimplifier::vertex_error(const SymetricMatrix& q, D x, D y, D z)
{
    return
        1 * q[0] * x*x
        + 2 * q[1] * x*y
        + 2 * q[2] * x*z
        + 2 * q[3] * x
        + 1 * q[4] * y*y
        + 2 * q[5] * y*z
        + 2 * q[6] * y
        + 1 * q[7] * z*z
        + 2 * q[8] * z
        + 1 * q[9];


}

D FastQuadricSimplifier::vertex_error(const SymetricMatrix& q, const vec3& v)
{
    return vertex_error(q, v.x, v.y, v.z);
}

// Error for one edge
D FastQuadricSimplifier::edge_error(const Vertex& v1, const Vertex& v2, vec3 &p_result, D& det)
{
    // compute interpolated vertex
    //SymetricMatrix q = v1.q + v2.q;
    SymetricMatrix q = v1.q;
    q += v2.q;
    bool   isBorder = v1.isBorder && v2.isBorder;
    D error = 0;
    det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);
    //if (det != 0 && !isBorder) -  we cant compare D with 0 
    if (abs(det) > 0.01 && !isBorder)   //is vertices are not in plane (value 0.01 is choisen coz it is best - dont try to make it smaller - smaller values causing bad locations of result collapsed point)
    {

        // q_delta is invertible
        D div1 = 1 / det;
        p_result.x = -div1*(q.det(1, 2, 3, 4, 5, 6, 5, 7, 8));	// vx = A41/det(q_delta)
        p_result.y = div1*(q.det(0, 2, 3, 1, 5, 6, 2, 7, 8));	// vy = A42/det(q_delta)
        p_result.z = -div1*(q.det(0, 1, 3, 1, 4, 6, 2, 5, 8));	// vz = A43/det(q_delta)

        error = vertex_error(q, p_result);
    }
    else
    {
        // vertices are in plane
        // det = 0 -> try to find best result
        //v1 origin 
        //vec3 p1 = v1.p;
        //vec3 p2 = v2.p;
        //vec3 p3 = (p1 + p2) / 2;
        //D error1 = vertex_error(q, p1);
        //D error2 = vertex_error(q, p2);
        //D error3 = vertex_error(q, p3);
        //error = min(error1, min(error2, error3));
        //if (error1 == error) p_result = p1;
        //if (error2 == error) p_result = p2;
        //if (error3 == error) p_result = p3;
        //v2 - avoid wrong collapse points thatn generates sharp triangles
        p_result = (v1.p + v2.p) / 2; //  always use mid point - this is the most safety point from making sharp edges triangles
        error = vertex_error(q, p_result);
    }
    return error;
}

P3 FastQuadricSimplifier::Convert(vec3 p)
{
    return P3(p.x, p.y, p.z);
}

void FastQuadricSimplifier::ChangeVertexRef(int v_id, vector<int> skip_T_ids, vector<int> add_T_ids, vector<int> T_vertexLocalIndexes)
{
    //cout << "ChangeVertexRef  v_id = " << v_id << endl;
    int addCount = add_T_ids.size();
    int skipCount = skip_T_ids.size();

    //update vertices[].tStart, tCount
    int oldStart = vertices[v_id].tstart;
    int oldCount = vertices[v_id].tcount;
    vertices[v_id].tstart = refs.size(); // we will append new reference list to the end
    vertices[v_id].tcount += addCount - skipCount;// change size of new ref list

    // copy old references
    loopi(0, oldCount)
    {
        int tiNext = refs[oldStart + i].tid;
        bool skiped = false;
        loopk(0, skipCount)
        {
            if (tiNext == skip_T_ids[k])
            {
                //cout << "   skiped " << v_id << endl;
                skiped = true;
                break;
            }
        }
        if (skiped) continue;
        Ref refOld = refs[oldStart + i];
        refs.push_back(refOld); // move reference to new position
    }

    // copy new references
    loopi(0, addCount)
    {
        Ref refNew;
        refNew.tid = add_T_ids[i];
        refNew.tvertex = T_vertexLocalIndexes[i];
        refs.push_back(refNew);
    }
}

int FastQuadricSimplifier::RemoveSharpTriangles(int& iterationsDone)
{
    int simplifiedCount = 0;
    if (!options.RemoveSharpTriangles) return 0;

    int trianglesCount = triangles.size();
    loopi(0, trianglesCount)
    {
        // this is to prevent very long loop execution - since this method is not so important we will terminate it if it run to long
        iterationsDone++;
        if (iterationsDone > 1024 * 1024 * 10)
        {
            return 0;// exit and say to stop calling this method
        }

        Triangle &t = triangles[i];

        // Skip non valid traingles
        if (t.isDeleted) continue;
        if (vertices[t.v[0]].isBorder) continue;
        if (vertices[t.v[1]].isBorder) continue;
        if (vertices[t.v[2]].isBorder) continue;
        if (t.RemoveSharpTriangles__wasAFriend)  continue;//if this trinagle is a old friend - dont simplify it

        // Check if triangle is sharp
        t.UpdateEdgeLengthsAndSharpness(vertices);
        if (t.sharpness > options.RemoveSharpTriangles_MinSharpness) continue; // remove traingles only if they sharpness is very low

        // Get biggest edge points, and shared point of smaller edges
        int viBig1 = 0; // point 1 of bigger edge
        int viBig2 = 1; // point 2 of bigger edge
        int viSmall = 2; // point of shared edges
        D pLen = t.edgesLengths[0];
        if (t.edgesLengths[1] > pLen)
        {
            viBig1 = 1;
            viBig2 = 2;
            viSmall = 0;
            pLen = t.edgesLengths[1];
        }
        if (t.edgesLengths[2] > pLen)
        {
            viBig1 = 2;
            viBig2 = 0;
            viSmall = 1;
            pLen = t.edgesLengths[2];
        }
        viBig1 = t.v[viBig1];// convert from local index to global
        viBig2 = t.v[viBig2];// convert from local index to global
        viSmall = t.v[viSmall];// convert from local index to global
        vec3 vBig1 = vertices[viBig1].p;
        vec3 vBig2 = vertices[viBig2].p;
        vec3 vSmall = vertices[viSmall].p;
        vec3 vBig1Middle = (vBig1 + vBig2) / 2;


        // Get triangle common to bigger edge
        int tiFriend = -1;
        loopj(0, trianglesCount)
        {
            // this is to prevent very long loop execution - since this method is not so important we will terminate it if it run to long
            iterationsDone++;
            if (iterationsDone > 1024 * 1024 * 10)
            {
                return 0;// exit and say to stop calling this method
            }

            Triangle &tFriend = triangles[j];
            if (i != j //  if this is not same sharp triangle that we testing
                && !tFriend.isDeleted // triangle not deleted
                && (tFriend.v[0] == viBig1 || tFriend.v[1] == viBig1 || tFriend.v[2] == viBig1) // if some triangle has viBig1
                && (tFriend.v[0] == viBig2 || tFriend.v[1] == viBig2 || tFriend.v[2] == viBig2))  // and if some triangle has viBig2
            {
                tiFriend = j; // we found common friend triangle
                break;
            }
        }

        // skip if not found (can happend if friend is newly generated triangle - we must in this case skip it to avoid forever loops)
        if (tiFriend == -1) continue;
        Triangle& tFriend = triangles[tiFriend];

        // dont merge two sharp triangles
        tFriend.UpdateEdgeLengthsAndSharpness(vertices);
        if (tFriend.sharpness < options.RemoveSharpTriangles_MaxFriendSharpness) continue; // if friend sharpness is also low - skipp such optimization, coz it can lead to flipped triangles

        // Get friend opposite vertex
        int tiFriendOppositeVertex = 0;
        if (tFriend.v[tiFriendOppositeVertex] == viBig1 || tFriend.v[tiFriendOppositeVertex] == viBig2) tiFriendOppositeVertex = 1;
        if (tFriend.v[tiFriendOppositeVertex] == viBig1 || tFriend.v[tiFriendOppositeVertex] == viBig2) tiFriendOppositeVertex = 2;
        tiFriendOppositeVertex = tFriend.v[tiFriendOppositeVertex];// convert from local index to global
        vec3 tFriendOppositeVertex = vertices[tiFriendOppositeVertex].p;


        //if (t.id != 8607 && t.id != 8731) continue;

        // DEBUG highlight sharp triangles
        if (options.DebugEnabled && options.Debug_HighlightSharpTriangles)
        {
            P3 c = Convert(t.Centroid(vertices));
            draw.AddLabel(c, to_string(t.id));
            draw.AddPoint(c, Color3d(1, 0, 0));
            P3 cFriend = Convert(tFriend.Centroid(vertices));
            draw.AddLabel(cFriend, "Friend " + to_string(tiFriend));
            draw.AddEdge(Convert(vBig1Middle), Convert(vSmall), Color3d(0, 0, 1)); // highlight separate line for removed traingle
            draw.AddEdge(Convert(vBig1Middle), Convert(tFriendOppositeVertex), Color3d(0, 1, 0)); // highlight separate line for removed traingle
            continue;
        }


        // Remove this sharp triangle and his friend
        t.isDeleted = true;
        tFriend.isDeleted = true;

        // Move viSmall[viSmall] to centroid of sharp triangle
        //vertices[viSmall].p = t.Centroid(vertices);

        // Create 2 new traingles in place of friend
        Triangle tFriendNew1;
        tFriendNew1.v[0] = tiFriendOppositeVertex;
        tFriendNew1.v[1] = viSmall;
        tFriendNew1.v[2] = viBig1;
        tFriendNew1.isDeleted = false;
        tFriendNew1.RemoveSharpTriangles__wasAFriend = true;
        int tiFriendNew1 = triangles.size();
        triangles.push_back(tFriendNew1);
        ChangeVertexRef(viBig1, { tiFriend , i }, { tiFriendNew1 }, { 2 }); //update vertex connection to new triangle tFriendNew1

        Triangle tFriendNew2;
        tFriendNew2.v[0] = tiFriendOppositeVertex;
        tFriendNew2.v[1] = viBig2;
        tFriendNew2.v[2] = viSmall;
        tFriendNew2.isDeleted = false;
        tFriendNew2.RemoveSharpTriangles__wasAFriend = true;
        int tiFriendNew2 = triangles.size();
        triangles.push_back(tFriendNew2);
        ChangeVertexRef(viBig2, { tiFriend , i }, { tiFriendNew2 }, { 1 }); //update vertex connection to new triangle tFriendNew2

        //cout << "tiFriendNew1 = " << tiFriendNew1 << "    tiFriendNew2 = " << tiFriendNew2 << endl;

        //update vertices[tiFriendOppositeVertex].tStart, tCount
        ChangeVertexRef(tiFriendOppositeVertex, { tiFriend }, { tiFriendNew1, tiFriendNew2 }, { 0,0 }); //update vertex connection to new triangle tFriendNew2

        // Update traingles that shares small edges common point
        ChangeVertexRef(viSmall, {  }, { tiFriendNew1, tiFriendNew2 }, { 1,2 }); //update vertex connection to new triangle tFriendNew2
        simplifiedCount++;
    }

    return simplifiedCount;
}