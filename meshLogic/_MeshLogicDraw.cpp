#include "stdafx.h"
#include "Mesh.h"
#include "_MeshLogicDraw.h"
#include "MeshSolverNrosy.h"
#include "MeshSurface.h"
#include "MeshStreams.h"
#include "MeshLoop.h"
#include "igl/parula.h"

const MeshLogicOptions_Draw& options = meshLogicOptions.Draw;



MeshLogicDraw::MeshLogicDraw(MeshSurface& srf)
    : mesh(srf.mesh), draw(srf.draw)
{
}

MeshLogicDraw::MeshLogicDraw(const Mesh& _mesh, ViewerDrawObjects& _draw)
    : mesh(_mesh), draw(_draw)
{
}


void MeshLogicDraw::DrawAll()
{
    if (options.Mesh.Id_Vertex) DrawId_Vertex();
    if (options.Mesh.Id_Edge) DrawId_Edge();
    if (options.Mesh.Id_Face) DrawId_Face();
    if (options.Mesh.Id_Mesh) DrawId_Mesh();
    if (options.Mesh.Id_MeshLoop) DrawId_MeshLoop();
    if (options.Mesh.Normals_Faces) DrawNormals(true, false, false);
    if (options.Mesh.Normals_Vertixes) DrawNormals(false, true, false);
    if (options.Mesh.Normals_Faces_Calculated) DrawNormals(false, false, true);

    if (options.Mesh.Borders) DrawBorders();
    if (options.Mesh.BorderIndexes) DrawBorderIndexes();
    if (options.Mesh.BorderLabels) DrawBorderLabels();
    if (options.Mesh.BorderSharpnessDots) DrawBorderSharpness();
    if (options.Mesh.BorderAngles) DrawBorderAngles();
    if (options.Mesh.Highlight.Enabled) DrawHighlights();
}

void MeshLogicDraw::DrawObject(int id, string name)
{
    if (options.Mesh.Obj_id && id != -1) Draw_Object_Id(id);
    if (options.Mesh.Obj_name && !empty(name)) Draw_Object_Name(name);
}


void MeshLogicDraw::Draw_BorderConstrainesIsoLines(const MeshSolverNrosy& solver, int borderConstrainesIsoLinesCount, int borderConstrainesIsoLinesCount_intensivity, bool drawLines, bool logMessages)
{
    if (borderConstrainesIsoLinesCount == 0) return;
    if (solver.Result.Field.size() == 0) return;

    int count = solver.Constrains.Constrains.size();

    //DEBUG show contrains
    //for (int i = 0; i < count; i++)
    //{
    //    int fid = solver.Constrains.Constrains[i].FaceId;
    //    V3 constrain = solver.Constrains.Constrains[i].Direction;
    //    P3 point = mesh.F_Barycenters.row(fid);
    //    draw.AddEdge(point, point + constrain*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
    //}

    //
    // Draw streamlines
    //

    MeshStreams s(draw, mesh, solver);
    s.Reserve(count);
    for (int i = 0; i < count; i++)
    {
        int fid = solver.Constrains.Constrains[i].FaceId;
        int eid = solver.Constrains.Constrains[i].EdgeId;
        //V3 constrain = solver.Constrains.Constrains[i].Direction_Corrected;   - incorrect here since solver may ignore correction direction     
        V3 constrain = solver.Result.Field[1].row(fid); // - so lets take direction from result
        //P3 point = mesh.F_Barycenters.row(fid);
        for (int k = 0; k < borderConstrainesIsoLinesCount_intensivity; k++)
        {
            D percent = (1.0 / (D)(borderConstrainesIsoLinesCount_intensivity +1))*(k + 1);
            P3 point = mesh.EdgePointAtPercent(eid, percent);
            s.Add(-1, -1, fid, MeshPointType::onEdge, eid, point, constrain, Color3d(0.6, 0.6, 0.7), -2);
        }
    }


    //
    // v1
    //
    //if (drawLines) draw.ReserveEdges(count*borderConstrainesIsoLinesCount); // reseve space - maximum of what we can add
    //int extendedCount = 0;
    //for (int i = 0; i < borderConstrainesIsoLinesCount; i++)
    //{
    //    int extendedCountI = s.ExtendStreams();
    //    if (extendedCountI == 0) break;
    //    extendedCount += extendedCountI;
    //    for (int i = 0; i < count; i++)
    //    {
    //        P3 p1 = s.start_points.row(i);
    //        P3 p2 = s.end_points.row(i);
    //        if (!s.finished(i))
    //        {
    //            if (drawLines)draw.AddEdge(p1, p2, s.colors.row(i));
    //        }
    //        //draw.AddLabel(p1, "  "+to_string(i));
    //    }
    //}
    //if (drawLines) cout << "Extended " << extendedCount << " times" << endl;


    //
    // v2
    //
    int extendedCount = s.ExtendStreams(borderConstrainesIsoLinesCount);
    if (drawLines)
    {
        draw.ReserveEdges(extendedCount);
        for (int i = 0; i < s.Count(); i++)
        {
            const MeshStream& points = s[i];
            for (int pi = 0; pi < points.size() - 1; pi++)
            {
                draw.AddEdge(points[pi].point, points[pi + 1].point, s[i].color);
            }
        }
        if (logMessages) cout << "Extended " << extendedCount << " times" << endl;
    }
}

void MeshLogicDraw::Draw_BorderConstrainesIsoLinesUV(const MeshSolverUV& solver, int borderConstrainesIsoLinesCount, int borderConstrainesIsoLinesCount_intensivity, bool drawLines, bool logMessages)
{
    if (borderConstrainesIsoLinesCount == 0) return;
    if (solver.Result.Field.size() == 0) return;

    int count = solver.Constrains.Constrains.size();

    //DEBUG show contrains
    //for (int i = 0; i < count; i++)
    //{
    //    int fid = solver.Constrains.Constrains[i].FaceId;
    //    V3 constrain = solver.Constrains.Constrains[i].Direction;
    //    P3 point = mesh.F_Barycenters.row(fid);
    //    draw.AddEdge(point, point + constrain*(mesh.avg_edge_length / 2), Color3d(1, 0, 0));
    //}

    //
    // Draw streamlines
    //
    MeshStreams s(draw, mesh, solver);
    s.Reserve(count);
    for (int i = 0; i < count; i++)
    {
        int inext = (i == count - 1) ? 0 : i + 1;
        if (solver.Constrains.Constrains[i].Type != MeshPointType::onVertex) continue;
        if (solver.Constrains.Constrains[inext].Type != MeshPointType::onVertex) continue;
        int vid0 = solver.Constrains.Constrains[i].vid_eid_fid;
        int vid1 = solver.Constrains.Constrains[inext].vid_eid_fid;
        int eid = mesh.CommonEdgeId_VertexVertex(vid0, vid1);
        if (eid == -1) continue;
        int commonFaceId1;
        int commonFaceId2;
        mesh.CommonFaceIds_VertexVertex(vid0, vid1, commonFaceId1, commonFaceId2);
        if (commonFaceId1 == -1) continue;
        P3 v0 = mesh.V.row(vid0);
        P3 v1 = mesh.V.row(vid1);
        for (int k = 0; k < borderConstrainesIsoLinesCount_intensivity; k++)
        {
            D percent = (1.0 / (D)(borderConstrainesIsoLinesCount_intensivity + 1))*(k + 1);
            P3 vmid = (v0 + v1) *percent;
            UV uv0 = solver.Constrains.Constrains[i].uv;
            UV uv1 = solver.Constrains.Constrains[inext].uv;
            MeshUV uvStart = MeshUV(MeshPointType::onEdge, eid, vmid, { (uv0[0] + uv1[0]) *percent, (uv0[1] + uv1[1]) *percent }); // at the middle of edge
            UV uvEnd = uvStart.uv.getUVEnd();
            s.Add(-1, -1, uvStart, uvEnd, Color3d(0.6, 0.6, 0.7), -2);
        }
    }


    //
    // v1
    //
    //if (drawLines) draw.ReserveEdges(count*borderConstrainesIsoLinesCount); // reseve space - maximum of what we can add
    //int extendedCount = 0;
    //for (int i = 0; i < borderConstrainesIsoLinesCount; i++)
    //{
    //    int extendedCountI = s.ExtendStreams();
    //    if (extendedCountI == 0) break;
    //    extendedCount += extendedCountI;
    //    for (int i = 0; i < count; i++)
    //    {
    //        P3 p1 = s.start_points.row(i);
    //        P3 p2 = s.end_points.row(i);
    //        if (!s.finished(i))
    //        {
    //            if (drawLines)draw.AddEdge(p1, p2, s.colors.row(i));
    //        }
    //        //draw.AddLabel(p1, "  "+to_string(i));
    //    }
    //}
    //if (drawLines) cout << "Extended " << extendedCount << " times" << endl;


    //
    // v2
    //
    int extendedCount = s.ExtendStreams(borderConstrainesIsoLinesCount);
    if (drawLines)
    {
        draw.ReserveEdges(extendedCount);
        for (int i = 0; i < s.Count(); i++)
        {
            const MeshStream& points = s[i];
            for (int pi = 0; pi < points.size() - 1; pi++)
            {
                draw.AddEdge(points[pi].point, points[pi + 1].point, s[i].color);
            }
        }
        if (logMessages) cout << "Extended " << extendedCount << " times" << endl;
    }
}

// Plots the mesh with an N-RoSy field and its singularities on top
// The constrained faces (b) are colored in red.
void MeshLogicDraw::draw_mesh_nrosy(const MeshSolverNrosy& solver, bool logMessages)
{
    int N = static_cast<int>(meshLogicOptions.Solver.N) + 1;
    if (options.Solver.ShowDirections && solver.Result.Field.size() > 0)
    {
        // Expand the representative vectors in the full vector set and plot them as lines
        const vector<V3s>& FF = solver.Result.Field;
        const Color3d c0 = Color3d(0, 1, 0);
        const Color3d c1 = Color3d(0, 0, 1);
        const P3s& C = mesh.F_Barycenters;
        draw.ReserveEdges(FF[0].rows()*N);
        D len = (mesh.avg_edge_length / 4);
        if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::FirstVectors)
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 dir0 = FF[0].row(fi);
                P3 c = C.row(fi);
                draw.AddEdge(c, c + dir0 * len, c0);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::SecondVectors)
        {
            for (int fi = 0; fi < FF[1].rows(); ++fi)
            {
                V3 dir1 = FF[1].row(fi);
                P3 c = C.row(fi);
                draw.AddEdge(c, c + dir1 * len, c1);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::TwoColoredVectors)
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 dir0 = FF[0].row(fi);
                V3 dir1 = FF[1].row(fi);
                P3 c = C.row(fi);
                draw.AddEdge(c, c + dir0 * len, c0);
                draw.AddEdge(c, c + dir1 * len, c1);

                //DEBUG - show angle between X and direction
               /* V3 x = mesh.F_Y.row(fi);
                D angle = utils::vector::Angle(x, dir1);
                draw.AddLabel(mesh.F_Barycenters.row(fi), "" + to_string(round(angle)));*/
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::TwoColoredVectorsBordersOnly)
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                if (mesh.F_isborder[fi])
                {
                    V3 dir0 = FF[0].row(fi);
                    V3 dir1 = FF[1].row(fi);
                    P3 c = C.row(fi);
                    draw.AddEdge(c, c + dir0 * len, c0);
                    draw.AddEdge(c, c + dir1 * len, c1);

                    //DEBUG - show angle between X and direction
                    /* V3 x = mesh.F_Y.row(fi);
                    D angle = utils::vector::Angle(x, dir1);
                    draw.AddLabel(mesh.F_Barycenters.row(fi), "" + to_string(round(angle)));*/
                }
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::TwoColoredWithBisectors)
        {
            V3s BIS1, BIS2;
            V3s FF1 = FF[0];
            V3s FF2 = FF[1];
            utils::mesh::compute_frame_field_bisectors(mesh.F_X, mesh.F_Y, FF1, FF2, BIS1, BIS2);
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 dir0 = FF[0].row(fi);
                V3 dir1 = FF[1].row(fi);
                V3 bi0 = BIS1.row(fi);
                V3 bi1 = BIS2.row(fi);
                P3 c = C.row(fi);

                draw.AddEdge(c, c + dir0 * len, c0);
                draw.AddEdge(c, c + dir1 * len, c1);
                draw.AddEdge(c, c + bi0 * (len / 3), c0);
                draw.AddEdge(c, c + bi1 * (len / 3), c1);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::TwoColoredWithXY)
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 dir0 = FF[0].row(fi);
                V3 dir1 = FF[1].row(fi);
                V3 x = mesh.F_X.row(fi);
                V3 y = mesh.F_Y.row(fi);
                P3 c = C.row(fi);

                draw.AddEdge(c, c + dir0 * len, c0);
                draw.AddEdge(c, c + dir1 * len, c1);
                draw.AddEdge(c, c + x * (len / 3), c0);
                draw.AddEdge(c, c + y * (len / 3), c1);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::XY)
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 x = mesh.F_X.row(fi);
                V3 y = mesh.F_Y.row(fi);
                P3 c = C.row(fi);

                draw.AddEdge(c, c + x * (len / 3), c0);
                draw.AddEdge(c, c + y * (len / 3), c1);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::TwoBisectors)
        {
            V3s BIS1, BIS2;
            V3s FF1 = FF[0];
            V3s FF2 = FF[1];
            utils::mesh::compute_frame_field_bisectors(mesh.F_X, mesh.F_Y, FF1, FF2, BIS1, BIS2);
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                V3 bi0 = BIS1.row(fi);
                V3 bi1 = BIS2.row(fi);
                P3 c = C.row(fi);


                draw.AddEdge(c, c + bi0 * (len / 3), c0);
                draw.AddEdge(c, c + bi1 * (len / 3), c1);
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::Weights)
        {
            //cout << "MeshLogicOptions_Draw_Solver::DirectionsStyle::Weights   N = " << N << endl;
            if (N == 4 || N == 2)
            {
                for (int fi = 0; fi < FF[0].rows(); ++fi)
                {
                    V3 dir0 = FF[0].row(fi);
                    V3 dir1 = FF[1].row(fi);
                    D weight1 = dir0.norm();
                    D weight2 = dir1.norm();
                    D weight = (weight1 + weight2) / 2;
                    draw.AddLabel(mesh.F_Barycenters.row(fi).transpose(), to_string(weight), Color4d(1, 0, 0, 1));
                }
            }
        }
        else if (options.Solver.ShowDirectionsStyle == MeshLogicOptions_Draw_Solver::DirectionsStyle::SolverQuuQuk)
        {
            vector<int> constrainedfids;
            for (int i = 0; i < solver.Constrains.Constrains.size(); i++)
            {
                constrainedfids.push_back(solver.Constrains.Constrains[i].FaceId);
            }
            int uu = 0;
            int uk = 0;
            for (int fid = 0; fid < mesh.FacesCount; ++fid)
            {
                P3 c = mesh.F_Barycenters.row(fid);
                if (utils::stdvector::exists(constrainedfids, fid))
                {
                    draw.AddLabel(c, "     " + to_string(uu), Color4d(1, 0, 0, 1), 3);
                    uu++;
                }
                else
                {
                    draw.AddLabel(c, "     " + to_string(uk), Color4d(0, 1, 0, 1), 3);
                    uk++;
                }
            }
        }
        else
        {
            for (int fi = 0; fi < FF[0].rows(); ++fi)
            {
                if (solver.Result.Field.size() ==2 && solver.Result.IsFieldSymetric)
                {
                    for (int ni = 0; ni < 2; ++ni)
                    {
                        V3 fieldDirection = FF[ni].row(fi);
                        P3 c = C.row(fi);
                        draw.AddEdge(c - fieldDirection * len, c + fieldDirection * len, c0);
                    }
                }
                else
                {
                    for (int ni = 0; ni < FF.size(); ++ni)
                    {
                        V3 fieldDirection = FF[ni].row(fi);
                        P3 c = C.row(fi);

                        draw.AddEdge(c, c + fieldDirection * len, c0);
                    }
                }
            }
        }
    }

    if (options.Solver.ShowDirectionsWeights && solver.Result.Field.size() > 0)
    {
        const vector<V3s>& YYY = solver.Result.Field;
        VectorXf W(YYY[0].rows());
        int NLocal = N;
        if (NLocal == 4) NLocal = 2;

        for (int i = 0; i < YYY[0].rows(); ++i)
        {
            DD weight = 0;
            for (int ni = 0; ni < NLocal; ++ni)
            {
                V3 fieldDirection = YYY[ni].row(i);
                weight += fieldDirection.norm();
            }
            W(i) = static_cast<float>(weight / NLocal);
            //draw.AddLabel(mesh.F_Barycenters.row(i), to_string(W(i)));
            //draw.AddLabel(mesh.F_Barycenters.row(i), to_string(YYY[0].row(i).norm())+" " + to_string(YYY[1].row(i).norm()));
        }
        float minweight = W.minCoeff();
        float maxweight = W.maxCoeff();
        float avgweight = W.size() == 0 ? 0 : W.sum() / W.size();
        //cout << "DirectionsWeights:  avg " << avgweight << "   min " << minweight << "   max " << maxweight << endl;
        //MatrixXf C;
        //igl::jet(W, true, draw.FaceColors);
        draw.meshColor.Scheme = MeshColorSceme::PerFace;
        igl::parula(W, false, draw.meshColor.Colors);
    }

    // Plot the singularities as colored dots (red for negative, blue for positive)
    if (options.Solver.ShowSingularities)
    {
        for (int i = 0; i < solver.Result.Singularities.size(); ++i)
        {
            if (solver.Result.Singularities(i) < -0.001)
            {
                draw.AddPoint(mesh.V.row(i), Color3d(1, 0, 0));
                //draw.AddLabel(V.row(i), "  " +to_string(S(i)));
                //draw.AddLabel(mesh.V.row(i), "  3");
            }
            else if (solver.Result.Singularities(i) > 0.001)
            {
                draw.AddPoint(mesh.V.row(i), Color3d(0, 0, 1));
                //draw.AddLabel(V.row(i), "  " + to_string(S(i)));
                //draw.AddLabel(mesh.V.row(i), "  5");
            }
        }
    }

    // Highlight constrained faces
    if (options.Cons.HighlightConstrainedFaces)
    {
        draw.meshColor.Scheme = MeshColorSceme::PerFace;
        draw.meshColor.Colors = MatrixXf::Constant(mesh.F.rows(), 3, 0.6f);//by default make all faces of silver color
        for (auto& c : solver.Constrains.Constrains)
        {
            draw.meshColor.Colors.row(c.FaceId)  = Color3f(1, 0, 0); // for boundary faces - use red color
        }
    }

    // Highlight constrains
    if (options.Cons.ShowBorderConstrains)
    {
        draw.ReserveEdges(solver.Constrains.Constrains.size());
        for (auto& c : solver.Constrains.Constrains)
        {
            P3 edgeMiddlePoint = mesh.EdgeMiddlePoint(c.EdgeId);
            draw.AddEdge(edgeMiddlePoint, edgeMiddlePoint + c.DirectionX * (mesh.avg_edge_length / 4), Color3d(1, 0, 0));
            draw.AddEdge(edgeMiddlePoint, edgeMiddlePoint + c.DirectionY_Corrected * (mesh.avg_edge_length / 4), Color3d(1, 0, 0));
        }
    }

    // Higlight sharp edges
    if (options.Cons.ShowEdgesSharpness)
    {
        D sharpAngle_COS = utils::angle::DegreesToCos(10);
        draw.ReserveLabels(mesh.EF.rows());
        for (int eid = 0; eid < mesh.EF.rows(); eid++)
        {
            if (mesh.E_isborder[eid]) continue;
            int fid0 = mesh.EF(eid, 0);
            int fid1 = mesh.EF(eid, 1);
            V3 normal0 = mesh.F_normals.row(fid0);
            V3 normal1 = mesh.F_normals.row(fid1);
            D angleCOS = utils::vector::Cos(normal0, normal1, true);
            if (angleCOS < sharpAngle_COS)
            {
                D angle = utils::vector::Angle(normal0, normal1, true);
                Color4d color = (angle < 30)
                    ? Color4d(0, 0, 0.6, 0.3)
                    : ((angle < 50)
                        ? Color4d(0.4, 0, 0.2, 0.4)
                        : Color4d(0.6, 0, 0, 0.6));
                draw.AddLabel(mesh.EdgeMiddlePoint(eid), "" + to_string(static_cast<int>(angle)), color);
            }
        }
    }

    // Highlight fixed p
    //if (options.nRosy_HighlightFixedP && solver.nrosy.IsInited)
    //{
    //    // prepare for batch edge adding
    //    int edgesCountWillBeAdded = 0;
    //    for (int eid = 0; eid < mesh.EF.rows(); ++eid)
    //        if (solver.nrosy.pFixed[eid])
    //            edgesCountWillBeAdded++;
    //    draw.ReserveEdges(edgesCountWillBeAdded);

    //    for (int eid = 0; eid < mesh.EF.rows(); ++eid)
    //    {
    //        if (solver.nrosy.pFixed[eid])
    //        {
    //            int v1index = mesh.EV(eid, 0);
    //            int v2index = mesh.EV(eid, 1);
    //            P3 v1 = V.row(v1index);
    //            P3 v2 = V.row(v2index);
    //            draw.AddEdge(v1, v2, Color3d(0, 0, 1));
    //        }
    //    }
    //}


    if (options.Solver.ShowBorderConstrainesIsoLines && options.Solver.ShowBorderConstrainesIsoLinesCount > 0)
    {
        //DEBUG test performance
        /*for (int i = 0; i < 100; i++)
        {
            Draw_BorderConstrainesIsoLines(draw, solver, mesh, options.nRosy_ShowBorderConstrainesIsoLinesCount, false);
        }*/
        Draw_BorderConstrainesIsoLines(solver, options.Solver.ShowBorderConstrainesIsoLinesCount, options.Solver.ShowBorderConstrainesIsoLinesCount_intensivity, true, logMessages);
    }
}

void MeshLogicDraw::draw_mesh_UV(const MeshSolverUV& solverUV, bool logMessages)
{
    if (meshLogicOptions.SolverUV.DebugEnabled && meshLogicOptions.SolverUV.debug_show_uv )
    {
        draw.ReserveLabels(solverUV.Result.Field.size());
        for (int vid = 0; vid < solverUV.Result.Field.size(); vid++)
        {
            const UV& uv = solverUV.Result.Field[vid];
            if (!meshLogicOptions.SolverUV.debug_show_uv__on_borders && mesh.V_isborder(vid)) continue;
            P3 p = solverUV.mesh.V.row(vid);
            draw.AddLabel(p, uv.uv_toString(), Color3d(0,0,0.7));
        }
    }

    if (meshLogicOptions.SolverUV.DebugEnabled && meshLogicOptions.SolverUV.debug_showBorderConstrainesIsoLines && meshLogicOptions.SolverUV.debug_showBorderConstrainesIsoLines__count > 0)
    {
        //DEBUG test performance
        /*for (int i = 0; i < 100; i++)
        {
            Draw_BorderConstrainesIsoLinesUV(solverUV, meshLogicOptions.SolverUV.debug_showBorderConstrainesIsoLines__count, false);
        }*/
        Draw_BorderConstrainesIsoLinesUV(solverUV, meshLogicOptions.SolverUV.debug_showBorderConstrainesIsoLines__count, meshLogicOptions.SolverUV.debug_showBorderConstrainesIsoLines__intensivity, true, logMessages);
    }
}

void MeshLogicDraw::DrawId_Vertex()
{
    draw.ReserveLabels(mesh.V.rows());
    for (int i = 0; i < mesh.V.rows(); ++i)
    {
        draw.AddLabel(mesh.V.row(i), "   " + to_string(i), options.Mesh.Id_Vertex_Color);
    }
}
void MeshLogicDraw::DrawId_Edge()
{
    draw.ReserveLabels(mesh.EV.rows());
    for (int i = 0; i < mesh.EV.rows(); ++i)
    {
        P3 midPoint = (mesh.V.row(mesh.EV(i, 0)) + mesh.V.row(mesh.EV(i, 1))) / 2;
        draw.AddLabel(midPoint, "   " + to_string(i), options.Mesh.Id_Edge_Color);
    }
}
void MeshLogicDraw::DrawId_Face()
{
    draw.ReserveLabels(mesh.F.rows());
    for (int i = 0; i < mesh.F.rows(); ++i)
    {
        P3 midPoint = mesh.F_Barycenters.row(i);
        draw.AddLabel(midPoint, to_string(i), options.Mesh.Id_Face_Color);
    }
}
void MeshLogicDraw::Draw_Object_Id(int id)
{
    P3 midPoint = mesh.V_Info.VcenterProjectedIndside;
    draw.AddLabel(midPoint, to_string(id), options.Mesh.Obj_Id_Color, 8);
}
void MeshLogicDraw::Draw_Object_Name(string name)
{
    P3 midPoint = mesh.V_Info.VcenterProjectedIndside;
    draw.AddLabel(midPoint, name, options.Mesh.Obj_Name_Color, 8);
}
void MeshLogicDraw::DrawId_Mesh()
{
    int meshId = mesh.id;
    P3 midPoint = mesh.V_Info.VcenterProjectedIndside;
    //P3 midPoint = mesh.V_Info.Vcenter;
    draw.AddLabel(midPoint, to_string(meshId), options.Mesh.Id_Mesh_Color, 8);
}
void MeshLogicDraw::DrawId_MeshLoop()
{
    for (const auto& loop : mesh.Loops)
    {
        P3 midPoint = loop.Centroid;
        draw.AddLabel(midPoint, to_string(loop.id), options.Mesh.Id_Mesh_Color, 5);
    }
}

void MeshLogicDraw::DrawNormals(bool faces, bool vertixes, bool faces_calculated)
{
    const P3s& V = mesh.V;
    const auto& F = mesh.F;
    const P3s& C = mesh.F_Barycenters;
    const V3s& V_normals = mesh.V_normals;
    const V3s& F_normals = mesh.F_normals;
    D len = mesh.avg_edge_length;

    if (faces)
    {
        draw.ReserveEdges(F.rows());
        for (int i = 0; i < F.rows(); ++i)
        {
            draw.AddEdge(C.row(i), C.row(i) + F_normals.row(i)*len, Color3d(0, 1, 0));
        }
    }

    if (vertixes)
    {
        draw.ReserveEdges(V.rows());
        for (int i = 0; i < V.rows(); ++i)
        {
            draw.AddEdge(V.row(i), V.row(i) + V_normals.row(i)*len, Color3d(1, 0, 0));
        }
    }


    if (faces_calculated)
    {
        V3s F_X;
        V3s F_Y;
        V3s F_normals_calculated;
        utils::mesh::local_basis(V, F, F_X, F_Y, F_normals_calculated);
        draw.ReserveEdges(F.rows());
        for (int i = 0; i < F.rows(); ++i)
        {
            draw.AddEdge(C.row(i), C.row(i) + F_normals_calculated.row(i)*len, Color3d(0, 0, 1));
        }
    }
}

void MeshLogicDraw::DrawBorders()
{
    // prepare for batch edge adding
    int adding = 0;
    for (const MeshLoop& loop : mesh.Loops)
        adding += loop.edges.size();

    draw.ReserveEdges(adding);
    for (const MeshLoop& loop : mesh.Loops)
    {
        Color3d color = loop.Type == MeshLoopType::Inner ? options.Mesh.TopologyConnection_Color_Inner : options.Mesh.TopologyConnection_Color_Outher;
        for (const MeshLoopEdge& edge : loop.edges)
        {
            draw.AddEdge(mesh.V.row(edge.StartVertexId), mesh.V.row(edge.EndVertexId), color);
        }
    }
}


void MeshLogicDraw::DrawBorderIndexes()
{
    // show label for each vertex
    int adding = 0;
    for (const MeshLoop& loop : mesh.Loops)
        adding += loop.edges.size();
    draw.ReserveLabels(adding);

    for (const MeshLoop& loop : mesh.Loops)
    {
        int edgenum = 0;
        for (const MeshLoopEdge& edge : loop.edges)
        {
            draw.AddLabel((mesh.V.row(edge.StartVertexId) + mesh.V.row(edge.EndVertexId)) / 2, to_string(edgenum++)); //  show numbers on each vertect from loop
        }
    }
}


void MeshLogicDraw::DrawBorderLabels()
{
    // prepare for batch edge adding
    int adding = 0;
    for (const MeshLoop& loop : mesh.Loops)
        adding += loop.edges.size();

    draw.ReserveLabels(adding);
    for (const MeshLoop& loop : mesh.Loops)
    {
        Color3d color = loop.Type == MeshLoopType::Inner ? options.Mesh.TopologyConnection_Color_Inner : options.Mesh.TopologyConnection_Color_Outher;
        for (const MeshLoopEdge& edge : loop.edges)
        {
            P3 p = (mesh.V.row(edge.StartVertexId) + mesh.V.row(edge.EndVertexId)) / 2;
            draw.AddLabel(p, to_string(edge.EdgeId), color);
        }
    }
}


void MeshLogicDraw::DrawBorderAngles()
{
    //options.Draw_Points_Color
    int adding = 0;
    for (const MeshLoop& loop : mesh.Loops)
        adding += loop.edges.size();

    draw.ReserveLabels(adding * 2);
    for (const MeshLoop& loop : mesh.Loops)
    {
        for (const MeshLoopPoint& p : loop.points)
        {
            P3 pointOut = mesh.VertexToPointOutOfFaces(p.VertexId, 1);
            string  angleStr = (p.AngleBetweenEdges < 0.01 || 1 < p.AngleBetweenEdges)
                ? to_string(static_cast<int>(round(p.AngleBetweenEdges)))
                : to_string(p.AngleBetweenEdges);
            draw.AddLabel(pointOut, angleStr, Vector4d(0.6, 0.6, 0.6, 0.4));
        }
        for (const MeshLoopPoint& p : loop.points)
        {
            P3 pointOut = mesh.VertexToPointOutOfFaces(p.VertexId, 1.5);
            draw.AddLabel(pointOut, to_string(static_cast<int>(round(p.AngleBetweenEdgesFull))), Vector4d(0.2, 0.2, 0.2, 1));
        }
    }

    //DEBUG - MeshLoop::GetSharpPoints_Indexes - code is here since add_label will be not visible otherwise
    //vector<MeshLoop> Loops;
    //MeshLoop::GetLoops(mesh, Loops);
}

void MeshLogicDraw::DrawHighlights()
{
    auto color = Color3d(0, 1, 1);
    auto fontIncrease = 7;
    if (utils::strings::IsInt(options.Mesh.Highlight.id))
    {
        for (int id : utils::strings::extractInts(utils::strings::Trim(options.Mesh.Highlight.id)))
        {
            switch (options.Mesh.Highlight.What)
            {
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::Vertex:
                {
                    int vid = id;
                    int maxvid = mesh.V.rows();
                    if (vid < 0 || vid >= maxvid)
                    {
                        //cout << "mesh highlighting:  invalid vid  - max vid=" << maxvid << endl;
                        continue; 
                    }
                    P3 v = mesh.V.row(vid);
                    draw.AddPoint(v, color);
                    string text = "  vid=" + to_string(vid);
                    draw.AddLabel(v, text, color, fontIncrease);
                }
                break;
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::Edge:
                {
                    int eid = id;
                    int maxeid = mesh.EV.rows();
                    if (eid < 0 || eid >= maxeid)
                    {
                        //cout << "mesh highlighting:  invalid eid  - max eid=" << maxeid << endl;
                        continue;
                    }

                    int vid0 = mesh.EV(eid, 0);
                    int vid1 = mesh.EV(eid, 1);
                    int fid0 = mesh.EF(eid, 0);
                    int fid1 = mesh.EF(eid, 1);
                    P3 v0 = mesh.V.row(vid0);
                    P3 v1 = mesh.V.row(vid1);
                    draw.AddEdgeBold(v0, v1, color);
                    string text = "  eid=" + to_string(eid);
                    string text_fids = "  fids={"+ to_string(fid0)+","+to_string(fid1)+"}";
                    string text_vids = "  vids={" + to_string(vid0) + "," + to_string(vid1) + "}";
                    draw.AddLabel((v0 + v1) / 2, text + "  " + text_fids + "  " + text_vids, color, fontIncrease);
                }
                break;
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::Face:
                {
                    int fid = id;
                    int maxfid = mesh.F.rows();
                    if (fid < 0 || fid >= maxfid)
                    {
                        //cout << "mesh highlighting:  invalid fid  - max fid=" << maxfid << endl;
                        continue;
                    }
                    string text = "  fid=" + to_string(fid);
                    string text_eids = "  eids={";
                    string text_vids = "  vids={";
                    for (int k = 0; k < 3; k++)
                    {
                        int eid = mesh.FE(fid, k);
                        int vid0 = mesh.EV(eid, 0);
                        int vid1 = mesh.EV(eid, 1);
                        P3 v0 = mesh.V.row(vid0);
                        P3 v1 = mesh.V.row(vid1);
                        draw.AddEdgeBold(v0, v1, color);
                        if (k != 0)text_eids += ",";
                        text_eids += to_string(eid);
                        if (k != 0)text_vids += ",";
                        text_vids += to_string(mesh.F(fid, k));
                    }
                    text_eids += "}";
                    text_vids += "}";
                    P3 midPoint = mesh.F_Barycenters.row(fid);
                    draw.AddLabel(midPoint, text+"  "+ text_eids+"  "+ text_vids, color, fontIncrease);
                }
                break;
                case MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::Mesh:
                {
                    int meshid = id;
                    if (mesh.id != meshid)
                    {
                        return;
                    }
                    for (int eid = 0; eid < mesh.EF.rows(); eid++)
                    {
                        int vid0 = mesh.EV(eid, 0);
                        int vid1 = mesh.EV(eid, 1);
                        P3 v0 = mesh.V.row(vid0);
                        P3 v1 = mesh.V.row(vid1);
                        draw.AddEdgeBold(v0, v1, color);
                    }
                    P3 midPoint = mesh.V_Info.VcenterProjectedIndside;
                    draw.AddLabel(midPoint, "  meshid=" + to_string(meshid), color, fontIncrease);
                }
                break;
                default:
                    //other values handled in Topology class
                    break;
            }
        }
    }

    if (options.Mesh.Highlight.What == MeshLogicOptions_Draw_Mesh::HighligtWhat::WhatType::ObjName)
    {
        vector<string> onlyObjectNames;
        utils::strings::split(options.Mesh.Highlight.id, onlyObjectNames, ";, ");
        for (string objName : onlyObjectNames)
        {
            if (mesh.Name == objName)
            {
                for (int eid = 0; eid < mesh.EF.rows(); eid++)
                {
                    int vid0 = mesh.EV(eid, 0);
                    int vid1 = mesh.EV(eid, 1);
                    P3 v0 = mesh.V.row(vid0);
                    P3 v1 = mesh.V.row(vid1);
                    draw.AddEdgeBold(v0, v1, color);
                }
                P3 midPoint = mesh.V_Info.VcenterProjectedIndside;
                draw.AddLabel(midPoint, "  mesh=" + objName, color, fontIncrease);
            }
        }
    }
}

    void MeshLogicDraw::DrawBorderSharpness()
    {
        for (const MeshLoop& loop : mesh.Loops)
        {
            // show sharp vertexes
            vector<int> vertexes;
            vector<D> angleChange;
            vector<int> pointIndexes;
            loop.GetSharpPoints(vertexes, angleChange, pointIndexes);
            for (int i = 0; i < vertexes.size(); i++)
            {
                P3 p = mesh.V.row(vertexes[i]);
                auto angle = angleChange[i];

                // show angle change for this sharp point
                if (options.Mesh.BorderSharpnessDotsAngle)
                {
                    P3 pointOut = mesh.VertexToPointOutOfFaces(vertexes[i], -1);
                    draw.AddLabel(pointOut, to_string(static_cast<int>(round(angle))), Vector4d(0, 0, 1, 1));
                }

                // show text cordinate
                //draw.AddLabel(p, to_string(c));

                // add label - how many devides should goes from this point
                int dividesCount = static_cast<int>((angle - 40) / 90);
                if (dividesCount > 0)
                {
                    draw.AddLabel(p, "  " + to_string(dividesCount));
                    draw.AddPoint(p, Color3d(0, 0.7, 0));
                }
                else
                {
                    draw.AddPoint(p, options.Mesh.Points_Color);
                }
            }
        }
    }

