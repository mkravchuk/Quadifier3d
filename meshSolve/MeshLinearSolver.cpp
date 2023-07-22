#include "stdafx.h"
#include "FaceConstrain.h"
#include "MeshLinearSolver.h"

const MeshLogicOptions_MeshLinearSolver& options = meshLogicOptions.MeshLinearSolver;
Elapsed_Solver& elapsed2 = elapsedTimers.Solver;

MeshLinearSolver_FaceData::MeshLinearSolver_FaceData(int _fid)
    : fid(_fid)
{
    pF = nullptr;
    F[0] = -1;
    F[1] = -1;
    F[2] = -1;
    F_isEliminated[0] = false;
    F_isEliminated[1] = false;
    F_isEliminated[2] = false;
    F_needsToUpdate[0] = false;
    F_needsToUpdate[1] = false;
    F_needsToUpdate[2] = false;
    needsToUpdate = false;
    depentOnCount = 0;
    dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList = -1;
    R[0] = complex<double>(0, 0);
    R[1] = complex<double>(0, 0);
    R[2] = complex<double>(0, 0);
    K = complex<double>(0, 0);
    C = complex<double>(0, 0);
    state = State::Unknown;
    Solution = complex<double>(0, 0);
}

void MeshLinearSolver_FaceData::MarkFriendFaceForUpdate(int fidFriend)
{
    needsToUpdate = true;
    bool found = false; // just for debug assert
    for (int k = 0; k < 3; k++)
    {
        if (F[k] == fidFriend)
        {
            F_needsToUpdate[k] = true;
            found = true;
            break;
        }
    }
    //assert(found); - not correct on backward stage - some faces eliminate others, and F can changed
}

int MeshLinearSolver_FaceData::EliminateDependicies()
{
    assert(state != State::Solved);
    int depentOnCount_before = depentOnCount;
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;
        if (!F_needsToUpdate[k]) continue;
        F_needsToUpdate[k] = false;  //clear flag

        MeshLinearSolver_FaceData& fFriend = pF[F[k]];

        // substitude solved
        if (fFriend.state == State::Solved)
        {
            K -= R[k] * fFriend.Solution;
            F_isEliminated[k] = true;
            depentOnCount--;
            continue;
        }

        // substitude single coo-dependency
        int fk;
        if (fFriend.depentOnCount == 1 && fFriend.GetSingleDependence(fk) == fid)
        {
            C -= (R[k] * fFriend.R[fk]) / fFriend.C;
            K -= (R[k] * fFriend.K) / fFriend.C;
            F_isEliminated[k] = true;
            depentOnCount--;
            //fFriend.state = State::Eliminated; - we deed it on 'Solve' method, and here we dont have to do this
            continue;
        }

        if (fFriend.depentOnCount == 2)
        {
            int fka = fFriend.GetDependenceNum(fid);
            int fkb = fFriend.GetSecondDependenceNum(fid);

            C -= (R[k] * fFriend.R[fka]) / fFriend.C; //TODO precalculate 'a.R[fka]/C' and use multiply instead of division
            K -= (R[k] * fFriend.K) / fFriend.C;
            R[k] = -(R[k] * fFriend.R[fkb]) / fFriend.C;
            F[k] = fFriend.F[fkb]; // change relation to point another face
        }
    }

    // merge duplicate dependencies
    for (int k = 0; k < 3; k++) // check each edge
    {
        int k2 = (k + 1) % 3;
        if (F[k] == F[k2]) // if we have 2 dependecies to 1 face - then we can merge them into one
        {
            int k1 = k;
            if (k1==2) swap(k1, k2);// this is important, since we assume in other algorithms that only second same fid is eliminated
            if (F_isEliminated[k2]) continue;
            F_isEliminated[k2] = true; // remove this dependency in favor of  'k' dependency
            R[k1] += R[k2];// merge this dependency into 'k'
            depentOnCount--; // count of dependencies decrease by one since we have meged 2 dependencies into 1
            break;
        }
    }

    // mark solved if there is no more dependencies
    if (depentOnCount == 0)
    {
        state = State::Solved;
        Solution = K / C;
    }
    return depentOnCount_before - depentOnCount;
}

bool MeshLinearSolver_FaceData::CanBeEliminated_DoubleDependency()
{
    int testedFriendsCount = 0; //  just for debug assertion
    assert(depentOnCount == 2);
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;// here we exlude as well all waiting faces
        MeshLinearSolver_FaceData& fFriend = pF[F[k]];
        //if (fFriend.state == State::Waiting) continue; - dont needed since we exluded this in condition above 'if (F_isEliminated[k]) continue;'
        testedFriendsCount++;
        if (fFriend.depentOnCount == 1)return false;// give a priority to resolving single dependencies
        if (fFriend.depentOnCount == 2 && fFriend.fid < fid) return false; // give a priority to resolving faces with lower 'fid' value - in this way we remove conflict of 'changing f to fFrriend and changing fFrriend to f'
    }
    assert(testedFriendsCount == 2);
    return true;
}

int MeshLinearSolver_FaceData::EliminateDoubleDependency()
{
    int resolvedDependenciesCount = 0;
    if (depentOnCount < 2) return 0; // can happend when 2 faces are merged, so we get some face with 1 dependency (last steps of solving iterations)

    //validate
    //assert(depentOnCount == 2);
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;
        MeshLinearSolver_FaceData& f = pF[F[k]];
        if (f.state == State::Eliminated) continue;
        assert(f.depentOnCount >= 2);
        if (f.depentOnCount == 2 && f.fid < fid) return 0;//f.depentOnCount can by only 2 or 3
    }

    // resolve dependency
    int kaFound = -1;
    int kbFound = -1;
    for (int ka = 0; ka < 3; ka++)
    {
        if (F_isEliminated[ka]) continue;
        MeshLinearSolver_FaceData& a = pF[F[ka]];
        if (a.state == State::Eliminated) continue;
        int fka = a.GetDependenceNum(fid);

        int kb = 0;
        for (; kb < 3; kb++)
        {
            if (ka == kb) continue;
            if (F_isEliminated[kb]) continue;
            if (pF[F[kb]].state == State::Eliminated) continue;
            break;
        }
        MeshLinearSolver_FaceData& b = pF[F[kb]];
        assert(a.fid != b.fid);
        //int fkb = b.GetDependenceNum(fid);

        kaFound = ka; // we can do it twice - since a and b is cyclic - no matter what is sequence 'a''b' or 'b''a'
        kbFound = kb;

        a.C -= (R[ka] * a.R[fka]) / C; //TODO precalculate 'a.R[fka]/C' and use multiply instead of division
        a.K -= (K* a.R[fka]) / C;
        a.F[fka] = b.fid; // change relation to point another face
        a.R[fka] = -(R[kb] * a.R[fka]) / C;
    }
    assert(kaFound != -1);
    assert(kbFound != -1);
    resolvedDependenciesCount = 1;

    // merge 2 dependecies to 1 face
    for (int abk : {kaFound, kbFound}) //  for faces 'a' and 'b'
    {
        for (int k = 0; k < 3; k++) // check each edge
        {
            MeshLinearSolver_FaceData& ab = pF[F[abk]];
            if (ab.F_isEliminated[k]) continue;
            if (ab.F_isEliminated[(k + 1) % 3]) continue;
            if (ab.F[k] == ab.F[(k + 1) % 3]) // if we have 2 dependecies to 1 face - then we can merge them into one
            {
                ab.F_isEliminated[(k + 1) % 3] = true; // remove this dependency in favor of  'k' dependency
                ab.R[k] += ab.R[(k + 1) % 3];// merge this dependency into 'k'
                ab.depentOnCount--; // count of dependencies decrease by one since we have meged 2 dependencies into 1
                //resolvedDependenciesCount++;
                break;
            }
        }
    }

    state = State::Eliminated;
    return resolvedDependenciesCount;
}


int MeshLinearSolver_FaceData::GetDependenceNum(int fid) const
{
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;
        if (F[k] == fid) return k;
    }
    assert(false);
    return -1;
}

int MeshLinearSolver_FaceData::GetSecondDependenceNum(int fid) const
{
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;
        if (F[k] != fid) return k;
    }
    assert(false);
    return -1;
}

int MeshLinearSolver_FaceData::GetSingleDependence(int& fk) const
{
    assert(state != State::Solved);

    int depentOnCount = 0; // just for debug assertion
    for (int k = 0; k < 3; k++)
    {
        if (F_isEliminated[k]) continue;
        depentOnCount++;
        fk = k;
    }
    assert(depentOnCount == 1);
    return F[fk];
}


//int MeshLinearSolver_FaceData::DependencesCount() const
//{
//    if (state == State::Solved) return 0;
//
//    int depentOnCount = 0;
//    for (int k = 0; k < 3; k++)
//    {
//        if (F_isEliminated[k]) continue;
//        depentOnCount++;
//    }
//    return depentOnCount;
//}

string MeshLinearSolver_FaceData::stateStr()
{
    switch (state)
    {
        case State::Eliminated:
            return "waiting";
        case State::Solved:
            return "solved";
        default:
            return "default";
    }
}

MeshLinearSolver::MeshLinearSolver(ViewerDrawObjects& _draw,
    const MatrixXd &_V,
    const MatrixXi &_F,
    const VectorXd &_F_Areas,
    const MatrixXd& _F_Barycenters,
    const MatrixXi& _EV,
    const MatrixXi& _FE,
    const MatrixXi& _EF,
    const VectorXd& _K,
    const MatrixXd& _F_X,
    const MatrixXd& _F_Y,
    const MatrixXd& _F_Z,
    const MatrixXi& _TT,
    const MatrixXi& _TTi,
    double _avg_edge_length,
    double _max_edge_length)
    : draw(_draw), V(_V), F(_F), F_Areas(_F_Areas), F_Barycenters(_F_Barycenters), EV(_EV), FE(_FE), EF(_EF), K(_K), F_X(_F_X), F_Y(_F_Y), F_Z(_F_Z), TT(_TT), TTi(_TTi),
    avg_edge_length(_avg_edge_length), max_edge_length(_max_edge_length)
{
    PopulateEQ();
}

void MeshLinearSolver::PopulateEQ()
{
    EQ.reserve(F.rows());

    // init empty structures
    for (int fid = 0; fid < F.rows(); fid++)
    {
        EQ.push_back(MeshLinearSolver_FaceData(fid));
    }

    for (int fid = 0; fid < F.rows(); fid++)
    {
        EQ[fid].pF = EQ.data();
    }

    // set relation between faces (property 'F')
    for (int fid = 0; fid < F.rows(); fid++)
    {
        for (int k = 0; k < 3; k++)
        {
            int fid_friend = TT(fid, k);
            if (fid_friend == -1)
            {
                EQ[fid].F_isEliminated[k] = true;
                continue;
            }
            EQ[fid].F[k] = fid_friend;
            EQ[fid].depentOnCount++;
        }
    }

    // set rotation coefficient between faces (property 'R')
    for (int eid = 0; eid < EF.rows(); eid++)
    {
        int fid0 = EF(eid, 0);
        int fid1 = EF(eid, 1);

        // if edge is border, then we dont have dependency - just mark it as unavailable
        if (fid1 == -1)
        {
            for (int k = 0; k < 3; k++)
            {
                if (FE(fid0, k) == eid)
                {
                    EQ[fid0].F_isEliminated[k] = true; // remove dependency on non existed face
                    break;
                }
            }
            continue;
        }

        complex<double> k10 = -std::polar(1., 1. * 4 * K[eid]);// origin
        complex<double> k01 = complex<double>(k10.real(), -k10.imag());
        for (int k = 0; k < 3; k++)
        {
            if (FE(fid0, k) == eid)
            {
                EQ[fid0].R[k] = k01;
                EQ[fid0].C += 1.0;
                break;
            }
        }
        for (int k = 0; k < 3; k++)
        {
            if (FE(fid1, k) == eid)
            {
                EQ[fid1].R[k] = k10;
                EQ[fid1].C += 1.0;
                break;
            }
        }
    }
}


void MeshLinearSolver::SetConstrains(
    const vector<FaceConstrain>& Constrains,  // Constrained faces representative vector
    double softPercent, // ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)        
    bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected)
{
    Timer timer;

    // Set weights base on edge length and triangle weight
    MatrixXd edgeWeights = MatrixXd::Constant(Constrains.size(), 2, 1);
    if (WeightIsRelativeToEdgeLength)
    {
        double maxedgeLength = 0;
        double maxfaceArea = 0;
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            Vector3d edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            double edgeLength = utils::vector::Length(edgeDirection);
            double faceArea = F_Areas(fid);
            maxedgeLength = max(maxedgeLength, edgeLength);
            maxfaceArea = max(maxfaceArea, faceArea);
        }
        for (int i = 0; i < Constrains.size(); ++i)
        {
            int fid = Constrains[i].FaceId;
            int eid = FE(fid, 0);
            Vector3d edgeDirection = V.row(EV(eid, 1)) - V.row(EV(eid, 0));
            double edgeLength = utils::vector::Length(edgeDirection);
            double faceArea = F_Areas(fid);
            //double edgeDirectionWeight = edgeLength / maxedgeLength;
            //double directionWeight = faceArea / maxfaceArea;
            //double weight = faceArea / maxfaceArea;// works bad
            double weight = edgeLength / maxedgeLength;// works ok
            double edgeDirectionWeight = weight;
            double directionWeight = weight;
            edgeWeights.row(i) << edgeDirectionWeight, directionWeight;
        }
    }

    // Set solver contrains base on face constrains and edge directions
    isFaceConstrained = VectorXb::Constant(F.rows(), false);
    faceConstrains = MatrixXd::Constant(F.rows(), 3 * 2, 0);

    for (int i = 0; i < Constrains.size(); ++i)
    {
        int fid = Constrains[i].FaceId;

        //v1
        //int eid = Constrains[i].EdgeId;
        //Point3d v0 = V.row(EV(eid, 0));
        //Point3d v1 = V.row(EV(eid, 1));
        //Vector3d edgeDirection = v1 - v0;
        Vector3d edgeDirection = Constrains[i].DirectionX;
        edgeDirection.normalize();

        //v2 - faster but requieres fix in mesh and dont work with solid meshes without borders
        //RowVector3d edgeDirection = F_X.row(fid);

        Vector3d direction = takeDirectionCorrected
            ? Constrains[i].DirectionY_Corrected
            : Constrains[i].DirectionY;
        direction.normalize();

        //DEBUG - check consistance of X directions for borders-constrains - they should all follow each others
        //Vector3d x = F_X.row(fid);
        //double dot = edgeDirection.dot(x);
        //if (dot < 0)
        //{
        //    cout << "wrong!!!   edgeDirection.dot(x) < 0  for fid=" << fid << endl;
        //}

        //if (eid == 135)
        //{
        //    Point3d edgemiddle = (v0 + v1) / 2;
        //    draw.AddPoint(v0, Color3d(0, 0, 0), "v0");
        //    draw.AddPoint(v1, Color3d(0,0,0), "v1");
        //    draw.AddPoint(edgemiddle);
        //    draw.AddEdge(edgemiddle, edgemiddle + edgeDirection*avg_edge_length, Color3d(0,1,0), "edgeDirection");
        //    //draw.AddEdge(edgemiddle, edgemiddle + direction*avg_edge_length, Color3d(0, 0, 1), "direction");
        //    e.directionToFace = mesh.EdgeNormalToFace(eid, fid);
        //}

        if (WeightIsRelativeToEdgeLength)
        {
            edgeDirection *= edgeWeights(i, 0);
            direction *= edgeWeights(i, 1);
        }
        double m = softPercent;
        // i dont know why results are wrong when softPercent ==1 - in this case we have to avoid multiply opperaion - why i dont know
        if (abs(1 - m) > 0.001)
        {
            direction *= m;
        }
        #if alternative_solvers_SUPPORTED
        if (meshLogicOptions.Solver.SolveForVectorsXY == MeshLogicOptions_Solver::SolveForVectorsXYEnum::xx) direction = edgeDirection;
        if (meshLogicOptions.Solver.SolveForVectorsXY == MeshLogicOptions_Solver::SolveForVectorsXYEnum::yy) edgeDirection = direction;
        #endif
        //faceConstrains.row(fid) << edgeDirection, direction;
        faceConstrains(fid, 0) = edgeDirection(0);
        faceConstrains(fid, 1) = edgeDirection(1);
        faceConstrains(fid, 2) = edgeDirection(2);
        faceConstrains(fid, 3) = direction(0);
        faceConstrains(fid, 4) = direction(1);
        faceConstrains(fid, 5) = direction(2);
        isFaceConstrained(fid) = true;
    }

    timer.stop(elapsed2.initSolver);
}

__forceinline complex<double> getComplex(int index, const MatrixXd &faceConstrains, const RowVector3d& x, const RowVector3d& y, int fid)
{
    const RowVector3d& w = faceConstrains.block(fid, 3 * index, 1, 3);
    double wx = w.dot(x);
    double wy = w.dot(y);

    if (abs(wx) < 0.0000000001) wx = 0;
    if (abs(wy) < 0.0000000001) wy = 0;
    if (abs(1 - wx) < 0.0000000001) wx = 1;
    if (abs(1 - wy) < 0.0000000001) wy = 1;
    return complex<double>(wx, wy);
};

void MeshLinearSolver::SetConstrains(int ni, vector<int>& constrainedFids, vector<MeshLinearSolver_FaceData>& eq)
{
    constrainedFids.clear();
    constrainedFids.reserve(isFaceConstrained.size());
    for (int fid = 0; fid < isFaceConstrained.size(); fid++)
    {
        if (!isFaceConstrained[fid]) continue;
        const RowVector3d& x = F_X.row(fid);
        const RowVector3d& y = F_Y.row(fid);
        complex<double> weight(1.);
        complex<double> c0 = getComplex(0, faceConstrains, x, y, fid);
        complex<double> c1 = getComplex(1, faceConstrains, x, y, fid);
        complex<double> constrain = (ni == 0)
            ? weight * (c0*c0 + c1 * c1)
            : weight * c0*c0 * weight * c1 * c1;
        eq[fid].state = MeshLinearSolver_FaceData::State::Solved;
        eq[fid].Solution = constrain;
        eq[fid].depentOnCount = 0;
        constrainedFids.push_back(fid);
    }
}

void MeshLinearSolver::Solve(vector<MatrixX3d> &Result_F, bool& Result_F_isSorted)
{
    bool debugShowSteps = options.DebugEnabled && options.debug_debugStepNum != -1;
    int ni = 0;
    //for (int ni = 0; ni < 2; ni++)
    {
        // copy faces data
        vector<MeshLinearSolver_FaceData> eq(EQ);
        for (int fid = 0; fid < F.rows(); fid++)
        {
            eq[fid].pF = eq.data();
        }

        MeshLinearSolver_DependsOnFaceList eliminatedDependecies(eq.size());

        // set constrains
        vector<int> constrainedFids;
        SetConstrains(ni, constrainedFids, eq);

        //VectorXi fid_to_update;
        //fid_to_update.resize(F.rows()); // allocate enought space for any possible size list of updated faces ids
        auto debugDrawLines = [&](int fid, const Color3d& color)
        {
            const MeshLinearSolver_FaceData& f = eq[fid];
            Point3d c = F_Barycenters.row(f.fid);
            for (int k = 0; k < 3; k++)
            {
                if (f.F_isEliminated[k]) continue;// here we exlude as well all waiting faces
                Point3d cFriend = F_Barycenters.row(f.F[k]);
                Vector3d line = (cFriend - c);
                Point3d cEnd = c + line * 0.6;
                draw.AddEdge(c, cEnd, color);
            }
        };

        auto debugDrawInfo = [&](int fid, const Color3d& color, string decription)
        {
            Point3d c = F_Barycenters.row(fid);
            draw.AddPoint(c, color);
            draw.AddLabel(c, "  " + decription, color, 3);
            debugDrawLines(fid, color);
        };

        auto debugDraw = [&](const vector<int>& changed, const vector<int>& update, int step)
        {
            cout << endl << "step#" << step << "   changed " << changed.size() << " faces, and " << " needs to update " << update.size() << " faces" << endl;
            for (const auto& f : eq)
            {
                Point3d c = F_Barycenters.row(f.fid);
                if (f.state == MeshLinearSolver_FaceData::State::Solved)
                {
                    draw.AddPoint(c, Color3d(0, 1, 0)); //, "solved"
                }
                if (f.state == MeshLinearSolver_FaceData::State::Eliminated)
                {
                    debugDrawInfo(f.fid, Color3d(1, 1, 0), to_string(eq[f.fid].depentOnCount) + "waiting");
                }
            }
            for (int fid : changed)
            {
                if (eq[fid].state == MeshLinearSolver_FaceData::State::Unknown
                    && !utils::stdvector::exists(update, fid))
                {
                    debugDrawInfo(fid, Color3d(0.8, 0.8, 0), to_string(eq[fid].depentOnCount) + "`");
                }
            }
            for (int fid : update)
            {
                if (eq[fid].state == MeshLinearSolver_FaceData::State::Unknown)
                {
                    debugDrawInfo(fid, Color3d(1, 0, 0), to_string(eq[fid].depentOnCount) + (utils::stdvector::exists(changed, fid) ? "`" : ""));
                }
            }
        };

        vector<int> update;
        update.reserve(eq.size());
        vector<int> changed = constrainedFids;
        changed.reserve(eq.size());
        int step = -1;
        while (changed.size() > 0 || update.size() > 0)
        {
            step++;
            bool debugShowCurrentStep = debugShowSteps && step == options.debug_debugStepNum;

            //
            // mark friend faces for update
            //
            for (int fid : changed)
            {
                MeshLinearSolver_FaceData& f = eq[fid];
                if (f.depentOnCount == 3)
                {
                    //TODO work on 3x3x2
                    continue; // changed face with 3 dependencies cant update any friend face
                }

                // changed face has 0 dependencies
                if (f.state == MeshLinearSolver_FaceData::State::Solved)  // if changed face is solved - we have to substitude solved value in friend faces (happend ussually on backward stage (when we will substitude solved values in waiting faces))
                {
                    if (f.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList == -1)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            if (EQ[fid].F_isEliminated[k]) continue;
                            MeshLinearSolver_FaceData& fFriend = eq[EQ[fid].F[k]]; // here we have to use original EQ and not eq - since in eq relations between face changed
                            if (fFriend.state != MeshLinearSolver_FaceData::State::Solved)
                            {
                                fFriend.MarkFriendFaceForUpdate(f.fid); // sets flag 'needsToUpdate' and flag of which face has to be eliminated from dependencies
                                f.F_isEliminated[k] = false;// open door for updating friend
                            }
                        }
                    }
                    else
                    {
                        int index = f.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList;
                        while (index != -1)
                        {
                            MeshLinearSolver_FaceData& fFriend = eq[eliminatedDependecies.list[index].fid];
                            if (fFriend.state != MeshLinearSolver_FaceData::State::Solved)
                            {
                                fFriend.MarkFriendFaceForUpdate(f.fid);
                                fFriend.state = MeshLinearSolver_FaceData::State::Unknown;
                            }
                            index = eliminatedDependecies.list[index].prevIndex;
                        }
                    }
                    continue;
                }

                // changed face has 1 dependency
                if (f.depentOnCount == 1)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        if (f.F_isEliminated[k]) continue;// here we exlude as well all waiting faces
                        MeshLinearSolver_FaceData& fFriend = eq[f.F[k]];
                        if (fFriend.state != MeshLinearSolver_FaceData::State::Solved)
                        {
                            fFriend.MarkFriendFaceForUpdate(f.fid); // sets flag 'needsToUpdate' and flag of which face has to be eliminated from dependencies
                            eliminatedDependecies.AddNextDependence(f.fid, fFriend.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList);
                        }
                    }
                    f.state = MeshLinearSolver_FaceData::State::Eliminated; // f will waint on friend to be solved, and then on backward stage friend solved value will be substituted in our face and make it also solved
                    continue;
                }

                // changed face has 2 dependencies
                if (f.depentOnCount == 2 && f.CanBeEliminated_DoubleDependency())
                {
                    assert(f.state == MeshLinearSolver_FaceData::State::Unknown);
                    for (int k = 0; k < 3; k++)
                    {
                        if (f.F_isEliminated[k]) continue;// here we exlude as well all waiting faces
                        MeshLinearSolver_FaceData& fFriend = eq[f.F[k]];
                        assert(fFriend.state == MeshLinearSolver_FaceData::State::Unknown);
                        if (fFriend.state != MeshLinearSolver_FaceData::State::Solved)
                        {
                            fFriend.MarkFriendFaceForUpdate(f.fid); // sets flag 'needsToUpdate' and flag of which face has to be eliminated from dependencies
                            eliminatedDependecies.AddNextDependence(f.fid, fFriend.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList);
                        }
                    }
                    f.state = MeshLinearSolver_FaceData::State::Eliminated; // f will waint on friend to be solved, and then on backward stage friend solved value will be substituted in our face and make it also solved
                    continue;
                }
            }

            //
            // create a list of faces that need to be updated
            //
            update.clear();
            for (int fid : changed)
            {
                MeshLinearSolver_FaceData& f = eq[fid];
                if (f.state == MeshLinearSolver_FaceData::State::Solved && f.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList != -1) // special case for solved - this is backward stage, when we solved some face and others that waiting for it need to be updated
                {
                    int index = f.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList;
                    while (index != -1)
                    {
                        MeshLinearSolver_FaceData& fFriend = eq[eliminatedDependecies.list[index].fid];
                        if (fFriend.needsToUpdate)
                        {
                            update.push_back(fFriend.fid);
                            fFriend.needsToUpdate = false; // clear a flag, to avoid double occurance of fid in a 'update' list 
                        }
                        index = eliminatedDependecies.list[index].prevIndex;
                    }
                    f.dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList = -1;
                    continue;
                }
                for (int k = 0; k < 3; k++)
                {
                    if (f.F_isEliminated[k]) continue;
                    MeshLinearSolver_FaceData& fFriend = eq[f.F[k]];
                    if (fFriend.needsToUpdate)
                    {
                        update.push_back(fFriend.fid);
                        fFriend.needsToUpdate = false; // clear a flag, to avoid double occurance of fid in a 'update' list 
                    }
                }
            }
            if (options.DebugEnabled)
            {
                utils::stdvector::sort(update); // DEBUG for easyier debuging
                if (debugShowCurrentStep)
                {
                    cout << endl << "step#" << step << "   marking friend faces for update for " << changed.size() << " fids" << endl;
                    for (int fid : update)
                    {
                        cout << "update  fid#" << fid << "     dependencies=" << eq[fid].depentOnCount << "     state=" << eq[fid].stateStr() << endl;
                    }
                }
            }
            if (update.size() == 0)
            {
                if (debugShowCurrentStep)
                {
                    cout << "no more faces for update. finished at step#" << step << endl;
                }
                break; // break solve
            }

            //
            // DEBUG show step
            //
            if (debugShowSteps && step == options.debug_debugStepNum)
            {
                for (int fid : changed)
                {
                    //cout << "fid=" << fid << "    state = " << f.stateStr() << "  dependOnCount=" << f.depentOnCount << " ";
                }
                for (int fid : update)
                {
                }
                debugDraw(changed, update, step);
                break;
            }


            //
            // Increase step
            //
            step++;
            debugShowCurrentStep = debugShowSteps && step == options.debug_debugStepNum;

            //
            // Eliminate dependencies
            //
            if (debugShowCurrentStep) cout << endl << "step#" << step << "   updating " << update.size() << " fids:" << endl;
            atomic_int eliminatedTotalCount = 0;
            changed.clear();
            for (int fid : update)
            {
                MeshLinearSolver_FaceData& f = eq[fid];

                if (debugShowCurrentStep) cout << "fid=" << fid << "    state = " << f.stateStr() << "  dependOnCount=" << f.depentOnCount << " ";
                int eliminatedCount = f.EliminateDependicies();
                eliminatedTotalCount += eliminatedCount;
                changed.push_back(fid);

                if (debugShowCurrentStep) cout << "  new:    state = " << f.stateStr() << "  dependOnCount=" << f.depentOnCount << (eliminatedCount ? "  changed" : "") << endl;
            }
            if (debugShowCurrentStep) cout << "  eliminatedCount=" << eliminatedTotalCount << endl;

            //
            // DEBUG draw step
            //
            if (debugShowCurrentStep)
            {
                debugDraw(changed, update, step);
                break;
            }
        }
    }

}