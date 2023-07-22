#pragma once
class ViewerDrawObjects;
struct FaceConstrain;

struct MeshLinearSolver_DependsOnFace
{
    int fid; // fid this is depend on current face
    int prevIndex; // prev index in list of same structure data 'MeshLinearSolver_FaceData_DependOnMe'
};

class MeshLinearSolver_DependsOnFaceList // holds MeshLinearSolver_DependOnFace, and manage thread savaty adding to this list
{
private:
    atomic_int lastAddedIndex;
public:
    vector<MeshLinearSolver_DependsOnFace> list;
    MeshLinearSolver_DependsOnFaceList(int size)
    {
        list.resize(size*2);//size for forward and size for backward
        Clear();
    }
    void Clear()
    {
        lastAddedIndex = -1;
    }
    void AddNextDependence(int fid, int& currentIndex)
    {
        int prevIndex = currentIndex;
        currentIndex = ++lastAddedIndex;
        assert(currentIndex < list.size());
        list[currentIndex] = {fid, prevIndex};
    }
};

struct MeshLinearSolver_FaceData
{
    enum class State
    {
        Unknown,   // uknown value in equation
        Eliminated, // if face is eliminated from equation and now is waiting for friends to be solved
        Solved        // if solution is already found for this face 
    };
    int fid; // indexfid of this face
    MeshLinearSolver_FaceData* pF;
    int F[3];// friend-faces. (defined as poiner, to be able join surfaces in one linear equation)
    bool F_isEliminated[3]; // if coefficient to friend-face is eliminated
    bool F_needsToUpdate[3]; // flag that used i 'solve' method to indentify friend-faces that will should updated in method 'Update'
    bool needsToUpdate; //  flag that used in 'solve' method to indentify faces that will be updated in step
    int depentOnCount;
    int dependsOnMe_last_index_in_MeshLinearSolver_DependsOnFaceList;
    complex<double> R[3]; // rotation coefficients to friend-faces
    complex<double> K; // know value of equation for this face
    complex<double> C; // coefficient of variable
    State state;  // 
    complex<double> Solution; // solution of linear equation for this face
    MeshLinearSolver_FaceData(int fid);
    void MarkFriendFaceForUpdate(int fidFriend); // sets flag 'needsToUpdate' and flag 'F_needsToUpdate' of which face has to be eliminated from dependencies
    int EliminateDependicies();
    bool CanBeEliminated_DoubleDependency();
    int EliminateDoubleDependency();
    int GetDependenceNum(int fid) const;
    int GetSecondDependenceNum(int fid) const;
    int GetSingleDependence(int& fk) const;
    //int DependencesCount() const;
    string stateStr();
};


class MeshLinearSolver
{
private:
    VectorXb isFaceConstrained;
    MatrixXd faceConstrains;
    vector<MeshLinearSolver_FaceData> EQ;
public:
    ViewerDrawObjects & draw;
    const MatrixXd& V;
    const MatrixXi& F;
    const VectorXd& F_Areas;
    const MatrixXd& F_Barycenters;
    const MatrixXi& EV;
    const MatrixXi& FE;
    const MatrixXi& EF;
    const VectorXd& K;
    const MatrixXd& F_X;
    const MatrixXd& F_Y;
    const MatrixXd& F_Z;
    const MatrixXi& TT;
    const MatrixXi& TTi;
    double avg_edge_length;
    double max_edge_length;

    MeshLinearSolver(ViewerDrawObjects& draw,
        const MatrixXd &V,
        const MatrixXi &F,
        const VectorXd &F_Areas,
        const MatrixXd& F_Barycenters,
        const MatrixXi& EV,
        const MatrixXi& FE,
        const MatrixXi& EF,
        const VectorXd& K,
        const MatrixXd& F_X,
        const MatrixXd& F_Y,
        const MatrixXd& F_Z,
        const MatrixXi& TT,
        const MatrixXi& TTi,
        double avg_edge_length,
        double max_edge_length);

    void SetConstrains(
        const vector<FaceConstrain>& Constrains,  // Constrained faces representative vector
        double softPercent, // ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)        
        bool WeightIsRelativeToEdgeLength, bool logMessages, bool ignore_x, bool takeDirectionCorrected);

    void Solve(vector<MatrixX3d> &Result_F, bool& Result_F_isSorted);

private:
    void SetConstrains(int ni, vector<int>& constrainedFids, vector<MeshLinearSolver_FaceData>& eq);
    void PopulateEQ();
};