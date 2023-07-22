#include "stdafx.h"
#include "ModelFile.h"
#include "ModelObject.h"
#include "Mesher.h"

atomic_int ModelFile_nextId = 1;


ModelFile::ModelFile(const Model& _model, const string& _filename, const string& loadedOnlyObjectNames)
    : nextSrfId(0), readonly_Id(ModelFile_nextId++), 
      Id(readonly_Id), model(_model), Filename(_filename), LoadedOnlyObjectNames(loadedOnlyObjectNames), IsActive(true), mesher(Mesher(*this))
{
}

int ModelFile::GetNextSrfUniqueId()
{
    int res = nextSrfId;
    ++nextSrfId;
    return res;
}

II ModelFile::SizeOF() const
{
    II r = sizeof(ModelFile);
    r += Filename.size();
    r += LoadedOnlyObjectNames.size();
    r += AllObjects.size() * sizeof(ModelObject*);
    r += ActiveObjects.size() * sizeof(ModelObject*);
    r += ActiveObjects_Ids.size() * sizeof(int);
    r += Topology.SizeOF();
    r += mesher.SizeOF();
    return r;

}

void ModelFile::UpdateMeshTopology(bool updateOnlyIfObjectsAreChanged)
{
    //cout << "Updating topology... ";
    Timer timer;
    Topology.Update(ActiveObjects, ActiveObjects_Ids, updateOnlyIfObjectsAreChanged);
    timer.stop(elapsedTimers.Mesh.Topology);
    //if (meshLogicOptions.showTimeElapsed) cout << " done in " << timer << endl; else cout << endl;
}

void ModelFile::RefreshActiveObjects()
{
    if (!IsActive)
    {
        ActiveObjects.clear();
        ActiveObjects_Ids.clear();
        // we do not clear Topology - we may reuse it
        return;
    }

    // check if refresh is needed
    vector<int> newActiveObjects_Ids;
    if (IsActive)
    {
        newActiveObjects_Ids.reserve(AllObjects.size());
        for (auto obj : AllObjects)
        {
            if (obj->IsActive)
            {
                newActiveObjects_Ids.push_back(obj->Id);
            }
        }
        utils::stdvector::sort(newActiveObjects_Ids);// probably this array will be always sorted, but lets sort it anyway to be 100% sure
    }

    // refresh if needed
    bool isRefreshNeeded = !utils::stdvector::same(ActiveObjects_Ids, newActiveObjects_Ids);
    if (isRefreshNeeded)
    {
        ActiveObjects_Ids = newActiveObjects_Ids;
        ActiveObjects.clear();
        if (IsActive)
        {
            ActiveObjects.reserve(AllObjects.size());
            for (auto obj : AllObjects)
            {
                if (obj->IsActive)
                {
                    ActiveObjects.push_back(obj);
                }
            }
        }
        UpdateMeshTopology(true);
    }
}

void ModelFile::ClearDrawObjects()
{
    Topology.draw.Clear();
    mesher.ClearDrawObjects();
}

