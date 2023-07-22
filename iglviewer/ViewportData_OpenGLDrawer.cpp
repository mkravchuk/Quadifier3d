#include "stdafx.h"
#include "ViewportData_OpenGLDrawer.h"
#include "Viewport.h"
#include "ViewportObject.h"
#include "TextRenderer.h"
#include "ViewportData_OpenGLshaders.h"
#include "OpenGL_Options.h"
#include "OpenGL_Shared.h"

#define elapsed elapsedTimers.Viewer
#define options openglOptions

V3 ZERO_NORMAL = V3(0, 0, 0);

__forceinline float min3(float a, float b, float c)
{
    //asm("cmp   %1,%0\n"
    //    "cmova %1,%0\n"
    //    "cmp   %2,%0\n"
    //    "cmova %2,%0\n"
    //    : "+r" (a) : "r" (b), "r" (c));
    //return a;
    return std::min({ a,b,c });
}


void sortFacesForTransparencyDraw_Simple(int count, const vector<float>& dist_to_faces, vector<int>& dist_to_faces_sorted_indexes, ViewportData_OpenGLshaders& opengl)
{
    // initialize original index locations
    std::iota(dist_to_faces_sorted_indexes.begin(), dist_to_faces_sorted_indexes.begin() + count, 0);

    // sort indexes based on comparing values in v
    const vector<float>& pdist_to_faces = dist_to_faces;
    sort(dist_to_faces_sorted_indexes.begin(), dist_to_faces_sorted_indexes.begin() + count, [&pdist_to_faces](unsigned i1, unsigned i2)
    {
        return pdist_to_faces[i1] > pdist_to_faces[i2];
    });
}

void sortFacesForTransparencyDraw_KeyValue(int count, const vector<float>& dist_to_faces, vector<int>& dist_to_faces_sorted_indexes, ViewportData_OpenGLshaders& opengl)
{
    TransparencySort_CachedData& sortData = opengl.TransparencySort_CachedData;
    auto& keyvalue = sortData.dist_to_faces_keyvalue;
    if (count > keyvalue.size())
    {
        keyvalue.resize(count);
    }
    for (int i = 0; i < count; i++)
    {
        keyvalue[i].first = dist_to_faces[i];
        keyvalue[i].second = i;//we need descending order
    }
    sort(keyvalue.begin(), keyvalue.begin() + count);
    for (int i = 0; i < count; i++)
    {
        dist_to_faces_sorted_indexes[i] = keyvalue[count - 1 - i].second;
    }
}

void merge(int*, int*, int, int, int);
void mergesort(int *a, int*b, int low, int high)
{
    int pivot;
    if (low < high)
    {
        pivot = (low + high) / 2;
        mergesort(a, b, low, pivot);
        mergesort(a, b, pivot + 1, high);
        merge(a, b, low, pivot, high);
    }
}
void merge(int *a, int *b, int low, int pivot, int high)
{
    int h, i, j, k;
    h = low;
    i = low;
    j = pivot + 1;

    while ((h <= pivot) && (j <= high))
    {
        if (a[h] <= a[j])
        {
            b[i] = a[h];
            h++;
        }
        else
        {
            b[i] = a[j];
            j++;
        }
        i++;
    }
    if (h > pivot)
    {
        for (k = j; k <= high; k++)
        {
            b[i] = a[k];
            i++;
        }
    }
    else
    {
        for (k = h; k <= pivot; k++)
        {
            b[i] = a[k];
            i++;
        }
    }
    for (k = low; k <= high; k++) a[k] = b[k];
}
void sortFacesForTransparencyDraw_ButchQuickSort(int count, const vector<float>& dist_to_faces, vector< int>& dist_to_faces_sorted_indexes, ViewportData_OpenGLshaders& opengl)
{
    if (count == 0) return;
    TransparencySort_CachedData& sortData = opengl.TransparencySort_CachedData;
    auto& keyvalue = sortData.dist_to_faces_keyvalue;
    if (count > keyvalue.size())
    {
        keyvalue.resize(count);
    }


    const vector<float>& items = dist_to_faces;
    //DEBUG
    //vector<float> items = {1,6,3,5,2,8,6,9,4,4,0,3};
    //for (int i = 0; i < count; i++)
    //{
    //    keyvalue[i] = { -1, -1 };
    //}
    //const int bucketDimension = 3;

    // Get min max
    float minValue = items[0];
    float maxValue = items[0];
    for (const float& val : items)
    {
        if (val < minValue) minValue = val;
        if (maxValue < val) maxValue = val;
    }

    // Set buckets count
    const int bucketDimension = 1024; //10 Kb -  this size can be any, but i think is shuld be not to big to be cachable
                                      //const int bucketDimension = 3; //10 Kb -  this size can be any, but i think is shuld be not to big to be cachable
    int bucketsCount = static_cast<int>(items.size() / bucketDimension); //we will split sorting for few pieces, so we will not need merging later one
    if (items.size() % bucketDimension > 0) bucketsCount++;

    // Set buckets Diapason End
    float bucketRelativeLength = (maxValue - minValue) / bucketsCount; //each bucket contains values from A to B, and length B-A == 'bucketRelativeLength'
                                                                       //int bucketIndexShift = static_cast<int>(
    bucketsCount += 3;// list will be actually bigger for 2 items (first and last items) and +1 just to be safer - we will not check bucket index of item - we must be quick - this algorithm is all about performance
    vector<pair<int, int>> bucketsDiapasonStartEnd(bucketsCount, pair<int, int>(0, 0)); // start index in keyvalue, and end index in keyvalue  
                                                                                        // set bucketsDiapasonStartEnd.second -  count of items in some bucket
    for (const float& val : items)
    {
        float relativeValue = val - minValue;
        unsigned int bucketIndex = static_cast<unsigned int>(relativeValue / bucketRelativeLength); // yes, we need just index - to wich bucket this value will belong
        bucketsDiapasonStartEnd[bucketIndex].second++;// increment count of times in this backet - temporally storing count
    }
    // set bucketsDiapasonStartEnd.first -  start position in keyvalue list
    int nextStart = 0;
    for (auto& bDiapason : bucketsDiapasonStartEnd)
    {
        bDiapason.first = nextStart;
        nextStart += bDiapason.second;
    }
    // create array of  buckets insert positions in keyvalue
    vector<int> bucketInsertPositions(bucketsCount, 0);
    for (int i = 0; i < bucketsCount; i++)
    {
        bucketInsertPositions[i] = bucketsDiapasonStartEnd[i].first;
    }
    // move items to buckets (actually into keyvalue, but at special positions - to bucket's diapasons)
    // this actually partially sorts items by splitting them into groups from lower values to higher
    for (int i = 0; i < items.size(); i++)
    {
        float val = items[i];
        float relativeValue = val - minValue;
        unsigned int bucketIndex = static_cast<unsigned int>(relativeValue / bucketRelativeLength); // yes, we need just index - to wich bucket this value will belong
        int& insertPos = bucketInsertPositions[bucketIndex];
        keyvalue[insertPos] = { val, i };
        insertPos++;
    }

    //sort buckets
    extern bool IsOmpEnabled;
    #pragma omp parallel for if(IsOmpEnabled)
    for (int i = 0; i < bucketsCount; i++)
    {
        sort(keyvalue.begin() + bucketsDiapasonStartEnd[i].first, keyvalue.begin() + bucketsDiapasonStartEnd[i].first + bucketsDiapasonStartEnd[i].second);
    }

    // return results - we dont need to merge sorted buckets - buckets already sorted by concept itself
    for (int i = 0; i < count; i++)
    {
        dist_to_faces_sorted_indexes[i] = keyvalue[count - 1 - i].second;
    }
}

void sortFacesForTransparencyDraw(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl)
{
    if (!options.Transparent.Enabled) return;
    if (!options.Transparent.sortTriangles) return;
    if (opengl.V.size() == 0) return;
    if (opengl.F.size() == 0) return;
    Timer timeTotal;

    TransparencySort_CachedData& sortData = opengl.TransparencySort_CachedData;
    auto& F_vbo_duplicate = sortData.F_vbo_duplicate;
    auto& Vdistances = sortData.Vdistances;
    auto& dist_to_faces = sortData.dist_to_faces;
    auto& dist_to_faces_sorted_indexes = sortData.dist_to_faces_sorted_indexes;

    auto currentTime = chrono::high_resolution_clock::now();
    if (!options.Transparent.sortTriangles_showtime_inconsole)
    {
        ////
        //// avoid rapid calls - allow calls to sortFacesForTransparencyDraw to acieve 60fps
        ////
        if (currentTime < sortData.nextTimeAllowedToSort)
        {
            //DEBUG - show trothle
            //auto possible_call_after_ms = chrono::duration_cast<chrono::milliseconds>(sortData.nextTimeAllowedToSort - currentTime).count();
            //std::cout << "sortFacesForTransparencyDraw   throtled because last time call was long. Next call will be possible after " << possible_call_after_ms << " ms" << endl;
            return;
        }

        ////
        //// avoid calls with same model and rotation
        ////
        if (sortData.latcall_V_vbo_size == opengl.V.size()
            && sortData.latcall_F_vbo_size == opengl.F.size()
            && sortData.latcall_model == v.model.cast<float>()
            && sortData.latcall_view == v.view.cast<float>())
        {
            //std::cout << "sortFacesForTransparencyDraw   skipp sorting for same model" << endl;
            return;
        }
        sortData.latcall_V_vbo_size = opengl.V.size();
        sortData.latcall_F_vbo_size = opengl.F.size();
        sortData.latcall_model = v.model.cast<float>();
        sortData.latcall_view = v.view.cast<float>();
    }
    //cout << "sortFacesForTransparencyDraw  sorting" << endl;

    //
    // #1 - calculate vertex distances
    //
    // v1 - simple and slow 
    // formula is take from: http://stackoverflow.com/questions/16131963/depth-as-distance-to-camera-plane-in-glsl
    //Vector4f position_eye = model*view*Vector4f(v.row(i)(0), v.row(i)(1), v.row(i)(2), 1);
    //for (int i = 0; i < v.rows(); ++i) {
    //float dist = -position_eye.z();
    //}

    // V2 - fast - calculated in matrix style -  compiler should  optimize for SSE 
    // https://open.gl/transformations - helped to understand optimization
    //MatrixXf v = opengl.V_vbo.transpose();
    //Matrix4f tranformationMarix = model*view;
    //Vector4f tranformationVectorForZ = tranformationMarix.row(2); // since we need only z coordinat after transformation - we will use only 3-th row to optimize peformance
    //Vector3f tranformationVectorForZ3 = tranformationVectorForZ.head<3>();
    //float tranformationValueForZ = tranformationVectorForZ(3);
    //Vdistances = -((v*tranformationVectorForZ3).array() + tranformationValueForZ);
    Timer timeDistances;
    Matrix4f tranformationMarix = (v.model*v.view).eval().cast<float>();
    Vector4f tranformationVectorForZ = tranformationMarix.row(2); // since we need only z coordinat after transformation - we will use only 3-th row to optimize peformance
    Vector3f tranformationVectorForZ3 = tranformationVectorForZ.head<3>();
    float tranformationValueForZ = tranformationVectorForZ(3);
    //v1 using matrix - slow - need memory allocation and slow access to matrix elements
    //    VectorXf Vdistances; // distances from V to viewport - used in transparent sorting - defined here to avoid every time memory allocation 
    //#if EROW
    //    Vdistances = -((opengl.V_vbo*tranformationVectorForZ3).array() + tranformationValueForZ);
    //#else
    //    Vdistances = -((opengl.V_vbo.transpose()*tranformationVectorForZ3).array() + tranformationValueForZ);
    //#endif
    //v2 using vector (V_vbo has (x,y,z) coordinates placed linear one-by-one (so we dont need to check EROW - just access values one-by-one)
    int Vdistances_newCount = opengl.V.size() / 3;
    if (Vdistances_newCount > Vdistances.size())
    {
        Vdistances.resize(Vdistances_newCount);
    }
    float * vi = opengl.V.data(); // get eigen iterator to matrix
    float tranformationVectorForZ3_x = tranformationVectorForZ3(0);
    float tranformationVectorForZ3_y = tranformationVectorForZ3(1);
    float tranformationVectorForZ3_z = tranformationVectorForZ3(2);
    float* pV = Vdistances.data();
    float * pVEnd = pV + Vdistances_newCount;
    while (pV < pVEnd)
    {
        float vi_x = *vi++;
        float vi_y = *vi++;
        float vi_z = *vi++;
        *pV = -(vi_x*tranformationVectorForZ3_x + vi_y * tranformationVectorForZ3_y + vi_z * tranformationVectorForZ3_z);
        pV++;
    }

    // DEBUG - show distances to verticies
    //MatrixXf v = opengl.V_vbo;
    //vector<pair<float, int>> dist_to_vert_index(v.rows());
    //for (int i = 0; i < v.rows(); ++i)
    //{
    //    float dist = Vdistances(i);
    //    dist_to_vert_index[i] = std::make_pair(dist, i);
    //}
    //sort(dist_to_vert_index.begin(), dist_to_vert_index.end());
    //data.clear(false, false, true); // -  clear labels to avoid doubling same information and slow down app
    //for (int i = 0; i < dist_to_vert_index.size(); ++i)
    //{
    //    auto dist = dist_to_vert_index[i].first;
    //    auto vIndex = dist_to_vert_index[i].second;
    //    data.add_label(v.row(vIndex).cast<double>(), std::to_string(dist)); // show relative distances from vector to eye 
    //    //data.add_label(v.row(vIndex).cast<double>(), std::to_string(i)); // shows what vector is closer to eye
    //}
    //return;


    //
    // #2 - calculate face distances (Summarize distances for face for each of its 3 vertexes)
    //
    int  dist_to_faces_newCount = opengl.F.rows();
    if (dist_to_faces_newCount > dist_to_faces.size())
    {
        dist_to_faces.resize(dist_to_faces_newCount);
    }
    unsigned int * fvi = opengl.F.data(); // get eigen iterator to matrix
    float* pD = dist_to_faces.data();
    float * pDEnd = pD + dist_to_faces_newCount;
    float* pVdistances = Vdistances.data();
    while (pD < pDEnd)
    {
        // for every face store min distance (min sidt from every vertex of face)
        //v1 - simple
        //double dist = Vdistances[*fvi++];  // dist from first vertex of face
        //dist = std::min(dist, Vdistances[*fvi++]); // dist from second vertex of face
        //dist = std::min(dist, Vdistances[*fvi++]); // dist from third vertex of face
        //*pD = dist; //  save summ to dist_to_face_index - pair sumOfVertexDists-faceindex
        //pD++;
        //v2-fast
        //float minDist = min3(Vdistances[*fvi++], Vdistances[*fvi++], Vdistances[*fvi++]);
        //*pD = minDist; //  save min dist
        //pD++;
        //v3 - fast - doesnt use std:min which call jump
        //register float dist1 = *(pVdistances + *fvi++);  // dist from first vertex of face
        //register float dist2 = *(pVdistances + *fvi++);  // dist from first vertex of face
        //register float dist3 = *(pVdistances + *fvi++);  // dist from first vertex of face
        //register float minDist = dist1 < dist2 ? dist1 : dist2;
        //minDist = minDist < dist3 ? minDist : dist3;
        //*pD = minDist; //  save min dist
        //pD++;
        //v4 - fast - doenst need to call min - better visual quality 
        int fid0 = *(fvi + 0);
        int fid1 = *(fvi + 1);
        int fid2 = *(fvi + 2);
        float dist1 = *(pVdistances + fid0);  // dist from first vertex of face
        float dist2 = *(pVdistances + fid1);  // dist from first vertex of face
        float dist3 = *(pVdistances + fid2);  // dist from first vertex of face
        *pD = dist1 + dist2 + dist3; //  save summ dist
        pD++;
        fvi += 3;
    }
    timeDistances.stop();

    //
    // 2) sort dist_to_faces
    //
    //v1 - simple - allocates memory every time - and works only with local vector
    // auto dist_to_faces_sorted_indexes = sort_indexes_desc(dist_to_faces);
    //v2 - fast - use preallocated memory and works with prealocated vector
    Timer timeSort;
    if (dist_to_faces_newCount > dist_to_faces_sorted_indexes.size())
    {
        dist_to_faces_sorted_indexes.resize(dist_to_faces_newCount);
    }
    switch (options.Transparent.sortTriangles_sort_algorithm)
    {
        case OpenGL_Options_Transparent::SortAlgorithm::KeyValue:
            sortFacesForTransparencyDraw_KeyValue(dist_to_faces_newCount, dist_to_faces, dist_to_faces_sorted_indexes, opengl);
            break;
        case OpenGL_Options_Transparent::SortAlgorithm::Batch:
            sortFacesForTransparencyDraw_ButchQuickSort(dist_to_faces_newCount, dist_to_faces, dist_to_faces_sorted_indexes, opengl);
            break;
        default:
            sortFacesForTransparencyDraw_Simple(dist_to_faces_newCount, dist_to_faces, dist_to_faces_sorted_indexes, opengl);
            break;
    }
    timeSort.stop();

    // DEBUG - show distances to faces
    //MatrixXf v = opengl.V_vbo;
    //MatrixXui f = opengl.F_vbo;
    //data.clear(false, false, true);// -  clear labels to avoid doubling same information and slow down app
    //for (int i = 0; i < dist_to_faces_sorted_indexes.size(); ++i)
    //{
    //    auto fIndex = dist_to_faces_sorted_indexes[i];
    //    auto dist = dist_to_faces[fIndex];
    //    Vector3f faceMidPoint = (v.row(f(fIndex, 0)) + v.row(f(fIndex, 1)) + v.row(f(fIndex, 2))) / 3;
    //    data.add_label(faceMidPoint.cast<double>(), std::to_string(dist)); // show relative distances from vector to eye 
    //   //data.add_label(faceMidPoint.cast<double>(), std::to_string(i)); // shows what vector is closer to eye
    //  //  data.add_label(faceMidPoint.cast<double>(), std::to_string(f(fIndex, 1)));
    //}
    //return;


    //
    // #3 - change F_vbo
    //    
    // v1 - using matrix - slow copy - spended here 5% of time
    //auto fduplicate = MatrixXui(opengl.F_vbo);
    // v2 - using linear array - fast copy and no need in alocation memory every time 
    Timer timeCopy;
    int F_vbo_duplicate_newCount = opengl.F.size();
    if (F_vbo_duplicate_newCount > F_vbo_duplicate.size())
    {
        F_vbo_duplicate.resize(F_vbo_duplicate_newCount);
    }
    memcpy(F_vbo_duplicate.data(), opengl.F.data(), F_vbo_duplicate_newCount * sizeof(unsigned int)); //copy F_vbo to F_vbo_duplicate (F_vbo_duplicate size can be higher from F_vbo)

                                                                                                          //
                                                                                                          // v0 - single thread
                                                                                                          //
    unsigned int * fviDst = opengl.F.data();
    int * pfIndex = dist_to_faces_sorted_indexes.data();
    int * pfIndexEnd = pfIndex + dist_to_faces_newCount;
    //for (int i = 0; i < dist_to_faces_sorted_indexes.size(); ++i)
    //for (int fIndex : dist_to_faces_sorted_indexes)
    while (pfIndex < pfIndexEnd)
    {
        auto fIndex = *pfIndex;
        //auto fIndex = dist_to_faces_sorted_indexes[i];
        // v1 - using matrix - slow copy - 5%
        //#if EROW
        //        *fviDst++ = fduplicate(fIndex, 0);
        //        *fviDst++ = fduplicate(fIndex, 1);
        //        *fviDst++ = fduplicate(fIndex, 2);
        //#else
        //        *fviDst++ = fduplicate(0, fIndex);
        //        *fviDst++ = fduplicate(1, fIndex);
        //        *fviDst++ = fduplicate(2, fIndex);
        //#endif
        // v2 - using liniar array - fast copy and no need in alocation memory every time 
        // no need in checking EROW
        unsigned int * fviSrc = &F_vbo_duplicate[fIndex * 3];
        *fviDst++ = *fviSrc; fviSrc++;
        *fviDst++ = *fviSrc; fviSrc++;
        *fviDst++ = *fviSrc;
        pfIndex++;
    }

    //
    // v1 - 2 threads - doesnt give any performance
    //
    //int n = 2;
    //extern bool IsOmpEnabled;
    //#pragma omp parallel for if(IsOmpEnabled)
    //for (int i = 0; i < n; i++)
    //{
    //    int count = opengl.F_vbo.rows();
    //    int shift = i * (count / n);
    //    int shiftEnd = (i + 1) * (count / n);
    //    unsigned int * fviDst = opengl.F_vbo.data() + shift*3;
    //    unsigned int * pfIndex = dist_to_faces_sorted_indexes.data() + shift;
    //    unsigned int * pfIndexEnd = (i==n-1) ? dist_to_faces_sorted_indexes.data() + count : dist_to_faces_sorted_indexes.data() + shiftEnd;
    //    //for (int i = 0; i < dist_to_faces_sorted_indexes.size(); ++i)
    //    //for (int fIndex : dist_to_faces_sorted_indexes)
    //    while (pfIndex < pfIndexEnd)
    //    {
    //        auto fIndex = *pfIndex;
    //        unsigned int * fviSrc = &F_vbo_duplicate[fIndex * 3];
    //        *fviDst++ = *fviSrc; fviSrc++;
    //        *fviDst++ = *fviSrc; fviSrc++;
    //        *fviDst++ = *fviSrc;
    //        pfIndex++;
    //    }
    //}

    opengl.dirty |= ViewportData::DIRTY_FACE;
    timeCopy.stop();


    timeTotal.stop(elapsedTimers.Viewer.TransparentSort);
    //
    // Set next allowed time to call this method
    //
    if (options.Transparent.sortTriangles_showtime_inconsole)
    {

        std::cout << "sortFacesForTransparencyDraw()  time=" << timeTotal << "    (calcdist=" << timeDistances << "    sort=" << timeSort << "    copy=" << timeCopy << ")" << endl;
    }
    else
    {
        const int DESIRED_FPS = 50;
        const int DISERED_MILLISECONDS_PER_CALL = 1000 / DESIRED_FPS;
        const int MAX_THROTHLE_TIME_MS = 500;// 0.5 seconds 
        auto endTime = chrono::high_resolution_clock::now();
        auto elapsed_ms = chrono::duration_cast<chrono::milliseconds>(endTime - currentTime).count();
        sortData.nextTimeAllowedToSort = endTime;
        if (elapsed_ms > DISERED_MILLISECONDS_PER_CALL)
        {
            int throtle_ms = static_cast<int>((elapsed_ms - DISERED_MILLISECONDS_PER_CALL)*DESIRED_FPS / 10.0);
            throtle_ms = std::min(throtle_ms, MAX_THROTHLE_TIME_MS);
            sortData.nextTimeAllowedToSort += chrono::milliseconds(throtle_ms);
            //DEBUG - show trothle
            //std::cout << "sortFacesForTransparencyDraw  time: " << elapsed_ms << ",   trothle for " << throtle_ms << endl;
        }
        else
        {
            //DEBUG - show trothle
            //std::cout << "sortFacesForTransparencyDraw  time: " << elapsed_ms << endl;
        }
    }
}

void RestoreBlendOptions()
{
    //gl::Disable(GL_COLOR_LOGIC_OP);
    gl::Enable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // forward approach for blending colors (from back to front)
    //glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_ONE);

    //glBlendFunc(GL_DST_ALPHA, GL_ONE_MINUS_DST_ALPHA); // backward approach for blending colors (from front to back)
    //glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_DST_ALPHA);
    //glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
    //glBlendFunc(GL_DST_ALPHA, GL_ONE_MINUS_DST_ALPHA);

    //glBlendEquation(GL_FUNC_ADD);
    //glBlendFuncSeparate(GL_DST_ALPHA, GL_ONE, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA);
}

bool DepthMask_value = true;
void DepthMask_Restore()
{
    if (DepthMask_value)
    {
        glDepthMask(GL_TRUE);
    }
    else
    {
        glDepthMask(GL_FALSE);
    }
}
void DepthMask_Disable()
{
    glDepthMask(GL_FALSE);
}
void DepthMask_set(bool enabled)
{
    DepthMask_value = enabled;
    DepthMask_Restore();
}


void Antialiasing_Restore()
{
    if (options.Antialiasing.Enabled && openglOptions.WindowHint.SAMPLES != OpenGLAntialisingSamples::_0)
    {
        gl::Enable(GL_MULTISAMPLE);
    }
    else
    {
        gl::Disable(GL_MULTISAMPLE);
    }

    if (options.Antialiasing.Enabled && openglOptions.Antialiasing.LineSmoothingEnabled)
    {
        gl::Enable(GL_LINE_SMOOTH);
        // doesnt have impact at all - all GPU render lines smooth with same quality - so lets comment it
        switch (openglOptions.Antialiasing.LineSmoothingEnabled_quality)
        {
            case OpenGLAntialisingQuality::low:
                glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
                break;
            case OpenGLAntialisingQuality::high:
            default:
                glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
                break;
        }
    }
    else
    {
        gl::Disable(GL_LINE_SMOOTH);
    }

    if (options.Antialiasing.Enabled && openglOptions.Antialiasing.PolygonSmoothingEnabled)
    {
        gl::Enable(GL_POLYGON_SMOOTH);
    }
    else
    {
        gl::Disable(GL_POLYGON_SMOOTH);
    }

    gl::Disable(GL_POLYGON_STIPPLE);
    gl::Disable(GL_STENCIL_TEST);
};

void Antialiasing_Disable()
{
    if (options.Antialiasing.Enabled && openglOptions.WindowHint.SAMPLES != OpenGLAntialisingSamples::_0)
    {
        gl::Disable(GL_MULTISAMPLE);
    }

    if (options.Antialiasing.Enabled && openglOptions.Antialiasing.LineSmoothingEnabled)
    {
        gl::Disable(GL_LINE_SMOOTH);
    }
};

void DrawViewportData_Begin(Viewport& v)
{
    const Vector4i& size_int = v.size.cast<int>();

    // i think we dont have to change viewport size - lets draw on all screen
    glViewport(size_int(0), size_int(1), size_int(2), size_int(3));

    Antialiasing_Restore();

}




void Draw_DepthTest(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl)
{
    bool isMeshEmpty = (data.meshes.VCountTotal == 0);
    if (isMeshEmpty) return;

    //
    // Init 'view, model, proj' matrixes
    //                                                  
    const Matrix4f& view = v.view.cast<float>();
    const Matrix4f& model = v.model.cast<float>();
    const Matrix4f& proj = v.proj.cast<float>();
    Matrix4f view_model = view * model; //https://open.gl/transformations
    Matrix4f proj_view_model = proj * view * model; //https://open.gl/transformations
    #if EROW
    Matrix4f viewT = view.transpose();
    Matrix4f modelT = model.transpose();
    Matrix4f projT = proj.transpose();
    Matrix4f view_modelT = view_model.transpose();
    Matrix4f proj_view_modelT = proj_view_model.transpose();
    #else
    Matrix4f viewT = view;
    Matrix4f modelT = model;
    Matrix4f projT = proj;
    Matrix4f view_modelT = view_model;
    Matrix4f proj_view_modelT = proj_view_model;
    #endif

    //
    // Init 'shader_mesh_for_dephTesting' (send data to gpu)
    //
    opengl.bind_mesh_for_dephTesting();

    GLint proj_view_modeli = opengl.program_mesh_for_dephTesting.getUniformId("proj_view_model");
    glUniformMatrix4fv(proj_view_modeli, 1, GL_FALSE, proj_view_modelT.data());

    //for (int i = 0; i < 4; i++)
    //{
    //    GLint proj_view_modeli = opengl.program_mesh_for_dephTesting.getAttribId("proj_view_model_row"+to_string(i));
    //    Vector4f row = proj_view_modelT.row(i);
    //    glVertexAttrib4fv(proj_view_modeli, row.data());
    //}

    //
    // Faces
    //
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); // disable write to color buffer, since we need only populate depth bufer
    //gl::Disable(GL_BLEND);
    //opengl.draw_mesh(true, GL_FRONT_AND_BACK);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (options.Performance.unindex_depth_test)
    {
        glDrawArrays(GL_TRIANGLES, 0, opengl.V_unindexed.rows());
    }
    else
    {
        glDrawElements(GL_TRIANGLES, opengl.F.size(), GL_UNSIGNED_INT, 0);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    //gl::Enable(GL_BLEND);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE); // enable writes to color buffer
}




void Draw_Mesh(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl, bool drawFacesFromCachedLayer,
    double transparent_alpha_front, double transparent_alpha_back, double transparent_alpha__wireframe_multipliyer, bool show_faces, bool show_wireframe, bool show_texture, GLenum FRONT_AND_BACK)
{
    bool isMeshEmpty = (data.meshes.VCountTotal == 0);
    if (isMeshEmpty) return;


    if (drawFacesFromCachedLayer)
    {
        //cout << endl << "drawFacesFromCachedLayer" << endl << endl;
        gl::Disable(GL_DEPTH_TEST);
        show_faces = false;
        GLuint layer1_id = gl::get_layers_ColorBuffer_1();
        gl::DrawColorBuffer(layer1_id, true);
        gl::Enable(GL_DEPTH_TEST);
    }



    // exit if there is nothing to show
    if (!(show_faces || (show_wireframe && transparent_alpha__wireframe_multipliyer > 0.0001)))
    {
        return;
    }

    //
    // Init 'view, model, proj' matrixes
    //                                                  
    const Matrix4f& view = v.view.cast<float>();
    const Matrix4f& model = v.model.cast<float>();
    const Matrix4f& proj = v.proj.cast<float>();
    Matrix4f view_model = view * model; //https://open.gl/transformations
    Matrix4f proj_view_model = proj * view * model; //https://open.gl/transformations
    #if EROW
    Matrix4f viewT = view.transpose();
    Matrix4f modelT = model.transpose();
    Matrix4f projT = proj.transpose();
    Matrix4f view_modelT = view_model.transpose();
    Matrix4f proj_view_modelT = proj_view_model.transpose();
    #else
    Matrix4f viewT = view;
    Matrix4f modelT = model;
    Matrix4f projT = proj;
    Matrix4f view_modelT = view_model;
    Matrix4f proj_view_modelT = proj_view_model;
    #endif

    //
    // Init 'shader_mesh' (send data to gpu)
    //
    opengl.bind_mesh(show_texture);


    //GLint modeli = opengl.program_mesh.getUniformId("model");
    //GLint viewi = opengl.program_mesh.getUniformId("view");
    //GLint proji = opengl.program_mesh.getUniformId("proj");
    GLint view_modeli = opengl.program_mesh.getUniformId("view_model");
    GLint proj_view_modeli = opengl.program_mesh.getUniformId("proj_view_model");
    GLint alpha_BackSideCoeffI = opengl.program_mesh.getUniformId("alpha_BackSideCoeff");

    //glUniformMatrix4fv(modeli, 1, GL_FALSE, modelT.data());
    //glUniformMatrix4fv(viewi, 1, GL_FALSE, viewT.data());
    //glUniformMatrix4fv(proji, 1, GL_FALSE, projT.data());
    glUniformMatrix4fv(view_modeli, 1, GL_FALSE, view_modelT.data());
    glUniformMatrix4fv(proj_view_modeli, 1, GL_FALSE, proj_view_modelT.data());
    glUniform1f(alpha_BackSideCoeffI, static_cast<float>(options.Transparent.Enabled ? options.Transparent.alpha_BackSideCoeff : options.Light.alpha_BackSideCoeff));

    // Light parameters
    GLint specular_exponenti = opengl.program_mesh.getUniformId("specular_exponent");
    GLint specular_intensityi = opengl.program_mesh.getUniformId("specular_intensity");
    //GLint light_position_worldi = opengl.program_mesh.getUniformId("light_position_world");
    GLint light_position_eyei = opengl.program_mesh.getUniformId("light_position_eye");
    GLint lighting_factori = opengl.program_mesh.getUniformId("lighting_factor");

    glUniform1f(specular_exponenti, static_cast<float>(options.Transparent.Enabled ? options.Transparent.shininess : options.Light.shininess));
    glUniform1f(specular_intensityi, static_cast<float>(options.Light.shininess_intensity));
    Vector3f light = options.Light.light_position.cast<float>();
    //glUniform3fv(light_position_worldi, 1, rev_light.data());
    Vector4f light_eye = viewT * Vector4f(light(0), light(1), light(2), 1);
    glUniform3fv(light_position_eyei, 1, light_eye.data()); // vec3 light_position_eye = vec3 (view * vec4 (light_position_world, 1.0));
    glUniform1f(lighting_factori, static_cast<float>(options.Transparent.Enabled ? options.Transparent.lighting_factor : options.Light.lighting_factor)); // enables lighting


    GLint transparent_alpha_frontI = opengl.program_mesh.getUniformId("transparent_alpha_front");
    GLint transparent_alpha_backI = opengl.program_mesh.getUniformId("transparent_alpha_back");
    glUniform1f(transparent_alpha_frontI, static_cast<float>(transparent_alpha_front));
    glUniform1f(transparent_alpha_backI, static_cast<float>(transparent_alpha_back));
    GLint fixed_colori = opengl.program_mesh.getUniformId("fixed_color");
    GLint texture_factori = -1;
    if (show_texture) texture_factori = opengl.program_mesh.getUniformId("texture_factor");

    //
    // Faces
    //
    if (show_faces)
    {
        Antialiasing_Disable(); // disable antialiasing for faces - perfromace and look optimization - 10% speed improvement and better coloring of faces that are under background - background not influence on the faces color
        if (!options.Transparent.Enabled) gl::Disable(GL_BLEND);
        glUniform4f(fixed_colori, 0.0f, 0.0f, 0.0f, 0.0f);
        if (show_texture) glUniform1f(texture_factori, show_texture ? 1.0f : 0.0f);
        opengl.draw_mesh(true, FRONT_AND_BACK);
        if (show_texture) glUniform1f(texture_factori, 0.0f);
        if (!options.Transparent.Enabled) gl::Enable(GL_BLEND);//restore flag
        Antialiasing_Restore();
    }

    //
    // Wireframe
    //
    if (show_wireframe && transparent_alpha__wireframe_multipliyer > 0.0001)
    {
        Antialiasing_Restore();
        Color4f wireframe_color = v.wireframe_color;
        if (!options.Colors.wireframe_blend)
        {
            gl::Disable(GL_BLEND);
            wireframe_color *= 0.3;
        }
        DepthMask_Disable(); // suspend writing to deph buffer, to avoid artifacts
        glLineWidth(static_cast<float>(options.Sizes.wireframe_size));
        glUniform4f(fixed_colori, wireframe_color[0], wireframe_color[1], wireframe_color[2], v.wireframe_color[3] * static_cast<float>(transparent_alpha__wireframe_multipliyer));
        opengl.draw_mesh(false, FRONT_AND_BACK);
        glUniform4f(fixed_colori, 0.0f, 0.0f, 0.0f, 0.0f);
        DepthMask_Restore(); // resume writing to deph buffer
    }
}


void Draw_Labels(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl,
    double transparent_alpha)
{
    // Stop drawing if overlay off
    if (!v.show_overlay)
    {
        return;
    }

    int labels_count = 0;
    for (auto& draw : data.draws) labels_count += draw.second.get().labels.size();

    // exit if there is nothing to draw - this will avoid clear stencil buffer and as result improve speed
    if (labels_count == 0 && !v.show_vertid && !v.show_faceid)
    {
        return;
    }

    TextRenderer& textrenderer = v.textrenderer;
    bool isMeshEmpty = (data.meshes.VCountTotal == 0);
    const Vector4i& size_int = v.size.cast<int>();

    //
    // Init 'view, model, proj' matrixes
    //                                                  
    const Matrix4d& view = v.view;
    const Matrix4d& model = v.model;
    const Matrix4d& proj = v.proj;


    textrenderer.BeginDraw(view, model, proj, size_int, v.object_scale);

    //
    // Labels
    //
    if (labels_count > 0)
    {
        for (auto& draw : data.draws)
        {
            for (auto& label : draw.second.get().labels)
            {
                Color4d color = label.IsDefaultColor() ? Color4d(0, 0, 0, 1) : label.color;
                textrenderer.DrawText(label.point, ZERO_NORMAL, label.caption, transparent_alpha, color, label.FontSizeRelativeIncrease);
            }
        }
    }


    //
    // Vertex Ids (obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx)
    //
    if (v.show_vertid && !isMeshEmpty)
    {
        int vid_added_count = 0;
        for (auto& m : data.meshes.meshes)
        {
            for (int vid = 0; vid < m.V.rows(); ++vid)
            {
                //textrenderer.DrawText(data.V.row(vid), data.V_normals.row(vid), to_string(vid), show_vertid_color);
                int vid_global = vid_added_count + vid;
                textrenderer.DrawText(m.V.row(vid), ZERO_NORMAL, "   " + to_string(vid), transparent_alpha, v.show_vertid_color);
            }
            vid_added_count += m.V.rows();
        }
    }

    //
    // Edges Ids  (obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx)
    // EV no more present in OpgnGLDrawer - we remove it since it needed only for this obsolete code but takes a lot of time to calculate it
    // 
    //if (v.show_edgeid && !isMeshEmpty)
    //{
    //    for (int i = 0; i < data.EV.rows(); ++i)
    //    {
    //        textrenderer.DrawText((data.V.row(data.EV(i, 0)) + data.V.row(data.EV(i, 1))) / 2, ZERO_NORMAL, "  " + to_string(i), transparent_alpha, v.show_edgeid_color);
    //    }
    //}

    //
    // Faces Ids (obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx)
    //
    if (v.show_faceid && !isMeshEmpty)
    {
        for (auto& m : data.meshes.meshes)
        {
            for (int fid = 0; fid < m.F.rows(); ++fid)
            {
                P3 faceCenter = P3(0, 0, 0);
                for (int k = 0; k < m.F.cols(); ++k)
                    faceCenter += m.V.row(m.F(fid, k));
                faceCenter /= (D)m.F.cols();

                //textrenderer.DrawText(p, data.F_normals.row(i), to_string(i), show_faceid_color);
                textrenderer.DrawText(faceCenter, ZERO_NORMAL, to_string(fid), transparent_alpha, v.show_faceid_color);
            }
        }
    }

    //
    // Srf ids (obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx)
    //
    //if (v.show_srfid && data.Id >= 0)
    //{
    //    textrenderer.DrawText(data.VcenterProjectedIndside, ZERO_NORMAL, "  " + to_string(data.Id), transparent_alpha, v.show_srfid_color, 8);
    //}

    //
    // Srf serial numbers (obsolete - doesnt work for separated surfaces - use option Draw_Id_xxx)
    //
    //if (v.show_srfSerialNumber && data.SerialNumber >= 0)
    //{
    //    textrenderer.DrawText(data.VcenterProjectedIndside, ZERO_NORMAL, "  " + to_string(data.SerialNumber), transparent_alpha, v.show_srfSerialNumber_color, 8);
    //}

    textrenderer.EndDraw();
    RestoreBlendOptions();
}

void Draw_Lines_Dots(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl,
    double transparent_alpha, double point_size = -1)
{
    // Stop drawing if overlay off
    if (!v.show_overlay)
    {
        return;
    }

    bool isMeshEmpty = (data.meshes.VCountTotal == 0);
    const Vector4i& size_int = v.size.cast<int>();

    //
    // Init 'view, model, proj' matrixes
    //                                                  
    const Matrix4f& view = v.view.cast<float>();
    const Matrix4f& model = v.model.cast<float>();
    const Matrix4f& proj = v.proj.cast<float>();
    Matrix4f proj_view_model = proj * view*model; //https://open.gl/transformations
    #if EROW
    Matrix4f viewT = view.transpose();
    Matrix4f modelT = model.transpose();
    Matrix4f projT = proj.transpose();
    Matrix4f proj_view_modelT = proj_view_model.transpose();
    #else
    Matrix4f viewT = view;
    Matrix4f modelT = model;
    Matrix4f projT = proj;
    Matrix4f proj_view_modelT = proj_view_model;
    #endif


    GLint modeli;
    GLint viewi;
    GLint proji;
    GLint proj_view_modeli;
    GLint transparent_alphaI;

    //
    // Lines
    //
    if (opengl.lines_V.rows() > 0)
    {
        bool is_dirty = opengl.bind_overlay_lines_program();
        //modeli = opengl.program_overlay_lines.getUniformId("model");
        //viewi = opengl.program_overlay_lines.getUniformId("view");
        //proji = opengl.program_overlay_lines.getUniformId("proj");
        proj_view_modeli = opengl.program_overlay_lines.getUniformId("proj_view_model");
        transparent_alphaI = opengl.program_overlay_lines.getUniformId("transparent_alpha");

        //glUniformMatrix4fv(modeli, 1, GL_FALSE, modelT.data());
        //glUniformMatrix4fv(viewi, 1, GL_FALSE, viewT.data());
        //glUniformMatrix4fv(proji, 1, GL_FALSE, projT.data());
        glUniformMatrix4fv(proj_view_modeli, 1, GL_FALSE, proj_view_modelT.data());
        glUniform1f(transparent_alphaI, static_cast<float>(transparent_alpha));
        // This must be enabled, otherwise glLineWidth has no effect
        //gl::Enable(GL_LINE_SMOOTH);
        if (opengl.lines_F.rows() > 0)
        {
            opengl.bind_overlay_lines_linesNormal(is_dirty);
            double line_width = options.Sizes.line_size;
            if (v.show_wireframe) line_width *= options.Sizes.wireframe_linewidth_factor;// if wireframe is visible - increase thikness of line to make borders more visible
            glLineWidth(static_cast<float>(line_width));
            opengl.draw_overlay_lines();
        }
        if (opengl.linesBold_F.rows() > 0)
        {
            opengl.bind_overlay_lines_linesBold(is_dirty);
            double line_width = options.Sizes.lineBold_size;
            if (v.show_wireframe) line_width *= options.Sizes.wireframe_linewidth_factor;// if wireframe is visible - increase thikness of line to make borders more visible
            glLineWidth(static_cast<float>(line_width));
            opengl.draw_overlay_linesBold();
        }
    }

    //
    // Points
    //
    if (opengl.points_V.rows() > 0)
    {
        opengl.bind_overlay_points();
        //modeli = opengl.program_overlay_points.getUniformId("model");
        //viewi = opengl.program_overlay_points.getUniformId("view");
        //proji = opengl.program_overlay_points.getUniformId("proj");
        proj_view_modeli = opengl.program_overlay_points.getUniformId("proj_view_model");
        transparent_alphaI = opengl.program_overlay_points.getUniformId("transparent_alpha");

        //glUniformMatrix4fv(modeli, 1, GL_FALSE, modelT.data());
        //glUniformMatrix4fv(viewi, 1, GL_FALSE, viewT.data());
        //glUniformMatrix4fv(proji, 1, GL_FALSE, projT.data());
        glUniformMatrix4fv(proj_view_modeli, 1, GL_FALSE, proj_view_modelT.data());
        glUniform1f(transparent_alphaI, static_cast<float>(transparent_alpha));
        if (point_size < 0) point_size = options.Sizes.point_size;
        if (v.show_wireframe) point_size *= options.Sizes.wireframe_pointsize_factor; // if wireframe is visible - increase size of points to make them more visible
        if (point_size > options.Sizes.point_size_MAX) point_size = options.Sizes.point_size_MAX;// limit point size to 12 (since transparent and solid models has different point sizes)
        glPointSize(static_cast<float>(point_size));

        opengl.draw_overlay_points();
    }
}

void DrawViewportData__4layers(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl)
{
    //gl::Disable(GL_BLEND);
    //Draw_Lines_Dots(v, data, opengl, 1);
    //return;

    // Allow transparency  ( allows mesh lines draw transparently (anyway nanogui switch on this option))
    RestoreBlendOptions();
    enum class PrevDrawType
    {
        None, Mesh, LinesDots, Labels
    };
    PrevDrawType prevDrawType = PrevDrawType::None;


    auto draw_mesh = [&](double transparent_alpha_front, double transparent_alpha_back, double transparent_alpha__wireframe_multipliyer, bool show_faces, bool show_wireframe, GLenum FRONT_AND_BACK, bool drawFacesFromCachedLayer)
    {
        Draw_Mesh(v, data, opengl, drawFacesFromCachedLayer, transparent_alpha_front, transparent_alpha_back, transparent_alpha__wireframe_multipliyer, show_faces, show_wireframe, v.show_texture && options.Texture.Enabled, FRONT_AND_BACK);
        prevDrawType = PrevDrawType::Mesh;
    };

    auto draw_lines_dots = [&](double transparent_alpha, bool smooth, double point_size)
    {
        if (smooth) Antialiasing_Restore(); else Antialiasing_Disable();
        Draw_Lines_Dots(v, data, opengl, transparent_alpha, point_size);
        prevDrawType = PrevDrawType::LinesDots;
    };

    auto draw_labels = [&](double transparent_alpha, bool smooth)
    {
        if (smooth) Antialiasing_Restore(); else Antialiasing_Disable();
        Draw_Labels(v, data, opengl, transparent_alpha);
        prevDrawType = PrevDrawType::Labels;
    };


    // sort triangles to avoid artifacts (needed for transparent and for solid)
    sortFacesForTransparencyDraw(v, data, opengl); // sort F_vbo before opengl.bind_mesh

    Timer time;


    //
    // draw non-transparent
    //
    //DepthMask_set(true); // enable write to depth buffer
    if (!options.Transparent.Enabled)
    {
        //gl::Enable(GL_RASTERIZER_DISCARD);
        //gl::Disable(GL_CULL_FACE); // draw front and back 
        //gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_ALWAYS);
        //DepthMask_set(true); // enable write to depth buffer
        //Draw_DepthTest(v, data, opengl); // draw only to depth buffer even without fragment shader (improves speed by 20%)

        gl::Disable(GL_POLYGON_SMOOTH);
        gl::Disable(GL_CULL_FACE); // draw front and back 
        gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
        //gl::Disable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
        draw_mesh(options.Transparent.Layer2Visible.alpha_front, options.Transparent.Layer2Visible.alpha_back, 1, v.show_faces, v.show_wireframe, GL_FRONT_AND_BACK, false);
        draw_lines_dots(0.8, true, options.Sizes.point_size);
        //DepthMask_Disable(); // suspend writing to deph buffer, and thus enabling early-z-test
        draw_labels(1, true);
        //DepthMask_Restore();
        time.stop(elapsedTimers.Viewer.OpenGLDraw);
        //gl::Disable(GL_RASTERIZER_DISCARD);
        gl::Disable(GL_POLYGON_SMOOTH);
        return;
    }



    //
    // draw transparent
    //




    //for more info check folder: K:\Algorithms\Quadifier3d_Viewport\TestTransparentDraw\
    //for more info check site: www.alecjacobson.com/weblog/?p=2750 and test method above 'DrawViewportData__CheapTricksForOpenGLTransparency'
//#define GL_LESS 0x0201
//#define GL_EQUAL 0x0202
//#define GL_LEQUAL 0x0203
//#define GL_GREATER 0x0204
//#define GL_NOTEQUAL 0x0205
//#define GL_GEQUAL 0x0206
//#define GL_ALWAYS 0x0207
    //gl::Enable(GL_DEPTH_CLAMP); // gl::Enable(GL_DEPTH_CLAMP). This will cause the clip-space Z to remain unclipped by the front and rear viewing volume.   https://www.khronos.org/opengl/wiki/Vertex_Post-Processing#Depth_clamping
    //glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_FASTEST);   //disable the view-volume clipping test
    //glHint(GL_CLIP_VOLUME_CLIPPING_HINT_EXT, GL_DONT_CARE); //enable the view-volume clipping test
    //glShadeModel(GL_FLAT); // - no effect at all, why...



    //
    // draw depth buffer  (render with alpha = 0, just to prime the depth buffer)
    //    
    bool drawFacesFromCachedLayer = false;
    if (options.Transparent.Layer0Depth.Enabled)
    { 
        gl::Disable(GL_CULL_FACE); // draw front and back 
        gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
        DepthMask_set(true); // enable write to depth buffer
        if (openglOptions.Performance.use_viewport_layers && v.show_faces
            //&& options.Transparent.Layer2Visible.Enabled_Front && options.Transparent.Layer2Visible.Enabled_Back
            ) 
        {
            // draw front faces in cache layer and at same time populate depth_test buffer
            GLuint attachments1[1] = { GL_COLOR_ATTACHMENT1 };
            glDrawBuffers(1, attachments1); // activate layer#1
            gl::Disable(GL_BLEND); // draw without blending to be able later draw with blending - this will avoid double blending
            double zero__wireframe_multipliyer = 0; // draw only surfaces
            draw_mesh(options.Transparent.Layer2Visible.alpha_front, options.Transparent.Layer2Visible.alpha_back, zero__wireframe_multipliyer, v.show_faces, false, GL_FRONT_AND_BACK, false); //always draw faces, in case of invisible - set alpha to zero
            drawFacesFromCachedLayer = true;
            gl::Enable(GL_BLEND);
            GLuint attachments0[1] = { GL_COLOR_ATTACHMENT0 };
            glDrawBuffers(1, attachments0); // activate default layer
        }
        else
        {
            // draw only to depth buffer even without fragment shader (improves speed by 20%)
            Draw_DepthTest(v, data, opengl);
        }
        DepthMask_Disable(); // suspend writing to deph buffer, and thus enabling early-z-test
    }

    //
    // draw hidden layers
    //
    if (options.Transparent.Layer1Hidden.Enabled)
    {
        gl::Enable(GL_CULL_FACE); glCullFace(GL_FRONT); // draw back only
        gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_GREATER); // draw hidden faces
        bool smoothLines = options.Antialiasing.Enabled && options.Antialiasing.LineSmoothingEnabled && (options.Antialiasing.LineSmoothingEnabled_inHiddenLayers || !v.show_faces); //  if faces not visible then its better to smooth hidden lines, because they are now much clear visible
        double point_size = options.Sizes.point_size;
        point_size *= options.Transparent.Layer1Hidden.pointsize_factor; // for transparent model we need to decrease point size, since big size is very annoing
        draw_lines_dots(options.Transparent.Layer1Hidden.alpha_lines, smoothLines, point_size); // first draw overlay
        draw_labels(1, smoothLines); // draw labels after lines and before faces - to aciheve some shadowing (not perfect but at least some)
        draw_mesh(options.Transparent.Layer1Hidden.alpha_front, options.Transparent.Layer1Hidden.alpha_back, 0, v.show_faces, v.show_wireframe, GL_BACK, false); //always draw faces
    }

    //
    // draw front layers
    //
    bool smoothLines = options.Antialiasing.Enabled && options.Antialiasing.LineSmoothingEnabled;
    if (options.Transparent.Layer2Visible.Enabled_Front && options.Transparent.Layer2Visible.Enabled_Back)
    {
        gl::Disable(GL_CULL_FACE); // draw front and back 
        gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL); // draw back sides, that are not hidden by any surfaces (must be GL_EQUAL to avoid artifacts)

        draw_mesh(options.Transparent.Layer2Visible.alpha_front, options.Transparent.Layer2Visible.alpha_back, 1.5, v.show_faces, v.show_wireframe, GL_FRONT_AND_BACK, drawFacesFromCachedLayer); //always draw faces, in case of invisible - set alpha to zero
        //glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE_MINUS_SRC_ALPHA); // forward approach for blending colors (from back to front)
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // forward approach for blending colors (from back to front)
        //glBlendFunc(GL_SRC_ALPHA, GL_CONSTANT_COLOR); // forward approach for blending colors (from back to front)
        //glBlendFunc(GL_CONSTANT_COLOR, GL_DST_COLOR);
        //glBlendFunc(GL_CONSTANT_COLOR, GL_SRC_ALPHA); // forward approach for blending colors (from back to front)
        //glBlendColor(0.2, 0.2, 0.2, 0.6);
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA); // forward approach for blending colors (from back to front)
        draw_lines_dots(0.8, smoothLines, options.Sizes.point_size); //draw overlay a bit transparenctly just to be in same tone with other transparent model
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // forward approach for blending colors (from back to front)
    }
    else
    {
        if (options.Transparent.Layer2Visible.Enabled_Back)
        {
            gl::Enable(GL_CULL_FACE); glCullFace(GL_FRONT); // draw back only
            gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL); // draw back sides, that are not hidden by any surfaces (must be GL_EQUAL to avoid artifacts)
            draw_mesh(options.Transparent.Layer2Visible.alpha_front, options.Transparent.Layer2Visible.alpha_back, 0.3, v.show_faces, v.show_wireframe, GL_BACK, false);//always draw faces
        }
        if (options.Transparent.Layer2Visible.Enabled_Front)
        {
            gl::Enable(GL_CULL_FACE); glCullFace(GL_BACK); // draw front only
            gl::Enable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL); // draw front side that are on top of back sides
            draw_mesh(options.Transparent.Layer2Visible.alpha_front, options.Transparent.Layer2Visible.alpha_back, 1.5, v.show_faces, v.show_wireframe, GL_FRONT, false); //always draw faces, in case of invisible - set alpha to zero
            draw_lines_dots(0.8, smoothLines, options.Sizes.point_size); //draw overlay a bit transparenctly just to be in same tone with other transparent model
        }
    }


    // 
    // Clear flags
    //
    glUseProgram(0);
    gl::Disable(GL_DEPTH_TEST);
    gl::Disable(GL_CULL_FACE);
    glDepthMask(GL_TRUE); // we must restore this value to avoid performance penalty
    if (!options.Performance.use_nanogui_layers) gl::Enable(GL_STENCIL_TEST);
    gl::Disable(GL_RASTERIZER_DISCARD);
    gl::Disable(GL_POLYGON_SMOOTH);
    time.stop(elapsedTimers.Viewer.OpenGLDraw);
}


void DrawViewportData(Viewport& v, ViewportData& data, ViewportData_OpenGLshaders& opengl)
{
    //
    // Update opengl if data has changed or new
    //
    if (data.dirty)
    {
        opengl.set_data(data, v.invert_normals, v.crease_normals);
        data.dirty = ViewportData::DIRTY_NONE;
    }
    if (data.IsEmpty()) return;
    // v1 - single draw 
    //DrawViewportData__1layer(v, data, opengl);

    // v2 - double draw - perfect result - no triangles sorting required - excelent speed and look
    //DrawViewportData__2layers(v, data, opengl);

    // v3 - good results but not perfect - required 4 redraws - example taken from http://www.alecjacobson.com/weblog/?p=2750
    //DrawViewportData__CheapTricksForOpenGLTransparency(v, data, opengl);

    // v4 - double draw - perfect result - no triangles sorting required - excelent speed and look
    //DrawViewportData__3layers(v, data, opengl);

    // v5 - double draw - perfect result - no triangles sorting required - excelent speed and look
    DrawViewportData__4layers(v, data, opengl);
}
