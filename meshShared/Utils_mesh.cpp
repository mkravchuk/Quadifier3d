#include "stdafx.h"
#include "Utils_mesh.h"
#include "Utils_cpu.h"
#include "Utils_sse.h"
#include "Utils_point.h"
#include "Utils_vector.h"
#include "Utils_stdvector.h"

namespace utils
{
    namespace mesh
    {
        P3 GetFaceCentroid(int faceId, const I3s& F, const P3s& V)
        {
            P3 v0 = V.row(F(faceId, 0));
            P3 v1 = V.row(F(faceId, 1));
            P3 v2 = V.row(F(faceId, 2));
            return (v0 + v1 + v2) / static_cast<D>(3);
        }

        void GetEdgesLenghts(const P3s& V, const I2s& EV, Ds& E_Length, bool useSSE)
        {
            const int* pv = EV.data();
            const int* pvEnd = pv + EV.rows() * 2; // we get 2 vertexes for one edge
            const int* pvEnd4 = pvEnd - 4 * 2; // we get 2 vertexes for one edge
            E_Length.resize(EV.rows());
            D* pLength = E_Length.data();

            auto get_DistToPointPow2 = [&]()
            {
                int vid0 = *pv;
                pv++;
                int vid1 = *pv;
                pv++;
                P3 v0 = V.row(vid0);
                P3 v1 = V.row(vid1);
                return utils::point::DistToPointPow2(v0, v1);
            };

            if (utils::cpu::isSupportedSSE && useSSE && EV.rows() > 4)
            {
                while (pv < pvEnd4)
                {
                    D distpow2_0 = get_DistToPointPow2();
                    D distpow2_1 = get_DistToPointPow2();
                    D distpow2_2 = get_DistToPointPow2();
                    D distpow2_3 = get_DistToPointPow2();
                    Vec4f v4(distpow2_0, distpow2_1, distpow2_2, distpow2_3);
                    sqrt(v4).store(pLength);
                    pLength += 4;
                }
            }

            while (pv < pvEnd)
            {
                *pLength = sqrt(get_DistToPointPow2());
                pLength++;
            }
        }

        void GetEdgesAngles(const I2s& EF, const V3s& F_normals, Ds& E_Angles, bool useSSE)
        {
            E_Angles.resize(EF.rows());
            for (int eid = 0; eid < EF.rows(); eid++)
            {
                int f0 = EF(eid, 0);
                int f1 = EF(eid, 1);
                E_Angles[eid] = (f1 == -1 || isnan(F_normals.row(f0)(0)) || isnan(F_normals.row(f1)(0)))
                    ? NAN
                    : utils::vector::Angle(F_normals.row(f0), F_normals.row(f1), true);
            }
        }

        D GetAvgEdgeLength(const Ds& E_Length, D& min_edge_length, D& max_edge_length)
        {
            DD sumLengths = 0;
            unsigned count = E_Length.size();
            const D* pLength = E_Length.data();
            const D* pLengthEnd = pLength + count;
            min_edge_length = 0;
            max_edge_length = 0;
            if (count != 0)
            {
                min_edge_length = *pLength;
                max_edge_length = *pLength;
            }
            while (pLength < pLengthEnd)
            {
                //D length = E_Length(ei);
                D length = *pLength;
                pLength++;
                //if (length < min_edge_length)  min_edge_length = length;
                min_edge_length = (length < min_edge_length) ? length : min_edge_length;
                max_edge_length = (length > max_edge_length) ? length : max_edge_length;

                sumLengths += length;
            }

            return static_cast<D>(sumLengths / static_cast<DD>(count));
        }

        // same as igl::avg_edge_length but 10x times faster
        D GetAvgEdgeLength(const P3s& V, const I2s& EV, const I2s& EF, D& min_edge_length, D& max_edge_length)
        {
            DD avg = 0;
            int rows = EV.rows();
            V3 v(0, 0, 0);
            int row0 = 0;
            int row0next = 0;
            int count = 0;
            min_edge_length = 0;
            max_edge_length = 0;
            bool firstTime = false;
            for (int ei = 0; ei < rows; ++ei)
            {
                row0 = EV(ei, 0);
                row0next = EV(ei, 1);
                v = V.row(row0) - V.row(row0next);
                D length = v.norm();
                if (firstTime)
                {
                    min_edge_length = length;
                    max_edge_length = length;
                }
                else
                {
                    if (length < min_edge_length)  min_edge_length = length;
                    if (length > max_edge_length)  max_edge_length = length;
                }

                avg += length;
                count++;
                if (EF(ei, 1) == -1) continue;// if edge has only 1 face dont add second time legnth
                avg += length; //if edge has 2 faces - add twice length to avg
                count++;
            }

            return static_cast<D>(avg / (DD)(count));
        }

        // same as igl::avg_edge_length but 3x times faster
        D GetAvgEdgeLength(const P3s& V, const I3s& F)
        {
            DD avg = 0;
            int rows = F.rows();
            V3 v(0, 0, 0);
            int row0 = 0;
            int row0next = 0;
            for (int i = 0; i < rows; ++i)
            {
                row0 = F(i, 0);
                row0next = F(i, 1);
                v = V.row(row0) - V.row(row0next);
                avg += v.norm();

                row0 = F(i, 1);
                row0next = F(i, 2);
                v = V.row(row0) - V.row(row0next);
                avg += v.norm();


                row0 = F(i, 2);
                row0next = F(i, 0);
                v = V.row(row0) - V.row(row0next);
                avg += v.norm();
            }

            return static_cast<D>(avg / (DD)(F.rows() * 3));
        }

        void GetDistancesPow2(const P3s& points, const P3& p, std::vector<D>& distances)
        {
            int rows = points.rows();
            assert(rows > 0);
            distances.resize(rows);
            for (int i = 0; i < rows; i++)
            {
                P3 pi = points.row(i);
                distances[i] = utils::point::DistToPointPow2(p, pi);
            }
        }
        void GetDistances(const P3s& points, const P3& p, std::vector<D>& distances)
        {
            int rows = points.rows();
            assert(rows > 0);
            distances.resize(rows);
            for (int i = 0; i < rows; i++)
            {
                P3 pi = points.row(i);
                distances[i] = utils::point::DistToPoint(p, pi);
            }
        }
        int GetClosestPointIndex(const P3s& points, const P3& p)
        {
            int rows = points.rows();
            assert(rows > 0);
            int closestIndex = 0;
            D closestDist = utils::point::DistToPointPow2(p, points.row(0));
            for (int i = 1; i < rows; i++)
            {
                P3 pi = points.row(i);
                D disti = utils::point::DistToPointPow2(p, pi);
                if (disti < closestDist)
                {
                    closestDist = disti;
                    closestIndex = i;
                }
            }
            return closestIndex;
        }
        void GetClosestPointIndexes(const P3s& points, const P3& p, std::vector<int>& closestPointsIndexes, int count)
        {
            closestPointsIndexes.clear();
            std::vector<D> distancesPow2;
            GetDistancesPow2(points, p, distancesPow2);
            std::vector<unsigned int> sortIndexes = utils::stdvector::sort_indexes(distancesPow2);

            closestPointsIndexes.reserve(count);
            for (int i = 0; i < min(count, points.rows()); i++)
            {
                closestPointsIndexes.push_back(sortIndexes[i]);
            }
        }

        //same as igl::per_vertex_normals but 3x time faster
        void GetVertexNormals(const P3s& V, const I3s& F, const V3s& F_normals, const Ds& F_Areas, V3s& V_normals)
        {
            V_normals.setZero(V.rows(), 3);

            // loop over faces
            for (int fid = 0; fid < F.rows(); fid++)
            {
                D weight = F_Areas(fid);
                V3 fnormal = F_normals.row(fid)*weight;
                auto f = F.row(fid);
                // throw normal at each corner
                V_normals.row(f(0)) += fnormal;
                V_normals.row(f(1)) += fnormal;
                V_normals.row(f(2)) += fnormal;
            }


            // take average via normalization
            for (int i = 0; i < V_normals.rows(); i++)
            {
                V_normals.row(i).normalize();
            }
        }

        //same as igl::triangle_triangle_adjacency_preprocess but faster and using Matrix instead of vector that make even dealocation faster
        // each row of TTT will have { v1 v2 fi eilocal}
        void triangle_triangle_adjacency_preprocess_simple_slow(const I3s& F, I4s& TTT)
        {
            I4s TTTunsorted;
            TTTunsorted.resize(F.rows() * 3, 4);
            int tttIndex = 0;
            int ei = 0;
            int vertexIndex1 = 0;
            int vertexIndex2 = 0;
            // for each face edge contruct relation : vertexIndex1,vertexIndex2, face, edgeindex(0,1,2)
            for (int fi = 0; fi < F.rows(); ++fi)
            {
                ei = 0;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 1);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                TTTunsorted(tttIndex, 0) = vertexIndex1;
                TTTunsorted(tttIndex, 1) = vertexIndex2;
                TTTunsorted(tttIndex, 2) = fi;
                TTTunsorted(tttIndex, 3) = ei;
                tttIndex++;

                ei = 1;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 2);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                TTTunsorted(tttIndex, 0) = vertexIndex1;
                TTTunsorted(tttIndex, 1) = vertexIndex2;
                TTTunsorted(tttIndex, 2) = fi;
                TTTunsorted(tttIndex, 3) = ei;
                tttIndex++;

                ei = 2;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 0);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                TTTunsorted(tttIndex, 0) = vertexIndex1;
                TTTunsorted(tttIndex, 1) = vertexIndex2;
                TTTunsorted(tttIndex, 2) = fi;
                TTTunsorted(tttIndex, 3) = ei;
                tttIndex++;
            }

            // sort TTT by vertexIndex1, vertexIndex2
            // we will do same as 'std::sort(TTT.begin(), TTT.end())' but much faster using indexes and static memory
            std::vector<unsigned> sorted_indexes = utils::stdvector::sort_indexes_custom(TTTunsorted.rows(), [&TTTunsorted](unsigned int i1, unsigned int i2)
            {
                int t1v1 = TTTunsorted(i1, 0);
                int t1v2 = TTTunsorted(i1, 1);
                int t2v1 = TTTunsorted(i2, 0);
                int t2v2 = TTTunsorted(i2, 1);
                return (t1v1 < t2v1) || (t1v1 == t2v1 && t1v2 < t2v2);
            });

            TTT.resize(TTTunsorted.rows(), TTTunsorted.cols());
            for (int i = 0; i < TTTunsorted.rows(); i++)
            {
                TTT.row(i) = TTTunsorted.row(sorted_indexes[i]);
            }
        }


        void triangle_triangle_adjacency_preprocess_correctFacesEdgeIndexes(const MatrixX3i& vertex_indexes, I3s& F, I4s& TTT)
        {
            int correctedEdgeIndexesCount = 0;
            std::vector<bool> edgeHasFriend(TTT.rows(), false);
            Is Fshift = Is::Zero(F.rows());
            int vi_max = vertex_indexes.rows();
            const int* pvi = vertex_indexes.data();
            const int* pviEnd = vertex_indexes.data() + vertex_indexes.size();
            //for (int vi = 0; vi < vi_max; vi++)
            while (pvi < pviEnd)
            {
                //int count = vertex_indexes(vi, 0);
                //if (count == 0) continue;
                //int startTTTindex = vertex_indexes(vi, 1); //each row have {count, start index in TTT, writeIndex}
                int count = *(pvi + 0);
                int startTTTindex = *(pvi + 1);
                pvi += 3;
                if (count == 0) continue;

                int vid2 = TTT(startTTTindex + 0, 1); // each row of TTT will have{ vid1 vid2 fid eidlocal }
                for (int i2 = startTTTindex + 1; i2 < startTTTindex + count; i2++)
                {
                    int vid2_next = TTT(i2, 1);
                    if (vid2 == vid2_next)
                    {
                        edgeHasFriend[i2 - 1] = true;
                        edgeHasFriend[i2] = true;
                    }
                    vid2 = vid2_next;
                }
                for (int i = startTTTindex; i < startTTTindex + count; i++)
                {
                    if (edgeHasFriend[i]) continue;
                    int eidLocal = TTT(i, 3);
                    if (eidLocal == 0) continue;
                    int fid = TTT(i, 2);
                    int shift = 3 - eidLocal;
                    Fshift(fid) = shift;
                    correctedEdgeIndexesCount++;
                    //cout << "edge is border but not of index 0:  fid=" << fid << "  eidLocal=" << eidLocal << endl;
                }
            }


            if (correctedEdgeIndexesCount > 0)
            {
                for (int fid = 0; fid < Fshift.size(); fid++)
                {
                    int shift = Fshift(fid);
                    if (shift == 0) continue;
                    auto vids_old = F.row(fid);
                    for (int k = 0; k < 3; k++)
                    {
                        F(fid, k) = vids_old((k + 3 - shift) % 3);
                    }
                }
                for (int i = 0; i < TTT.rows(); i++)
                {
                    int fid = TTT(i, 2);
                    int shift = Fshift(fid);
                    if (shift == 0) continue;
                    int& eidLocal = TTT(i, 3);
                    eidLocal = (eidLocal + shift) % 3;
                }
            }
            //cout << "correctedEdgeIndexesCount=" << correctedEdgeIndexesCount << endl;
        }

        //same as igl::triangle_triangle_adjacency_preprocess but faster and using Matrix instead of vector that make even dealocation faster
        // each row of TTT will have { v1 v2 fi eilocal}
        void triangle_triangle_adjacency_preprocess_fast(I3s& F, const P3s& V, I4s& TTT, bool correctFacesEdgeIndexes)
        {
            I4s TTTunsorted;
            TTTunsorted.resize(F.rows() * 3, 4);
            int ei = 0;
            int vertexIndex1 = 0;
            int vertexIndex2 = 0;

            // build vertex occurances (this will help to add to TTTunsorted already sorted by vertexIndex1)
            MatrixX3i vertex_indexes;
            vertex_indexes.setZero(V.rows(), 3); //first value - how many times vertex counted, second value - start index in TTTunsorted, thrith value - current write index in TTTunsorted
            for (int fi = 0; fi < F.rows(); fi++)
            {
                ei = 0;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 1);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                vertex_indexes(vertexIndex1, 0) += 1; //increment counter of vertex occurance

                ei = 1;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 2);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                vertex_indexes(vertexIndex1, 0) += 1; //increment counter of vertex occurance

                ei = 2;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 0);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                vertex_indexes(vertexIndex1, 0) += 1; //increment counter of vertex occurance
            }

            // build vertex indexes in TTTunsorted (this will help to add to TTTunsorted already sorted by vertexIndex1)
            int currentTTTIndex = 0;
            for (int i = 0; i < vertex_indexes.rows(); i++)
            {
                int count = vertex_indexes(i, 0);
                if (count == 0) continue;
                vertex_indexes(i, 1) = currentTTTIndex; // start index in TTTunsorted
                vertex_indexes(i, 2) = currentTTTIndex; // current write index in TTTunsorted
                currentTTTIndex += count;
            }
            if (currentTTTIndex != TTTunsorted.rows())
            {
                cout << endl << endl << "!!! Error in utils::mesh::triangle_triangle_adjacency_preprocess_fast   - currentTTTIndex != TTTunsorted.rows()" << endl << endl << endl;
                assert(currentTTTIndex == TTTunsorted.rows());
            }

            // for each face edge contruct relation : vertexIndex1,vertexIndex2, face, edgeindex(0,1,2)
            // since we use 'vertex_indexes' we will add to 'TTTunsorted' in a partally sorted order - sorted by vertexIndex1 but no by vertexIndex2 - what we will do later in sort section
            int writeTTTIndex;
            for (int fi = 0; fi < F.rows(); fi++)
            {
                ei = 0;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 1);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                writeTTTIndex = vertex_indexes(vertexIndex1, 2); // get current write index in TTTunsorted
                TTTunsorted(writeTTTIndex, 0) = vertexIndex1;
                TTTunsorted(writeTTTIndex, 1) = vertexIndex2;
                TTTunsorted(writeTTTIndex, 2) = fi;
                TTTunsorted(writeTTTIndex, 3) = ei;
                vertex_indexes(vertexIndex1, 2) += 1;// increment write index, so next items will come after

                ei = 1;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 2);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                writeTTTIndex = vertex_indexes(vertexIndex1, 2); // get current write index in TTTunsorted
                TTTunsorted(writeTTTIndex, 0) = vertexIndex1;
                TTTunsorted(writeTTTIndex, 1) = vertexIndex2;
                TTTunsorted(writeTTTIndex, 2) = fi;
                TTTunsorted(writeTTTIndex, 3) = ei;
                vertex_indexes(vertexIndex1, 2) += 1;// increment write index, so next items will come after

                ei = 2;
                vertexIndex1 = F(fi, ei);
                vertexIndex2 = F(fi, 0);
                if (vertexIndex1 > vertexIndex2) std::swap(vertexIndex1, vertexIndex2);
                writeTTTIndex = vertex_indexes(vertexIndex1, 2); // get current write index in TTTunsorted
                TTTunsorted(writeTTTIndex, 0) = vertexIndex1;
                TTTunsorted(writeTTTIndex, 1) = vertexIndex2;
                TTTunsorted(writeTTTIndex, 2) = fi;
                TTTunsorted(writeTTTIndex, 3) = ei;
                vertex_indexes(vertexIndex1, 2) += 1;// increment write index, so next items will come after
            }

            // sort TTTunsorted by vertexIndex1, vertexIndex2
            // since we already have sorted by first value 'vertexIndex1' - we need to sort only by second, and thus we need su sort subsets
            // this is because add to TTTunsorted in such manner that 'vertexIndex1' will be already in order
            TTT.resize(TTTunsorted.rows(), TTTunsorted.cols());
            std::vector<pair<int, int>> sort_keyvalue;
            for (int vi = 0; vi < vertex_indexes.rows(); vi++)
            {
                int count = vertex_indexes(vi, 0);
                if (count == 0) continue;
                int startTTTindex = vertex_indexes(vi, 1);
                //int vertexIndex1 = TTTunsorted(startTTTindex, 0);
                // having 'startTTTindex' and 'count' - we need to sort subset by 'vertexIndex2'
                if (sort_keyvalue.size() < count)sort_keyvalue.resize(count);
                for (int i = 0; i < count; i++)
                {
                    int ttindex = startTTTindex + i;
                    sort_keyvalue[i] = { TTTunsorted(ttindex, 1), ttindex };
                }
                sort(sort_keyvalue.begin(), sort_keyvalue.begin() + count);

                //Vector4i tttuns1 = TTTunsorted.row(startTTTindex + 0);
                //Vector4i tttuns2 = TTTunsorted.row(startTTTindex + 1);
                //Vector4i tttuns3 = TTTunsorted.row(startTTTindex + 2);
                //Vector4i tttuns4 = TTTunsorted.row(startTTTindex + 3);
                //Vector4i tttuns5 = TTTunsorted.row(startTTTindex + 4);
                //Vector4i tttuns6 = TTTunsorted.row(startTTTindex + 5);
                for (int subi = 0; subi < count; subi++)
                {
                    int tttIndexUnsorted = startTTTindex + subi;
                    int tttIndexSorted = sort_keyvalue[subi].second;
                    TTT.row(tttIndexUnsorted) = TTTunsorted.row(tttIndexSorted);
                }
            }


            //correctFacesEdgeIndexes
            if (correctFacesEdgeIndexes) triangle_triangle_adjacency_preprocess_correctFacesEdgeIndexes(vertex_indexes, F, TTT);
        }

        inline void minmax(float& a, float& b)
        {
            auto asse = _mm_set_ss(a);
            auto bsse = _mm_set_ss(b);
            _mm_store_ss(&a, _mm_min_ss(asse, bsse));
            _mm_store_ss(&b, _mm_max_ss(asse, bsse));
        }
        inline void minmax(int& a, int& b)
        {
            auto asse = _mm_set1_epi32(a);
            auto bsse = _mm_set1_epi32(b);
            a = _mm_min_epi32(asse, bsse).m128i_i32[0];
            b = _mm_max_epi32(asse, bsse).m128i_i32[0];
        }

        // same as 'triangle_triangle_adjacency_preprocess_fast' but faster since it uses SSE commands 
        // same as igl::triangle_triangle_adjacency_preprocess but faster and using Matrix instead of vector that make even dealocation faster
        // each row of TTT will have { v1 v2 fi eilocal}
        void triangle_triangle_adjacency_preprocess_sse(I3s& F, const P3s& V, I4s& TTT, bool correctFacesEdgeIndexes)
        {
            TTT.resize(F.rows() * 3, 4);


            // build vertex occurances (this will help to add to TTTunsorted already sorted by vertexIndex1)
            MatrixX3i vertex_indexes;//first value - how many times vertex counted, second value - start index in TTTunsorted, thrith value - current write index in TTTunsorted
            //auto inc_counts_vertex_indexes = [&vertex_indexes](int vid0, int vid1)
            //{
            //    //v0
            //    //if (vid0 > vid1) std::swap(vid0, vid1);
            //    //vertex_indexes(vid0, 0) += 1; //increment counter of vertex occurance

            //    //v1
            //    __m128i vids;
            //    vids.m128i_i32[0] = vid0;
            //    vids.m128i_i32[1] = vid1;
            //    utils::sse::sort2ints(vids);
            //    vertex_indexes(vids.m128i_i32[0], 0) += 1; //increment counter of vertex occurance
            //};
            vertex_indexes.setZero(V.rows(), 3);
            const int* pfid = F.data();
            const int* pfid_end = F.data() + F.size();
            while (pfid < pfid_end)
            {
                int vid0 = *(pfid + 0);
                int vid1 = *(pfid + 1);
                int vid2 = *(pfid + 2);

                //v0,v1
                //inc_counts_vertex_indexes(vid0, vid1);
                //inc_counts_vertex_indexes(vid1, vid2);
                //inc_counts_vertex_indexes(vid2, vid0);

                //v2
                __m128i vids;
                vids.m128i_i32[0] = vid0;
                vids.m128i_i32[1] = vid1;
                vids.m128i_i32[2] = vid2;
                utils::sse::_4::sort3ints(vids);
                vertex_indexes(vids.m128i_i32[0], 0) += 2; //increment counter of vertex occurance: one vertex will be lowest from others 2
                vertex_indexes(vids.m128i_i32[1], 0) += 1; //increment counter of vertex occurance: one vertex will be lower from 1 and higher from 1
                pfid += 3;

                //v3 - since we need only 

            }

            // build vertex indexes in TTTunsorted (this will help to add to TTTunsorted already sorted by vertexIndex1)
            int currentTTTIndex = 0;
            for (int i = 0; i < vertex_indexes.rows(); i++)
            {
                int count = vertex_indexes(i, 0);
                if (count == 0) continue;
                vertex_indexes(i, 1) = currentTTTIndex; // start index in TTTunsorted
                vertex_indexes(i, 2) = currentTTTIndex; // current write index in TTTunsorted
                currentTTTIndex += count;
            }
            if (currentTTTIndex != TTT.rows())
            {
                cout << endl << endl << "!!! Error in utils::mesh::triangle_triangle_adjacency_preprocess_fast   - currentTTTIndex != TTTunsorted.rows()" << endl << endl << endl;
                assert(currentTTTIndex == TTT.rows());
            }




            // clear TTT - just for debuging - we dont need to clear buffer since all data will be populated
            //TTT.setConstant(F.rows() * 3, 4, -1);
            //__m128i all_minus_1 = _mm_set1_epi32(-1);
            //__m128i* pttt = (__m128i*)TTT.data();
            //__m128i* ptttEnd = (__m128i*)(TTT.data() + TTT.size());
            //__m128i* ptttEnd4 = ptttEnd - 8;
            //while (pttt < ptttEnd4)
            //{
            //    _mm_storeu_si128(pttt, all_minus_1);
            //    _mm_storeu_si128(pttt + 1, all_minus_1);
            //    _mm_storeu_si128(pttt + 2, all_minus_1);
            //    _mm_storeu_si128(pttt + 3, all_minus_1);
            //    _mm_storeu_si128(pttt + 4, all_minus_1);
            //    _mm_storeu_si128(pttt + 5, all_minus_1);
            //    _mm_storeu_si128(pttt + 6, all_minus_1);
            //    _mm_storeu_si128(pttt + 7, all_minus_1);
            //    pttt += 8;
            //}
            //while (pttt < ptttEnd)
            //{
            //    _mm_storeu_si128(pttt, all_minus_1);
            //    pttt++;
            //}

            // for each face edge contruct relation : vertexIndex1,vertexIndex2, face, edgeindex(0,1,2)
            // since we use 'vertex_indexes' we will add to 'TTTunsorted' in a partally sorted order - sorted by vertexIndex1 but no by vertexIndex2 - what we will do later in sort section
            //
            // v0 - simple
            //
            auto TTTunsorted_pushback = [&TTT, &vertex_indexes](int vid0, int vid1, int fid, int eidk)
            {
                //v0
                if (vid0 > vid1) std::swap(vid0, vid1);
                int& writeTTTIndex = vertex_indexes(vid0, 2); // get current write index in TTTunsorted
                TTT(writeTTTIndex, 0) = vid0;
                TTT(writeTTTIndex, 1) = vid1;
                TTT(writeTTTIndex, 2) = fid;
                TTT(writeTTTIndex, 3) = eidk;
                //                TTTunsorted.row(writeTTTIndex) = Vector4i(vid0, vid1, fid, eid);
                writeTTTIndex++;// increment write index, so next items will come after

                //v1
                //__m128i vids;
                //vids.m128i_i32[0] = vid0;
                //vids.m128i_i32[1] = vid1;
                //utils::sse::sort2ints(vids);
                //int& writeTTTIndex = vertex_indexes(vids.m128i_i32[0], 2);
                //vids.m128i_i32[2] = fid;
                //vids.m128i_i32[3] = eid;
                //int* pwrite = TTTunsorted.data() + writeTTTIndex*4;
                //writeTTTIndex++;// increment write index, so next items will come after
                //_mm_storeu_si128(reinterpret_cast<__m128i*>(pwrite), vids);
            };
            int fid_next = 0;
            pfid = F.data();
            while (pfid < pfid_end)
            {
                int vid0 = *(pfid + 0);
                int vid1 = *(pfid + 1);
                int vid2 = *(pfid + 2);
                TTTunsorted_pushback(vid0, vid1, fid_next, 0);
                TTTunsorted_pushback(vid1, vid2, fid_next, 1);
                TTTunsorted_pushback(vid2, vid0, fid_next, 2);
                pfid += 3;
                fid_next++;
            }

            //
            // v1 - sse - same speed :(
            // 
            //__m128i fast_fid_next = _mm_set1_epi32(0);
            //__m128i fast_fid_next_inc = _mm_set_epi32(0, 1, 0, 0);
            //__m128i fast_eidk0 = _mm_set_epi32(0, 0, 0, 0);
            //__m128i fast_eidk1 = _mm_set_epi32(1, 0, 0, 0);
            //__m128i fast_eidk2 = _mm_set_epi32(2, 0, 0, 0);
            //pfid = F.data();
            //while (pfid < pfid_end)
            //{
            //    __m128i vids = _mm_loadu_si128((__m128i const*)pfid); // read all vids at once
            //    vids.m128i_i32[3] = vids.m128i_i32[0]; // set last 4-th vid as vid0 - to be able to compare it with vid2
            //    // now vids have values: (vid0, vid1, vid2, vid0)
            //    // sort 2 pair indexes at once: (vid0, vid1) and (vid2, vid0)
            //    __m128i vids_sorted_01_20 = vids;
            //    utils::sse::_4::sort2ints(vids_sorted_01_20);
            //    // sort 1 pair (vid1, vid2)
            //    __m128i vids_sorted_12 = _mm_bsrli_si128(vids, 4);
            //    utils::sse::_4::sort2ints(vids_sorted_12);

            //    __m128i vids_sorted_01 = _mm_move_epi64(vids_sorted_01_20); //  make (vid0, vid1, 0, 0)
            //    vids_sorted_12 = _mm_move_epi64(vids_sorted_12); //  make (vid1, vid2, 0, 0)
            //    __m128i vids_sorted_20 = _mm_bsrli_si128(vids_sorted_01_20, 8); //  make (vid2, vid0, 0, 0)

            //    int& writeTTTIndex0 = vertex_indexes(vids_sorted_01.m128i_i32[0], 2);
            //    int* write0 = TTT.data() + writeTTTIndex0 * 4;
            //    writeTTTIndex0++;
            //    int& writeTTTIndex1 = vertex_indexes(vids_sorted_12.m128i_i32[0], 2);
            //    int* write1 = TTT.data() + writeTTTIndex1 * 4;
            //    writeTTTIndex1++;
            //    int& writeTTTIndex2 = vertex_indexes(vids_sorted_20.m128i_i32[0], 2);
            //    int* write2 = TTT.data() + writeTTTIndex2 * 4;
            //    writeTTTIndex2++;

            //    __m128i val0 = _mm_add_epi32(vids_sorted_01, fast_fid_next); // no need to add fast_eidk1 since its all value zero
            //    _mm_storeu_si128((__m128i*)(write0), val0);
            //    __m128i val1 = _mm_add_epi32(_mm_add_epi32(vids_sorted_12, fast_fid_next), fast_eidk1);
            //    _mm_storeu_si128((__m128i*)(write1), val1);
            //    __m128i val2 = _mm_add_epi32(_mm_add_epi32(vids_sorted_20, fast_fid_next), fast_eidk2);
            //    _mm_storeu_si128((__m128i*)(write2), val2);

            //    pfid += 3;
            //    fast_fid_next = _mm_add_epi32(fast_fid_next, fast_fid_next_inc);
            //}

            // sort TTTunsorted by vertexIndex1, vertexIndex2
            // since we already have sorted by first value 'vertexIndex1' - we need to sort only by second, and thus we need su sort subsets
            // this is because add to TTTunsorted in such manner that 'vertexIndex1' will be already in order

            std::vector<pair<int, int>> sort_keyvalue;
            //int maxCount = 0;
            Vec4i*  p_TTT_data = (Vec4i*)TTT.data(); //  1 row have 4 ints what is exactly 128 bits
            for (int vi = 0; vi < vertex_indexes.rows(); vi++)
            {
                int count = vertex_indexes(vi, 0);
                if (count == 0) continue;
                int startTTTindex = vertex_indexes(vi, 1);

                // having 'startTTTindex' and 'count' - we need to sort subset by 'vertexIndex2'

                // v0 - sorting
                //if (sort_keyvalue.size() < count)sort_keyvalue.resize(count);
                //for (int i = 0; i < count; i++)
                //{
                //    sort_keyvalue[i] = { TTT(startTTTindex + i, 1), i }; // vertexIndex2, index local (from 0 to count of vertexIndex2 for vertexIndex1)
                //}
                //sort(sort_keyvalue.begin(), sort_keyvalue.begin() + count);
                //for (int subi = 0; subi < count; subi++)
                //{
                //    int tttIndexUnsorted = startTTTindex + subi;
                //    int tttIndexSorted = startTTTindex + sort_keyvalue[subi].second;
                //    TTT.row(tttIndexUnsorted) = TTT.row(tttIndexSorted);
                //}

                // v1 - copy inplace - 2x faster 
                Vec4i* p1 = p_TTT_data + startTTTindex;
                Vec4i* pEnd = p_TTT_data + startTTTindex + count;
                while (p1 < pEnd)
                {
                    int vid1 = (*p1)[1];
                    Vec4i* p2 = p1 + 1;
                    while (p2 < pEnd)
                    {
                        int vid2 = (*p2)[1];
                        if (vid1 == vid2)
                        {
                            p1++;
                            swap(*p1, *p2);
                        }
                        p2++;
                    }

                    p1++;
                }
            }

            //correctFacesEdgeIndexes
            if (correctFacesEdgeIndexes) triangle_triangle_adjacency_preprocess_correctFacesEdgeIndexes(vertex_indexes, F, TTT);
        }

        template <int VERTEX_INDEXES_SIZE>
        void triangle_triangle_adjacency_preprocess_correctFacesEdgeIndexes_fast(const Matrix<int, Dynamic, VERTEX_INDEXES_SIZE> & vertex_indexes, I3s& F, I4s& TTT)
        {
            int correctedEdgeIndexesCount = 0;
            Bs edgeHasFriend = Bs::Constant(TTT.rows(), false);
            Is Fshift = Is::Zero(F.rows());
            int vi_max = vertex_indexes.rows();
            const int* pvi = vertex_indexes.data();
            const int* pviEnd = vertex_indexes.data() + vertex_indexes.size();
            //for (int vi = 0; vi < vi_max; vi++)
            while (pvi < pviEnd)
            {
                //int count = vertex_indexes(vi, 0);
                //if (count == 0) continue;
                //int startTTTindex = vertex_indexes(vi, 1); //each row have {count, start index in TTT, writeIndex}
                int count = *(pvi + 0);
                int startTTTindex = *(pvi + 1);
                pvi += VERTEX_INDEXES_SIZE;
                if (count == 0) continue;

                int vid2 = TTT(startTTTindex + 0, 1); // each row of TTT will have{ vid1 vid2 fid eidlocal }
                for (int i2 = startTTTindex + 1; i2 < startTTTindex + count; i2++)
                {
                    int vid2_next = TTT(i2, 1);
                    if (vid2 == vid2_next)
                    {
                        edgeHasFriend[i2 - 1] = true;
                        edgeHasFriend[i2] = true;
                    }
                    vid2 = vid2_next;
                }
                for (int i = startTTTindex; i < startTTTindex + count; i++)
                {
                    if (edgeHasFriend[i]) continue;
                    int eidLocal = TTT(i, 3);
                    if (eidLocal == 0) continue;
                    int fid = TTT(i, 2);
                    int shift = 3 - eidLocal;
                    Fshift(fid) = shift;
                    correctedEdgeIndexesCount++;
                    //cout << "edge is border but not of index 0:  fid=" << fid << "  eidLocal=" << eidLocal << endl;
                }
            }


            if (correctedEdgeIndexesCount > 0)
            {
                for (int fid = 0; fid < Fshift.size(); fid++)
                {
                    int shift = Fshift(fid);
                    if (shift == 0) continue;
                    I3 vids_old = F.row(fid);
                    for (int k = 0; k < 3; k++)
                    {
                        F(fid, k) = vids_old((k + 3 - shift) % 3);
                    }
                }
                for (int i = 0; i < TTT.rows(); i++)
                {
                    int fid = TTT(i, 2);
                    int shift = Fshift(fid);
                    if (shift == 0) continue;
                    int& eidLocal = TTT(i, 3);
                    eidLocal = (eidLocal + shift) % 3;
                }
            }
            //cout << "correctedEdgeIndexesCount=" << correctedEdgeIndexesCount << endl;
        }

        // same as 'triangle_triangle_adjacency_preprocess_sse' but faster 
        // same as igl::triangle_triangle_adjacency_preprocess but faster and using Matrix instead of vector that make even dealocation faster
        // each row of TTT will have { v1 v2 fi eilocal}
        void triangle_triangle_adjacency_preprocess_sse4(I3s& F, const P3s& V, I4s& TTT, bool correctFacesEdgeIndexes)
        {
            I4s TTTunsorted;
            TTTunsorted.resize(F.rows() * 3, 4);

            // build vertex occurances (this will help to add to TTTunsorted already sorted by vertexIndex1)
            MatrixX2i vertex_indexes;//first value - how many times vertex counted, second value - start index in TTTunsorted
            Is vertex_indexes_writeIndex;// current write index in TTTunsorted
            vertex_indexes.setZero(V.rows(), 2);
            vertex_indexes_writeIndex.setZero(V.rows());
            const int* pfid = F.data();
            const int* pfid_end = F.data() + F.size();
            while (pfid < pfid_end)
            {
                __m128i vids = _mm_load_si128((const __m128i*)pfid);
                utils::sse::_4::sort3ints(vids);
                vertex_indexes(vids.m128i_i32[0], 0) += 2; //increment counter of vertex occurance: one vertex will be lowest from others 2
                vertex_indexes(vids.m128i_i32[1], 0) += 1; //increment counter of vertex occurance: one vertex will be lower from 1 and higher from 1
                pfid += 3;
            }

            // build vertex indexes in TTTunsorted (this will help to add to TTTunsorted already sorted by vertexIndex1)
            int currentTTTIndex = 0;
            for (int i = 0; i < vertex_indexes.rows(); i++)
            {
                int count = vertex_indexes(i, 0);
                if (count == 0) continue;
                vertex_indexes(i, 1) = currentTTTIndex; // start index in TTTunsorted
                vertex_indexes_writeIndex(i) = currentTTTIndex; // current write index in TTTunsorted
                currentTTTIndex += count;
            }
            if (currentTTTIndex != TTTunsorted.rows())
            {
                cout << endl << endl << "!!! Error in utils::mesh::triangle_triangle_adjacency_preprocess_fast   - currentTTTIndex != TTTunsorted.rows()" << endl << endl << endl;
                assert(currentTTTIndex == TTTunsorted.rows());
            }

            // for each face edge contruct relation : vertexIndex1,vertexIndex2, face, edgeindex(0,1,2)
            // since we use 'vertex_indexes' we will add to 'TTTunsorted' in a partally sorted order - sorted by vertexIndex1 but no by vertexIndex2 - what we will do later in sort section
            auto TTTunsorted_pushback = [&TTTunsorted, &vertex_indexes_writeIndex](__m128i vid0_vid1, int fid, int eid)
            {
                int vid_low = vid0_vid1.m128i_i32[0];
                int& writeTTTIndex = vertex_indexes_writeIndex(vid_low); // get current write index in TTTunsorted
                vid0_vid1.m128i_i32[2] = fid;
                vid0_vid1.m128i_i32[3] = eid;
                _mm_storeu_si128((__m128i*)(TTTunsorted.data() + writeTTTIndex * 4), vid0_vid1);
                writeTTTIndex++;// increment write index, so next items will come after
            };
            int fid_next = 0;
            pfid = F.data();
            while (pfid < pfid_end)
            {
                __m128i vids = _mm_load_si128((const __m128i*)pfid);
                TTTunsorted_pushback(utils::sse::_4::take2sortedInts<0, 1>(vids), fid_next, 0);
                TTTunsorted_pushback(utils::sse::_4::take2sortedInts<1, 2>(vids), fid_next, 1);
                TTTunsorted_pushback(utils::sse::_4::take2sortedInts<2, 0>(vids), fid_next, 2);

                pfid += 3;
                fid_next++;
            }


            // sort TTTunsorted by vertexIndex1, vertexIndex2
            // since we already have sorted by first value 'vertexIndex1' - we need to sort only by second, and thus we need su sort subsets
            // this is because add to TTTunsorted in such manner that 'vertexIndex1' will be already in order
            TTT.resize(TTTunsorted.rows(), TTTunsorted.cols());
            int maxVCount = 0;
            for (int vi = 0; vi < vertex_indexes.rows(); vi++)
            {
                int count = vertex_indexes(vi, 0);
                maxVCount = count > maxVCount ? count : maxVCount;
            }
            //std::vector<int> counts(32, 0);
            if (maxVCount < 33)
            {
                //StaticSort<2> ss2;
                //StaticSort<3> ss3;
                //StaticSort<4> ss4;
                //StaticSort<5> ss5;
                //StaticSort<6> ss6;
                //StaticSort<7> ss7;
                //StaticSort<8> ss8;
                //StaticSort<9> ss9;
                //StaticSort<10> ss10;
                // this code will cover 99.999 percent of all meshes
                pair<int, int> sort_keyvalue[32];
                for (int vi = 0; vi < vertex_indexes.rows(); vi++)
                {
                    int count = vertex_indexes(vi, 0);
                    //counts[count]++;
                    if (count == 0) continue;
                    int startTTTindex = vertex_indexes(vi, 1);
                    //int vertexIndex1 = TTTunsorted(startTTTindex, 0);
                    // having 'startTTTindex' and 'count' - we need to sort subset by 'vertexIndex2'
                    for (int i = 0; i < count; i++)
                    {
                        int vid2 = TTTunsorted(startTTTindex + i, 1);
                        sort_keyvalue[i] = { vid2, i }; // vertexIndex2, index local (from 0 to count of vertexIndex2 for vertexIndex1)
                    }

                    //if (count == 4)
                    //    ss4(sort_keyvalue);
                    //else if (count == 6)
                    //    ss6(sort_keyvalue);
                    ////else if (count == 3)
                    ////    ss3(sort_keyvalue);
                    //else if (count == 5)
                    //    ss5(sort_keyvalue);
                    ////else if (count == 2)
                    ////    ss2(sort_keyvalue);
                    ////else if (count == 7)
                    ////    ss7(sort_keyvalue);
                    //else if (count == 8)
                    //    ss8(sort_keyvalue);
                    ////else if (count == 9)
                    ////    ss9(sort_keyvalue);
                    ////else if (count == 10)
                    ////    ss10(sort_keyvalue);
                    //else 
                    if (count > 1)
                        sort(sort_keyvalue, sort_keyvalue + count);


                    for (int subi = 0; subi < count; subi++)
                    {
                        int tttIndexUnsorted = startTTTindex + subi;
                        int tttIndexSorted = startTTTindex + sort_keyvalue[subi].second;
                        TTT.row(tttIndexUnsorted) = TTTunsorted.row(tttIndexSorted);
                    }
                }
            }
            else
            {
                // this code will probably never run, but we have to make shure that program will never crashe event for the very uncommon cases
                std::vector<pair<int, int>> sort_keyvalue;
                for (int vi = 0; vi < vertex_indexes.rows(); vi++)
                {
                    int count = vertex_indexes(vi, 0);
                    if (count == 0) continue;
                    int startTTTindex = vertex_indexes(vi, 1);
                    //int vertexIndex1 = TTTunsorted(startTTTindex, 0);
                    // having 'startTTTindex' and 'count' - we need to sort subset by 'vertexIndex2'
                    if (sort_keyvalue.size() < count)sort_keyvalue.resize(count);
                    for (int i = 0; i < count; i++)
                    {
                        sort_keyvalue[i] = { TTTunsorted(startTTTindex + i, 1), i }; // vertexIndex2, index local (from 0 to count of vertexIndex2 for vertexIndex1)
                    }
                    sort(sort_keyvalue.begin(), sort_keyvalue.begin() + count);
                    for (int subi = 0; subi < count; subi++)
                    {
                        int tttIndexUnsorted = startTTTindex + subi;
                        int tttIndexSorted = startTTTindex + sort_keyvalue[subi].second;
                        TTT.row(tttIndexUnsorted) = TTTunsorted.row(tttIndexSorted);
                    }
                }
            }


            //cout << "maxVCount = "<< maxVCount<<"     counts[1..12] =    {"  << counts[1]<<", "<< counts[2] << ", " << counts[3] << ", " << counts[4] << ", " << counts[5] << ", " << counts[6] << ", " << counts[7] << ", " << counts[8] << ", " << counts[9] << ", " << counts[10] << ", " << counts[11] << ", " << counts[12] << "} " << endl;

            //correctFacesEdgeIndexes
            if (correctFacesEdgeIndexes) triangle_triangle_adjacency_preprocess_correctFacesEdgeIndexes_fast<2>(vertex_indexes, F, TTT);
        }

        //same as igl::is_edge_manifold but using TTT as MatrixX4i which is faster
        int get_manifold_edges_count(const I3s& F, const I4s& TTT)
        {
            int count = 0;
            for (int i = 2; i < TTT.rows(); ++i)
            {
                // Check any edges occur 3 times
                const I4& r1 = TTT.row(i - 2);
                const I4& r2 = TTT.row(i - 1);
                const I4& r3 = TTT.row(i);
                if ((r1[0] == r2[0] && r2[0] == r3[0])
                    &&
                    (r1[1] == r2[1] && r2[1] == r3[1]))
                {
                    cout << "warning:  manifold edge found  " << r1[0] << "," << r1[1] << endl;
                    count++;
                }
            }
            if (count != 0)
            {
                cout << "warning:  Found " << count << "manifold edges. Please check your input mesh." << endl;
            }
            return count;
        }

        //remove faces with manifold edges - our algorithms assume that mesh in non-manifold
        void remove_manifold_edges(I3s& F, I4s& TTT, int& removedFacesCount)
        {
            removedFacesCount = 0;
            int manifoldEdgesCount = 0;
            if (TTT.rows() < 3)
            {
                return;
            }
            //TTT has values:   vertexIndexStart, vertexIndexEnd, face, edgeIndexInFace
            Bs deletedF; // stored deleted faces because they have manifold edge - only 2 faces allowed to have same edge - 3-th one will be deleted
            int prevprev0 = TTT(0, 0);
            int prevprev1 = TTT(0, 1);
            int prev0 = TTT(1, 0);
            int prev1 = TTT(1, 1);
            for (int i = 2; i < TTT.rows(); ++i)
            {
                int current0 = TTT(i, 0);
                int current1 = TTT(i, 1);
                // Check any edges occur 3 times
                if ((prevprev0 == prev0 && prev0 == current0)
                    &&
                    (prevprev1 == prev1 && prev1 == current1))
                {
                    // create only if we need in this
                    if (deletedF.size() == 0)
                    {
                        deletedF = Bs::Constant(F.rows(), false);
                    }
                    manifoldEdgesCount++;
                    int fid = TTT(i, 2);
                    deletedF(fid) = true;
                    //cout << "warning:  manifold edge found  " << r1[0] << "," << r1[1] << endl;
                }
                prevprev0 = prev0;
                prevprev1 = prev1;
                prev0 = current0;
                prev1 = current1;
            }

            //delete if found
            if (manifoldEdgesCount != 0)
            {
                #if DEBUG
                cout << endl << "!!! warning:  Found " << manifoldEdgesCount << " manifold edges. Please check your input mesh." << endl;
                #endif
                //cout << "!!! warning:  Removing manifold edges and faces ...";
                // remove from F manifold faces
                int removedCount = 0;
                Is f_new_indexes;
                f_new_indexes.resize(F.rows());
                for (int i = 0; i < F.rows(); ++i)
                {
                    if (deletedF(i))
                    {
                        removedCount++;
                    }
                    else
                    {
                        F(i - removedCount, 0) = F(i, 0);
                        F(i - removedCount, 1) = F(i, 1);
                        F(i - removedCount, 2) = F(i, 2);
                    }
                    f_new_indexes(i) = i - removedCount;
                }
                F.conservativeResize(F.rows() - removedCount, 3);
                removedFacesCount = removedCount;

                // remove from TTT manifold faces
                removedCount = 0;
                for (int i = 0; i < TTT.rows(); ++i)
                {
                    int fid = TTT(i, 2);
                    if (deletedF(fid))
                    {
                        removedCount++;
                    }
                    else
                    {
                        TTT(i - removedCount, 0) = TTT(i, 0);
                        TTT(i - removedCount, 1) = TTT(i, 1);
                        TTT(i - removedCount, 2) = f_new_indexes(fid);
                        TTT(i - removedCount, 3) = TTT(i, 3);
                    }
                }
                TTT.conservativeResize(TTT.rows() - removedCount, 4);
                //cout << "done." << endl;
                #if DEBUG
                cout << "!!! warning:  Deleted " << removedFacesCount << " manifold faces. Please check your input mesh." << endl;
                #endif
            }
        }


        //same as igl::edge_topology but using TTT as MatrixX4i which is faster
        void edge_topology(const P3s& V, const I3s& F, const I4s& TTT, I3s& FE, I2s& EF, Bs& E_isborder,
            I2s& EV, I2s& EFi, bool calculate_EV_EFi)
        {
            // Only needs to be edge-manifold
            if (V.rows() == 0 || F.rows() == 0)
            {
                FE = I3s::Constant(0, 3, -1);
                EF = I2s::Constant(0, 2, -1);
                if (calculate_EV_EFi)
                {
                    EV = I2s::Constant(0, 2, -1);
                }
                return;
            }

            // count the number of edges (assume manifoldness)
            //v0 - simple
            //int Ensimple = 1; // the last is always counted
            //for (int i = 0; i < int(TTT.rows()) - 1; ++i)
            //    if (!((TTT(i, 0) == TTT(i + 1, 0)) && (TTT(i, 1) == TTT(i + 1, 1))))
            //        Ensimple++;
            //v1 - sse
            __m128i* pttt = (__m128i*)TTT.data();
            __m128i* ptttEnd = pttt + TTT.rows();
            __m128i* ptttEnd4 = ptttEnd - 5;
            __m128i ttti0 = *pttt;
            pttt++;
            int En = 1;
            //while (pttt < ptttEnd4)
            //{
            //    __m128i ttti1 = *(pttt);
            //    __m128i ttti2 = *(pttt + 1);
            //    __m128i ttti3 = *(pttt + 2);
            //    __m128i ttti4 = *(pttt + 3);
            //    __m128i eq01 = _mm_cmpeq_epi64(ttti0, ttti1);
            //    __m128i eq12 = _mm_cmpeq_epi64(ttti1, ttti2);
            //    __m128i eq23 = _mm_cmpeq_epi64(ttti2, ttti3);
            //    __m128i eq34 = _mm_cmpeq_epi64(ttti3, ttti4);
            //    ttti0 = ttti4;
            //    int c0 = !(eq01.m128i_i32[0]) & 1;
            //    int c1 = !(eq12.m128i_i32[0]) & 1;
            //    int c2 = !(eq23.m128i_i32[0]) & 1;
            //    int c3 = !(eq34.m128i_i32[0]) & 1;
            //    En += c0+c1+c2+c3;
            //    pttt += 4;
            //}
            while (pttt < ptttEnd)
            {
                __m128i ttti1 = *pttt;
                __m128i eq = _mm_cmpeq_epi64(ttti0, ttti1);
                ttti0 = ttti1;
                int c = !(eq.m128i_i32[0]) & 1;
                En += c;
                pttt++;
            }
            //assert(Ensimple == En);

            EF.resize(En, 2);
            E_isborder.resize(En);
            FE.resize(F.rows(), 3);
            if (calculate_EV_EFi)
            {
                EV.resize(En, 2);
                EFi.resize(En, 2);
            }
            En = 0;

            int TTT_rows_minus_1 = TTT.rows() - 1;

            for (int i = 0; i < TTT.rows(); ++i)
            {
                const I4 r1 = TTT.row(i); //vertexIndexStart, vertexIndexEnd, fid, edgeIndexInFace
                const I4 r2 = (i != TTT_rows_minus_1) ? TTT.row(i + 1) : r1;
                bool isNotBorder = (i != TTT_rows_minus_1) && ((r1(0) == r2(0)) && (r1(1) == r2(1)));

                if (isNotBorder)
                {
                    E_isborder(En) = false;
                    EF(En, 0) = r1[2];//fid
                    EF(En, 1) = r2[2];//fid
                    FE(r1[2], r1[3]) = En;
                    FE(r2[2], r2[3]) = En;
                }
                else
                {
                    // Border edge
                    E_isborder(En) = true;
                    EF(En, 0) = r1[2]; //fid
                    EF(En, 1) = -1;
                    FE(r1[2], r1[3]) = En; //edgeIndexInFace
                }

                if (calculate_EV_EFi)
                {
                    if (isNotBorder)
                    {
                        EV(En, 0) = r1[0];
                        EV(En, 1) = r1[1];
                        EFi(En, 0) = r1[3];//edgeIndexInFace
                        EFi(En, 1) = r2[3];//edgeIndexInFace
                    }
                    else
                    {
                        EV(En, 0) = r1[0]; //vertexIndexStart
                        EV(En, 1) = r1[1]; //vertexIndexEnd
                        EFi(En, 0) = r1[3]; //edgeIndexInFace
                        EFi(En, 1) = -1;
                    }
                }

                if (isNotBorder) i++; // skip the next one
                En++;
            }


            if (calculate_EV_EFi)
            {
                // Sort the relation EF, accordingly to EV
                // the first one is the face on the left of the edge
                for (int i = 0; i < EF.rows(); ++i)
                {
                    //v1
                    // search for edge EV.row(i)
                    //int fid = EF(i, 0);
                    //bool flip = true;
                    //for (int j = 0; j < 3; ++j)
                    //{
                    //    if ((F(fid, j) == EV(i, 0)) && (F(fid, (j + 1) % 3) == EV(i, 1)))
                    //        flip = false;
                    //}
                    //v2
                    int e_v0 = EV(i, 0);
                    int e_v1 = EV(i, 1);
                    int fid1 = EF(i, 0);
                    int v0 = F(fid1, 0);
                    int v1 = F(fid1, 1);
                    if (v0 == e_v0 && v1 == e_v1) continue;
                    int v2 = F(fid1, 2);
                    if (v1 == e_v0 && v2 == e_v1)continue;
                    if (v2 == e_v0 && v0 == e_v1)continue;
                    //int tmp = EF(i, 0);
                    //EF(i, 0) = EF(i, 1);
                    //EF(i, 1) = tmp;
                    if (EF(i, 1) == -1) // if there is only one face contacted with edge, we have to swap edge directoin, in order to guarantee that first face is on the left side of the edge
                    {
                        swap(EV(i, 0), EV(i, 1));
                    }
                    else
                    {
                        swap(EF(i, 0), EF(i, 1));
                        swap(EFi(i, 0), EFi(i, 1));
                    }
                    assert(EF(i, 0) != -1);
                }
            }
        }

        //same as igl::triangle_triangle_adjacency but using TTT as MatrixX4i which is faster
        void triangle_triangle_adjacency(const I3s& F, const I4s& TTT,
            I3s& FF, I3s& FFi)
        {
            FF.setConstant(F.rows(), F.cols(), -1);
            FFi.setConstant(F.rows(), F.cols(), -1);
            for (int i = 1; i < TTT.rows(); ++i)
            {
                const I4&  r1 = TTT.row(i - 1);
                const I4&  r2 = TTT.row(i);
                if ((r1[0] == r2[0]) && (r1[1] == r2[1]))
                {
                    // FF
                    FF(r1[2], r1[3]) = r2[2];
                    FF(r2[2], r2[3]) = r1[2];

                    // FFi
                    FFi(r1[2], r1[3]) = r2[3];
                    FFi(r2[2], r2[3]) = r1[3];
                }
            }
        }

        // same as igl::barycenter
        void barycenter_fast(const P3s& V, const I3s& F, P3s& BC)
        {
            BC.setZero(F.rows(), 3);
            // Loop over faces
            int count = F.rows();
            for (int i = 0; i < count; i++)
            {
                // loop around face
                auto viii = F.row(i);
                P3 v0 = V.row(viii(0));
                P3 v1 = V.row(viii(1));
                P3 v2 = V.row(viii(2));
                BC.row(i) = (v0 + v1 + v2) / D(3);
            }
        }

        void vertex_triangle_adjacency_old(const P3s& V, const I3s& F, CompactVectorVector<int>& VF, CompactVectorVector<int>& VFi)
        {
            // resize
            int n = V.rows();

            // set capacity for each individual vertex-vector
            //cout << endl << endl << " --- vertex_triangle_adjacency --- " << endl << endl;
            //cout <<  " --- get counts " << endl;
            VF.resizeBegin(n);
            for (int fid = 0; fid < F.rows(); ++fid)
            {
                int vid0 = F(fid, 0);
                int vid1 = F(fid, 1);
                int vid2 = F(fid, 2);
                VF.size(vid0)++;
                VF.size(vid1)++;
                VF.size(vid2)++;
                //if (vid0 == 3) cout << "vid = 3, fid = " << fid << "   size = " << VF.size(vid0) << endl;
                //if (vid1 == 3) cout << "vid = 3, fid = " << fid << "   size = " << VF.size(vid1) << endl;
                //if (vid2 == 3) cout << "vid = 3, fid = " << fid << "   size = " << VF.size(vid2) << endl;
            }
            VFi.resizeBegin(n, true);
            for (int i = 0; i < n; i++)
            {
                VFi.size(i) = VF.size(i);
            }
            VF.resizeEnd();// do only after copy size to VFi, because after this call sizes will be zero
            VFi.resizeEnd();

            // populate result-vectors
            //cout << " --- populate result-vectors " << endl;
            for (int fid = 0; fid < F.rows(); ++fid)
            {
                for (int k = 0; k < 3; ++k)
                {
                    auto vid = F(fid, k);
                    //if (vid == 3) cout << "vid = 3, fid = " << fid << "   size = " << VF.size(vid) << endl;
                    VF.add(vid, fid);
                    VFi.add(vid, k);
                }
            }
            //cout << " --- END --- " << endl << endl;
        }

        void vertex_triangle_adjacency(const P3s& V, const I3s& F, CompactVectorVector<int>& VF, CompactVectorVector<int>& VFi)
        {
            //return vertex_triangle_adjacency_old(V, F, VF, VFi);
            // resize
            int n = V.rows();

            // set capacity for each individual vertex-vector
            //cout << endl << endl << " --- vertex_triangle_adjacency --- " << endl << endl;
            //cout <<  " --- get counts " << endl;
            VF.resizeBegin(n);
            VFi.resizeBegin(n, true);
            if (F.size() == 0)
            {
                VF.resizeEnd();
                VFi.resizeEnd();
                return;
            }
            const int* pvid = F.data();
            const int* pvidEnd = pvid + F.size();
            auto infoVF = VF.InfoPOINTER();
            auto infoVFi = VFi.InfoPOINTER();
            while (pvid < pvidEnd)
            {
                infoVF[*(pvid + 0)].size++;
                infoVF[*(pvid + 1)].size++;
                infoVF[*(pvid + 2)].size++;
                pvid += 3;
            }
            int totalSize = VF.resizeEnd();
            //for (int i = 0; i < n; i++)  infoVFi[i] = infoVF[i];
            memcpy(infoVFi, infoVF, n * sizeof(infoVF[0])); // VFi and VF will have same information ( the difference will be that VF store fid and VFi stores eidk)
            VFi.resizeEnd(totalSize);

            // populate result-vectors
            auto dataVF = VF.DataPOINTER();
            auto dataVFi = VFi.DataPOINTER();
            auto pVFi = VFi;
            pvid = F.data();
            int fid = 0;
            while (pvid < pvidEnd)
            {
                for (int k = 0; k < 3; ++k)
                {
                    auto vid = *pvid;
                    //VF.add(vid, fid);
                    CompactVectorVector<int>::add_direct(infoVF, dataVF, vid, fid);  //faster version from VF.add becuase compiler not enought smart to unreference VF pointers
                    //VFi.add(vid, k);
                    CompactVectorVector<int>::add_direct(infoVFi, dataVFi, vid, k); //faster version from VFi.add becuase compiler not enought smart to unreference VFi pointers
                    pvid++;
                }
                fid++;
            }
            //cout << " --- END --- " << endl << endl;
        }

        void vertex_edge_adjacency(const P3s& V, const I3s& F, const CompactVectorVector<int>& VF, const CompactVectorVector<int>& VFi, const I3s& FE, CompactVectorVector<int>& VE)
        {
            // resize
            int n = V.rows();

            // set capacity
            VE.resizeBegin(n, true);
            for (int vid = 0; vid < V.rows(); vid++)
            {
                int facesCount = VF.size(vid);
                VE.size(vid) = facesCount * 2;// count of edges will be max 2x times of face count
            }
            VE.resizeEnd();

            // populate eid's
            for (int vid = 0; vid < V.rows(); vid++)
            {
                const int* pfid = VF.pointer(vid);
                const int* pfedgeindex = VFi.pointer(vid);

                int* VE_start = VE.pointer(vid);
                int* VE_end = VE_start + VE.size(vid);
                int count = 0;
                int facesCount = VF.size(vid);
                for (int i = 0; i < facesCount; i++)
                {
                    //int fid = VF(vid, i);
                    //int fedgeindex1 = VFi(vid, i);
                    int fid = *pfid;
                    pfid++;
                    int fedgeindex1 = *pfedgeindex;
                    pfedgeindex++;
                    I3 fe = FE.row(fid);
                    int eid1 = fe(fedgeindex1);
                    int fedgeindex2 = (fedgeindex1 + 2) % 3;
                    int eid2 = fe(fedgeindex2);
                    //v0 - simple
                    //if (!VE.exists(vid, eid1)) VE.add(vid, eid1);
                    //if (!VE.exists(vid, eid2)) VE.add(vid, eid2);
                    //v1 - 3x faster - minimum if conditions and iterate array only once
                    auto p = VE_start;
                    bool found1 = false;
                    bool found2 = false;
                    while (p < VE_end)
                    {
                        found1 |= (*p == eid1);
                        found2 |= (*p == eid2);
                        p++;
                    }
                    if (found1)
                    {
                        *VE_end = eid1;
                        VE_end++;
                        count++;
                    }
                    if (found2)
                    {
                        *VE_end = eid2;
                        VE_end++;
                        count++;
                    }
                }
                VE.size(vid) = count;
            }

            VE.Compact();
        }

        void area_of_quad(const P3s& V, const I3s& F, Ds& F_Areas)
        {
            //TODO optimize doublearea_quad - dont form new structure - just calculate on the fly 
            assert(V.cols() == 3); // Only supports points in 3D
            assert(F.cols() == 4); // Only support quads

            // Split the quads into triangles
            I3s Ft(F.rows() * 2, 3);

            for (size_t i = 0; i < F.rows(); ++i)
            {
                Ft.row(i * 2) = I3(F(i, 0), F(i, 1), F(i, 2));
                Ft.row(i * 2 + 1) = I3(F(i, 2), F(i, 3), F(i, 0));
            }

            // Compute areas
            Ds doublearea_tri;
            area_of_triangle(V, Ft, doublearea_tri);

            F_Areas.resize(F.rows());
            for (unsigned i = 0; i < F.rows(); ++i)
                F_Areas(i) = doublearea_tri(i * 2) + doublearea_tri(i * 2 + 1);

        }

        void area_of_triangle(const P3s& V, const I3s& F, Ds& F_Areas)
        {
            // quads are handled by a specialized function
            if (F.cols() == 4)
            {
                return area_of_quad(V, F, F_Areas);
            }

            // Only support triangles
            assert(F.cols() == 3);
            const int F_count = F.rows();

            F_Areas.resize(F_count);
            if (F_count == 0) return;

            //
            // v1 - origin
            //
            //const auto & proj_doublearea = [&V, &F](const int x, const int y, const int f)->D
            //{
            //    auto rx = V(F(f, 0), x) - V(F(f, 2), x);
            //    auto sx = V(F(f, 1), x) - V(F(f, 2), x);
            //    auto ry = V(F(f, 0), y) - V(F(f, 2), y);
            //    auto sy = V(F(f, 1), y) - V(F(f, 2), y);
            //    return rx * sy - ry * sx;
            //};
            //for (int fid = 0; fid < F_count; fid++)
            //{
            //    F_Areas(fid) = 0;
            //    for (int d = 0; d < 3; d++)
            //    {
            //        D dblAd = proj_doublearea(d, (d + 1) % 3, f);
            //        F_Areas(fid) += dblAd * dblAd;
            //    }
            //    F_Areas(fid) = sqrt(F_Areas(fid));
            //}

            //
            // v2 - fast
            //
            auto vid0 = F(0, 0);
            auto vid1 = F(0, 1);
            auto vid2 = F(0, 2);
            const auto & proj_doublearea_fast = [&V, &vid0, &vid1, &vid2](const int x, const int y)->D
            {
                auto v2x = V(vid2, x);
                auto v2y = V(vid2, y);
                auto rx = V(vid0, x) - v2x;
                auto sx = V(vid1, x) - v2x;
                auto ry = V(vid0, y) - v2y;
                auto sy = V(vid1, y) - v2y;
                return rx * sy - ry * sx;
            };

            D maxDiff = 0;
            for (int fid = 0; fid < F_count; fid++)
            {
                vid0 = F(fid, 0);
                vid1 = F(fid, 1);
                vid2 = F(fid, 2);
                D doublearea = 0;
                for (int k = 0; k < 3; k++)
                {
                    D dblAd = proj_doublearea_fast(k, (k + 1) % 3);
                    doublearea += dblAd * dblAd;
                }
                D area = sqrt(doublearea) / 2;
                F_Areas(fid) = area;

                //P3 v0 = V.row(vid0);
                //P3 v1 = V.row(vid1);
                //P3 v2 = V.row(vid2);
                //D a = utils::point::DistToPoint(v0, v1);
                //D b = utils::point::DistToPoint(v1, v2);
                //D c = utils::point::DistToPoint(v2, v0);
                //D s = (a + b + c) * 0.5f;
                //D areaNEW =  sqrt(s*(s - a)*(s - b)*(s - c));
                //
                ////cout  << "area = " << area << "   new area = " << area <<  "   diff = "<<abs(area  - areaNEW)<< endl;
                //maxDiff = max(maxDiff, abs(area - areaNEW));
            }
            //cout << "maxDiff = " << maxDiff << endl;
        }

        void area_of_triangle_fast(const I3s& FE, const Ds& E_Length, Ds& F_Areas, bool useSSE)
        {
            // Only support triangles
            assert(FE.cols() == 3);
            const int F_count = FE.rows();

            F_Areas.resize(F_count);
            if (F_count == 0) return;

            const int* peid = FE.data();
            const int* peidEnd = FE.data() + F_count * 3; // 3 edge-indexes in a row
            const int* peidEnd4 = peidEnd - 4 * 3;// 3 edge-indexes in a row
            D* pArea = F_Areas.data();

            auto getAreaPow2 = [&]()
            {
                int eid0 = *peid; peid++;
                int eid1 = *peid; peid++;
                int eid2 = *peid; peid++;
                D a = E_Length(eid0);
                D b = E_Length(eid1);
                D c = E_Length(eid2);
                D s = (a + b + c) * 0.5f;
                D areaPow2 = s * (s - a)*(s - b)*(s - c);
                return areaPow2;
            };

            if (utils::cpu::isSupportedSSE && useSSE && FE.rows() > 4)
            {
                while (peid < peidEnd4)
                {
                    D areapow2_0 = getAreaPow2();
                    D areapow2_1 = getAreaPow2();
                    D areapow2_2 = getAreaPow2();
                    D areapow2_3 = getAreaPow2();
                    Vec4f v4(areapow2_0, areapow2_1, areapow2_2, areapow2_3);
                    sqrt(v4).store(pArea);
                    pArea += 4;
                }
            }

            while (peid < peidEnd)
            {
                *pArea = sqrt(getAreaPow2());
                pArea++;
            }
        }


        int local_basis(const P3s& V, const I3s& F, V3s& F_X, V3s& F_Y, V3s& F_Z)
        {
            int invalidNormalsCount = 0;
            int rows = F.rows();
            F_X.resize(rows, 3);
            F_Y.resize(rows, 3);
            F_Z.resize(rows, 3);

            for (int i = 0; i < rows; ++i)
            {
                int fi0 = F(i, 0);
                int fi1 = F(i, 1);
                int fi2 = F(i, 2);
                P3 vertex0 = V.row(fi0);
                P3 vertex1 = V.row(fi1);
                P3 vertex2 = V.row(fi2);
                V3 v1n = vertex1 - vertex0;
                V3 t = vertex2 - vertex0;
                v1n.normalize();
                V3 v3n = utils::vector::Cross(v1n, t);
                v3n.normalize();
                V3 v2n = utils::vector::Cross(v1n, v3n);
                //v2n.normalize(); //  no need in normalization since this is cross of 2 normalized vectors
                F_X(i, 0) = v1n(0);
                F_X(i, 1) = v1n(1);
                F_X(i, 2) = v1n(2);
                F_Y(i, 0) = -v2n(0);
                F_Y(i, 1) = -v2n(1);
                F_Y(i, 2) = -v2n(2);
                F_Z(i, 0) = v3n(0);
                F_Z(i, 1) = v3n(1);
                F_Z(i, 2) = v3n(2);
                if (isnan(v3n(0))) invalidNormalsCount++;
            }
            return invalidNormalsCount;
        }

        int local_basis(const P3s& V, const I3s& F, const I3s& FE, const Ds& E_Length, V3s& F_X, V3s& F_Y, V3s& F_Z)
        {
            int invalidNormalsCount = 0;
            int rows = F.rows();
            F_X.resize(rows, 3);
            F_Y.resize(rows, 3);
            F_Z.resize(rows, 3);

            const int* pF = F.data();
            const int* pE = FE.data();
            const int* pFEnd = pF + rows * 3;
            #ifdef USE_EIGEN
            D* pFx = F_X.data();
            D* pFy = F_Y.data();
            D* pFz = F_Z.data();
            #else
            #endif

            int row = 0;
            while (pF < pFEnd)
            {
                int vid0 = *pF; pF++;
                int vid1 = *pF; pF++;
                int vid2 = *pF; pF++;
                P3 vertex0 = V.row(vid0);
                P3 vertex1 = V.row(vid1);
                P3 vertex2 = V.row(vid2);

                // edge #1 normalized
                V3 dir01 = vertex1 - vertex0;
                int eid0 = *pE; pE += 3;
                dir01 /= E_Length(eid0);

                // edge #3
                V3 dir02 = vertex2 - vertex0;

                V3 z = utils::vector::Cross(dir01, dir02);
                D zlength = utils::vector::Length(z);
                z /= zlength; // we should normalize z since angle between edge#1 and edge#3 can be different from 90 degree
                V3 y = utils::vector::Cross(z, dir01);
                #ifdef USE_EIGEN
                *pFx = x(0); pFx++;
                *pFx = x(1); pFx++;
                *pFx = x(2); pFx++;
                *pFy = y(0); pFy++;
                *pFy = y(1); pFy++;
                *pFy = y(2); pFy++;
                *pFz = z(0); pFz++;
                *pFz = z(1); pFz++;
                *pFz = z(2); pFz++;
                #else
                F_X.row(row) = dir01;
                F_Y.row(row) = y;
                F_Z.row(row) = z;
                row++;
                #endif

                if (isnan(z(0)))
                {
                    V3 zdebug = utils::vector::Cross(dir01, dir02);
                    invalidNormalsCount++;
                }
                if (isnan(dir01(0)) || isnan(y(0)))
                {
                    invalidNormalsCount++;
                }
            }
            return invalidNormalsCount;
        }

        // same as igl::compute_frame_field_bisectors but faster (returns invalidNormalsCount) 
        void compute_frame_field_bisectors(
            const V3s& F_X, const V3s& F_Y,
            const V3s& FF1, const V3s& FF2,
            V3s& BIS1, V3s& BIS2)
        {
            BIS1.resize(FF1.rows(), 3);
            BIS2.resize(FF1.rows(), 3);
            //DEBUG - zero to have better debug
            //BIS1.setZero(FF1.rows(), 3);
            //BIS2.setZero(FF1.rows(), 3);
            //extern bool IsOmpEnabled;
        //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
            D ZERO = 0;
            D PI = (D)M_PI;
            for (int i = 0; i < FF1.rows(); ++i)
            {
                int fid = i; // here FF1 must be of same length as checkForFaces

                // project onto the tangent plane and convert to angle
                // Convert to angle
                D a1 = atan2(F_Y.row(i).dot(FF1.row(i)), F_X.row(i).dot(FF1.row(i)));
                //make it positive by adding some multiple of 2pi
                a1 += std::ceil(std::max(ZERO, -a1) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                a1 = fmod(a1, (PI * 2));
                D a2 = atan2(F_Y.row(i).dot(FF2.row(i)), F_X.row(i).dot(FF2.row(i)));
                //make it positive by adding some multiple of 2pi
                a2 += std::ceil(std::max(ZERO, -a2) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                a2 = fmod(a2, (PI * 2));

                D b1 = (a1 + a2) / 2;
                //make it positive by adding some multiple of 2pi
                b1 += std::ceil(std::max(ZERO, -b1) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                b1 = fmod(b1, (PI * 2));

                D b2 = b1 + (PI / 2);

                //make it positive by adding some multiple of 2pi
                b2 += std::ceil(std::max(ZERO, -b2) / (PI * 2)) * (PI * 2);

                //take modulo 2pi
                b2 = fmod(b2, (PI * 2));

                BIS1.row(i) = cos(b1) * F_X.row(i) + sin(b1) * F_Y.row(i);
                BIS2.row(i) = cos(b2) * F_X.row(i) + sin(b2) * F_Y.row(i);

            }
        }
        // same as igl::compute_frame_field_bisectors but faster (returns invalidNormalsCount) 
        void compute_frame_field_bisectors(const std::vector<bool>& checkForFaces,
            const V3s& F_X, const V3s& F_Y,
            const V3s& FF1, const V3s& FF2,
            V3s& BIS1, V3s& BIS2)
        {
            BIS1.resize(FF1.rows(), 3);
            BIS2.resize(FF1.rows(), 3);
            //DEBUG - zero to have better debug
            //BIS1.setZero(FF1.rows(), 3);
            //BIS2.setZero(FF1.rows(), 3);
            //extern bool IsOmpEnabled;
        //#pragma omp parallel for schedule(static) if(IsOmpEnabled)
            D ZERO = 0;
            D PI = (D)M_PI;
            for (int i = 0; i < FF1.rows(); ++i)
            {
                int fid = i; // here FF1 must be of same length as checkForFaces
                if (!checkForFaces[fid]) continue;

                // project onto the tangent plane and convert to angle
                // Convert to angle
                D a1 = atan2(F_Y.row(i).dot(FF1.row(i)), F_X.row(i).dot(FF1.row(i)));
                //make it positive by adding some multiple of 2pi
                a1 += std::ceil(std::max(ZERO, -a1) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                a1 = fmod(a1, (PI * 2));
                D a2 = atan2(F_Y.row(i).dot(FF2.row(i)), F_X.row(i).dot(FF2.row(i)));
                //make it positive by adding some multiple of 2pi
                a2 += std::ceil(std::max(ZERO, -a2) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                a2 = fmod(a2, (PI * 2));

                D b1 = (a1 + a2) / 2;
                //make it positive by adding some multiple of 2pi
                b1 += std::ceil(std::max(ZERO, -b1) / (PI * 2)) * (PI * 2);
                //take modulo 2pi
                b1 = fmod(b1, (PI * 2));

                D b2 = b1 + (PI / 2);

                //make it positive by adding some multiple of 2pi
                b2 += std::ceil(std::max(ZERO, -b2) / (PI * 2)) * (PI * 2);

                //take modulo 2pi
                b2 = fmod(b2, (PI * 2));

                BIS1.row(i) = cos(b1) * F_X.row(i) + sin(b1) * F_Y.row(i);
                BIS2.row(i) = cos(b2) * F_X.row(i) + sin(b2) * F_Y.row(i);

            }
        }


        void MakeMeshFlat(const I3s& F, const V3s& F_normals, P3s& V)
        {
            int countNormals = F_normals.rows() == 0 ? 1 : F_normals.rows(); // protection from devision by zero exception
            DD normal_sum_x = 0;
            DD normal_sum_y = 0;
            DD normal_sum_z = 0;
            for (int i = 0; i < F_normals.rows(); ++i)
            {
                normal_sum_x += F_normals(i, 0);
                normal_sum_y += F_normals(i, 1);
                normal_sum_z += F_normals(i, 2);
            }
            V3 normal_avg = V3(normal_sum_x / countNormals, normal_sum_y / countNormals, normal_sum_z / countNormals);
            normal_avg.normalize();

            // get avg point
            int countPoints = V.rows() == 0 ? 1 : V.rows(); // protection from devision by zero exception
            DD point_sum_x = 0;
            DD point_sum_y = 0;
            DD point_sum_z = 0;
            for (int i = 0; i < V.rows(); ++i)
            {
                point_sum_x += V(i, 0);
                point_sum_y += V(i, 1);
                point_sum_z += V(i, 2);
            }
            P3 point_avg = P3(point_sum_x / countPoints, point_sum_y / countPoints, point_sum_z / countPoints);

            //extern Model model;
            //model.draw.AddPoint(point_avg, Color3d(1, 0, 0));
            //model.draw.AddEdge(point_avg, point_avg + point_avg, Color3d(1, 0, 0));
            //for (int i = 0; i < V.rows(); ++i)
            //{
            //    cout << utils::point::DistToPlane(V.row(i), point_avg, normal_avg) << endl;
            //}

            for (int i = 0; i < V.rows(); i++)
            {
                V.row(i) = utils::point::ProjectToPlane(V.row(i), point_avg, normal_avg);
            }
        }
    }
}
