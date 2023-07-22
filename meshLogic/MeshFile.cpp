#include "stdafx.h"
#include "MeshFile.h"
#include "Mesh.h"
#include "PolygonMesh.h"
#include "ViewerDrawObjects.h"
#include "igl/readSTL.h"


//******************************************************************
// LoadFromObj
//******************************************************************


enum class charType
{
    Unkown, Delimiter, Number
};
charType charTypes[255];
void init_charTypes()
{
    for (int i = 0; i < 255; i++)
    {
        charTypes[i] = charType::Unkown;
    }
    charTypes[static_cast<int>(' ')] = charType::Delimiter;  //32
    charTypes[static_cast<int>('/')] = charType::Delimiter;  //47
    charTypes[static_cast<int>('+')] = charType::Number;  //43
    charTypes[static_cast<int>('-')] = charType::Number;  //45
    charTypes[static_cast<int>('.')] = charType::Number;  //46
    charTypes[static_cast<int>('0')] = charType::Number;  //48
    charTypes[static_cast<int>('1')] = charType::Number;
    charTypes[static_cast<int>('2')] = charType::Number;
    charTypes[static_cast<int>('3')] = charType::Number;
    charTypes[static_cast<int>('4')] = charType::Number;
    charTypes[static_cast<int>('5')] = charType::Number;
    charTypes[static_cast<int>('6')] = charType::Number;
    charTypes[static_cast<int>('7')] = charType::Number;
    charTypes[static_cast<int>('8')] = charType::Number;
    charTypes[static_cast<int>('9')] = charType::Number;
    charTypes[static_cast<int>('e')] = charType::Number;   //101
    charTypes[static_cast<int>('E')] = charType::Number;   //101
}



bool get3doubles(char* line, D* double3Values)
{
    std::vector<int> ints;
    std::vector<D> doubles;
    int ints_added_count;
    int doubles_added_count;
    if (utils::strings::extractValues(line, ints, doubles, ints_added_count, doubles_added_count) && doubles_added_count == 3)
    {
        double3Values[0] = doubles[0];
        double3Values[1] = doubles[1];
        double3Values[2] = doubles[2];
        return true;
    }
    return false;
}



int extractDoubleValues(const char* str, D* doubles, int expectedCount)
{
    int ints_added_count = 0;
    int doubles_added_count = 0;

    const char* p = str;
    bool isValuesStarted = false;
    bool isDouble = false;
    bool isExponent = false;
    bool isExponentMinus = false;
    bool isNegative = false;
    int i = 0;
    long long d = 0;
    int dn = 0;
    int e = 0;
    const D dnm[16] = { 1, 0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f, 0.000001f, 0.0000001f, 0.00000001f, 0.000000001f, 0.0000000001f, 0.00000000001f, 0.000000000001f, 0.0000000000001f, 0.00000000000001f, 0.000000000000001f };

    auto finishValue = [&]()
    {
        if (!isValuesStarted) return;

        if (isDouble)
        {
            D r = (D)i;
            if (dn < 16)
            {
                r += dnm[dn] * d;
            }
            else
            {
                r += d / std::pow(10.0f, (D)dn);
            }
            if (isNegative)
            {
                r = -r;
            }

            if (isExponent)
            {
                D exp = std::pow(10.0f, (D)e);
                if (isExponentMinus)
                {
                    r /= exp;
                }
                else
                {
                    r *= exp;
                }
            }

            doubles_added_count++;
            if (ints_added_count + doubles_added_count <= expectedCount)
            {
                *doubles = static_cast<D>(r);
                doubles++;
            }
        }
        else
        {
            if (isNegative)
            {
                i = -i;
            }
            ints_added_count++;
            if (ints_added_count + doubles_added_count <= expectedCount)
            {
                *doubles = (D)i;
                doubles++;
            }
        }

        i = 0;
        d = 0;
        dn = 0;
        e = 0;
        isValuesStarted = false;
        isNegative = false;
        isDouble = false;
        isExponent = false;
        isExponentMinus = false;
    };

    while (*p != 0)
    {
        while (*p >= '0')
        {
            if (*p != 'e' && *p != 'E')
            {
                isValuesStarted = true;
                if (isExponent)
                {
                    e = e * 10 + (*p - '0');
                }
                else if (isDouble)
                {
                    d = d * 10 + (*p - '0');
                    dn++;
                }
                else
                {
                    i = i * 10 + (*p - '0');
                }
                p++;
            }
            else
            {
                isExponent = true;
                e = 0;
                p++;
            }
        }
        switch (*p)
        {
            case '.':
                isDouble = true;
                break;
            case 'e':
            case 'E':
                isExponent = true;
                e = 0;
                break;
            case '-':
                if (isExponent)
                {
                    isExponentMinus = true;
                }
                else
                {
                    isNegative = true;
                }
                break;
            default:
                finishValue();
                break;
        }
        if (*p == 0) break;
        p++;
    }
    finishValue();

    return ints_added_count + doubles_added_count;
}

bool get3doubles_fast(char* line, D* double3Values)
{
    if (extractDoubleValues(line, double3Values, 3) == 3)
    {
        return true;
    }
    return false;
}



int readNgon(char* line, int* vid, int expectedVidsInLineCount)
{
    //if (vid != nullptr)
    //{
    //    cout << "line=" << line;
    //}
    int intsCount = 0;
    char* p = line;

    char c = *p;
    char* numberStart = nullptr;
    auto process = [&](char* numberEnd)
    {
        if (numberStart == nullptr) return;
        intsCount++; // new int added
        char* pInt = numberStart;
        numberStart = nullptr;
        if (vid == nullptr) return;// skip atoi in case we need only count of ints
        if (intsCount > expectedVidsInLineCount) return; // if count of ints greater from expected - fail
        int i = 0;
        while (pInt < numberEnd)
        {
            i = i * 10 + (*pInt - '0');
            pInt++;
        }
        //cout << "    " << i;
        *vid = i - 1; // write converted int to buffer and minus 1 since Obj index starts from 1 but in C++ index starts from 0
        vid++; // increment position of next writing
    };
    charType waitingForType = charType::Number;
    while (c != 0)
    {
        charType cType = charTypes[c];
        if (cType == waitingForType)
        {
            switch (cType)
            {
                case charType::Number:
                    numberStart = p;
                    waitingForType = charType::Delimiter;
                    do
                    {
                        p++;
                    } while (*p >= '0'); // skipp all digits
                    c = *p;
                    continue;
                case charType::Delimiter:
                    process(p);
                    if (c == ' ') waitingForType = charType::Number;
                    break;
                default:
                    break;
            };
        }
        p++;
        c = *p;
    }
    process(p);
    //if (vid != nullptr)
    //{
    //    cout << endl;
    //}
    return intsCount;
}

int getNgonSize(char* line)
{
    return readNgon(line, nullptr, 0);
}


int readNgon_fast(char* line, int* pvid, int& max_vid)
{
    max_vid = 0;
    int intsCount = 0;
    char* c = line;
    do
    {
        if (*c++ == ' ')
        {
            if (*c >= '0')
            {
                intsCount++;
                int i = 0;
                do
                {
                    i = i * 10 + (*c - '0');
                    c++;
                } while (*c >= '0'); // skipp all digits
                //cout << "    " << i;
                int vid = i - 1; // write converted int to buffer and minus 1 since Obj index starts from 1 but in C++ index starts from 0;
                *pvid = vid;
                pvid++; // increment position of next writing
                if (vid > max_vid) // get max vid, to make sure we will not crash app if some vid is higher from possible
                {
                    max_vid = vid;
                }
            }
        }
    } while (*c != 0);
    return intsCount;
}

int getNgonSize_fast(char* line)
{
    int intsCount = 0;
    char* c = line;
    do
    {
        if (*c++ == ' ')
        {
            if (*c >= '0')
            {
                intsCount++;
                do
                {
                    c++;
                } while (*c >= '0'); // skipp all digits
            }
        }
    } while (*c != 0);
    return intsCount;
}



//static inline int popcnt128(unsigned long long n)
//{
//    const __m128i cnt64 = __popcnt64(n);
//    const __m128i cnt64_hi = _mm_unpackhi_epi64(cnt64, cnt64);
//    const __m128i cnt128 = _mm_add_epi32(cnt64, cnt64_hi);
//    return _mm_cvtsi128_si32(cnt128);
//}


bool __inline ctz(unsigned long value, unsigned long& index)
{
    return _BitScanForward(&index, value);
}

bool __inline clz(unsigned long value, unsigned long& index)
{
    return _BitScanReverse(&index, value);
}

bool MeshFile::LoadFromObj(const string& filename, vector<PolygonMesh>& meshes, const vector<string>& loadOnlyGroups, bool separateGroups)
{
    extern bool IsOmpEnabled;

    init_charTypes();


    meshes.clear();

    //
    // read file in buffer
    //
    int sse_buffer_size = 256; // reserve bytes for sse commands
    //vector<char> buffer;
    Vector<char> buffer;
    if (!utils::file::Read(filename, buffer, 1, 3 + sse_buffer_size)) // reserve bytes for sse commands
    {
        cout << "!!!   error:   Mesh::LoadObj - Cannot read from file '" << filename << "'" << endl;
        return false;
    }
    buffer[0] = '\n'; // add first symbol 'newline' because we want to have at least 1 line before v,f,g
    buffer[buffer.size() - 3 - sse_buffer_size] = '\n'; // add pre-pre-last symbol 'newline' because we want to have at least 1 line in file and we use condition 'if (prevSymbolWasNewLine)'
    buffer[buffer.size() - 2 - sse_buffer_size] = ' '; // add pre-last symbol 'space' because we use conditions like:   if (line[0] == 'v' && line[1] == ' ')
    buffer[buffer.size() - 1 - sse_buffer_size] = ' '; // add last symbol 'space' because we use conditions like:   *(p+1)
    for (int i = 0; i < sse_buffer_size; i++)
    {
        buffer[buffer.size() - 0 - sse_buffer_size + i] = ' '; // reserve bytes for sse commands
    }

    //
    // Get lines positions in buffer
    //
    enum class LineType
    {
        Unknown, V, F, G
    };
    struct LineInfo
    {
        LineType type;
        char* chars;
    };
    std::vector<LineInfo> lines;
    lines.reserve(static_cast<int>(buffer.size() / 10)); // reserve 10x times less space for indexes
    int v_count = 0;
    int f_count = 0;
    int g_count = 0;
    int buffer_size = buffer.size(); // increase speed in DEBUG mode
    char* pBlock = buffer.data();
    char* pEnd = buffer.data() + buffer_size - sse_buffer_size;
    char* pNewLine = buffer.data();

    const __m128i char_slash = _mm_set1_epi8('\\');
    const __m128i char_enter1 = _mm_set1_epi8('\n');
    const __m128i char_enter2 = _mm_set1_epi8('\r');
    while (pBlock < pEnd)
    {
        const __m128i blockOfChars = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pBlock));
        const __m128i eq_slash = _mm_cmpeq_epi8(char_slash, blockOfChars);
        const __m128i eq_enter1 = _mm_cmpeq_epi8(char_enter1, blockOfChars);
        const __m128i eq_enter2 = _mm_cmpeq_epi8(char_enter2, blockOfChars);
        int mask = _mm_movemask_epi8(_mm_or_si128(eq_slash, _mm_or_si128(eq_enter1, eq_enter2)));
        while (mask != 0)
        {

            unsigned long bitpos;
            ctz(mask, bitpos); // always true, becase we check it before  'while (mask != 0)'
            auto p = pBlock + bitpos;
            char c = *p;
            switch (c)
            {
                case '\\':
                {
                    if (p + 1 < pEnd && (*(p + 1) == '\n' || *(p + 1) == '\r'))
                    {
                        *(p) = ' ';
                        *(p + 1) = ' ';
                        mask &= ~(1UL << (bitpos + 1));
                        if (p + 2 < pEnd && (*(p + 2) == '\n' || *(p + 2) == '\r'))
                        {
                            *(p + 2) = ' ';
                            mask &= ~(1UL << (bitpos + 2));
                        }
                    }
                    break;
                }
                case '\n':
                case '\r':
                {
                    *p = 0; //replace all newlines with 0 - this will make ours lines ready for passing to method 'sscanf'
                    char nextCharAterBreakLine1 = *(p + 1);
                    if (nextCharAterBreakLine1 != '\n' && nextCharAterBreakLine1 != '\r')
                    {
                        pNewLine = p + 1; // remember this new line
                        LineInfo line = { LineType::Unknown, pNewLine };
                        char nextCharAterBreakLine2 = *(p + 2);
                        if (nextCharAterBreakLine2 == ' ')
                        {
                            if (nextCharAterBreakLine1 == 'v')
                            {
                                v_count++;
                                line.type = LineType::V;
                            }
                            if (nextCharAterBreakLine1 == 'f')
                            {
                                f_count++;
                                line.type = LineType::F;
                            }
                            if (nextCharAterBreakLine1 == 'g')
                            {
                                g_count++;
                                line.type = LineType::G;
                            }
                        }
                        lines.push_back(line);
                    }
                    break;
                }
                default:
                    break;
            }
            mask &= ~(1UL << bitpos);
        }
        pBlock += sizeof(char_slash);
    }


    //
    // get all vertex,groups,faces lines
    //
    vector<char*> linesV(v_count);
    char** linesVnext = linesV.data();
    vector<char*> linesF(f_count);
    char** linesFnext = linesF.data();
    int linesFIndex = 0;
    struct Group
    {
        char* name;
        int linesF_startIndex;
        int linesF_endIndex;
        int linesF_count;
    };
    vector<Group> linesG(g_count);
    Group* linesGnext = linesG.data();
    int linesGIndex = 0;
    for (LineInfo line : lines)
    {
        //char c1 = line.chars[1];
        //if (c1 != ' ') continue;// vertexes,faces,groups defined as 'v ...' 'f ...' 'g ...' - space must be after letters vfg
        //char c0 = line.chars[0];
        if (line.type == LineType::F)
        {
            *linesFnext = line.chars;
            linesFnext++;
            linesFIndex++;
        }
        else if (line.type == LineType::V)
        {
            *linesVnext = line.chars;
            linesVnext++;
        }
        else if (line.type == LineType::G)
        {
            *linesGnext = { line.chars + 2, linesFIndex, 0, 0 };
            linesGnext++;
            linesGIndex++;
        }
    }
    if (f_count > 0)
    {
        if (g_count == 0 || linesG[0].linesF_startIndex != 0) // ensure first faces will always belong to group, even it was not defined in obj file
        {
            linesG.insert(linesG.begin(), { nullptr, 0, 0,0 });
            g_count++;
        }
        for (int i = 0; i < linesG.size() - 1; i++)
        {
            Group& g = linesG[i];
            Group& gNext = linesG[i + 1];
            g.linesF_endIndex = gNext.linesF_startIndex;
            g.linesF_count = g.linesF_endIndex - g.linesF_startIndex;
        }
        Group& gLast = linesG.back();
        gLast.linesF_endIndex = linesF.size();
        gLast.linesF_count = gLast.linesF_endIndex - gLast.linesF_startIndex;
    }
    if (linesV.size() != v_count)
    {
        cout << "!!! error:  MeshFile::LoadFromObj()  get all vertexes failed" << endl;
        assert(linesV.size() == v_count && "MeshFile::LoadFromObj()  get all vertexes failed");
        return false;
    }
    if (linesF.size() != f_count)
    {
        cout << "!!! error:  MeshFile::LoadFromObj()  get all faces failed" << endl;
        assert(linesF.size() == f_count && "MeshFile::LoadFromObj()  get all faces failed");
        return false;
    }
    if (linesG.size() != g_count)
    {
        cout << "!!! error:  MeshFile::LoadFromObj()  get all groups failed" << endl;
        assert(linesG.size() == g_count && "MeshFile::LoadFromObj()  get all groups failed");
        return false;
    }
    // return empty result if there are not faces
    if (f_count == 0)
    {
        return true;
    }


    //
    // Create list of loaded groups - to be able to load only few groups without peformance penalty
    //
    std::vector<int> gis(g_count);
    std::iota(gis.begin(), gis.end(), 0); // by default load all groups
    //std::vector<int> linesFs(linesF.size());
    //std::iota(linesFs.begin(), linesFs.end(), 0); // by default load all faces
    if (loadOnlyGroups.size() != 0)
    {
        // test in parallel - because we use 'utils::stdvector::exists'
        #pragma omp parallel for schedule(static) if(IsOmpEnabled)
        for (int gi = 0; gi < g_count; gi++)
        {
            Group& g = linesG[gi];
            string groupName = g.name != nullptr ? g.name : to_string(gi);
            groupName = utils::strings::Trim(groupName);
            if (!utils::stdvector::exists(loadOnlyGroups, groupName))
            {
                gis[gi] = -1; // remove group from load
            }
        }
        // and fix results in single core
        int nextIndex = 0;
        for (int gi : gis) if (gi != -1)
        {
            gis[nextIndex++] = gi;
        }
        gis.resize(nextIndex);

        // limit loading faces from linesF
        //nextIndex = 0;
        //for (int gii = 0; gii < gis.size(); gii++)
        //{
        //    int gi = gis[gii];
        //    Group& g = linesG[gi];
        //    for (int i = g.linesF_startIndex; i < g.linesF_endIndex; i++)
        //    {
        //    }
        //}
    }

    // remove empty meshes
    for (int i = gis.size() - 1; i >= 0; i--)
    {
        int gi = gis[i];
        if (linesG[gi].linesF_count == 0)
        {
            utils::stdvector::remove_at(gis, i);
        }
    }

    //
    // Read F
    //


    //
    // allocate space for ngons
    //
    bool IsOmpEnabled_IsOnlyOneGroup = (gis.size() == 1);
    CompactVectorVector<int> ngons;
    bool skipZeroingInfo = (loadOnlyGroups.size() == 0); // skip zeroing if all data will be loaded from obj, so all ngons data will be populated
    ngons.resizeBegin(f_count, skipZeroingInfo);
    bool cycle_getNgonSize_failed = false;
    #pragma omp parallel for schedule(static)  if(IsOmpEnabled && !IsOmpEnabled_IsOnlyOneGroup)
    for (int gii = 0; gii < gis.size(); gii++)
    {
        int gi = gis[gii];
        Group& g = linesG[gi];
        #pragma omp parallel for schedule(static)  if(IsOmpEnabled && IsOmpEnabled_IsOnlyOneGroup)
        for (int i = g.linesF_startIndex; i < g.linesF_endIndex; i++)
        {
            char* line = linesF[i];
            line++; //skip 'f'
            int size = getNgonSize_fast(line);
            ngons.size(i) = size;
            if (size < 3)
            {
                // fail
                cout << "!!! MeshFile:  failed to read face line: " << line << endl;
                assert(false && "!!! MeshFile:  failed to read face line");
                cycle_getNgonSize_failed = true;
            }
        }
    }
    if (cycle_getNgonSize_failed)
    {
        return false;
    }
    ngons.resizeEnd();

    //
    // read ngons
    //
    atomic_int F_max_vid = 0;
    #pragma omp parallel for schedule(static)  if(IsOmpEnabled && !IsOmpEnabled_IsOnlyOneGroup)
    //for (int ii = 0; ii < linesFs.size(); ii++)
    //{
    //    int i = linesFs[ii];
    for (int gii = 0; gii < gis.size(); gii++)
    {
        int gi = gis[gii];
        Group& g = linesG[gi];
        #pragma omp parallel for schedule(static)  if(IsOmpEnabled && IsOmpEnabled_IsOnlyOneGroup)
        for (int i = g.linesF_startIndex; i < g.linesF_endIndex; i++)
        {
            char* line = linesF[i];
            line++; //skip 'f'
            int capacity = ngons.capacity(i);
            int max_vid;
            int readedVidsCount = readNgon_fast(line, ngons.pointer(i), max_vid);

            utils::num::update_maximum(F_max_vid, max_vid); // get max vid, to make sure we will not crash app if some vid is higher from possible

            ngons.size(i) = min(readedVidsCount, capacity); // set size as well, since we want use direct assignment instead of calling method 'ngons.add' just improve performance
            if (readedVidsCount != capacity)
            {
                // fail
                cout << "!!! MeshFile:  failed to read face line: " << line << endl;
                assert(false && "!!! MeshFile:  failed to read face line");
                //return false; we can continue of program execution, but some ngon will be broken
            }
        }
    }



    //
    // Read V
    //
    P3s V_file;
    V_file.resize(v_count, 3);
    std::vector<int> linesV_isUsedBylinesF(linesV.size());
    std::iota(linesV_isUsedBylinesF.begin(), linesV_isUsedBylinesF.end(), 0); // by default load all vertexes
    if (loadOnlyGroups.size() != 0)
    {
        for (int i = 0; i < linesV_isUsedBylinesF.size(); i++) // make all indexes negative, means
        {
            linesV_isUsedBylinesF[i] = -(linesV_isUsedBylinesF[i] + 1); // make negative and minus 1 to be able eliminate 0 index
        }
        for (int gii = 0; gii < gis.size(); gii++)
        {
            int gi = gis[gii];
            Group& g = linesG[gi];
            for (int ni = g.linesF_startIndex; ni < g.linesF_endIndex; ni++)
            {
                for (int i = 0; i < ngons.size(ni); i++)
                {
                    int vid = ngons(ni, i);
                    assert(vid < linesV_isUsedBylinesF.size() && "vid out of range of 'linesV'");
                    if (linesV_isUsedBylinesF[vid] < 0)
                    {
                        linesV_isUsedBylinesF[vid] = -(linesV_isUsedBylinesF[vid] + 1); // mark index positive, means we will use this vertex
                    }
                }
            }
        }
        int nextIndex = 0;
        for (int i : linesV_isUsedBylinesF) if (i >= 0)
        {
            linesV_isUsedBylinesF[nextIndex++] = linesV_isUsedBylinesF[i];
        }
        linesV_isUsedBylinesF.resize(nextIndex);
    }
    bool cycle_get3doubles_failed = false;
    #pragma omp parallel for schedule(static)  if(IsOmpEnabled)
    for (int ii = 0; ii < linesV_isUsedBylinesF.size(); ii++)
    {
        int i = linesV_isUsedBylinesF[ii];
        char* line = linesV[i];
        line++; //skip 'v'
        double v0, v1, v2;
        if (get3doubles_fast(line, &V_file(i, 0)))
        {
            // success
        }
        else if (sscanf_s(line, "%lf %lf %lf", &v0, &v1, &v2) == 3)
        {
            // success
            V_file(i, 0) = (D)v0;
            V_file(i, 1) = (D)v1;
            V_file(i, 2) = (D)v2;
        }
        else
        {
            // fail
            cout << "!!! MeshFile:  failed to read vertex line: " << line << endl;
            assert(false && "!!! MeshFile:  failed to read vertex line");
            cycle_get3doubles_failed = true;
        }
    }
    if (cycle_get3doubles_failed)
    {
        return false;
    }


    //
    // Validate 
    //
    bool is_out_of_range_vid_detected = F_max_vid > V_file.rows() - 1;
    if (is_out_of_range_vid_detected)
    {
        cout << "!!! error in MeshFile.LoadFromObj:  vertex index is out of range in F:  max vid in F = " << F_max_vid + 1 << "  count of vertexes in V = " << V_file.rows() << endl;
        return false;
    }

    //
    // Return result IF there is only 1 group OR we dont want to separare groups
    //
    // !! we cant do this, because our algorithms require that there must be no one unused vertex 
    //     - so we have to filter and remove all unused vertexes
    // 
    //  so insted of returning as is - we will merge all groups into one and process as usual
    //
    //if (g_count == 1 || (!separateGroups && loadOnlyGroups.size() == 0))
    //{
    //    ! string groupName = linesG[0].name != nullptr ? linesG[0].name : "0";
    //    ! groupName = utils::strings::Trim(groupName);
    //    ! meshes.push_back(PolygonMesh(1, groupName));
    //    ! PolygonMesh& mesh = meshes.back();
    //    ! mesh.V = V_file;
    //    ! mesh.ngons = ngons;
    //    ! return true;
    //}

    // force 'separateGroups' if we load obj file partially
    if (loadOnlyGroups.size() != 0)
    {
        separateGroups = true;
    }

    // merge all groups if we dont want to separate groups    
    char merged_groups_name[] = "separateGroups == false\0";
    if (separateGroups == false && g_count > 0)
    {
        gis.clear();
        gis.push_back(0); // only 1 group - first one
        Group g = linesG[0];
        g.linesF_endIndex = linesG.back().linesF_endIndex;
        g.linesF_count = g.linesF_endIndex - g.linesF_startIndex;
        g.name = merged_groups_name;
        linesG.clear();
        linesG.push_back(g);
    }

    //
    // Ungroup F
    //
    // allocate 'meshes' for multithreading
    meshes.reserve(gis.size());
    for (int gii = 0; gii < gis.size(); gii++)
    {
        int gi = gis[gii];
        Group& g = linesG[gi];
        string groupName = g.name != nullptr ? g.name : to_string(gi);
        groupName = utils::strings::Trim(groupName);
        meshes.push_back(PolygonMesh(gii + 1, groupName));
    }

    // for every group of faces
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int gii = 0; gii < gis.size(); gii++)
    {
        int gi = gis[gii];
        Group& g = linesG[gi];
        PolygonMesh& mesh = meshes[gii];
        if (g.linesF_count == 0) continue;

        int* vidStart = ngons.pointer(g.linesF_startIndex);
        int* vidEnd = ngons.pointer(g.linesF_endIndex - 1) + ngons.size(g.linesF_endIndex - 1);

        // detect vMin and vMax to work only in that diapason - speed optimization for many small groups - anyway small group of faces present in some small diapason of V
        int vidMin = INT_MAX;
        int vidMax = 0;
        int* nextvid = vidStart;
        while (nextvid < vidEnd) // iterate all vertexes in this group
        {
            int vid = *nextvid;
            if (vid < vidMin) vidMin = vid;
            if (vid > vidMax) vidMax = vid;
            nextvid++;
        }
        if (vidMax < vidMin)
        {
            cout << "!!! MeshFile: failed to load group #" << gi << "-" << g.name << " since vidMax<vidMin=" << vidMax << "<" << vidMin << endl;
            assert(vidMax >= vidMin && "MeshFile: failed to load group");
            continue;
        }

        // detect what vertices are used by face
        int vidMinMax_size = vidMax - vidMin + 1;
        //vector<bool>  vused_bool(vidMinMax_size, false); // bit array
        const int bitsperint = sizeof(int) * 8;
        Vector<bool>  vused_bits;
        vused_bits.setConstant(vidMinMax_size, false); // bit array

        //v0 - one by one
        int vcount_local = 0;
        nextvid = vidStart;
        while (nextvid < vidEnd) // iterate all vertexes in this group
        {
            int vid = *nextvid;
            int index = vid - vidMin;
            //vused_bool[index] = true;
            int bitsIndex = index / bitsperint;
            int mask = 1 << (index % bitsperint);
            int value = vused_bits.data()[bitsIndex];
            if ((value & mask) == 0) vcount_local++;
            vused_bits.data()[bitsIndex] = value | mask;
            nextvid++;
        }
        //cout << "vcount_local=" << vcount_local << "   countOfValues=" << vused_bits.countOfValues(true) << endl;

        // copy vertixes
        // also, create mapping from global vertex index to local index - to be able later convert in opposite
        mesh.V.resize(vcount_local, 3);
        int vid_local = 0;
        Is map_global_local;
        map_global_local.resize(vidMinMax_size);

        //v0 - iterate bit per bit
        //for (int i = 0; i < vidMinMax_size; i++)
        //{
        //    if (vused_bits[i])
        //    {
        //        int vid = vidMin + i;
        //        mesh.V(vid_local, 0) = V_file(vid, 0); 
        //        mesh.V(vid_local, 1) = V_file(vid, 1);
        //        mesh.V(vid_local, 2) = V_file(vid, 2);
        //        map_global_local[i] = vid_local;// set mapping from file index to output index
        //        vid_local++;
        //    }
        //}
        // v1 - iterate int and inside int bits  - much faster
        P3* pVfilemin = &V_file.row(vidMin);
        P3* pVmesh = &mesh.V.row(0);
        int ibits_max_128 = vused_bits.size() / 128;
        for (int ibits = 0; ibits < ibits_max_128; ibits++)
        {
            __m128i bits128 = vused_bits.data128()[ibits];
            if (utils::sse::bitsIsSetCount(bits128) == 0) continue;
            for (int k = 0; k < 4; k++)
            {
                int bits = bits128.m128i_i32[k];
                for (int bitIndex = k * 32; bitIndex < k * 32 + 32; bitIndex++)
                {
                    if (bits & 1)
                    {
                        int i = ibits * 128 + bitIndex;
                        //v0
                        //int vid = vidMin + i;
                        //mesh.V.row(vid_local) = V_file.row(vid);
                        //v1
                        pVmesh[vid_local] = pVfilemin[i];
                        map_global_local[i] = vid_local;// set mapping from file index to output index
                        vid_local++;
                    }
                    bits >>= 1; // shift bits right
                }
            }
        }
        for (int i = vused_bits.size() - (vused_bits.size() % 128); i < vused_bits.size(); i++)
        {
            if (vused_bits[i])
            {
                int vid = vidMin + i;
                mesh.V.row(vid_local) = V_file.row(vid);
                map_global_local[i] = vid_local;// set mapping from file index to output index
                vid_local++;
            }
        }


        //copy faces
        mesh.ngons.resizeBegin(g.linesF_count, true);
        //v0 - simple
        //for (int i = 0; i < g.linesF_count; i++)
        //{
        //    mesh.ngons.capacity(i) = ngons.capacity(g.linesF_startIndex + i);
        //}
        //mesh.ngons.refreshAfterCapacitiesChanges();
        //for (int i = 0; i < g.linesF_count; i++)
        //{
        //    mesh.ngons.size(i) = mesh.ngons.capacity(i);
        //}

        //v1 - direct - faster
        auto info_local = mesh.ngons.InfoPOINTER();
        auto info_global = ngons.InfoPOINTER() + g.linesF_startIndex;
        int totalSize = 0;
        for (int i = 0; i < g.linesF_count; i++)
        {
            int size = info_global->size;
            info_local->size = size;
            info_local->positionInData = totalSize;
            totalSize += size;
            info_local++;
            info_global++;
        }
        mesh.ngons.resizeEnd(totalSize); // pass totalSize since we done direct initialization

        nextvid = vidStart;
        int* nextvid_write = mesh.ngons.pointer(0);
        while (nextvid < vidEnd) // iterate all vertexes in this group
        {
            int vid = *nextvid;
            *nextvid_write = map_global_local[vid - vidMin];
            nextvid++;
            nextvid_write++;
        }
    }


    return true;
}

bool MeshFile::LoadFromStl(const string& filename, vector<PolygonMesh>& meshes)
{
    meshes.clear();
    vector<vector<D>> V;
    vector<vector<I>> F;
    vector<vector<D>> N;
    if (!igl::readSTL(filename, V, F, N))
    {
        cout << "failed to read from stl file" << endl;
        return false;
    }

    meshes.push_back(PolygonMesh(1, "stl"));
    PolygonMesh& mesh = meshes.back();

    //
    // Copy verticies
    //
    mesh.V.resize(V.size(), 3);
    for (int i = 0; i < V.size(); i++)
    {
        assert(V[i].size() == 3);
        V3 v(V[i][0], V[i][1], V[i][2]);
        mesh.V.row(i) = v;
    }

    //
    // Copy faces
    //
    mesh.ngons.InitFromVectors(F);

    //
    // Merge vertexes
    //
    mesh.MergeVertexes(0.0001);

    return true;
}

bool MeshFile::LoadFrom(const string& filename, vector<PolygonMesh>& meshes, const vector<string>& loadOnlyGroups, bool separateGroups)
{
    bool loaded = false;
    meshes.clear();
    string ext = utils::strings::ToLower(utils::file::ExtractExtension(filename));
    ext = utils::strings::ToLower(ext);
    if (ext == "obj")
    {
        loaded = LoadFromObj(filename, meshes, loadOnlyGroups, separateGroups);
    }
    if (ext == "stl")
    {
        loaded = LoadFromStl(filename, meshes);
    }
    if (!loaded) return false;

    extern bool IsOmpEnabled;
    bool IsOmpEnabled_IsOnemesh = meshes.size() < 3;
    #pragma omp parallel for  if(IsOmpEnabled && !IsOmpEnabled_IsOnemesh)
    for (int i = 0; i < meshes.size(); i++)
    {
        meshes[i].Heal(IsOmpEnabled && IsOmpEnabled_IsOnemesh && meshes[i].V.rows() > 1000);
    }

    return loaded && meshes.size() > 0;
}


//******************************************************************
// SaveToObj
//******************************************************************


void VertexesToStr(const P3s& V, const Is& V_IndexesInFile_GlobalOrLocal, string& s)
{
    s = "";
    bool checkGlobalIndexes = (V_IndexesInFile_GlobalOrLocal.size() != 0);
    for (int vi = 0; vi < V.rows(); vi++)
    {
        if (checkGlobalIndexes && V_IndexesInFile_GlobalOrLocal[vi] < 0) continue; // write only local vertexes
        s += "v";
        for (int k = 0; k < 3; k++)
        {
            s += " " + to_string(V(vi, k));//TODO optimize to avoid memory realocations + use direct pointer instead of V(vi, k)
        }
        s += "\n";
    }
}

void NgonsToStr(const CompactVectorVector<int>& ngons, const Is& V_IndexesInFile_GlobalOrLocal, int vertextesStartIndexShiftGlobal, int vertextesStartIndexShiftLocal, string& s)
{
    s = "";
    bool checkGlobalIndexes = (V_IndexesInFile_GlobalOrLocal.size() != 0);
    for (int fi = 0; fi < ngons.size(); fi++)
    {
        s += "f";
        for (int i = 0; i < ngons.size(fi); i++)
        {
            int vi = ngons(fi, i);
            if (checkGlobalIndexes)
            {
                int index = V_IndexesInFile_GlobalOrLocal[vi];
                if (index < 0)
                {
                    // global index
                    vi = (-index - 1);
                }
                else
                {
                    // local index
                    vi = (index - 1) + vertextesStartIndexShiftLocal;
                }
            }
            else
            {
                vi += vertextesStartIndexShiftLocal;
            }
            vi += vertextesStartIndexShiftGlobal;
            vi++;
            s += " " + to_string(vi);//TODO optimize to avoid memory realocations
        }
        s += "\n";
    }
}

bool MeshFile::SaveToObj(const string& filename, const PolygonMeshes& polygonMeshes, int vertextesStartIndexShift)
{
    const vector<const PolygonMesh*>& meshes = polygonMeshes.meshes;


    vector<string> textes;
    textes.resize(meshes.size() * 3 + 100);
    int textesadded = 0;
    extern bool IsOmpEnabled;


    //
    // Add header
    //
    if (vertextesStartIndexShift != 0) // if we are appending few files into one obj
    {
        textes[textesadded++] = "\n\n\n==========================================================================\n\n\n";
    }
    else
    {
        textes[textesadded++] = "# PolygonMeshes exporter\n";
        textes[textesadded++] = "# File Created: ";
        textes[textesadded++] = utils::time::TimeToStr();
        textes[textesadded++] = "\n";
    }
    textes[textesadded++] = "# " + to_string(polygonMeshes.GetVertexesCount()) + " vertices\n";
    textes[textesadded++] = "# " + to_string(polygonMeshes.GetNgonsCount()) + " faces\n";
    textes[textesadded++] = "# " + to_string(polygonMeshes.GetGroupsCount()) + " groups\n";
    textes[textesadded++] = "\n";
    textes[textesadded++] = "\n";


    //
    // Gather common vertexes from all meshes 
    //
    P3s V_common;     // Mesh vertexes (each row has 3 vertexes coordinates)
    polygonMeshes.GetCommonVertexes(V_common);

    //
    // Add vertexes
    //
    textes[textesadded++] = "\n";
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size() + 1; i++)
    {
        if (i == 0)
        {
            VertexesToStr(V_common, Is(), textes[textesadded + i]);
        }
        else
        {
            const auto &m = meshes[i - 1];
            VertexesToStr(m->V, m->VMergeInfo.V_IndexesInFile_GlobalOrLocal, textes[textesadded + i]);
        }
    }
    textesadded += 1 + meshes.size();


    //
    // Calculate shift indexes
    //
    vector<int> vertextesStartIndexShiftLocals;
    vertextesStartIndexShiftLocals.resize(meshes.size());
    int vertextesStartIndexShiftLocal = V_common.size(); // start writing local vertexes just after common vertexes
    for (int i = 0; i < meshes.size(); i++)
    {
        vertextesStartIndexShiftLocals[i] = vertextesStartIndexShiftLocal;
        vertextesStartIndexShiftLocal += meshes[i]->GetVertexesCountLocal();
    }


    //
    // Add faces
    //
    textes[textesadded++] = "\n";
    #pragma omp parallel for  if(IsOmpEnabled)
    for (int i = 0; i < meshes.size(); i++)
    {
        string& facesHeader = textes[textesadded + i * 2 + 0];
        string& facesBody = textes[textesadded + i * 2 + 1];
        const auto &m = meshes[i];
        facesHeader = "";
        if (i == 0 || meshes[i - 1]->groupid != meshes[i]->groupid) // some meshes may belong to one original mesh, so start groups only once per each original mesh
        {
            string groupname = empty(meshes[i]->groupname) ? "M" + to_string(m->groupid) : meshes[i]->groupname;
            facesHeader += "\n\n#\n#  " + groupname + "\n#\ng " + groupname + "\n\n";
        }
        facesHeader += "# " + to_string(m->ngons.size()) + " faces\n";
        NgonsToStr(m->ngons, m->VMergeInfo.V_IndexesInFile_GlobalOrLocal, vertextesStartIndexShift, vertextesStartIndexShiftLocals[i], facesBody);
    }
    textesadded += meshes.size() * 2;


    //
    // Save to file
    //
    bool appendToFile = (vertextesStartIndexShift > 0);
    textes.resize(textesadded);
    bool res = utils::file::Write(filename, textes, appendToFile);
    return res;
}

bool MeshFile::IsFileTypeSupported(const string& filename)
{
    auto supported = SupportedFileExtensions();
    string ext = utils::strings::ToLower(utils::file::ExtractExtension(filename));
    return utils::stdvector::exists(supported, ext);
}

vector<string> MeshFile::SupportedFileExtensions()
{
    vector<string> supported;
    supported.push_back("obj");
    supported.push_back("stl");
    return supported;
}
