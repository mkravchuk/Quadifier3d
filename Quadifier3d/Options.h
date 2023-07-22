#pragma once

enum class MeshConstrainsType;
enum class MeshSimplificationAlgorithm;


class Options
{
public:
    string APPName;
    Options();
    static void Update_OmpCpuCores();
    void SetOpenGLOptions();
    void SetMeshLogicOptions(OpenGL_Window& window);
    void SetViewerOptions(OpenGL_Window& window);
    void AddOptions(OpenGL_Window& window);

    void AddOptions_Common(OpenGL_Window& window);
    /**** App ************************************************/
    void AddOptions_App(OpenGL_Window& window);

    /**** GuiStyle ************************************************/
    NanoguiTheme::NanoguiThemeColorStyle ThemeColorStyle;
    void AddOptions_GuiStyle(OpenGL_Window& window);
    function<void()> On_OptionChanged_GuiStyle;

    /**** MeshSimplification ***********************************/
    void AddOptions_MeshSimplification(OpenGL_Window& window);
    function<void()> On_OptionChanged_MeshSimplification;

    /**** Draw ***************************************************/
    void AddOptions_Viewer_Viewport(OpenGL_Window& window);
    void AddOptions_Viewer_Show(OpenGL_Window& window);
    void AddOptions_DrawDebugInfo(OpenGL_Window& window);
    void AddOptions_Highlight(OpenGL_Window& window);
    void AddOptions_OpenGL(OpenGL_Window& window, bool show_tech_options = true, bool show_visual_options = true);
    function<void()> On_OptionChanged_Draw;
    function<void(bool crease_normals, D crease_normals_angle)> On_OptionChanged_Draw_Crease;
    function<void(bool reinit_opengl)> On_OptionChanged_OpenGL;

    /**** Mesh *********************************************/
    void AddOptions_Mesh(OpenGL_Window& window);
    function<void()> On_OptionChanged_Mesh;
    void AddOptions_MeshHeal(OpenGL_Window& window);
    function<void()> On_MeshHeal_Executed;


    /**** Mesh Topology ******************************************/
    void AddOptions_MeshTopology(OpenGL_Window& window);
    function<void()> On_OptionChanged_MeshTopology;
    function<void()> On_OptionChanged_MeshTopology_TestPerformanceCall;

    /**** Stream *********************************************/
    void AddOptions_Stream(OpenGL_Window& window);
    function<void()> On_OptionChanged_Stream;


    /**** Stream Topology ******************************************/
    void AddOptions_StreamTopology(OpenGL_Window& window);
    function<void()> On_OptionChanged_StreamTopology;
    function<void()> On_OptionChanged_StreamTopology_TestPerformanceCall;


    /**** CONSTRAINS *****************************************/
    void AddOptions_Constrain(OpenGL_Window& window);
    function<void()> On_OptionChanged_Constrain;

    /**** CONSTRAINS UV **************************************/
    void AddOptions_ConstrainUV(OpenGL_Window& window);


    /**** NROSY  FIELD ****************************************/
    void AddOptions_MeshLinearSolver(OpenGL_Window& window);
    void AddOptions_LDLTFloatSolver(OpenGL_Window& window);
    void AddOptions_LinearEquationSolver(OpenGL_Window& window);
    void AddOptions_SolverNrosy__short(OpenGL_Window& window, bool add_performance_options);
    void AddOptions_SolverNrosy(OpenGL_Window& window, bool showNrosyDrawOptions, bool showNrosyOptions);
    void AddOptions_SolverAngleBound__short(OpenGL_Window& window, bool add_performance_options);
    void AddOptions_SolverAngleBound(OpenGL_Window& window);
    void AddOptions_SolverUV__short(OpenGL_Window& window, bool add_performance_options);
    void AddOptions_SolverUV(OpenGL_Window& window);
    function<void()> On_OptionChanged_Solver;


    /**** Divider *********************************************/
    void AddOptions_Divider(OpenGL_Window& window);
    void AddOptions_Divider__short(OpenGL_Window& window);
    function<void()> On_OptionChanged_Divider;
    void AddOptions_StreamAdjuster(OpenGL_Window& window);
    void AddOptions_Joiner(OpenGL_Window& window);
    void AddOptions_Joiner___short(OpenGL_Window& window);
    function<void()> On_OptionChanged_Joiner;
    void AddOptions_DividerIterator(OpenGL_Window& window);
    void AddOptions_DividerIterator__short(OpenGL_Window& window);
    function<void()> On_OptionChanged_DividerIterator;
    void Options::AddOptions_DividerOptimalConnector(OpenGL_Window& window);
    function<void()> On_OptionChanged_DividerOptimalConnector;
    void Options::AddOptions_DividerLogicConnector(OpenGL_Window& window);
    function<void()> On_OptionChanged_DividerLogicConnector;


    void AddOptions_MeshCutter(OpenGL_Window& window);
    function<void()> On_OptionChanged_MeshCutter;

    /**** Mesher  *********************************************/
    void AddOptions_Mesher(OpenGL_Window& window);
    void AddOptions_Mesher__short(OpenGL_Window& window);
    function<void(bool call_UpdateViewerMeshFromModel)> On_OptionChanged_Mesher;
    bool Mesher__debug_draw_cuttedMesh;
    void Update_Mesher__saved_draw_cutted_mesh();
    function<void(string)> On_Mesher_SaveDebugedMeshToFile;
    function<void(string)> On_Mesher_SaveQuadMeshToFile;


    /**** FILE ************************************************/
    enum class TestFiles
    {
        circle, circle1, circle2, test3, qq1, qq2, /*qq2Ind, */ quad, quad_ngon, ex1,/*ex2, */ trymach1, trymach2, dysk, dysk_quad,

        korzyna_small_1, korzyna_small_2, korzyna_small_3, korzyna_small_4, korzyna_small_5, korzyna_small_3_triangle,
        _152687_1, _152687_2, _152687_3,
        Lock_door_1, Lock_door_2, Lock_door_3, Lock_door_4,
        door1, door7, door8, kolo, detal1, detal11, detal2, detal22, detal222, detal2222, _132, vypuklistj, 

        charA, charF, charH, charn, charR, charS, charT,
        korzyna_small, _100750, _119276, _139244, _2, trymach3, trymach4, _10308589, _139242, _143205, f1, f2, f3, JARWIT1, JARWIT2, coffee2_dno, _042, uu, spiral, _114575, _13, _272, DaskLamp27, DaskLamp29, DaskLamp50, DaskLamp502, 
        pan, chair02, krug, d1, kyt, kyt_adaptive,
        dush1, dush1D, dush1I, dush1A, dush2,
        fandisk, joint, beetle, sculpt, casting, pear, mask, cup, planesphere, sharpsphere, twirl, block, mannequinmc, holes3, bunny,
        korzyna, _191117, _191117M, coffee1, coffee2, coffee3, mug, chair
    };
    vector<string>  TestFilesStr = {
        "#trivial", "circle", "circle1", "circle2", "test3", "qq1", "qq2", /*"qq2Ind", */ "quad", "quad_ngon", "ex1", /*"ex2", */ "trymach1","trymach2", "dysk", "dysk_quad",

        "#simple", "korzyna_small_1", "korzyna_small_2" , "korzyna_small_3", "korzyna_small_4", "korzyna_small_5" , "korzyna_small_3_triangle",
        "152687_1", "152687_2", "152687_3",
        "Lock_door_1", "Lock_door_2", "Lock_door_3", "Lock_door_4",
        "door1", "door7", "door8", "kolo", "detal1", "detal11", "detal2", "detal22", "detal222", "detal2222","132","vypuklistj",

        "#chars", "charA", "charF", "charH", "charn", "charR", "charS", "charT",
        "#complex", "korzyna_small", "100750", "119276", "139244", "2", "trymach3", "trymach4", "10308589", "139242", "143205", "f1", "f2", "f3", "JARWIT1", "JARWIT2", "coffee2_dno","042","uu", "spiral", "114575", "13", "272","DaskLamp27","DaskLamp29","DaskLamp50","DaskLamp502", 
        "#complex", "pan", "chair02", "krug","d1", "kyt", "kyt_adaptive",
        "#dush", "dush1", "dush1D", "dush1I", "dush1A", "dush2",
        "#organic", "fandisk", "joint", "beetle", "sculpt", "casting", "pear", "mask", "cup", "planesphere", "sharpsphere", "twirl", "block", "mannequinmc", "holes3", "bunny",
        "#huge", "korzyna", "191117", "191117M", "coffee1", "coffee2", "coffee3", "mug", "chair"
    };
    enum class TestFilesDensity
    {
        low, medium, high
    };
    vector<string>  TestFilesDensityStr = { "low", "medium", "high" };
    struct PresetFile
    {
        int Index; // index in 'vector<PresetFile> PresetFiles'
        string customTestFile;
        TestFiles testFile;
        TestFilesDensity density;
        double meshSize;
        string loadOnlyObjectNames;
        string ToString(const Options& options) const
        {
            string meshSizeStr = to_string(meshSize);
            while (meshSizeStr.back() == '0') meshSizeStr.resize(meshSizeStr.size() - 1);
            string file = customTestFile.empty()
                ? options.TestFilesStr[static_cast<int>(testFile)] + "   " + options.TestFilesDensityStr[static_cast<int>(density)]
                : customTestFile;
            return file
                + "   " + meshSizeStr
                + "   " + loadOnlyObjectNames;
        }
    };


    bool PrecacheMeshFile;
    string CustomTestFile;
    TestFiles TestFile;
    TestFilesDensity TestFileDensity;
    string ShowOnlyMeshIds;
    string ShowOnlyObjectIds;
    string LoadOnlyObjectNames;
    bool ConvertMeshToFlatMesh;
    bool TestSpeed_ReadFileFromObj;
    bool TestSpeed_AddMeshes;
    bool SeparateGroups;

    string GetTestFileName();
    string GetOptionFile(string filename);
    void AddOptions_File(OpenGL_Window& window);
    function<void(bool alignCamera, bool resolve, bool testFileChanged)> On_OptionChanged_File;
    function<void()> On_RememberModelLocation_File;
    function<void(string filename)> On_OpenObjFile_File;
    function<void()> On_SelectByMouse_ShowOnly;



    vector<PresetFile> PresetFiles;
    int CurrentPresetFileIndex;
    void AddPresetTestGroup(string presetGroupName);
    void AddPresetTestFile(TestFiles testFile, TestFilesDensity density, double meshSize = 1, string loadOnlyObjectNames = "");
    void AddPresetTestFile(string testFile, double meshSize = 1, string loadOnlyObjectNames = "");
    void SetPresetFileIndex(int index);
    void AddOptions_TestFilesPreset(OpenGL_Window& window);


    /**** KEYBOARD SHORTUCTS *******************************/
    bool callback_key_down(OpenGL_Window& window, int key, int modifier);
    bool callback_key_pressed(OpenGL_Window& window, unsigned int unicode_key, int modifiers);


private:
    //bool AddOptions_Test(OpenGL_Window& window);
};
