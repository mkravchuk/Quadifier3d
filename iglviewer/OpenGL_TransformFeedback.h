#pragma once
#include "OpenGL_Shared.h"

// This class wraps an OpenGL program composed of three shaders
class OpenGL_TransformFeedback
{
private:
    bool isCreated;
    bool RASTERIZER_DISCARD;
    bool query_primitivesWRITTEN;
    GLuint query;
public:
    GLenum primitiveMode;       // GL_POINTS, GL_TRIANGLES
    GLuint   primitivesWRITTEN; // primitives written after Begin+End

    GLuint tbo;                        // transformFeedback buffer id
    vector<string> variables;      // cached vaiables from shader
    vector<int> variablesSizesInBytes;  // sizes of those variables in bytes

    OpenGL_TransformFeedback();
    ~OpenGL_TransformFeedback();
    void Create(GLenum primitiveMode, bool RASTERIZER_DISCARD, bool query_primitivesWRITTEN);
    void Delete();
    void ClearCounters();  // call this method before draw (anyway)
    void Begin_CatchShadersOutput(); // call this method before draw (only if TransformFeedback is Enabled)
    void End_CatchShadersOutput(bool showDebugInfo = false);   // call this method after draw (only if TransformFeedback is Enabled)
    void BindBuffer(const MatrixXf& buffer, int countOfElementsInRow);
    int  GetVariablesSumSizeInBytes();
    template <class T>
    Matrix<T, Dynamic, Dynamic> LoadShadersOutputFromGPU() // load data catched by 'Begin_CatchShadersOutput'+'End_CatchShadersOutput' from GPU to CPU
    {
        if (primitivesWRITTEN == 0)
        {
            cout << "! warning   no data is catched by transform feedback, but requsted by user!" << endl;
            //assert(primitivesWRITTEN > 0 && "! warning   no data is catched by transform feedback, but requsted by user!");
        }
        Matrix<T, Dynamic, Dynamic> feedback;
        int feedback_index = 0;
        int floats_count = GetVariablesSumSizeInBytes() / sizeof(T);
        feedback.resize(primitivesWRITTEN, floats_count);
        glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, feedback_index, feedback.size() * sizeof(T), feedback.data());
        return feedback;
    }
};