#pragma once


struct FaceConstrain
{
    int Id;// index in loop that holds this struct
    int FaceId;
    int EdgeId;

    V3 DirectionX;
    V3 DirectionY;
    V3 DirectionY_Corrected;
    D DirectionY_Corrected_Angle;

    FaceConstrain(int _id, const int& _faceId, const int& _edgeId)
        : Id(_id), FaceId(_faceId), EdgeId(_edgeId), DirectionX(V3(0, 0, 0)), DirectionY(V3(0, 0, 0)), DirectionY_Corrected(V3(0, 0, 0)), DirectionY_Corrected_Angle(0)
    {
    }
};