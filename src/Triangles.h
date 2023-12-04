
#ifndef TRIANGLES_H
#define TRIANGLES_H

#include <arm_neon.h>
#include <simd/simd.h>
#include <array>
#include <iostream>
#include "RenderTypes.h"

// In most senarios, SIMD_X will make a huge improvement
// SIMD_X Will affect functions that are called T*R*R times

// In most senarios, SIMD_Y will make a little difference and may have a negative impact on performance
// SIMD_Y Will affect functions that are called T*R times

#define SIMD

enum StepIndex{
    XBCoordStepIndex = 0,
    YBCoordStepIndex = 1,
    _SpacerA = 2,
    _SpacerB = 3,

    ColorRStepIndex = 4,
    ColorGStepIndex = 5,
    ColorBStepIndex = 6,
    DepthStepIndex = 7
};


typedef struct PreCalValue{
    float o,x,y;
    PreCalValue() = default;
    PreCalValue(float oIn, float xIn, float yIn):o(oIn),x(xIn),y(yIn){};
    PreCalValue(PreCalValue xBC, PreCalValue yBC, double a, double b, double c);
}PreCalValue;


typedef struct PreCalTriangle{
    PreCalTriangle() = default;
    PreCalTriangle(Vertex vertices[3], int resolution);


    // Triangle Bounds are in pixels (unsigned short)
    // x,z = lowX,highX
    // y,w = lowY,highY
    simd::ushort4 bounds;
    simd::float4 normal;
    PreCalValue xBCoord;
    PreCalValue yBCoord;
    PreCalValue depth;
    PreCalValue colorR;
    PreCalValue colorG;
    PreCalValue colorB;

}PreCalTriangle;


typedef struct ShaderTriangle{
    simd::float4 normal;
    simd::float4 color;
}ShaderTriangle;


typedef struct SteppedTriangle{

    public:

    const PreCalTriangle *pcTriangle;

    alignas(16) std::array<float, 8> values;
    SteppedTriangle() = default;
    SteppedTriangle(PreCalTriangle triangle, int resolution_, int threadCount, int threadIndex);

    void StepX();
    void StepY();

    void SetX(int xp);

    private:

    void SetConstants(int resolution_, int stepSizeY_, int offset_);
    void SetXValues(const std::array<PreCalValue, 8> &preCalValues);
    void SetYValues(const std::array<PreCalValue, 8> &preCalValues);

    // --- Step Values --- //

    #ifdef SIMD
    float32x4x2_t v_deltaXs;
        float32x4x2_t v_initials;
        float32x4x2_t v_deltaYs;
        float32x4x2_t v_valueYs;

    #else
        std::array<float, 8> deltaXs;
        std::array<float, 8> initials;
        std::array<float, 8> deltaYs;
        std::array<float, 8> valueYs;
    #endif

    // --- Constants --- //

    float invRez;

    int resolution;

    int stepSizeY;

    int offset;

}SteppedTriangle;

#endif