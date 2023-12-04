
#ifndef TRIANGLES_H
#define TRIANGLES_H

#include <arm_neon.h>
#include <simd/simd.h>
#include <array>
#include <iostream>
#include "RenderTypes.h"

enum StepIndex{
    XBCoordStepIndex = 0,
    YBCoordStepIndex = 1,
    XUVStepIndex = 2,
    YUVStepIndex = 3,

    XNormalStepIndex = 4,
    YNormalStepIndex = 5,
    ZNormalStepIndex = 5,
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
    PreCalTriangle(Triangle &triangle, int resolution);
    simd::ushort4 bounds;
    PreCalValue xBCoord;
    PreCalValue yBCoord;
    PreCalValue xUV;
    PreCalValue yUV;
    PreCalValue xNormal;
    PreCalValue yNormal;
    PreCalValue zNormal;
    PreCalValue depth;
    unsigned int meshID;
}PreCalTriangle;


typedef struct ShaderTriangle{
    simd::float4 normal;
    simd::float2 uv;
    unsigned int meshID;
}ShaderTriangle;


typedef struct SteppedTriangle{

    public:
    unsigned int meshID;
    simd::ushort4 bounds;

    alignas(16) std::array<float, 8> values;
    SteppedTriangle() = default;
    SteppedTriangle(PreCalTriangle triangle, int resolution_);

    void StepX();
    void StepY();

    void SetX(int xSet);
    void SetY(int ySet);

    private:

    void SetConstants(int resolution_);
    void SetXValues(const std::array<PreCalValue, 8> &preCalValues);
    void SetYValues(const std::array<PreCalValue, 8> &preCalValues);

    // --- Step Values --- //

    float32x4x2_t v_deltaXs;
    float32x4x2_t v_initials;
    float32x4x2_t v_deltaYs;
    float32x4x2_t v_valueYs;

    // --- Constants --- //

    float invRez;

    int resolution;

}SteppedTriangle;

#endif