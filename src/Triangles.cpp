
#include "Triangles.h"

// --- Stepped Triangle --- //

// --- Constructor --- //

SteppedTriangle::SteppedTriangle(PreCalTriangle triangle, int resolution_, int threadCount, int threadIndex){

    SetConstants(resolution_, threadCount, threadIndex);

    std::array<PreCalValue, 8> preCalValues;

    preCalValues[XBCoordStepIndex] = triangle.xBCoord;
    preCalValues[YBCoordStepIndex] = triangle.yBCoord;
    preCalValues[_SpacerA] = PreCalValue{0,0,0};
    preCalValues[_SpacerB] = PreCalValue{0,0,0};

    preCalValues[ColorRStepIndex] = triangle.colorR;
    preCalValues[ColorGStepIndex] = triangle.colorG;
    preCalValues[ColorBStepIndex] = triangle.colorB;
    preCalValues[DepthStepIndex] = triangle.depth;

    SetXValues(preCalValues);

    SetYValues(preCalValues);

    SetX(0);

}

// --- Step Functions --- //

// --- Step X --- //

#ifdef SIMD

void SteppedTriangle::StepX(){

    // Values += DeltaX

    float32x4_t v_values1 = vld1q_f32(values.data());                           // Load Values
    float32x4_t v_values2 = vld1q_f32(values.data() + 4);

    v_values1 = vaddq_f32(v_values1, v_deltaXs.val[0]);                         // Add DeltaX To Values
    v_values2 = vaddq_f32(v_values2, v_deltaXs.val[1]);

    vst1q_f32(values.data(), v_values1);                                        // Store Values
    vst1q_f32(values.data() + 4, v_values2);

}

void SteppedTriangle::SetX(int x){

    // Values = Initial + Y + x * DeltaX

    float32x4_t v_values1 = vaddq_f32(v_initials.val[0], v_valueYs.val[0]);     // Set Values To Initial + Y
    float32x4_t v_values2 = vaddq_f32(v_initials.val[1], v_valueYs.val[1]);

    float32x4_t v_stepX1 = vmulq_n_f32(v_deltaXs.val[0], x);                    // Multiply DeltaX By X
    float32x4_t v_stepX2 = vmulq_n_f32(v_deltaXs.val[1], x);

    v_values1 = vaddq_f32(v_values1, v_stepX1);                                 // Add StepX To Values
    v_values2 = vaddq_f32(v_values2, v_stepX2);

    vst1q_f32(values.data(), v_values1);                                        // Store Values
    vst1q_f32(values.data() + 4, v_values2);

}

void SteppedTriangle::StepY(){

    // Y Values += DeltaY
    // Values = Initial Values + Y Values

    v_valueYs.val[0] = vaddq_f32(v_valueYs.val[0], v_deltaYs.val[0]);           // Add DeltaY To Y ValueS
    v_valueYs.val[1] = vaddq_f32(v_valueYs.val[1], v_deltaYs.val[1]);

    alignas(16) float32x4x2_t v_values;                                         // Declare Values (Uninitialized)

    v_values.val[0] = vaddq_f32(v_initials.val[0], v_valueYs.val[0]);           // Set Values To Initial + Y
    v_values.val[1] = vaddq_f32(v_initials.val[1], v_valueYs.val[1]);

    vst1q_f32(values.data(), v_values.val[0]);                                  // Store Values
    vst1q_f32(values.data() + 4, v_values.val[1]);

}

#else

void SteppedTriangle::StepX(){

    for(int i = 0; i < 8; ++i){
        values[i] += deltaXs[i];
    }

}

void SteppedTriangle::SetX(int x){
    
    for(int i = 0; i < 8; ++i){
        values[i] = initials[i] + valueYs[i] + x * deltaXs[i];
    }
    
}

void SteppedTriangle::StepY(){

    for(int i = 0; i < 8; ++i){
        float valueY = valueYs[i] + deltaYs[i];
        valueYs[i] = valueY;
        values[i] = initials[i] + valueY;
    }

}

#endif

// --- Step Y --- //





void SteppedTriangle::SetConstants(int resolution_, int stepSizeY_, int offset_){
    invRez = 2.0 / resolution_;
    resolution = resolution_;
    stepSizeY = stepSizeY_;
    offset = offset_;
}

// --- Private --- //

// --- Set Functions --- //

#ifdef SIMD

void SteppedTriangle::SetXValues(const std::array<PreCalValue, 8> &preCalValues){
    
    // --- Delta X Values --- // --- ( X * invRez ) --- //

    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[XBCoordStepIndex].x, v_deltaXs.val[0], 0);
    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[YBCoordStepIndex].x, v_deltaXs.val[0], 1);
    v_deltaXs.val[0] = vsetq_lane_f32(0, v_deltaXs.val[0], 2);
    v_deltaXs.val[0] = vsetq_lane_f32(0, v_deltaXs.val[0], 3);

    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[ColorRStepIndex].x, v_deltaXs.val[1], 0);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[ColorGStepIndex].x, v_deltaXs.val[1], 1);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[ColorBStepIndex].x, v_deltaXs.val[1], 2);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[DepthStepIndex].x, v_deltaXs.val[1], 3);

}

void SteppedTriangle::SetYValues(const std::array<PreCalValue, 8> &preCalValues){

    // --- Initial Values --- // --- ( O - X + Y * (offset * invRez - 1) ) --- //

    v_initials.val[0] = vsetq_lane_f32(preCalValues[XBCoordStepIndex].o - preCalValues[XBCoordStepIndex].x + preCalValues[XBCoordStepIndex].y * (offset * invRez - 1), v_initials.val[0], 0);
    v_initials.val[0] = vsetq_lane_f32(preCalValues[YBCoordStepIndex].o - preCalValues[YBCoordStepIndex].x + preCalValues[YBCoordStepIndex].y * (offset * invRez - 1), v_initials.val[0], 1);
    v_initials.val[0] = vsetq_lane_f32(0, v_initials.val[0], 2);
    v_initials.val[0] = vsetq_lane_f32(0, v_initials.val[0], 3);

    v_initials.val[1] = vsetq_lane_f32(preCalValues[ColorRStepIndex].o - preCalValues[ColorRStepIndex].x + preCalValues[ColorRStepIndex].y * (offset * invRez - 1), v_initials.val[1], 0);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[ColorGStepIndex].o - preCalValues[ColorGStepIndex].x + preCalValues[ColorGStepIndex].y * (offset * invRez - 1), v_initials.val[1], 1);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[ColorBStepIndex].o - preCalValues[ColorBStepIndex].x + preCalValues[ColorBStepIndex].y * (offset * invRez - 1), v_initials.val[1], 2);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[DepthStepIndex].o - preCalValues[DepthStepIndex].x + preCalValues[DepthStepIndex].y * (offset * invRez - 1), v_initials.val[1], 3);

    // --- Delta Y Values --- // --- ( Y * invRez * stepSizeY ) --- //

    v_deltaYs.val[0] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[XBCoordStepIndex].y, v_deltaYs.val[0], 0);
    v_deltaYs.val[0] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[YBCoordStepIndex].y, v_deltaYs.val[0], 1);
    v_deltaYs.val[0] = vsetq_lane_f32(0, v_deltaYs.val[0], 2);
    v_deltaYs.val[0] = vsetq_lane_f32(0, v_deltaYs.val[0], 3);

    v_deltaYs.val[1] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[ColorRStepIndex].y, v_deltaYs.val[1], 0);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[ColorGStepIndex].y, v_deltaYs.val[1], 1);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[ColorBStepIndex].y, v_deltaYs.val[1], 2);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * stepSizeY * preCalValues[DepthStepIndex].y, v_deltaYs.val[1], 3);

    // --- Y Values --- // --- ( 0 ) --- //

    v_valueYs.val[0] = vmovq_n_f32(0);

    v_valueYs.val[1] = vmovq_n_f32(0);

}

#else

void SteppedTriangle::SetXValues(const std::array<PreCalValue, 8> &preCalValues){
    // DeltaX Values = x * invRez
    // InvRez = 2.0 / resolution. So resolution * DeltaX = 2*x
    // Initial values is o - x - y. Initial value + DeltaX * resolution = o + x - y

    for(int i = 0; i < 8; i++){
        deltaXs[i] = preCalValues[i].x * invRez;
    }

}

void SteppedTriangle::SetYValues(const std::array<PreCalValue, 8> &preCalValues){

    // DeltaY Value = y * invRez * stepSizeY
    // y * invRez is the change in y per pixel
    // StepSizeY is to account for thread offset. (StepSizeY == ThreadCount)

    for(int i = 0; i < 8; i++){
        deltaYs[i] = preCalValues[i].y * invRez * stepSizeY;
    }

    // Initial Value = o - x - y + y * offset * invRez
    // Since coordinates start at -1 and go to 1, the initial value is o - x - y, (origin + offset at (-1,-1))
    // Each thread starts at a different y value, so y * offset * invRez is added, (deltaY * threadIndex)

    for(int i = 0; i < 8; i++){
        valueYs[i] = 0;
        initials[i] = preCalValues[i].o - preCalValues[i].x - preCalValues[i].y + preCalValues[i].y * offset * invRez;
    }

}

#endif

// --- PreCalTriangle --- //

// Converts 3 vertices into a precalculated triangle
// PreCalculated triangles are used to speed up the rendering process
// PreCalTriangles:
// PreCalTriangles have 3 values, o, x and y
// To calculate a value, the following can be used:
// Value = oValue + xCoord * xValue + yCoord * yValue
// xCoord and yCoord are the coordinates -1 to 1 of the pixel being drawn
PreCalTriangle::PreCalTriangle(Vertex vertices[3], int resolution){

    // Doubles are used to reduce innacuracy errors
    // Per pixel this is constant time
    // However, the final triangle uses floats

    // --- Positions --- //

    simd_double3 p0 = (simd_double3){vertices[0].position.x,vertices[0].position.y,vertices[0].position.z};
    simd_double3 p1 = (simd_double3){vertices[1].position.x,vertices[1].position.y,vertices[1].position.z};
    simd_double3 p2 = (simd_double3){vertices[2].position.x,vertices[2].position.y,vertices[2].position.z};

    // --- Barycentric Coordinates --- //

    double dInv = 1.0f / ( (p1.y - p2.y) * (p0.x - p2.x) - (p2.y - p0.y) * (p2.x - p1.x) );

    xBCoord.o = (p2.x * (p2.y - p1.y) - p2.y * (p2.x - p1.x)) * dInv;
    yBCoord.o = (p2.y * (p2.x - p0.x) - p2.x * (p2.y - p0.y)) * dInv;

    xBCoord.x = ((1 - p2.x) * (p1.y - p2.y) - p2.y * (p2.x - p1.x)) * dInv - xBCoord.o;
    yBCoord.x = ((1 - p2.x) * (p2.y - p0.y) - p2.y * (p0.x - p2.x)) * dInv - yBCoord.o;

    xBCoord.y = ((1 - p2.y) * (p2.x - p1.x) - p2.x * (p1.y - p2.y)) * dInv - xBCoord.o;
    yBCoord.y = ((1 - p2.y) * (p0.x - p2.x) - p2.x * (p2.y - p0.y)) * dInv - yBCoord.o;

    // --- Depth --- //

    double z0 = - p0.z / (abs(p0.z) + 1);
    double z1 = - p1.z / (abs(p1.z) + 1);
    double z2 = - p2.z / (abs(p2.z) + 1);

    depth = PreCalValue(xBCoord, yBCoord, z0, z1, z2);

    // --- Color --- //

    colorR = PreCalValue(xBCoord, yBCoord, vertices[0].color.r, vertices[1].color.r, vertices[2].color.r);
    colorG = PreCalValue(xBCoord, yBCoord, vertices[0].color.g, vertices[1].color.g, vertices[2].color.g);
    colorB = PreCalValue(xBCoord, yBCoord, vertices[0].color.b, vertices[1].color.b, vertices[2].color.b);

    // --- Normal --- //

    normal.xyz = simd::normalize(simd::cross(vertices[1].position - vertices[0].position, vertices[2].position - vertices[0].position));
    if(normal.z > 0){normal = -normal;}

    normal.w = 0;

    // --- Bounds --- //

    int xPixel0 = std::floor((p0.x + 1) * 0.5 * resolution);
    int xPixel1 = std::floor((p1.x + 1) * 0.5 * resolution);
    int xPixel2 = std::floor((p2.x + 1) * 0.5 * resolution);

    int yPixel0 = std::floor((p0.y + 1) * 0.5 * resolution);
    int yPixel1 = std::floor((p1.y + 1) * 0.5 * resolution);
    int yPixel2 = std::floor((p2.y + 1) * 0.5 * resolution);

    int bias = 1; // Prevents edge pixels from being culled

    const int maxUShort = 65535;

    bounds.x = (unsigned short)std::max(std::min(std::min(std::min(xPixel0,xPixel1),xPixel2) - bias,maxUShort),0);      // Low X
    bounds.z = (unsigned short)std::max(std::min(std::max(std::max(xPixel0,xPixel1),xPixel2) + bias,maxUShort),0);      // High X
    
    bounds.y = (unsigned short)std::max(std::min(std::min(std::min(yPixel0,yPixel1),yPixel2) - bias,maxUShort),0);      // Low Y
    bounds.w = (unsigned short)std::max(std::min(std::max(std::max(yPixel0,yPixel1),yPixel2) + bias,maxUShort),0);      // High Y

}



PreCalValue::PreCalValue(PreCalValue xBC, PreCalValue yBC, double a, double b, double c){
    o = a * xBC.o + b * yBC.o + c * (1.0 - xBC.o - yBC.o);
    x = a * xBC.x + b * yBC.x - c * (xBC.x + yBC.x);
    y = a * xBC.y + b * yBC.y - c * (xBC.y + yBC.y);
}