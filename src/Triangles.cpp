
#include "Triangles.h"

/* 
A Stepped Trianle is basically a position and two vectors.
When vector X is added to the position, it is equvilent to calculating the next pixel in the x direction.
When vector Y is added to the position, it is equvilent to calculating the next pixel in the y direction.
Position represents the pixel at screen coordinates (0,0). (The pixel in the middle of the screen)
Triangles can be updated with StepX() and StepY(), to 'move' the triangle to the next pixel.
Or values can be set with SetX() and SetY().

StepX() and StepY() exclusively use addition. (Faster)
SetX() and SetY() use multiplication and addition. (Still fast but slower than Step)

PreCalTriangles have these values calculated, but no methods implemented.
PreCalTriangles are only used to pass information around, later to be converted to SteppedTriangles.
PreCalTriangles may be removed in the future with SteppedTriangles being used instead.
*/

// --- Stepped Triangle --- //

// --- Constructor --- //

SteppedTriangle::SteppedTriangle(PreCalTriangle triangle, int resolution_){

    SetConstants(resolution_);

    meshID = triangle.meshID;
    bounds = triangle.bounds;

    std::array<PreCalValue, 8> preCalValues;

    preCalValues[XBCoordStepIndex] = triangle.xBCoord;
    preCalValues[YBCoordStepIndex] = triangle.yBCoord;
    preCalValues[XUVStepIndex] = triangle.xUV;
    preCalValues[YUVStepIndex] = triangle.yUV;

    preCalValues[XNormalStepIndex] = triangle.xNormal;
    preCalValues[YNormalStepIndex] = triangle.yNormal;
    preCalValues[ZNormalStepIndex] = triangle.zNormal;
    preCalValues[DepthStepIndex] = triangle.depth;


    SetXValues(preCalValues);

    SetYValues(preCalValues);

    SetY(0);

    SetX(0);

}

// --- Step Functions --- //

// --- Step X --- //

void SteppedTriangle::StepX(){

    // Values += DeltaX

    float32x4_t v_values1 = vld1q_f32(values.data());                           // Load Values
    float32x4_t v_values2 = vld1q_f32(values.data() + 4);

    v_values1 = vaddq_f32(v_values1, v_deltaXs.val[0]);                         // Add DeltaX To Values
    v_values2 = vaddq_f32(v_values2, v_deltaXs.val[1]);

    vst1q_f32(values.data(), v_values1);                                        // Store Values
    vst1q_f32(values.data() + 4, v_values2);

}

void SteppedTriangle::SetX(int xSet){

    // Values = Initial + Y + x * DeltaX

    float32x4_t v_values1 = vaddq_f32(v_initials.val[0], v_valueYs.val[0]);     // Set Values To Initial + Y
    float32x4_t v_values2 = vaddq_f32(v_initials.val[1], v_valueYs.val[1]);

    float32x4_t v_stepX1 = vmulq_n_f32(v_deltaXs.val[0], xSet);                 // Multiply DeltaX By xSet
    float32x4_t v_stepX2 = vmulq_n_f32(v_deltaXs.val[1], xSet);

    v_values1 = vaddq_f32(v_values1, v_stepX1);                                 // Add StepX To Values
    v_values2 = vaddq_f32(v_values2, v_stepX2);

    vst1q_f32(values.data(), v_values1);                                        // Store Values
    vst1q_f32(values.data() + 4, v_values2);

}

// --- Step Y --- //

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

// --- Set Y --- //

void SteppedTriangle::SetY(int ySet){

    // Y Values = Y * DeltaY
    // Values = Initial Values + Y Values

    v_valueYs.val[0] = vmulq_n_f32(v_deltaYs.val[0], ySet);                     // Set Y Values To DeltaY * ySet
    v_valueYs.val[1] = vmulq_n_f32(v_deltaYs.val[1], ySet);

    alignas(16) float32x4x2_t v_values;                                         // Declare Values (Uninitialized)

    v_values.val[0] = vaddq_f32(v_initials.val[0], v_valueYs.val[0]);           // Set Values To Initial + Y
    v_values.val[1] = vaddq_f32(v_initials.val[1], v_valueYs.val[1]);

    vst1q_f32(values.data(), v_values.val[0]);                                  // Store Values
    vst1q_f32(values.data() + 4, v_values.val[1]);

}



// --- Private --- //

void SteppedTriangle::SetConstants(int resolution_){
    invRez = 2.0 / resolution_;
    resolution = resolution_;
}

// --- Set Functions (Constructor) --- //

void SteppedTriangle::SetXValues(const std::array<PreCalValue, 8> &preCalValues){
    
    // --- Delta X Values --- // --- ( X * invRez ) --- //

    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[0].x, v_deltaXs.val[0], 0);
    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[1].x, v_deltaXs.val[0], 1);
    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[2].x, v_deltaXs.val[0], 2);
    v_deltaXs.val[0] = vsetq_lane_f32(invRez * preCalValues[3].x, v_deltaXs.val[0], 3);

    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[4].x, v_deltaXs.val[1], 0);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[5].x, v_deltaXs.val[1], 1);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[6].x, v_deltaXs.val[1], 2);
    v_deltaXs.val[1] = vsetq_lane_f32(invRez * preCalValues[7].x, v_deltaXs.val[1], 3);

}

void SteppedTriangle::SetYValues(const std::array<PreCalValue, 8> &preCalValues){

    // --- Initial Values --- // --- ( O - X - Y ) --- //

    v_initials.val[0] = vsetq_lane_f32(preCalValues[0].o - preCalValues[0].x - preCalValues[0].y, v_initials.val[0], 0);
    v_initials.val[0] = vsetq_lane_f32(preCalValues[1].o - preCalValues[1].x - preCalValues[1].y, v_initials.val[0], 1);
    v_initials.val[0] = vsetq_lane_f32(preCalValues[2].o - preCalValues[2].x - preCalValues[2].y, v_initials.val[0], 2);
    v_initials.val[0] = vsetq_lane_f32(preCalValues[3].o - preCalValues[3].x - preCalValues[3].y, v_initials.val[0], 3);

    v_initials.val[1] = vsetq_lane_f32(preCalValues[4].o - preCalValues[4].x - preCalValues[4].y, v_initials.val[1], 0);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[5].o - preCalValues[5].x - preCalValues[5].y, v_initials.val[1], 1);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[6].o - preCalValues[6].x - preCalValues[6].y, v_initials.val[1], 2);
    v_initials.val[1] = vsetq_lane_f32(preCalValues[7].o - preCalValues[7].x - preCalValues[7].y, v_initials.val[1], 3);

    // --- Delta Y Values --- // --- ( Y * invRez ) --- //

    v_deltaYs.val[0] = vsetq_lane_f32(invRez * preCalValues[0].y, v_deltaYs.val[0], 0);
    v_deltaYs.val[0] = vsetq_lane_f32(invRez * preCalValues[1].y, v_deltaYs.val[0], 1);
    v_deltaYs.val[0] = vsetq_lane_f32(invRez * preCalValues[2].y, v_deltaYs.val[0], 2);
    v_deltaYs.val[0] = vsetq_lane_f32(invRez * preCalValues[3].y, v_deltaYs.val[0], 3);

    v_deltaYs.val[1] = vsetq_lane_f32(invRez * preCalValues[4].y, v_deltaYs.val[1], 0);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * preCalValues[5].y, v_deltaYs.val[1], 1);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * preCalValues[6].y, v_deltaYs.val[1], 2);
    v_deltaYs.val[1] = vsetq_lane_f32(invRez * preCalValues[7].y, v_deltaYs.val[1], 3);

    // --- Y Values --- // --- ( 0 ) --- //
    v_valueYs.val[0] = vmovq_n_f32(0.0f);
    v_valueYs.val[1] = vmovq_n_f32(0.0f);

}


// --- PreCalTriangle --- //

// Converts 3 vertices into a precalculated triangle
// PreCalculated triangles are used to speed up the rendering process
// PreCalTriangles:
// PreCalTriangles have 3 values, o, x and y
// To calculate a value, the following can be used:
// Value = oValue + xCoord * xValue + yCoord * yValue
// xCoord and yCoord are the coordinates -1 to 1 of the pixel being drawn
PreCalTriangle::PreCalTriangle(Triangle &triangle, int resolution){

    // Doubles are used to reduce innacuracy errors
    // Per pixel this is constant time
    // However, the final triangle uses floats

    meshID = triangle.meshID;

    // --- Positions --- //

    simd_double3 p0 = (simd_double3){triangle.vertices[0].x,triangle.vertices[0].y,triangle.vertices[0].z};
    simd_double3 p1 = (simd_double3){triangle.vertices[1].x,triangle.vertices[1].y,triangle.vertices[1].z};
    simd_double3 p2 = (simd_double3){triangle.vertices[2].x,triangle.vertices[2].y,triangle.vertices[2].z};

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

    // --- UV --- //

    xUV = PreCalValue(xBCoord, yBCoord, (double)triangle.textureCoordinates[0].x, (double)triangle.textureCoordinates[1].x, (double)triangle.textureCoordinates[2].x);
    yUV = PreCalValue(xBCoord, yBCoord, (double)triangle.textureCoordinates[0].y, (double)triangle.textureCoordinates[1].y, (double)triangle.textureCoordinates[2].y);

    // --- Normal --- //

    xNormal = PreCalValue(xBCoord, yBCoord, (double)triangle.normals[0].x, (double)triangle.normals[1].x, (double)triangle.normals[2].x);
    yNormal = PreCalValue(xBCoord, yBCoord, (double)triangle.normals[0].y, (double)triangle.normals[1].y, (double)triangle.normals[2].y);
    zNormal = PreCalValue(xBCoord, yBCoord, (double)triangle.normals[0].z, (double)triangle.normals[1].z, (double)triangle.normals[2].z);

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


// --- PreCalValue --- //
// xBC and yBC are basicaly barycentric coordinates.
// These are used to calculate the PreCalValue.
// Just a matrix multiplication.
PreCalValue::PreCalValue(PreCalValue xBC, PreCalValue yBC, double a, double b, double c){
    o = a * xBC.o + b * yBC.o + c * (1.0 - xBC.o - yBC.o);
    x = a * xBC.x + b * yBC.x - c * (xBC.x + yBC.x);
    y = a * xBC.y + b * yBC.y - c * (xBC.y + yBC.y);
}