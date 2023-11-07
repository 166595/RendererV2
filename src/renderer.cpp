#include "renderer.h"
#include "RenderTypes.h"
#include "Shaders.h"
#include <simd/simd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <arm_neon.h>

void Renderer::DrawEquation(){

    resolution = 2048;
    ColorBuffer pixelBuffer;
    pixelBuffer.length = 4*resolution*resolution;
    //pixelBuffer.c = new char[pixelBuffer.length];
    FileInterface fileInterface;
    fileInterface.LoadFrameToBuffer(&pixelBuffer,1);

    // Buffers used to store previous pixel row
    // Buffers are alternated between
    bool EquationLineBufferA[resolution+1];
    bool EquationLineBufferB[resolution+1];
    bool writeBufferA = false;

    // Initial Parameters
    float xScale = 5;
    float yScale = 5;
    float dx = xScale*(2.0f/(float)resolution);
    float dy = yScale*(2.0f/(float)resolution);

    // Create First EquationBufferLine
    FillEquationLineBuffer(EquationLineBufferA,xScale,-yScale-dy,resolution);
    // Draw each row of pixels
    for(int y = 0; y < resolution; y++){
        float _y = yScale*(2*(float)y/(float)resolution - 1);

        // Check current buffer
        if(writeBufferA){
            FillEquationLineBuffer(EquationLineBufferA,xScale,_y,resolution);
        }
        else{
            FillEquationLineBuffer(EquationLineBufferB,xScale,_y,resolution);
        }
        // Switch Write Buffer
        writeBufferA = !writeBufferA;

        for(int x = 0; x < resolution; x++){
            // Check pixel edges
            int edgeCount = 0;
            if(EquationLineBufferA[x]){edgeCount++;};
            if(EquationLineBufferA[x+1]){edgeCount++;};
            if(EquationLineBufferB[x]){edgeCount++;};
            if(EquationLineBufferB[x+1]){edgeCount++;};

            // If 1,2 or 3 edges, draw pixel
            if((edgeCount > 0)&&(edgeCount < 4)){
                char colorValue = (char)255;
                int index = 4*resolution*y + 4*x;
                pixelBuffer.c[index] = colorValue;
                pixelBuffer.c[index+1] = colorValue;
                pixelBuffer.c[index+2] = colorValue;
                pixelBuffer.c[index+3] = (char)255;
            }
        }
    }

    // Write Pixel Buffer To File
    fileInterface.resolution = resolution;
    fileInterface.CreateBMPFileWithBuffer(pixelBuffer);
    std::cout << "Finished Render\n";
}

void Renderer::FillEquationLineBuffer(bool *eLineBuffer, float xScale, float y, float resolution){
    for(int x = 0; x < resolution+1; x++){
        float _x = xScale*(2*(float)x/(float)resolution - 1);
        eLineBuffer[x] = (EquationShader(_x,y)>0);
    }
}

void Renderer::DrawShaderScreen(){

    resolution = 2048;

    FileInterface fileInterface;
    fileInterface.resolution = resolution;

    fileInterface.PrepBMPFileWithSize(4*resolution*resolution);

    unsigned int numThreads = std::thread::hardware_concurrency();

    std::cout << "Num Threads: " << numThreads << "\n";

    std::vector<std::thread> threads;

    std::mutex mtx;

    int bufferSize = 4*resolution;

    for(unsigned int i = 0; i < numThreads; i++){
        threads.push_back(std::thread([&, i]{

            float invDRez = 2.0f/(float)resolution;

            char *charBuffer = new char[bufferSize];

            for(int y = i; y < resolution; y += numThreads){

                int index = 0;

                for(int x = 0; x < resolution; x++){
                    FragmentShader(float(x)*invDRez - 1, float(y)*invDRez - 1, *(Color*)(&charBuffer[index]));
                    index += 4;
                }

                mtx.lock();
                fileInterface.SubmitCharBufferAsRow(charBuffer, bufferSize, y);
                mtx.unlock();

            }

            delete[] charBuffer;
        }));
    }

    for(auto& thread : threads){
        thread.join();
    }

    fileInterface.CloseBMPFile();
}

enum StepIndex{
    XBCoordStepIndex,
    YBCoordStepIndex,
    DepthStepIndex,
    ColorRStepIndex,
    ColorGStepIndex,
    ColorBStepIndex
};

//#define SIMD_X
//#define SIMD_Y

/* --- SIMD Tests ---
Rez: 4096 x 4096 
Triangles: 100
Threads: 8

No SIMD: 7 sec
SIMD X: 4 sec
SIMD Y: 7 sec
SIMD X & Y: 22 sec

*/



typedef struct SteppedTriangle{

    alignas(16) std::array<float, 8> values;

    #ifdef SIMD_X
        float32x4x2_t v_deltaXs;
    #else
        std::array<float, 8> deltaXs;
    #endif

    #ifdef SIMD_Y
        float32x4x2_t v_initials;
        float32x4x2_t v_deltaYs;

    #else
        std::array<float, 8> initials;
        std::array<float, 8> deltaYs;
    #endif

    std::array<float, 8> valueYs;

    double inverseDResolution;

    SteppedTriangle() = default;

    SteppedTriangle(PreCalTriangle triangle, int resolution, int stepSizeY, int offset){
        inverseDResolution = 2.0 / resolution;

        SetValues(triangle.xBCoord.o, triangle.xBCoord.x, triangle.xBCoord.y, resolution, stepSizeY, offset, XBCoordStepIndex);
        SetValues(triangle.yBCoord.o, triangle.yBCoord.x, triangle.yBCoord.y, resolution, stepSizeY, offset, YBCoordStepIndex);

        SetValues(triangle.depth.o, triangle.depth.x, triangle.depth.y, resolution, stepSizeY, offset, DepthStepIndex);

        SetValues(triangle.colorR.o, triangle.colorR.x, triangle.colorR.y, resolution, stepSizeY, offset, ColorRStepIndex);
        SetValues(triangle.colorG.o, triangle.colorG.x, triangle.colorG.y, resolution, stepSizeY, offset, ColorGStepIndex);
        SetValues(triangle.colorB.o, triangle.colorB.x, triangle.colorB.y, resolution, stepSizeY, offset, ColorBStepIndex);

        //xBCoord = SteppedValue(triangle.xBCoord.o, triangle.xBCoord.x, triangle.xBCoord.y, resolution, stepSizeY, offset);
        //yBCoord = SteppedValue(triangle.yBCoord.o, triangle.yBCoord.x, triangle.yBCoord.y, resolution, stepSizeY, offset);

        //depth = SteppedValue(triangle.depth.o, triangle.depth.x, triangle.depth.y, resolution, stepSizeY, offset);
        //colorR = SteppedValue(triangle.colorR.o, triangle.colorR.x, triangle.colorR.y, resolution, stepSizeY, offset);
        //colorG = SteppedValue(triangle.colorG.o, triangle.colorG.x, triangle.colorG.y, resolution, stepSizeY, offset);
        //colorB = SteppedValue(triangle.colorB.o, triangle.colorB.x, triangle.colorB.y, resolution, stepSizeY, offset);

    }


    void SetValues(float _o, float _x, float _y, int _resolution, int _stepSizeY, int offset, int index){

        values[index] = _o - _x - _y + _y * offset * inverseDResolution;
        valueYs[index] = 0;
        SetLaneValues(_o, _x, _y, _resolution, _stepSizeY, offset, index);

    }

    void SetLaneValues(float _o, float _x, float _y, int _resolution, int _stepSizeY, int offset, int index){
        #ifdef SIMD_X
            v_deltaXs.val[index] = vsetq_lane_f32(_x * inverseDResolution, v_deltaXs.val[index], lane);
            //vsetq_lane_f32(_x * inverseDResolution, v_deltaXs.val[index/4] + index%4, 0);
        #else
            deltaXs[index] = _x * inverseDResolution;
        #endif

        #ifdef SIMD_Y
            vsetq_lane_f32(_o - _x - _y + _y * offset * inverseDResolution, v_initials.val[index/4], 0);
            vsetq_lane_f32(_y * inverseDResolution * _stepSizeY, v_deltaYs.val[index/4], 0);
        #else
            initials[index] = values[index];
            deltaYs[index] = _y * inverseDResolution * _stepSizeY;
        #endif
    }


    void StepX(){

        #ifdef SIMD_X

            alignas(16) float32x4_t v_values1 = vld1q_f32(values.data());
            alignas(16) float32x4_t v_values2 = vld1q_f32(values.data() + 4);

            v_values1 = vaddq_f32(v_values1, v_deltaXs.val[0]);
            v_values2 = vaddq_f32(v_values2, v_deltaXs.val[1]);

            vst1q_f32(values.data(), v_values1);
            vst1q_f32(values.data() + 4, v_values2);

        #else

        for(int i = 0; i < 6; i++){
            values[i] += deltaXs[i];
        }

        #endif
    }

    void StepY(){

        #ifdef SIMD_Y

        alignas(16) float32x4_t v_valueY1 = vld1q_f32(valueYs.data());
        alignas(16) float32x4_t v_valueY2 = vld1q_f32(valueYs.data() + 4);

        v_valueY1 = vaddq_f32(v_valueY1, v_deltaYs.val[0]);
        v_valueY2 = vaddq_f32(v_valueY2, v_deltaYs.val[1]);

        alignas(16) float32x4x2_t v_values;

        v_values.val[0] = vaddq_f32(v_initials.val[0], v_valueY1);
        v_values.val[1] = vaddq_f32(v_initials.val[1], v_valueY2);

        vst1q_f32(values.data(), v_values.val[0]);
        vst1q_f32(values.data() + 4, v_values.val[1]);

        vst1q_f32(valueYs.data(), v_valueY1);
        vst1q_f32(valueYs.data() + 4, v_valueY2);
        
        #else

        for(int i = 0; i < 6; i++){
            valueYs[i] += deltaYs[i];
            values[i] = initials[i] + valueYs[i];
        }
        #endif
    }

}SteppedTriangle;

void Renderer::DrawTriangle(std::vector<Triangle> _triangles){

    resolution = RESOLUTION;

    std::vector<PreCalTriangle> triangles;
    for(auto t : _triangles){
        triangles.push_back(CreatePreCalTriangleFromVertices(t.vertex));
    }

    FileInterface fileInterface;
    fileInterface.resolution = resolution;

    fileInterface.PrepBMPFileWithSize(4*resolution*resolution);

    unsigned int numThreads = std::thread::hardware_concurrency();

    if(numThreads > MAX_THREADS){numThreads = MAX_THREADS;}

    //std::cout << "Num Threads: " << numThreads << "\n";

    std::vector<std::thread> threads;

    std::mutex mtx;

    int bufferSize = 4*resolution;

    int triangleCount = triangles.size();

    for(unsigned int threadIndex = 0; threadIndex < numThreads; threadIndex++){
        threads.push_back(std::thread([&, triangles, threadIndex]{
            
            // --- Initialize Variables --- //

            const float invDRez = 2.0f/(float)resolution;

            char *charBuffer = new char[bufferSize]; /* Heap Allocated Buffer */

            const int triangleCount = triangles.size();

            //float zValues[triangleCount];
            SteppedTriangle sTriangles[triangleCount];

            for(int i = 0; i < triangleCount; i++){
                sTriangles[i] = SteppedTriangle(triangles[i], resolution, numThreads, threadIndex);

                //float _bX = triangle.BarycentricCoords.x0 + x * triangle.BarycentricCoords.x1 + y * triangle.BarycentricCoords.x2;
                //float _bY = triangle.BarycentricCoords.y0 + x * triangle.BarycentricCoords.y1 + y * triangle.BarycentricCoords.y2;
                //float _bZ = 1 - _bX - _bY;
            }

            // --- Start Y Loop --- //

            for(int y = threadIndex; y < resolution; y += numThreads){

                int index = 0;

                const float yCoord = float(y)*invDRez - 1;

                // --- Start X Loop --- //

                for(int x = 0; x < resolution; x++){

                    const float xCoord = float(x)*invDRez - 1;

                    charBuffer[index] = 0;      // Blue
                    charBuffer[index+1] = 0;    // Green
                    charBuffer[index+2] = 0;    // Red
                    charBuffer[index+3] = 0;    // Alpha Channel Used As Depth Buffer

                    // --- Start Triangle Loop --- //

                    for(int i = 0; i < triangleCount; i++){

                        float zDepth = (float)charBuffer[index+3];

                        float zValue = sTriangles[i].values[DepthStepIndex] + 64;

                        if (zValue >= zDepth) // Depth Buffer Check
                        {
                            //double xBCoord = sTriangles[i].values[XBCoordStepIndex];
                            //double yBCoord = sTriangles[i].values[YBCoordStepIndex];

                            if((sTriangles[i].values[XBCoordStepIndex] >= 0.0)&&(sTriangles[i].values[YBCoordStepIndex] >= 0.0)&&(1.0 - sTriangles[i].values[XBCoordStepIndex] - sTriangles[i].values[YBCoordStepIndex] >= 0.0)) // Rasterization Check
                            {
                                ShaderTriangle shaderTriangle;
                                shaderTriangle.normal = triangles[i].normal;
                                shaderTriangle.color = simd::float4{(float)sTriangles[i].values[ColorRStepIndex], (float)sTriangles[i].values[ColorGStepIndex], (float)sTriangles[i].values[ColorBStepIndex], 1.0};
                                Color colorOut;
                                TriangleFragment(xCoord, yCoord, shaderTriangle, colorOut);
                                charBuffer[index] = (char)(colorOut.b);
                                charBuffer[index+1] = (char)(colorOut.g);
                                charBuffer[index+2] = (char)(colorOut.r);
                                charBuffer[index+3] = (char)zValue;

                            }
                        }

                        sTriangles[i].StepX();
                    }

                    charBuffer[index+3] = (char)255; // Alphe Channel Is Set To 255 For File Write
                    index += 4;
                }

                // --- Submit Row --- //

                int yinv = resolution - y - 1;
                mtx.lock();
                fileInterface.SubmitCharBufferAsRow(charBuffer, bufferSize, yinv);
                mtx.unlock();

                for(int i = 0; i < triangleCount; i++){
                    sTriangles[i].StepY();
                }
            }

            delete[] charBuffer;
        }));
    }

    for(auto& thread : threads){
        thread.join();
    }

    fileInterface.CloseBMPFile();
}

/*void Renderer::SetMiniTriangles(std::vector<Vertex[3]> triangleVertices){
    int count = 1;
    MiniTriangle *triangles[count];
    for(int i = 0; i < count; i++){
        triangles[i] = CreateMiniTriangleFromVertices(triangleVertices[i]);
        TriCoord triCoord = CreateBarycentricCoordFromVertices(triangleVertices[i]);
        triangles[i]->BarycentricCoords = TriCoordToMiniTriCoord(triCoord);
    }
}*/

PreCalTriangle Renderer::CreatePreCalTriangleFromVertices(Vertex vertices[3]){

    simd_double3 p1 = (simd_double3){vertices[0].position.x,vertices[0].position.y,vertices[0].position.z};
    simd_double3 p2 = (simd_double3){vertices[1].position.x,vertices[1].position.y,vertices[1].position.z};
    simd_double3 p3 = (simd_double3){vertices[2].position.x,vertices[2].position.y,vertices[2].position.z};

    PreCalTriangle out;


    double dInv = 1.0f / ( (p2.y - p3.y) * (p1.x - p3.x) - (p3.y - p1.y) * (p3.x - p2.x) );

    out.xBCoord.o = (- p3.x * (p2.y - p3.y) - p3.y * (p3.x - p2.x)) * dInv;
    out.yBCoord.o = (- p3.x * (p3.y - p1.y) - p3.y * (p1.x - p3.x)) * dInv;

    out.xBCoord.x = ((1 - p3.x) * (p2.y - p3.y) - p3.y * (p3.x - p2.x)) * dInv - out.xBCoord.o;
    out.yBCoord.x = ((1 - p3.x) * (p3.y - p1.y) - p3.y * (p1.x - p3.x)) * dInv - out.yBCoord.o;

    out.xBCoord.y = (- p3.x * (p2.y - p3.y) + (1 - p3.y) * (p3.x - p2.x)) * dInv - out.xBCoord.o;
    out.yBCoord.y = (- p3.x * (p3.y - p1.y) + (1 - p3.y) * (p1.x - p3.x)) * dInv - out.yBCoord.o;

    out.depth = GetPCValue(p1.z,p2.z,p3.z,out.xBCoord,out.yBCoord);
    out.colorR = GetPCValue(vertices[0].color.r,vertices[1].color.r,vertices[2].color.r,out.xBCoord,out.yBCoord);
    out.colorG = GetPCValue(vertices[0].color.g,vertices[1].color.g,vertices[2].color.g,out.xBCoord,out.yBCoord);
    out.colorB = GetPCValue(vertices[0].color.b,vertices[1].color.b,vertices[2].color.b,out.xBCoord,out.yBCoord);

    out.normal.xyz = simd::normalize(simd::cross(vertices[1].position - vertices[0].position, vertices[2].position - vertices[0].position));
    if(out.normal.z > 0){out.normal = -out.normal;}

    out.normal.w = 0;

    return out;
}

//ZA * out.x0 + x * ZA * out.x1 + y * ZA * out.x2
//ZB * out.y0 + x * ZB * out.y1 + y * ZB * out.y2
//ZC*(1 - out.x0 - out.y0) - x * ZC * (out.x1 + out.y1) - y * ZC * (out.x2 + out.y2);

PreCalValue Renderer::GetPCValue(double a, double b, double c, PreCalValue xBC, PreCalValue yBC){
    PreCalValue value;
    value.o = a * xBC.o + b * yBC.o + c * (1.0 - xBC.o - yBC.o);
    value.x = a * xBC.x + b * yBC.x - c * (xBC.x + yBC.x);
    value.y = a * xBC.y + b * yBC.y - c * (xBC.y + yBC.y);
    return value;
}

/*Vector3 BarycentricCoord(float x, float y, int i){
    Vector3 p1 = _vertices[_triangles[i].v[0]];
    Vector3 p2 = _vertices[_triangles[i].v[1]];
    Vector3 p3 = _vertices[_triangles[i].v[2]];

    float _x = (x - p3.x) * (p2.y - p3.y) + (y - p3.y) * (p3.x - p2.x);
    float _y = (x - p3.x) * (p3.y - p1.y) + (y - p3.y) * (p1.x - p3.x);
    float D = (p2.y - p3.y) * (p1.x - p3.x) - (p3.y - p1.y) * (p3.x - p2.x);
    _x /= D;
    _y /= D;

    Vector3 bCoord = Vector3(_x,_y,0);
    return bCoord;
}*/



/*
bool CheckAxisLine(float x, float y, float dx, float dy){
    return (((round(x)>x)^(round(x)>x+dx)));
}
Color Frame1Shader(unsigned int inx, unsigned int iny, unsigned int resolution){
    float xScale = 5;
    float yScale = 5;
    float lineSize = 2;
    float x = xScale*(2*(float)inx/(float)resolution - 1);
    float y = yScale*(2*(float)iny/(float)resolution - 1);
    float dx = lineSize*xScale*(2.0f/(float)resolution);
    float dy = lineSize*yScale*(2.0f/(float)resolution);

    if(((0>x)^(0>x+2*dx))||((0>y)^(0>y+2*dy))){return (Color){51,51,51,255};}

    if((round(x)>x)^(round(x)>x+dx)){return (Color){51,51,51,255};}
    if((round(y)>y)^(round(y)>y+dy)){return (Color){51,51,51,255};}

    if((round(5*x)>5*x)^(round(5*x)>5*x+2.5*dx)){return (Color){85,85,85,255};}
    if((round(5*y)>5*y)^(round(5*y)>5*y+2.5*dy)){return (Color){85,85,85,255};}

    return (Color){255,255,255,255};
}
*/