#include <iostream>
#include "FileInterface.h"
#include "renderer.h"
#include "RenderTypes.h"
#include <chrono>
#include <ctime>

class Timer{
    std::chrono::high_resolution_clock::time_point start;
    public:
    Timer(){
        start = std::chrono::high_resolution_clock::now();
    }
    void Reset(){
        start = std::chrono::high_resolution_clock::now();
    }
    double GetTime(){
        return std::chrono::duration_cast< std::chrono::duration<double> >(std::chrono::high_resolution_clock::now() - start).count();
    }
    void ReadTime(){
        double time = GetTime();
        std::cout << "Time " << time << " (" << floor(1000*time) << "ms)\n";
        std::cout << "Equivelent FPS: " << 1.0/time << "\n";
    }
};

simd_float3 randomPos(){
    return (simd_float3){(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1};
}

Color randomColor(){
    return (Color){(unsigned char)(rand()%255),(unsigned char)(rand()%255),(unsigned char)(rand()%255)};
}

Color randomHalfColor(){
    return (Color){(unsigned char)(64 + rand()%128),(unsigned char)(64 + rand()%128),(unsigned char)(64 + rand()%128)};
}

Vertex randomVertex(Vertex origin){
    Vertex vertex;
    Color color = randomHalfColor();
    color.r += origin.color.r;
    color.g += origin.color.g;
    color.b += origin.color.b;
    color.a = 255;

    vertex.position = 0.25*randomPos() + 0.25*randomPos() + origin.position;
    vertex.color = color;
    return vertex;
}


Triangle randomTriangle(int seed){
    Triangle triangle;
    Color mainColor = randomColor();
    Vertex originVertex = {0.5*randomPos(),randomHalfColor()};

    triangle.vertex[0] = randomVertex(originVertex);
    triangle.vertex[1] = randomVertex(originVertex);
    triangle.vertex[2] = randomVertex(originVertex);
    return triangle;
}

simd::float3 cubePos[] = {
    (simd_float3){-1,-1,-1}, // cubePos1
    (simd_float3){1,-1,-1},  // cubePos2
    (simd_float3){-1,1,-1},  // cubePos3
    (simd_float3){1,1,-1},   // cubePos4
    (simd_float3){-1,-1,1},  // cubePos5
    (simd_float3){1,-1,1},   // cubePos6
    (simd_float3){-1,1,1},   // cubePos7
    (simd_float3){1,1,1}     // cubePos8
};

int tIndex[] = {
    1,2,4, // Face 1
    1,4,3, // Face 1
    2,6,8, // Face 2
    2,8,4, // Face 2
    3,4,8, // Face 3
    3,8,7, // Face 3
    1,5,6, // Face 4
    1,6,2, // Face 4
    5,7,8, // Face 5
    5,8,6, // Face 5
    1,3,7, // Face 6
    1,7,5  // Face 6
};

simd::float3 RotateVector3(simd::float3 v, float rx, float ry, float rz) {
    rx = rx * M_PI / 180.0f;
    ry = ry * M_PI / 180.0f;
    rz = rz * M_PI / 180.0f;

    simd::float3x3 Rx = {
        simd::float3{1.0f, 0.0f, 0.0f},
        simd::float3{0.0f, cos(rx), -sin(rx)},
        simd::float3{0.0f, sin(rx), cos(rx)}
    };

    simd::float3x3 Ry = {
        simd::float3{cos(ry), 0.0f, sin(ry)},
        simd::float3{0.0f, 1.0f, 0.0f},
        simd::float3{-sin(ry), 0.0f, cos(ry)}
    };

    simd::float3x3 Rz = {
        simd::float3{cos(rz), -sin(rz), 0.0f},
        simd::float3{sin(rz), cos(rz), 0.0f},
        simd::float3{0.0f, 0.0f, 1.0f}
    };

    simd::float3x3 R = Rz * Ry * Rx;
    simd::float3 rotated_v = R * v;
    return rotated_v;
}

void RA(){
    Timer timer;
    Renderer renderer;

    std::vector<Triangle> triangles;

    int triangleCount = 16;

    for(int i = 0; i < triangleCount; i++){

        triangles.push_back(randomTriangle(i));

    }

    renderer.DrawTriangle(triangles);

    timer.ReadTime();
}

void RB(int resolution = 1024){
    Renderer renderer;
    Timer timer;

    Triangle triangleA;
    triangleA.vertex[0] = (Vertex){(simd_float3){-1,-1,-12},(Color){255,0,0}};
    triangleA.vertex[1] = (Vertex){(simd_float3){-0.25,1,-10},(Color){0,255,0}};
    triangleA.vertex[2] = (Vertex){(simd_float3){0.5,-1,-12},(Color){0,0,255}};

    Triangle triangleB;
    triangleB.vertex[0] = (Vertex){(simd_float3){-0.5,1,-12},(Color){0,255,255}};
    triangleB.vertex[1] = (Vertex){(simd_float3){0.25,-1,-10},(Color){255,0,255}};
    triangleB.vertex[2] = (Vertex){(simd_float3){1,1,-12},(Color){255,255,0}};

    std::vector<Triangle> triangles;

    triangles.push_back(triangleA);
    triangles.push_back(triangleB);

    timer.Reset();

    renderer.DrawTriangle(triangles, resolution);

    timer.ReadTime();

}

double DrawBenchmarkA(int resolution){
    Renderer renderer;
    Timer timer;

    Triangle triangleA;
    triangleA.vertex[0] = (Vertex){(simd_float3){-1,-1,-12},(Color){255,0,0}};
    triangleA.vertex[1] = (Vertex){(simd_float3){-0.25,1,-10},(Color){0,255,0}};
    triangleA.vertex[2] = (Vertex){(simd_float3){0.5,-1,-12},(Color){0,0,255}};

    Triangle triangleB;
    triangleB.vertex[0] = (Vertex){(simd_float3){-0.5,1,-12},(Color){0,255,255}};
    triangleB.vertex[1] = (Vertex){(simd_float3){0.25,-1,-10},(Color){255,0,255}};
    triangleB.vertex[2] = (Vertex){(simd_float3){1,1,-12},(Color){255,255,0}};

    std::vector<Triangle> triangles;

    triangles.push_back(triangleA);
    triangles.push_back(triangleB);

    timer.Reset();

    renderer.DrawTriangle(triangles, resolution);

    return timer.GetTime();
}

double DrawBenchmarkB(int resolution){
    Renderer renderer;
    Timer timer;

    std::vector<Triangle> triangles;

    int triangleCount = 512;

    for(int i = 0; i < triangleCount; i++){

        triangles.push_back(randomTriangle(i));

    }

    timer.Reset();

    renderer.DrawTriangle(triangles, resolution);

    return timer.GetTime();
}

double DrawBenchmarkC(int resolution){
    Renderer renderer;
    Timer timer;

    std::vector<Triangle> triangles;


    for(int i = 0; i < 1024; i++){

        simd::float3 centerPos = randomPos() + (simd_float3){0,0,-10};
        Color color = randomColor();

        float rx = (float)rand()/RAND_MAX*360;
        float ry = (float)rand()/RAND_MAX*360;
        float rz = (float)rand()/RAND_MAX*360;

        for(int j = 0; j < 12; j++){

            Triangle triangle;

            simd::float3 aPos = RotateVector3(cubePos[tIndex[3*j]-1],rx,ry,rz);
            simd::float3 bPos = RotateVector3(cubePos[tIndex[3*j+1]-1],rx,ry,rz);
            simd::float3 cPos = RotateVector3(cubePos[tIndex[3*j+2]-1],rx,ry,rz);

            triangle.vertex[0] = (Vertex){centerPos + 0.02 * aPos, color};
            triangle.vertex[1] = (Vertex){centerPos + 0.02 * bPos, color};
            triangle.vertex[2] = (Vertex){centerPos + 0.02 * cPos, color};

            triangles.push_back(triangle);
        }
    }

    timer.Reset();

    renderer.DrawTriangle(triangles, resolution);

    return timer.GetTime();
}

void RunBenchmark(){
    Timer timer;
    int resolutions[5] = {256,512,1024,2048,4096};
    double timesA[5];
    double timesB[5];
    double timesC[5];


    const double msScale = 10000 / 3;

    for(int i = 0; i < 5; i++){

        double timeA = DrawBenchmarkA(resolutions[i]);
        double timeB = DrawBenchmarkB(resolutions[i]);
        double timeC = DrawBenchmarkC(resolutions[i]);

        timeB += DrawBenchmarkB(resolutions[i]);
        timeC += DrawBenchmarkC(resolutions[i]);
        timeA += DrawBenchmarkA(resolutions[i]);

        timeC += DrawBenchmarkC(resolutions[i]);
        timeA += DrawBenchmarkA(resolutions[i]);
        timeB += DrawBenchmarkB(resolutions[i]);

        timesA[i] = round(msScale * timeA) / 10;
        timesB[i] = round(msScale * timeB) / 10;
        timesC[i] = round(msScale * timeC) / 10;

    }

    double totalTime = timer.GetTime();

    std::cout << "Benchmark Complete, Total Time: " << totalTime << " (" << floor(1000*totalTime) << "ms)\n";

    std::cout << "// R ";
    for(int i = 0; i < 5; i++){
        std::cout << "/ " << resolutions[i] << " / ";
    }

    std::cout << "\n// A ";
    for(int i = 0; i < 5; i++){
        std::cout << "/ " << timesA[i] << "ms / ";
    }

    std::cout << "\n// B ";
    for(int i = 0; i < 5; i++){
        std::cout << "/ " << timesB[i] << "ms / ";
    }

    std::cout << "\n// C ";
    for(int i = 0; i < 5; i++){
        std::cout << "/ " << timesC[i] << "ms / ";
    }

    std::cout << "\n";
}

void RenderScreen(int resolution = 1024){
    Renderer renderer;
    Timer timer;

    Triangle triangleA;
    triangleA.vertex[0] = (Vertex){(simd_float3){-1,-1,-1},(Color){255,255,255}};
    triangleA.vertex[1] = (Vertex){(simd_float3){1,-1,-1},(Color){255,255,255}};
    triangleA.vertex[2] = (Vertex){(simd_float3){1,1,-1},(Color){255,255,255}};

    Triangle triangleB;
    triangleB.vertex[0] = (Vertex){(simd_float3){1,1,-1},(Color){255,255,255}};
    triangleB.vertex[1] = (Vertex){(simd_float3){-1,1,-1},(Color){255,255,255}};
    triangleB.vertex[2] = (Vertex){(simd_float3){-1,-1,-1},(Color){255,255,255}};

    std::vector<Triangle> triangles;

    triangles.push_back(triangleA);
    triangles.push_back(triangleB);

    timer.Reset();

    renderer.DrawTriangle(triangles, resolution);

    timer.ReadTime();
}

void RenderCubes(int cubeCount, int resolution = 1024){

    Renderer renderer;
    Timer timer;

    std::vector<Triangle> triangles;


    for(int i = 0; i < cubeCount; i++){

        simd::float3 centerPos = randomPos() + (simd_float3){0,0,-10};
        Color color = randomColor();

        float rx = (float)rand()/RAND_MAX*360;
        float ry = (float)rand()/RAND_MAX*360;
        float rz = (float)rand()/RAND_MAX*360;

        for(int j = 0; j < 12; j++){

            Triangle triangle;

            simd::float3 aPos = RotateVector3(cubePos[tIndex[3*j]-1],rx,ry,rz);
            simd::float3 bPos = RotateVector3(cubePos[tIndex[3*j+1]-1],rx,ry,rz);
            simd::float3 cPos = RotateVector3(cubePos[tIndex[3*j+2]-1],rx,ry,rz);

            triangle.vertex[0] = (Vertex){centerPos + 0.02 * aPos, color};
            triangle.vertex[1] = (Vertex){centerPos + 0.02 * bPos, color};
            triangle.vertex[2] = (Vertex){centerPos + 0.02 * cPos, color};

            triangles.push_back(triangle);
        }
    }

    renderer.DrawTriangle(triangles, resolution);

    timer.ReadTime();
}
/* Benchmarks (Rounded to 5ms) ----------------------------------------------------------------- */

// 8 Threads
// R / 256 / / 512 / / 1024 / / 2048 / / 4096 / 
// A / 4.5ms / / 5.9ms / / 14.5ms / / 42.8ms / / 149.7ms / 
// B / 10.1ms / / 18.8ms / / 60.7ms / / 181ms / / 682.1ms / 
// C / 38.9ms / / 56.5ms / / 106ms / / 219.2ms / / 558.1ms / 

// Single Thread
// A / 8.1ms / / 18ms / / 65.8ms / / 249.9ms / / 988.9ms / 
// B / 55.4ms / / 126ms / / 407.4ms / / 1280.4ms / / 4883.9ms / 
// C / 204.2ms / / 359.1ms / / 704.4ms / / 1545.8ms / / 3939.3ms / 

int main(){

    //RenderScreen();
    //RunBenchmark();
    RenderCubes(1024, 1024);


    //RB();

    return 0;
}