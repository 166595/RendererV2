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
    }
};

// DrawEquation() 
// Will Draw White Lines When The Equation Is Equal To Zero

// DrawShaderScreen()
// Per Pixel Fragment Shader

simd_float3 randomPos(){
    return (simd_float3){(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1};
}

Color randomColor(){
    return (Color){(unsigned char)(rand()%255),(unsigned char)(rand()%255),(unsigned char)(rand()%255),255};
}

Color randomHalfColor(){
    return (Color){(unsigned char)(64 + rand()%128),(unsigned char)(64 + rand()%128),(unsigned char)(64 + rand()%128),255};
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

void RA(){
    Timer timer;
    Renderer renderer;

    std::vector<Triangle> triangles;

    int triangleCount = 256;
    double time[triangleCount];
    double maxTime = 0;

    for(int i = 0; i < triangleCount; i++){

        triangles.push_back(randomTriangle(i));

    }

    renderer.DrawTriangle(triangles);

    timer.ReadTime();
    std::cout << "Equivelent FPS: " << 1.0/timer.GetTime() << "\n";
}

void RB(){
    Timer timer;
    Renderer renderer;

    Triangle triangleA;
    triangleA.vertex[0] = (Vertex){(simd_float3){-1,-1,-1},(Color){255,0,0,255}};
    triangleA.vertex[1] = (Vertex){(simd_float3){-0.25,1,1},(Color){0,255,0,255}};
    triangleA.vertex[2] = (Vertex){(simd_float3){0.5,-1,-1},(Color){0,0,255,255}};

    Triangle triangleB;
    triangleB.vertex[0] = (Vertex){(simd_float3){-0.5,1,-1},(Color){0,255,255,255}};
    triangleB.vertex[1] = (Vertex){(simd_float3){0.25,-1,1},(Color){255,0,255,255}};
    triangleB.vertex[2] = (Vertex){(simd_float3){1,1,-1},(Color){255,255,0,255}};

    std::vector<Triangle> triangles;

    triangles.push_back(triangleA);
    triangles.push_back(triangleB);

    timer.Reset();

    renderer.DrawTriangle(triangles);

    timer.ReadTime();
    std::cout << "Equivelent FPS: " << 1.0/timer.GetTime() << "\n";
}


int main(){

    RA();

    return 0;
}