#include <iostream>
#include "FileInterface.h"
#include "renderer.h"
#include "RenderTypes.h"
#include "Triangles.h"
#include <chrono>
#include <ctime>
#include "Mesh.h"

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

simd::float4 randomPos(){
    return (simd::float4){(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1,(float)rand()/RAND_MAX*2-1,1};
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


simd::float4 cubePos[] = {
    (simd::float4){-1,-1,-1,1}, // cubePos1
    (simd::float4){1,-1,-1,1},  // cubePos2
    (simd::float4){-1,1,-1,1},  // cubePos3
    (simd::float4){1,1,-1,1},   // cubePos4
    (simd::float4){-1,-1,1,1},  // cubePos5
    (simd::float4){1,-1,1,1},   // cubePos6
    (simd::float4){-1,1,1,1},   // cubePos7
    (simd::float4){1,1,1,1}     // cubePos8
};

simd::float4 cubeNormals[] = {
    (simd::float4){0,0,-1,0}, // cubeNormal1
    (simd::float4){0,0,1,0},  // cubeNormal2
    (simd::float4){0,-1,0,0}, // cubeNormal3
    (simd::float4){0,1,0,0},  // cubeNormal4
    (simd::float4){-1,0,0,0}, // cubeNormal5
    (simd::float4){1,0,0,0}   // cubeNormal6
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

simd::float4x4 GetTransformMatrix(simd::float3 position, simd::float3 rotation, float scale){
    float rx = rotation.x * M_PI / 180.0f;
    float ry = rotation.y * M_PI / 180.0f;
    float rz = rotation.z * M_PI / 180.0f;

    simd::float4x4 rxMatrix = {
        simd::float4{1.0f, 0.0f, 0.0f, 0.0f},
        simd::float4{0.0f, cos(rx), -sin(rx), 0.0f},
        simd::float4{0.0f, sin(rx), cos(rx), 0.0f},
        simd::float4{0.0f, 0.0f, 0.0f, 1.0f}
    };

    simd::float4x4 ryMatrix = {
        simd::float4{cos(ry), 0.0f, sin(ry),0},
        simd::float4{0.0f, 1.0f, 0.0f,0},
        simd::float4{-sin(ry), 0.0f, cos(ry),0},
        simd::float4{0.0f, 0.0f, 0.0f, 1.0f}
    };

    simd::float4x4 rzMatrix = {
        simd::float4{cos(rz), -sin(rz), 0.0f, 0.0f},
        simd::float4{sin(rz), cos(rz), 0.0f, 0.0f},
        simd::float4{0.0f, 0.0f, 1.0f, 0.0f},
        simd::float4{0.0f, 0.0f, 0.0f, 1.0f}
    };

    simd::float4x4 translationMatrix = {
        simd::float4{scale, 0.0f, 0.0f, 0.0f},
        simd::float4{0.0f, scale, 0.0f, 0.0f},
        simd::float4{0.0f, 0.0f, scale, 0.0f},
        simd::float4{position.x, position.y, position.z, 1.0f}
    };

    return translationMatrix * rzMatrix * ryMatrix * rxMatrix;
}

/*void RenderScreen(int resolution = 1024){
    Renderer renderer;
    Timer timer;

    Triangle triangleA;
    triangleA.vertices[0] = (simd::float4){-1,-1,-1,1};
    triangleA.vertices[1] = (simd::float4){1,-1,-1,1};
    triangleA.vertices[2] = (simd::float4){1,1,-1,1};
    triangleA.normals[0] = (simd::float4){0,0,1,0};
    triangleA.normals[1] = (simd::float4){0,0,1,0};
    triangleA.normals[2] = (simd::float4){0,0,1,0};
    triangleA.color[0] = (Color){255,255,255};
    triangleA.color[1] = (Color){255,255,255};
    triangleA.color[2] = (Color){255,255,255};

    Triangle triangleB;
    triangleB.vertices[0] = (simd::float4){1,1,-1,1};
    triangleB.vertices[1] = (simd::float4){-1,1,-1,1};
    triangleB.vertices[2] = (simd::float4){-1,-1,-1,1};
    triangleB.normals[0] = (simd::float4){0,0,1,0};
    triangleB.normals[1] = (simd::float4){0,0,1,0};
    triangleB.normals[2] = (simd::float4){0,0,1,0};
    triangleB.color[0] = (Color){255,255,255};
    triangleB.color[1] = (Color){255,255,255};
    triangleB.color[2] = (Color){255,255,255};

    std::vector<Triangle> triangles;

    triangles.push_back(triangleA);
    triangles.push_back(triangleB);

    timer.Reset();

    renderer.DrawTriangles(triangles, resolution);

    timer.ReadTime();
}

void RenderCubes(int cubeCount, int resolution = 1024){
    std::cout << "Cube Cal Start\n";
    std::cout.flush();

    Renderer renderer;
    Timer timer;

    std::vector<Triangle> triangles;


    for(int i = 0; i < cubeCount; i++){

        simd::float4 centerPos = randomPos() + (simd::float4){0,0,-10,1};
        Color color = randomColor();

        float rx = (float)rand()/RAND_MAX*360;
        float ry = (float)rand()/RAND_MAX*360;
        float rz = (float)rand()/RAND_MAX*360;

        simd::float4x4 rotationMatrix = GetRotationMatrix(rx,ry,rz);

        for(int j = 0; j < 12; j++){

            Triangle triangle;

            float size = 0.05;// 0.005;

            simd::float4 aPos = rotationMatrix * cubePos[tIndex[3*j]-1];

            simd::float4 bPos = rotationMatrix * cubePos[tIndex[3*j+1]-1];

            simd::float4 cPos = rotationMatrix * cubePos[tIndex[3*j+2]-1];

            simd::float4 normal = rotationMatrix * cubeNormals[j/2];

            triangle.vertices[0] = centerPos + size * aPos;
            triangle.vertices[1] = centerPos + size * bPos;
            triangle.vertices[2] = centerPos + size * cPos;
            triangle.normals[0] = normal;
            triangle.normals[1] = normal;
            triangle.normals[2] = normal;
            triangle.color[0] = color;
            triangle.color[1] = color;
            triangle.color[2] = color;

            triangles.push_back(triangle);
        }
    }
    std::cout << "Cube Cal End\n";
    std::cout.flush();

    renderer.DrawTriangles(triangles, resolution);

    timer.ReadTime();
}*/

void DrawMesh(std::string meshName, int resolution = 1024){
    Renderer renderer;
    Timer timer;

    std::vector<Mesh> meshes;
    Mesh mesh = CreateMesh(meshName);
    simd::float3 position = {0.2,0,-10};
    simd::float3 rotation = {0,90,0};
    mesh.transform = GetTransformMatrix(position, rotation, 1.0f);
    meshes.push_back(mesh);

    renderer.DrawMeshes(meshes, resolution);
    timer.ReadTime();

}

// --- Benchmark Results --- //

// 8 Threads
// R / 512 / / 1024 / / 2048 /
// A / 5ms / / 12.4ms / / 40.3ms /
// B / 17.9ms / / 53ms / / 167.6ms /
// C / 45.6ms / / 78.1ms / / 163.5ms /


int main(){

    //RunBenchmark();
    //std::cout << "Main Start\n";
    //std::cout.flush();
    //RenderCubes((1 << 16), 2048);
    //RenderScreen();

    //DrawMesh("city");
    DrawMesh("spot");
    //RenderScreen();



    return 0;
}