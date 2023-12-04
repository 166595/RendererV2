
#include "FileInterface.h"
#include "RenderTypes.h"
#include "Triangles.h"
#include "Shaders.h"
#include "Mesh.h"

#include <iostream>
#include <sstream>
#include <simd/simd.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <bit>

#ifndef Renderer_h
#define Renderer_h

#define MAX_THREADS 8

// CHUNK_COUNT must be greater than the number of threads (Values lower will be clamped to the number of threads)
#define CHUNK_COUNT 64

#define VERBOSE
//#define CHUNK_VISUALIZATION

#ifdef __cplusplus
   extern "C"
   {
#endif
    class Renderer{

        public:
        
        void DrawTriangles(std::vector<Triangle> &triangles, int resolution = 1024);
        void DrawMeshes(std::vector<Mesh> &meshes, int resolution = 1024);

        private:
        int resolution;
        void PrintDraw(int triangleCount);
        bool CheckSteppedTriangle(SteppedTriangle &triangle, unsigned short x, unsigned short y, float depth);
        
    };

#ifdef __cplusplus
   } // extern "C"
#endif

#endif