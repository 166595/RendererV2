
#include "FileInterface.h"
#include "RenderTypes.h"
#include "Triangles.h"
#include "Shaders.h"

#include <iostream>
#include <simd/simd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <bit>

#ifndef Renderer_h
#define Renderer_h

#define MAX_THREADS 8
#define XCHUNK_SIZE 64

//#define VERBOSE


#undef EQUATION_RENDERER


#ifdef __cplusplus
   extern "C"
   {
#endif
    class Renderer{

        public:
        
        void DrawTriangle(std::vector<Triangle> triangles, int resolution = 1024);

        #ifdef EQUATION_RENDERER
        void DrawEquation();
        void FillEquationLineBuffer(bool *eLineBuffer, float xScale, float y, float resolution);
        #endif

        private:
        int resolution;
        void PrintDraw(int triangleCount);
        bool CheckSteppedTriangle(SteppedTriangle &triangle, unsigned short x, unsigned short y, float depth);


        //void SetMiniTriangles(std::vector<Vertex[3]> triangleVertices);
        
    };

#ifdef __cplusplus
   } // extern "C"
#endif

#endif