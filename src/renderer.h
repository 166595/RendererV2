#include "FileInterface.h"
#include "RenderTypes.h"
#include <iostream>

#ifndef Renderer_h
#define Renderer_h

#define MAX_THREADS 8
#define RESOLUTION 4096

#ifdef __cplusplus
   extern "C"
   {
#endif
    class Renderer{
        int resolution;
        //void SetTriangleBarycentricCoord(Triangle *t);
        public:
        PreCalTriangle CreatePreCalTriangleFromVertices(Vertex vertices[3]);
        void DrawEquation();
        void DrawTriangle(std::vector<Triangle> triangles);
        void DrawShaderScreen();
        void FillEquationLineBuffer(bool *eLineBuffer, float xScale, float y, float resolution);
        PreCalValue GetPCValue(double a, double b, double c, PreCalValue xBC, PreCalValue yBC);
        //void SetMiniTriangles(std::vector<Vertex[3]> triangleVertices);
        
    };

#ifdef __cplusplus
   } // extern "C"
#endif

#endif