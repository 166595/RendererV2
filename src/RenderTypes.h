
#ifndef RenderTypes_h
#define RenderTypes_h

#include <simd/simd.h>
#include <iostream>
#include <bit>
#include "MiniFValue.h"

#ifdef __cplusplus
   extern "C"
   {
#endif

    // full = 64 bits = 8 bytes
    // short = 16 bits = 2 bytes

    /* Full Render Types ----------------------------------------------------------------------- */

    #pragma pack(push, 1)
    typedef struct Color{

        unsigned char b,g,r,a;

        Color():r(0),g(0),b(0),a(255){};

        Color(float rIn, float gIn, float bIn, float aIn):r(rIn),g(gIn),b(bIn),a(aIn){};
        Color(float rIn, float gIn, float bIn):r(rIn),g(gIn),b(bIn),a(255){};

        Color(unsigned char rIn, unsigned char gIn, unsigned char bIn, unsigned char aIn):r(rIn),g(gIn),b(bIn),a(aIn){};
        Color(unsigned char rIn, unsigned char gIn, unsigned char bIn):r(rIn),g(gIn),b(bIn),a(255){};

        Color(char rIn, char gIn, char bIn, char aIn):r(rIn),g(gIn),b(bIn),a(aIn){};
        Color(char rIn, char gIn, char bIn):r(rIn),g(gIn),b(bIn),a(255){};

        Color(int rIn, int gIn, int bIn, int aIn):r(rIn),g(gIn),b(bIn),a(aIn){};
        Color(int rIn, int gIn, int bIn):r(rIn),g(gIn),b(bIn),a(255){};

        Color(simd::float4 color):r(color.x),g(color.y),b(color.z),a(color.w){};
        Color(simd::float3 color):r(color.x),g(color.y),b(color.z),a(255){};

    }Color;
    #pragma pack(pop)

    typedef struct{
        simd_float3 position;
        Color color;
    }Vertex;

    typedef struct{
        float x0,x1,x2;
        float y0,y1,y2;
        float z0,z1,z2;
    }TriCoord;

    typedef struct{
        Vertex vertex[3];
        TriCoord BarycentricCoords;
    }Triangle;


    // --- Not Current --- //

    /* Mini Render Types ----------------------------------------------------------------------- */

    // Used in place of float
    // 16 bits / 2 bytes / 1 short
    typedef short MiniFValue;

    // 16 bits / 2 bytes / 1 short
    typedef unsigned MiniColor;

    // 48 bits / 6 bytes / 3 short
    typedef struct{
        MiniFValue x,y,z;
    }MiniVector3;

    // MiniVector3 position 
    // MiniColor color
    // 64 bits / 8 bytes / 1 full
    typedef struct{
        MiniVector3 position;
        MiniColor color;
    }MiniVertex;

    
    // (Origin): MiniFValue x0, y0   
    // (X Axis): MiniFValue x1, y1     
    // (Y Axis): MiniFValue x2, y2     
    // 96 bits / 12 bytes / 6 short
    typedef struct{
        MiniFValue x0, y0; // Origin
        MiniFValue x1, y1; // Relative X Axis
        MiniFValue x2, y2; // Relative Y Axis
    }MiniTriCoord;

    // 256 bits / 32 bytes / 4 full -- Not Current --
    // 288 bits / 36 bytes / 9 short
    typedef struct{
        MiniVertex vertex[3];
        MiniTriCoord BarycentricCoords;
    }MiniTriangle;



    /* Buffer Types ---------------------------------------------------------------------------- */

    typedef struct ColorBuffer{
        unsigned int length;
        char *c;
    }ColorBuffer;

    /* Create Functions ------------------------------------------------------------------------ */

    MiniTriangle *CreateMiniTriangleFromTriangle(Triangle triangleIn);
    MiniTriangle *CreateMiniTriangleFromVertices(Vertex vertices[3]);

    /* Convert Functions ----------------------------------------------------------------------- */

    void CopyTriangleToMiniTriangle(Triangle triangleIn, MiniTriangle *mTriangle);

    MiniColor ColorToMiniColor(Color color);
    MiniVector3 Vector3ToMiniVector3(simd_float3 vector);
    MiniVertex VertexToMiniVertex(Vertex vertex);
    MiniTriCoord TriCoordToMiniTriCoord(TriCoord triCoord);

#ifdef __cplusplus
   } // extern "C"
#endif

#endif