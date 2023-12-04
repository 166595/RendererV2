
#ifndef RenderTypes_h
#define RenderTypes_h

#include <simd/simd.h>
#include <iostream>
#include <bit>

#ifdef __cplusplus
   extern "C"
   {
#endif

    // Definitely not the best way to define color, but it works for now

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

    typedef struct Vertex{
        simd_float4 position;
        //simd_float3 normal; // Normals are currently calculated at runtime, this will be implemented later
        Color color;
    }Vertex;

    typedef struct{
        unsigned int meshID;
        simd::float4 vertices[3];
        simd::float4 normals[3];
        simd::float2 textureCoordinates[3];
    }Triangle;

#ifdef __cplusplus
   } // extern "C"
#endif

#endif