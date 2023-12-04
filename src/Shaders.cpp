
#include "Shaders.h"
#include <simd/simd.h>

#include <bit>

simd::float4 Project(simd::float4 position, simd::float4x4 &projectionMatrix){
    simd::float4 projected = projectionMatrix * position;
    float Z = 1.0f;//5.0f / (5 - position.z);
    projected.x *= Z;
    projected.y *= Z;
    return projected;
}

void TriangleShader(Triangle &triangle, simd::float4x4 &rotationMatrix){
    for(int i = 0; i < 3; i++){
        triangle.vertices[i] = Project(triangle.vertices[i], rotationMatrix);
        triangle.normals[i] =  Project(triangle.normals[i], rotationMatrix);
    }
}

const simd::float4 _Yellow = (simd::float4){255.0f, 215.0f, 1.0f, 1.0f};
const simd::float4 _Blue = (simd::float4){1.0f, 1.0f, 255.0f, 1.0f};
const simd::float4 _Green = (simd::float4){1.0f, 255.0f, 1.0f, 1.0f};
const simd::float4 _Red = (simd::float4){255.0f, 1.0f, 1.0f, 1.0f};

simd::float4 Sample(simd::float2 uv, unsigned int meshID){
    float V = floor(((2 + cos(uv.x * 256) * cos(uv.y * 256)))) / 3.0f;
    return V * (simd::float4){255.0f, 255.0f, 255.0f, 255.0f};
    /*switch(meshID%4){
        case 0 : return (0.5f + V) * _Yellow;
        case 1 : return (0.5f + V) * _Blue;
        case 2 : return (0.5f + V) * _Green;
        case 3 : return (0.5f + V) * _Red;
        default: break;
    }

    return (0.5f + V) * _Yellow;*/
}

Color FragmentShader(const ShaderTriangle &triangle, float x, float y){

    simd::float4 light = (simd_float4){0,0,1,0};
    simd::float4 normal = simd::normalize(triangle.normal);
    float dot = simd::dot(normal, simd::normalize(light));
    if(dot < 0){dot = 0;}
    dot = 0.05f + dot * 0.8f + 0.15f * std::max(2 * dot * dot - 1,0.0f);
    simd::float4 color = Sample(triangle.uv, triangle.meshID);
    color *= dot;
    return (Color)color;
}