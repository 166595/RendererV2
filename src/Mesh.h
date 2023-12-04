
#ifndef MESH_H
#define MESH_H
#include <iostream>
#include <fstream>
#include <sstream>
#include "FileInterface.h"
#include "RenderTypes.h"
#include "Triangles.h"
enum InputType{
    VERTEX = (int)'v' + ((int)' ' << 8),
    VERTEX_NORMAL = (int)'v' + ((int)'n' << 8),
    TEXTURE_COORDINATE = (int)'v' + ((int)'t' << 8),
    FACE = (int)'f' + ((int)' ' << 8)
};

typedef struct QuadIndices{
    simd::uint4 vertex, normal, textureCoordinate;
}QuadIndices;

typedef struct Mesh{
    simd::float4x4 transform;
    std::vector<QuadIndices> quadIndices;
    std::vector<simd::float4> vertices;
    std::vector<simd::float4> normals;
    std::vector<simd::float2> textureCoordinates;
}Mesh;

Mesh CreateMesh(std::string meshName);

#endif