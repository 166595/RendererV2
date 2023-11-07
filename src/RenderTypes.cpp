
#include "RenderTypes.h"

MiniTriangle *CreateMiniTriangleFromTriangle(Triangle triangleIn){
    MiniTriangle *triangle = (MiniTriangle *) malloc(sizeof(MiniTriangle));
    triangle->vertex[0] = VertexToMiniVertex(triangleIn.vertex[0]);
    triangle->vertex[1] = VertexToMiniVertex(triangleIn.vertex[1]);
    triangle->vertex[2] = VertexToMiniVertex(triangleIn.vertex[2]);
    triangle->BarycentricCoords = TriCoordToMiniTriCoord(triangleIn.BarycentricCoords);
    return triangle;
}

void CopyTriangleToMiniTriangle(Triangle triangleIn, MiniTriangle *mTriangle){
    mTriangle->vertex[0] = VertexToMiniVertex(triangleIn.vertex[0]);
    mTriangle->vertex[1] = VertexToMiniVertex(triangleIn.vertex[1]);
    mTriangle->vertex[2] = VertexToMiniVertex(triangleIn.vertex[2]);
    mTriangle->BarycentricCoords = TriCoordToMiniTriCoord(triangleIn.BarycentricCoords);
}

MiniTriangle *CreateMiniTriangleFromVertices(Vertex vertices[3]){
    MiniTriangle *triangle = (MiniTriangle *) malloc(sizeof(MiniTriangle));
    triangle->vertex[0] = VertexToMiniVertex(vertices[0]);
    triangle->vertex[1] = VertexToMiniVertex(vertices[1]);
    triangle->vertex[2] = VertexToMiniVertex(vertices[2]);
    return triangle;
}

MiniColor ColorToMiniColor(Color incolor){
    int r = fmax(fmin(incolor.r * EIGHT_BIT_TO_FIVE_BIT, 31),0);
    int g = fmax(fmin(incolor.g * EIGHT_BIT_TO_FIVE_BIT, 31),0);
    int b = fmax(fmin(incolor.b * EIGHT_BIT_TO_FIVE_BIT, 31),0);
    int a = (incolor.a > 63) ? 1 : 0;

    MiniColor color;
    color = r | (g << 5) | (b << 10) | (a << 15);
    return color;
}

MiniTriCoord TriCoordToMiniTriCoord(TriCoord triCoord){
    MiniTriCoord miniTriCoord;
    miniTriCoord.x0 = MFV::FloatToMiniFValue(triCoord.x0);
    miniTriCoord.y0 = MFV::FloatToMiniFValue(triCoord.y0);
    miniTriCoord.x1 = MFV::FloatToMiniFValue(triCoord.x1);
    miniTriCoord.y1 = MFV::FloatToMiniFValue(triCoord.y1);
    miniTriCoord.x2 = MFV::FloatToMiniFValue(triCoord.x2);
    miniTriCoord.y2 = MFV::FloatToMiniFValue(triCoord.y2);
    return miniTriCoord;
}


MiniVector3 Vector3ToMiniVector3(simd_float3 vector){
    MiniVector3 miniVector;
    miniVector.x = MFV::FloatToMiniFValue(vector.x);
    miniVector.y = MFV::FloatToMiniFValue(vector.y);
    miniVector.z = MFV::FloatToMiniFValue(vector.z);
    return miniVector;
}

MiniVertex VertexToMiniVertex(Vertex vertex){
    MiniVertex miniVertex;
    miniVertex.position = Vector3ToMiniVector3(vertex.position);
    miniVertex.color = ColorToMiniColor(vertex.color);
    return miniVertex;
}