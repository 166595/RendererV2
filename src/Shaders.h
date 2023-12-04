#include "RenderTypes.h"
#include "Triangles.h"
#include <cmath>
#include <iostream>
#include <simd/simd.h>

#ifndef Shaders_h
#define Shaders_h

#ifdef __cplusplus
   extern "C"
   {
#endif

   float EquationShader(float x, float y);

#ifdef __cplusplus
   } // extern "C"
#endif

   Vertex VertexShader(Vertex vertex);
   void FragmentShader(float x, float y, Color &color);
   Color TriangleFragment(const ShaderTriangle &triangle, float x, float y);

#endif