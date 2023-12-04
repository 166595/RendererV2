
#include "Shaders.h"

#include <bit>

Vertex VertexShader(Vertex vertex){
    return vertex;
}

void FragmentShader(float x, float y, Color &color){
    /*float cx = x * 1.1f - 0.25f;
    float cy = y * 1.1f;

    // Initialize variables for Mandelbrot calculation
    float zx = 0.0f, zy = 0.0f;
    int iteration = 0, maxIterations = 1000;

    // Perform Mandelbrot calculation
    while(zx * zx + zy * zy <= 4.0f && iteration < maxIterations) {
        float tmp = zx * zx - zy * zy + cx;
        zy = 2.0f * zx * zy + cy;
        zx = tmp;
        iteration++;
    }

    // Generate color based on the number of iterations
    if(iteration == maxIterations) {
        color = Color{0, 0, 0, 255}; // Black for points inside the Mandelbrot set
    } else {
        // Use a gradient for points outside the Mandelbrot set
        unsigned char r = static_cast<unsigned char>(255 * sin(0.016 * iteration + 4));
        unsigned char g = static_cast<unsigned char>(255 * sin(0.013 * iteration + 2));
        unsigned char b = static_cast<unsigned char>(255 * sin(0.01 * iteration + 1));
        color = Color{r, g, b, 255};
    }*/
}

float EquationShader(float x,float y){
    //float value = x*x + y*y - 0.5;
    float A = 1.0f, B = 1.0f, C = 0.0f, D = 0.0f;
    float value = y - (A * std::sin(B * (x - C)) + D);
    return value;
}


Color TriangleFragment(const ShaderTriangle &triangle, float x, float y){

    simd::float4 light = (simd_float4){0,0,0.5,0};
    float dot = -simd::dot(triangle.normal, simd::normalize(light));
    if(dot < 0){dot = 0;}

    float z = (float)triangle.color.w / (1.0f - (float)triangle.color.w / 255.0f) / 255.0f;

    float distanceSqrd = 1 + 2*((light.x - x)*(light.x - x) + (light.y - y)*(light.y - y) + (light.z - z)*(light.z - z));

    simd::float4 color = triangle.color * dot / distanceSqrd;
    return (Color)color;
}