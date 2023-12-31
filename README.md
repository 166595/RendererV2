# RendererV2

Was inspired and wanted to know how computers generate graphics, this program runs entirely on the CPU. _(Not fast)_

## Features include:
- Parsing a .obj file, _(Quads only)_
- Creating a BMP image from scratch
- Use fragment shaders
- Triangle Draws
- Mesh Draws
- Multithreading

## My Optimisations
While creating this, I had to make 3 major optimisations.

### 1. SteppedTriangles
Recalculating triangle data for each pixel is far too expensive.
To remedy this I created a SteppedTriangle which does as much precalculation as possible.
For each variable associated with a triangle, it calculates an origin value, a deltaX value, and a deltaY value.
With this, it is possible to only use addition to calculate all triangle bounds, depth, and screen area.

### 2. SIMD
With my implementation of my SteppedTriangle, my program was doing huge amounts of addition.
For each triangle, 8 integer additions were happening per pixel.
For a resolution of 2048x2048 (My goal), 1 triangle took about 34 million floating point operations. (\*GPU stares intensely\*)
So for 128 cubes, 1,536 triangles, my CPU was performing 6.4 Billion addition operations.
Using SIMD, I am able to load two floats (32-bit) into a register (64-bit arch) and perform two operations at once.
After implementing this, I saw my render times half instantly.

### 3. Bounds & Tiling
It is inefficient to iterate through all triangles per pixel, especially when multithreading.
So I split my triangles into buckets depending on their bounding box and gave each thread its own bucket.
Each thread then further split the triangles into chunks of around 64 x 64 pixels.
From this, I saw another huge increase in performance.

---
From these 3 changes, I saw roughly a 40x improvement on my machine. (M1 Mac 10 Core)

I have enjoyed working on this project and learnt a lot, but it is unlikely I will continue.

