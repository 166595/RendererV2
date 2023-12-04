
#include "renderer.h"

void Renderer::DrawTriangle(std::vector<Triangle> _triangles, int _resolution){

    if(_resolution < 256){
        std::cout << "Resolution must be equal or greater than 256:\n";
        return;
    }

    if(_resolution > 4096){
        std::cout << "Resolution must be equal or less than 4096:\n";
        return;
    }

    resolution = _resolution;

    #ifdef VERBOSE
        PrintDraw(_triangles.size());
    #endif

    int triangleCount = _triangles.size();

    // --- Create PreCalculated Triangles --- //

    std::vector<PreCalTriangle> triangles;
    for(auto t : _triangles){

        if((t.vertex[0].position.z > 0)&&(t.vertex[1].position.z > 0)&&(t.vertex[2].position.z > 0))
        {continue;}

        triangles.push_back(PreCalTriangle(t.vertex, resolution));

    }

    // --- Initialize File Interface --- //

    FileInterface fileInterface;
    fileInterface.resolution = resolution;

    fileInterface.PrepBMPFileWithSize(4*resolution*resolution);

    int bufferSize = 4*resolution;

    // --- Define Threads --- //

    unsigned int numThreads = std::thread::hardware_concurrency();

    if(numThreads > MAX_THREADS){numThreads = MAX_THREADS;}

    //std::cout << "Num Threads: " << numThreads << "\n";

    std::vector<std::thread> threads;

    std::mutex mtx;

    // --- Debug --- //

    #ifdef VERBOSE
        int triangleInSize = sizeof(PreCalTriangle) * triangleCount;
        int triangleSize = sizeof(SteppedTriangle) * triangleCount;
        float triangleInSizeKB = (float)(triangleInSize/100)/10.0f;
        float triangleSizeKB = (float)(triangleSize/100)/10.0f;
        std::cout << "Triangle In Size: " << sizeof(PreCalTriangle) << "B, Triangle Size: " << sizeof(SteppedTriangle) << "B\n";
        std::cout << "Triangle In Total Size: " << triangleInSizeKB << "KB, " << triangleInSize << "B\n";
        std::cout << "Triangle Total Size: " << triangleSizeKB << "KB, " << triangleSize << "B\n";
        int totalSize = (sizeof(PreCalTriangle) + sizeof(SteppedTriangle)) * triangleCount * numThreads;
        float totalSizeKB = (float)(totalSize/100)/10.0f;
        float totalSizeMB = (float)(totalSize/100000)/10.0f;
        if(totalSizeMB > 1.0f){
            std::cout << "Total Memory Size: " << totalSizeMB << "MB, " << totalSize << "B\n";
        }
        else{
            std::cout << "Total Memory Size: " << totalSizeKB << "KB, " << totalSize << "B\n";
        }
    #endif

    // --- Start Threads --- //

    for(unsigned int threadIndex = 0; threadIndex < numThreads; threadIndex++){
        threads.push_back(std::thread([&, triangles, threadIndex]{
            
            // --- Initialize Variables --- //

            const float invDRez = 2.0f/(float)resolution;

            const int triangleCount = triangles.size();
            
            // --- Heap Allocated Buffers --- //

            char *charBuffer = new char[bufferSize];

            SteppedTriangle *sTriangles = new SteppedTriangle[triangleCount];

            for(int i = 0; i < triangleCount; i++){
                sTriangles[i] = SteppedTriangle(triangles[i], resolution, numThreads, threadIndex);
                sTriangles[i].pcTriangle = &triangles[i];
            }

            // --- Start Y Loop --- //

            for(unsigned short y = threadIndex; y < resolution; y += numThreads){

                int charBufferIndex = 0;

                const float yCoord = float(y)*invDRez - 1;

                // --- Start X Loop --- //

                const unsigned short xChunkSize = XCHUNK_SIZE;

                const unsigned short xChunkCount = resolution / xChunkSize;

                std::vector<SteppedTriangle*> validYTriangles[xChunkCount];

                for(int i = 0; i < triangleCount; i++){

                    // --- Bounds Check --- //

                    simd::ushort4 bounds = triangles[i].bounds;
                    
                    if((y < bounds.y)||(y > bounds.w))
                    {continue;}

                    // --- Check Side --- //

                    for(int j = 0; j < xChunkCount; j++){

                        unsigned short low = j * xChunkSize;
                        unsigned short high = (j+1) * xChunkSize;

                        if((bounds.x < high) && (bounds.z > low)){
                            validYTriangles[j].push_back(&sTriangles[i]);
                        }

                    }

                }

                SteppedTriangle *priorityTrianglePtr = nullptr;

                int validYTrianglesIndex = 0;


                for(unsigned short x = 0; x < resolution; x++){

                    const float xCoord = float(x)*invDRez - 1;

                    charBuffer[charBufferIndex] = 0;              // Blue
                    charBuffer[charBufferIndex+1] = 0;            // Green
                    charBuffer[charBufferIndex+2] = 0;            // Red
                    charBuffer[charBufferIndex+3] = (char)255;    // Alpha

                    // --- Start Triangle Loop --- //

                    float depth = 1.0f;

                    SteppedTriangle *selectedTrianglePtr = nullptr;

                    if(priorityTrianglePtr != nullptr){

                        if(CheckSteppedTriangle(*priorityTrianglePtr, x, y, depth)){

                            selectedTrianglePtr = priorityTrianglePtr;
                            depth = priorityTrianglePtr->values[DepthStepIndex];
                            
                        }
                        else{

                            priorityTrianglePtr->StepX();
                            priorityTrianglePtr = nullptr;

                        }

                    }

                    for(SteppedTriangle *trianglePtr : validYTriangles[validYTrianglesIndex]){

                        if(trianglePtr == priorityTrianglePtr)
                        {continue;}   // Skip Priority Triangle

                        SteppedTriangle &triangle = *trianglePtr;

                        // --- Depth Buffer Check --- //

                        float zValue = triangle.values[DepthStepIndex];

                        if((zValue > depth)||(zValue < 0))
                        {triangle.StepX(); continue;}

                        // --- Rasterization Check --- //

                        float bX = triangle.values[XBCoordStepIndex];

                        if(bX < 0.0)
                        {triangle.StepX(); continue;}

                        float bY = triangle.values[YBCoordStepIndex];

                        if((bY < 0.0)||(bX + bY > 1.0))
                        {triangle.StepX(); continue;}

                        // --- Bounds Check --- //

                        simd::ushort4 bounds = triangle.pcTriangle->bounds;

                        if((x < bounds.x)||(x > bounds.z))
                        {triangle.StepX(); continue;}

                        // --- Set Depth --- //

                        depth = zValue;

                        // --- Update Selected Triangle --- //

                        if(selectedTrianglePtr != nullptr){
                            selectedTrianglePtr->StepX();
                        }

                        selectedTrianglePtr = trianglePtr;

                    }

                    // --- Fragment Shader --- //

                    if(selectedTrianglePtr != nullptr){
                        
                        // --- Get Color --- //

                        Color colorOut = TriangleFragment(
                            ShaderTriangle{
                                .normal = selectedTrianglePtr->pcTriangle->normal,
                                .color = *reinterpret_cast<simd::float4*>(&selectedTrianglePtr->values[ColorRStepIndex])
                            }, 
                            xCoord, yCoord
                        );

                        // --- Step X --- //

                        selectedTrianglePtr->StepX();

                        // --- Set Char Buffer --- //

                        memcpy(&charBuffer[charBufferIndex], &colorOut, 4);
                        charBuffer[charBufferIndex+3] = (char)255; // Alpha Channel

                        // --- Set Priority Triangle --- //

                        priorityTrianglePtr = selectedTrianglePtr;

                    }

                    // --- Increment Index --- //

                    charBufferIndex += 4;

                    // --- Check Side --- //

                    if(x >= (validYTrianglesIndex+1) * xChunkSize){

                        // --- Switch Side --- //

                        validYTrianglesIndex++;
                        priorityTrianglePtr = nullptr;

                        // --- Compensate For Switch --- //

                        for(SteppedTriangle *trianglePtr : validYTriangles[validYTrianglesIndex]){
                            trianglePtr->SetX(x);
                        }

                    }

                }

                // --- Submit Row --- //

                int yinv = resolution - y - 1;

                mtx.lock();
                fileInterface.SubmitCharBufferAsRow(charBuffer, bufferSize, yinv);
                mtx.unlock();

                for(int i = 0; i < triangleCount; i++){
                    sTriangles[i].StepY();
                }
            }

            // --- Delete Heap Allocated Buffers --- //

            delete[] charBuffer;
            delete[] sTriangles;
            
        }));
    }

    for(auto& thread : threads){
        thread.join();
    }

    fileInterface.CloseBMPFile();
    #ifdef VERBOSE
        std::cout << "Finished Render\n";
    #endif
}

bool Renderer::CheckSteppedTriangle(SteppedTriangle &triangle, unsigned short x, unsigned short y, float depth){

    // --- Depth Buffer Check --- //

    float zValue = triangle.values[DepthStepIndex];

    if((zValue < 0)||(zValue > depth))
    {return false;}

    // --- Rasterization Check --- //

    float bX = triangle.values[XBCoordStepIndex];
    float bY = triangle.values[YBCoordStepIndex];

    if((bX < 0.0)||(bY < 0.0)||(bX + bY > 1.0))
    {return false;}

    // --- Bounds Check --- //

    simd::ushort4 bounds = triangle.pcTriangle->bounds;

    if((x < bounds.x)||(x > bounds.z))
    {return false;}

    return true;
}

// --- Utility Functions --- //

/*Vector3 BarycentricCoord(float x, float y, int i){
    Vector3 p0 = _vertices[_triangles[i].v[0]];
    Vector3 p1 = _vertices[_triangles[i].v[1]];
    Vector3 p2 = _vertices[_triangles[i].v[2]];

    float _x = (x - p2.x) * (p1.y - p2.y) + (y - p2.y) * (p2.x - p1.x);
    float _y = (x - p2.x) * (p2.y - p0.y) + (y - p2.y) * (p0.x - p2.x);
    float D = (p1.y - p2.y) * (p0.x - p2.x) - (p2.y - p0.y) * (p2.x - p1.x);
    _x /= D;
    _y /= D;

    Vector3 bCoord = Vector3(_x,_y,0);
    return bCoord;
}*/

// --- Debug Functions --- //

void Renderer::PrintDraw(int triangleCount){
    int actualThreads = std::thread::hardware_concurrency();
    if(actualThreads > MAX_THREADS){actualThreads = MAX_THREADS;}
    std::cout << "Draw\n";
    std::cout << "Resolution: " << resolution << "x" << resolution << "\n";
    std::cout << "Resolution Squared: " << resolution * resolution << "\n";
    std::cout << "Max Threads: " << MAX_THREADS << " Actual Threads: " << actualThreads << "\n";
    std::cout << "Triangle Count: " << triangleCount << "\n";
    std::cout << "Options: ";
    #ifdef SIMD_X
        std::cout << "SIMD X ";
    #endif
    #ifdef SIMD_Y
        std::cout << "SIMD Y ";
    #endif
    std::cout << "\n";
}

// --- Unused / Unoptimised --- //

// --- Equation Renderer --- //

#ifdef EQUATION_RENDERER

void Renderer::DrawEquation(){

    resolution = 2048;
    ColorBuffer pixelBuffer;
    pixelBuffer.length = 4*resolution*resolution;
    //pixelBuffer.c = new char[pixelBuffer.length];
    FileInterface fileInterface;
    fileInterface.LoadFrameToBuffer(&pixelBuffer,1);

    // Buffers used to store previous pixel row
    // Buffers are alternated between
    bool EquationLineBufferA[resolution+1];
    bool EquationLineBufferB[resolution+1];
    bool writeBufferA = false;

    // Initial Parameters
    float xScale = 5;
    float yScale = 5;
    float dx = xScale*(2.0f/(float)resolution);
    float dy = yScale*(2.0f/(float)resolution);

    // Create First EquationBufferLine
    FillEquationLineBuffer(EquationLineBufferA,xScale,-yScale-dy,resolution);
    // Draw each row of pixels
    for(int y = 0; y < resolution; y++){
        float _y = yScale*(2*(float)y/(float)resolution - 1);

        // Check current buffer
        if(writeBufferA){
            FillEquationLineBuffer(EquationLineBufferA,xScale,_y,resolution);
        }
        else{
            FillEquationLineBuffer(EquationLineBufferB,xScale,_y,resolution);
        }
        // Switch Write Buffer
        writeBufferA = !writeBufferA;

        for(int x = 0; x < resolution; x++){
            // Check pixel edges
            int edgeCount = 0;
            if(EquationLineBufferA[x]){edgeCount++;};
            if(EquationLineBufferA[x+1]){edgeCount++;};
            if(EquationLineBufferB[x]){edgeCount++;};
            if(EquationLineBufferB[x+1]){edgeCount++;};

            // If 1,2 or 3 edges, draw pixel
            if((edgeCount > 0)&&(edgeCount < 4)){
                char colorValue = (char)255;
                int index = 4*resolution*y + 4*x;
                pixelBuffer.c[index] = colorValue;
                pixelBuffer.c[index+1] = colorValue;
                pixelBuffer.c[index+2] = colorValue;
                pixelBuffer.c[index+3] = (char)255;
            }
        }
    }

    // Write Pixel Buffer To File
    fileInterface.resolution = resolution;
    fileInterface.CreateBMPFileWithBuffer(pixelBuffer);
    std::cout << "Finished Render\n";
}

void Renderer::FillEquationLineBuffer(bool *eLineBuffer, float xScale, float y, float resolution){
    for(int x = 0; x < resolution+1; x++){
        float _x = xScale*(2*(float)x/(float)resolution - 1);
        eLineBuffer[x] = (EquationShader(_x,y)>0);
    }
}

#endif