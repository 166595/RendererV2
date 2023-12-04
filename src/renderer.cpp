
#include "renderer.h"

void Renderer::DrawTriangles(std::vector<Triangle> &triangles, int resolution){
    /*Mesh mesh;
    int tSize = triangles.size();
    for(int i = 0; i < tSize; i++){
        Triangle t = triangles[i];
        mesh.vertices.push_back(t.vertices[0]);
        mesh.vertices.push_back(t.vertices[1]);
        mesh.vertices.push_back(t.vertices[2]);
        mesh.normals.push_back(t.normals[0]);
        mesh.normals.push_back(t.normals[1]);
        mesh.normals.push_back(t.normals[2]);
        mesh.textureCoordinates.push_back(t.textureCoordinates[0]);
        mesh.textureCoordinates.push_back(t.textureCoordinates[1]);
        mesh.textureCoordinates.push_back(t.textureCoordinates[2]);
        QuadIndices quad;
        quad.vertex.x = 3*i;
        quad.vertex.y = 3*i + 1;
        quad.vertex.z = 3*i + 2;
        quad.vertex.w = 3*i + 2;
        quad.normal.x = 3*i;
        quad.normal.y = 3*i + 1;
        quad.normal.z = 3*i + 2;
        quad.normal.w = 3*i + 2;
        quad.textureCoordinate.x = 3*i;
        quad.textureCoordinate.y = 3*i + 1;
        quad.textureCoordinate.z = 3*i + 2;
        quad.textureCoordinate.w = 3*i + 2;
        mesh.quadIndices.push_back(quad);
    }

    simd::float4x4 identityMatrix = {
        simd::float4{1.0f, 0.0f, 0.0f, 0.0f},
        simd::float4{0.0f, 1.0f, 0.0f, 0.0f},
        simd::float4{0.0f, 0.0f, 1.0f, 0.0f},
        simd::float4{0.0f, 0.0f, 0.0f, 1.0f}
    };
    
    DrawMesh(mesh, identityMatrix, resolution);*/
}


void Renderer::DrawMeshes(std::vector<Mesh> &meshes,  int _resolution){

    #ifdef VERBOSE
        std::cout << "Render Start\n";
        std::cout.flush();
    #endif

    if(_resolution < 256){
        std::cout << "Resolution must be equal or greater than 256:\n";
        std::cout.flush();
        return;
    }

    if(_resolution > 4096){
        std::cout << "Resolution must be equal or less than 4096:\n";
        std::cout.flush();
        return;
    }

    std::vector<Triangle> _triangles = {};
    Color color = Color(255,255,255);
    for(int meshID = 0; meshID < meshes.size(); meshID++){
        Mesh &mesh = meshes[meshID];
        for(QuadIndices quad : mesh.quadIndices){
            simd::float4 v0 = mesh.vertices[quad.vertex.x];
            simd::float4 v1 = mesh.vertices[quad.vertex.y];
            simd::float4 v2 = mesh.vertices[quad.vertex.z];
            simd::float4 v3 = mesh.vertices[quad.vertex.w];

            Triangle triangleA;
            triangleA.meshID = meshID;
            triangleA.vertices[0] = v0;
            triangleA.vertices[1] = v1;
            triangleA.vertices[2] = v2;
            if(quad.normal.x != 0){
                triangleA.normals[0] = mesh.normals[quad.normal.x-1];
                triangleA.normals[1] = mesh.normals[quad.normal.y-1];
                triangleA.normals[2] = mesh.normals[quad.normal.z-1];
            }
            else{
                simd::float3 normal = simd::normalize(simd::cross(v1.xyz - v0.xyz, v2.xyz - v0.xyz));
                triangleA.normals[0] = (simd::float4){normal.x,normal.y,normal.z,0};
                triangleA.normals[1] = (simd::float4){normal.x,normal.y,normal.z,0};
                triangleA.normals[2] = (simd::float4){normal.x,normal.y,normal.z,0};
            }
            triangleA.textureCoordinates[0] = mesh.textureCoordinates[quad.textureCoordinate.x];
            triangleA.textureCoordinates[1] = mesh.textureCoordinates[quad.textureCoordinate.y];
            triangleA.textureCoordinates[2] = mesh.textureCoordinates[quad.textureCoordinate.z];


            Triangle triangleB;
            triangleB.meshID = meshID;
            triangleB.vertices[0] = v2;
            triangleB.vertices[1] = v3;
            triangleB.vertices[2] = v0;
            if(quad.normal.z != 0){
                triangleB.normals[0] = mesh.normals[quad.normal.z-1];
                triangleB.normals[1] = mesh.normals[quad.normal.w-1];
                triangleB.normals[2] = mesh.normals[quad.normal.x-1];
            }
            else{
                simd::float3 normal = simd::normalize(simd::cross(v3.xyz - v2.xyz, v0.xyz - v2.xyz));
                triangleB.normals[0] = (simd::float4){normal.x,normal.y,normal.z,0};
                triangleB.normals[1] = (simd::float4){normal.x,normal.y,normal.z,0};
                triangleB.normals[2] = (simd::float4){normal.x,normal.y,normal.z,0};
            }
            triangleB.textureCoordinates[0] = mesh.textureCoordinates[quad.textureCoordinate.z];
            triangleB.textureCoordinates[1] = mesh.textureCoordinates[quad.textureCoordinate.w];
            triangleB.textureCoordinates[2] = mesh.textureCoordinates[quad.textureCoordinate.x];

            TriangleShader(triangleA, mesh.transform);
            TriangleShader(triangleB, mesh.transform);

            _triangles.push_back(triangleA);
            _triangles.push_back(triangleB);
        }
    }
    

    // --- Initialize Constant Variables --- //

    const unsigned int numThreads = std::min((unsigned int)MAX_THREADS, std::thread::hardware_concurrency());

    const int initTriangleCount = _triangles.size();

    resolution = _resolution;

    const float invDRez = 2.0f/(float)resolution;

    const unsigned short chunkBias = 1;

    const unsigned short chunkTSize = (resolution + numThreads - 1) / numThreads;
    const unsigned short chunkTCount = numThreads;

    const unsigned short chunkXCount = std::max((unsigned int)CHUNK_COUNT, numThreads+1);
    const unsigned short chunkYCount = (std::max((unsigned int)CHUNK_COUNT, numThreads+1) + numThreads - 1) / numThreads;

    const unsigned short chunkXSize = (resolution + chunkXCount - 1) / chunkXCount;
    const unsigned short chunkYSize = (chunkTSize + chunkYCount - 1) / chunkYCount;


    // --- Create PreCalTriangle Triangles --- //

    std::vector<PreCalTriangle> triangles;

    std::cout << "PreCalTriangle Start\n";
    std::cout.flush();

    for(Triangle &t : _triangles){

        if((t.vertices[0].z > 0)&&(t.vertices[1].z > 0)&&(t.vertices[2].z > 0))
        {continue;}

        triangles.push_back(PreCalTriangle(t, resolution));


    }
    std::cout << "PreCalTriangle End\n";
    std::cout.flush();

    // --- Initialize File Interface --- //

    FileInterface *fileInterface = new FileInterface();
    fileInterface->resolution = resolution;

    bool fileISuccess = fileInterface->OpenImageFile();
    if(!fileISuccess){
        std::cerr << "Error : Renderer : fileInterface failure\n";
        delete fileInterface;
        return;
    }

    const int bufferSize = 4*resolution;

    // --- Debug --- //

    #ifdef VERBOSE
        int tCount = triangles.size();
        PrintDraw(tCount);
        int triangleInSize = sizeof(PreCalTriangle) * tCount;
        int triangleSize = sizeof(SteppedTriangle) * tCount;
        float triangleInSizeKB = (float)(triangleInSize/100)/10.0f;
        float triangleSizeKB = (float)(triangleSize/100)/10.0f;
        std::cout << "Triangle In Size: " << sizeof(PreCalTriangle) << "B, Triangle Size: " << sizeof(SteppedTriangle) << "B\n";
        std::cout << "Triangle In Total Size: " << triangleInSizeKB << "KB, " << triangleInSize << "B\n";
        std::cout << "Triangle Total Size: " << triangleSizeKB << "KB, " << triangleSize << "B\n";
        int totalSize = (sizeof(PreCalTriangle) + sizeof(SteppedTriangle)) * tCount * numThreads;
        float totalSizeKB = (float)(totalSize/100)/10.0f;
        float totalSizeMB = (float)(totalSize/100000)/10.0f;
        if(totalSizeMB > 1.0f){
            std::cout << "Total Triangle Memory Size: " << totalSizeMB << "MB, " << totalSize << "B\n";
        }
        else{
            std::cout << "Total Triangle Memory Size: " << totalSizeKB << "KB, " << totalSize << "B\n";
        }
        std::cout.flush();
    #endif

    // --- Start Threads --- //

    std::vector<std::thread> threads;

    std::mutex mtx;
    #ifdef VERBOSE
        std::cout << "Thead Define Start\n";
        std::cout.flush();
    #endif

    for(unsigned int threadIndex = 0; threadIndex < numThreads; threadIndex++){

        // --- Create Valid Triangle List For Current Thread --- //

        std::vector<PreCalTriangle*> validPreCalTriangles;

        for(int i = 0; i < initTriangleCount; i++){

            simd::ushort4 bounds = triangles[i].bounds;

            // --- Check Thread Chunk --- //

            unsigned short low = threadIndex * chunkTSize;
            unsigned short high = (threadIndex+1) * chunkTSize;

            if((bounds.y < high + chunkBias) && (bounds.w + chunkBias > low)){
                validPreCalTriangles.push_back(&triangles[i]);
            }

        }

        // --- Push Thread --- //
        
        threads.push_back(std::thread([&, validPreCalTriangles, threadIndex]{
            
            // --- Initialize Variables --- //

            const unsigned short threadYOffset = chunkTSize * threadIndex;

            const int triangleCount = validPreCalTriangles.size();

            // --- Heap Allocated Buffers --- //

            char *charBuffer = new char[bufferSize];

            SteppedTriangle *sTriangles = new SteppedTriangle[triangleCount];

            // --- Create Stepped Triangles --- //

            for(int i = 0; i < triangleCount; i++){
                sTriangles[i] = SteppedTriangle(*validPreCalTriangles[i], resolution);
                sTriangles[i].SetY(chunkTSize * threadIndex);
            }

            // --- Create Valid Y Triangle Lists --- //

            std::vector<SteppedTriangle*> validYTriangles[chunkYCount];

            for(int i = 0; i < triangleCount; i++){

                simd::ushort4 bounds = sTriangles[i].bounds;

                // --- Check Y Chunk --- //

                for(int j = 0; j < chunkYCount; j += 1){

                    unsigned short low = j * chunkYSize + threadYOffset;
                    unsigned short high = (j+1) * chunkYSize + threadYOffset;

                    if((bounds.y < high + chunkBias) && (bounds.w + chunkBias > low)){
                        validYTriangles[j].push_back(&sTriangles[i]);
                    }

                }

            }

            int validYTrianglesIndex = 0;

            // --- Start Y Loop --- //

            for(unsigned short y = threadYOffset; y < threadYOffset + chunkTSize; y++){

                std::fill_n(charBuffer, bufferSize, 0);

                int charBufferIndex = 0; // Index Of Current Pixel In Row

                const float yCoord = float(y)*invDRez - 1;

                // --- Create Valid X Triangle Lists --- //

                std::vector<SteppedTriangle*> validXTriangles[chunkXCount];

                for(SteppedTriangle *trianglePtr : validYTriangles[validYTrianglesIndex]){

                    // --- Bounds Check --- //

                    simd::ushort4 bounds = trianglePtr->bounds;
                    
                    if((y < bounds.y)||(y > bounds.w))
                    {continue;}

                    // --- Check X Chunk --- //

                    for(int j = 0; j < chunkXCount; j++){

                        unsigned short low = j * chunkXSize;
                        unsigned short high = (j+1) * chunkXSize;

                        if((bounds.x < high + chunkBias) && (bounds.z + chunkBias > low)){
                            validXTriangles[j].push_back(trianglePtr);
                        }

                    }

                }

                int validXTrianglesIndex = 0;

                // --- Start X Loop --- //

                SteppedTriangle *priorityTrianglePtr = nullptr;         // Last Triangle Drawn To Pixel

                for(unsigned short x = 0; x < resolution; x++){

                    const float xCoord = float(x)*invDRez - 1;

                    // --- Start Triangle Loop --- //

                    float depth = 1.0f;

                    SteppedTriangle *selectedTrianglePtr = nullptr;     // Current Triangle To Be Drawn To Pixel

                    if(priorityTrianglePtr != nullptr){                 // If Triangle Was Drawn To Last Pixel, Check That Triangle First

                        if(CheckSteppedTriangle(*priorityTrianglePtr, x, y, depth)){

                            selectedTrianglePtr = priorityTrianglePtr;
                            depth = priorityTrianglePtr->values[DepthStepIndex];
                            
                        }
                        else{

                            priorityTrianglePtr->StepX();
                            priorityTrianglePtr = nullptr;

                        }

                    }

                    for(SteppedTriangle *trianglePtr : validXTriangles[validXTrianglesIndex]){

                        if(trianglePtr == priorityTrianglePtr)
                        {continue;}   // Skip Priority Triangle, Already Checked

                        SteppedTriangle &triangle = *trianglePtr;

                        // --- Depth Buffer Check --- //

                        float zValue = triangle.values[DepthStepIndex];

                        if((zValue > depth)||(zValue < 0))
                        {triangle.StepX(); continue;}

                        // --- Bounds Check --- //

                        simd::ushort4 bounds = triangle.bounds;

                        if((x < bounds.x)||(x > bounds.z))
                        {triangle.StepX(); continue;}
                        
                        // --- Rasterization Check --- //

                        float bX = triangle.values[XBCoordStepIndex];

                        if(bX < 0.0)
                        {triangle.StepX(); continue;}

                        float bY = triangle.values[YBCoordStepIndex];

                        if((bY < 0.0)||(bX + bY > 1.0))
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
                        
                        // --- Get Color For Pixel --- //

                        Color colorOut = FragmentShader(
                            ShaderTriangle{
                                .normal = (simd::float4){selectedTrianglePtr->values[XNormalStepIndex],selectedTrianglePtr->values[YNormalStepIndex],selectedTrianglePtr->values[ZNormalStepIndex],0.0f},
                                .uv = (simd::float2){selectedTrianglePtr->values[XUVStepIndex],selectedTrianglePtr->values[YUVStepIndex]},
                                .meshID = selectedTrianglePtr->meshID
                            }, 
                            xCoord, yCoord
                        );                        

                        #ifdef CHUNK_VISUALIZATION
                            int R = (int)(255.0f * (float)validXTrianglesIndex / (float)chunkXCount);
                            int G = (int)(255.0f * (float)validYTrianglesIndex / (float)chunkYCount);
                            if(threadIndex % 2){
                                colorOut = Color(R,G,0);
                            }
                            else{
                                colorOut = Color(255 - R,255 - G,255);
                            }
                        #endif

                        // --- Step X --- //

                        selectedTrianglePtr->StepX();

                        // --- Set CharBuffer To Pixel Color --- //

                        memcpy(&charBuffer[charBufferIndex], &colorOut, 4);

                        // --- Set Priority Triangle --- //

                        priorityTrianglePtr = selectedTrianglePtr;

                    }

                    // --- Increment Pixel Index --- //

                    charBufferIndex += 4; // RGBA (4)

                    // --- Check If In Next Chunk --- //

                    if(x >= (validXTrianglesIndex+1) * chunkXSize - 1){

                        // --- Switch Chunk --- //

                        validXTrianglesIndex++;
                        priorityTrianglePtr = nullptr;

                        // --- Compensate For Switch --- //

                        for(SteppedTriangle *trianglePtr : validXTriangles[validXTrianglesIndex]){
                            trianglePtr->SetX(x);
                        }

                    }

                }

                // --- Submit Row --- //

                for(int i = 0; i < bufferSize; i += 4){
                    charBuffer[i+3] = (char)255; // Set All Alpha Values To 255
                }
                unsigned short yInv = resolution - y - 1;
                mtx.lock();
                fileInterface->SubmitCharBufferAsRow(charBuffer, bufferSize, yInv);
                mtx.unlock();

                // --- Check If In Next Chunk --- //

                if(y >= (validYTrianglesIndex+1) * chunkYSize + threadYOffset - 1){

                    // --- Switch Chunk --- //

                    validYTrianglesIndex++;

                    // --- Compensate For Switch --- //

                    for(SteppedTriangle *trianglePtr : validYTriangles[validYTrianglesIndex]){
                        trianglePtr->SetY(y);
                    }

                }
                else{

                    for(SteppedTriangle *trianglePtr : validYTriangles[validYTrianglesIndex]){
                        trianglePtr->StepY();
                    }

                }

            }

            // --- Delete Heap Allocated Buffers --- //

            delete[] charBuffer;
            delete[] sTriangles;
            
        }));
    }

    // --- Join Threads --- //
    #ifdef VERBOSE
        std::cout << "Join Threads\n";
        std::cout.flush();
    #endif

    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();

    for(auto& thread : threads){
        thread.join();
    }

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    double time = std::chrono::duration_cast< std::chrono::duration<double> >(t1 - t0).count();
    std::cout << "True Render Time: " << time << " (" << floor(1000*time) << "ms)\n";

    #ifdef VERBOSE
        std::cout << "Render End\n";
        std::cout.flush();
    #endif

    // --- Close File --- //

    fileInterface->CloseImageFile();
    #ifdef VERBOSE
        std::cout << "Finished Render\n";
        std::cout.flush();
    #endif

    // --- Delete Stack Memory --- //

    delete fileInterface;
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

    simd::ushort4 bounds = triangle.bounds;

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
    std::cout << "Vertex Count: " << 3*triangleCount << "\n";
    std::cout << "Options: ";
    #ifdef SIMD
        std::cout << "SIMD";
    #endif
    std::cout << "\n";
    std::cout.flush();
}