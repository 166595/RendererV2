
#include "Mesh.h"

simd::float4 GetVector3(char vertexBuf[256], float w = 0.0f){
    std::stringstream ss(vertexBuf);
    simd::float4 vertex;
    char num[256];
    float f;
    ss >> num;

    ss >> num;
    sscanf(num,"%f", &f);
    vertex.x = f;

    ss >> num;
    sscanf(num,"%f", &f);
    vertex.y = f;

    ss >> num;
    sscanf(num,"%f", &f);
    vertex.z = f;

    vertex.w = w;
    
    return vertex;
}

simd::float2 GetVector2(char vertexBuf[256]){
    std::stringstream ss(vertexBuf);
    simd::float2 vertex;
    char num[256];
    float f;
    ss >> num;

    ss >> num;
    sscanf(num,"%f", &f);
    vertex.x = f;

    ss >> num;
    sscanf(num,"%f", &f);
    vertex.y = f;
    
    return vertex;
}

QuadIndices GetQuad3(char quadBuf[256]){
    std::stringstream ss(quadBuf);
    QuadIndices quad;
    std::string num;
    uint i;

    int iSize = 0;

    ss >> num;
    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.x = i - 1;

    sscanf(&num[iSize], "%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.textureCoordinate.x = i - 1;

    sscanf(&num[iSize], "%d", &i);
    iSize = 0;
    quad.normal.x = i;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.y = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.textureCoordinate.y = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.normal.y = i;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.z = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.textureCoordinate.z = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.normal.z = i;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.w = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.textureCoordinate.w = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.normal.w = i;

    return quad;
}

QuadIndices GetQuad2(char quadBuf[256]){
    std::stringstream ss(quadBuf);
    QuadIndices quad;
    std::string num;
    uint i;

    int iSize = 0;

    ss >> num;
    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.x = i - 1;

    sscanf(&num[iSize], "%d", &i);
    iSize = 0;
    quad.textureCoordinate.x = i - 1;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.y = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.textureCoordinate.y = i - 1;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.z = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.textureCoordinate.z = i - 1;

    ss >> num;

    sscanf(&num[iSize],"%d", &i);
    iSize += std::to_string(i).length() + 1;
    quad.vertex.w = i - 1;

    sscanf(&num[iSize],"%d", &i);
    iSize = 0;
    quad.textureCoordinate.w = i - 1;

    quad.normal.x = quad.normal.y = quad.normal.z = quad.normal.w = 0;

    return quad;
}

Mesh CreateMesh(std::string meshName){
    std::cout << "Creating Mesh\n";
    Mesh mesh;
    std::ifstream fileStream;
    std::string fileName = "Resources/" + meshName + ".obj";
    if(!GetObjectFile(fileStream, fileName)){
        std::cerr << "Error : Mesh : Could not open file\n";
        return mesh;
    }

    char buffer[256];
    memset(buffer,0,256);
    while(fileStream.peek() != 'v'){
        fileStream.getline(buffer,256);
    }
    bool hasNormals = false;
    InputType inputType = VERTEX;
    while(fileStream.peek() != std::char_traits<wchar_t>::eof()){
        fileStream.getline(buffer,256);
        inputType = (InputType)((int)buffer[0] + ((int)buffer[1] << 8));
        switch(inputType){
            case VERTEX : {
                mesh.vertices.push_back(GetVector3(buffer, 1.0f)); 
                break;}

            case VERTEX_NORMAL : {
                if(!hasNormals){hasNormals = true;}
                mesh.normals.push_back(GetVector3(buffer)); 
                break;}

            case TEXTURE_COORDINATE : {
                mesh.textureCoordinates.push_back(GetVector2(buffer)); 
                break;}

            case FACE : {
                if(hasNormals){
                    mesh.quadIndices.push_back(GetQuad3(buffer));
                }
                else{
                    mesh.quadIndices.push_back(GetQuad2(buffer));
                }
                break;}

            default : break;
        }
    }

    return mesh;
}