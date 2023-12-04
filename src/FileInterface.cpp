#include "FileInterface.h"

FileInterface::~FileInterface(){
    CloseBMPFile();
}

void FileInterface::CreateBMPHeader(){
    int h_values[] = {66, 77, 138, 138, 124, (resolution >> 8), 256 - (resolution >> 8), 255, 255, 1, 32, 3, 64, 19, 11, 19, 11, 255, 255, 255, 255, 66, 71, 82, 115};
    int h_index[] = {0, 1, 2, 10, 14, 19, 23, 24, 25, 26, 28, 30, 36, 38, 39, 42, 43, 56, 59, 62, 69, 70, 71, 72, 73};
    memset(bmpHeader,0,138);
    for (int i = 0; i < 25; i++)
    {
        bmpHeader[h_index[i]] = (char)h_values[i];
    }
}

void FileInterface::LoadFrameToBuffer(ColorBuffer *buffer, unsigned int frame){
    std::ifstream loadFile;

    std::filesystem::path filePath = std::filesystem::current_path();

    switch(frame){
        case 1 : filePath += "/Resources/Frame1.bmp"; break;
        case 2 : filePath += "/Resources/Frame2.bmp"; break;
        default : return;
    }

    if(std::filesystem::exists(filePath)){
        loadFile.open(filePath, std::ios::in | std::ios::binary );
    }
    else{
        std::cout << "Failed to load file\n";
        return;
    }
    
    {
        loadFile.seekg(19);
        char sizeX = loadFile.get();
        loadFile.seekg(23);
        char sizeY = loadFile.get();
        if((sizeX != (char)8)&&(sizeX != (char)248)){
            std::cerr << "Error : Failed To Load Frame (Expected a size of 2048)\n";
            return;
        }
    }

    std::cout << "New Buffer\n";
    unsigned int length = 4*2048*2048;
    buffer->length = length;
    char charBuf[length];
    buffer->c = charBuf;
    loadFile.seekg(BMPHEAD);
    loadFile.read((char *)(buffer->c),length);
    std::cout << "Finish Buffer\n";
}


// Open BMP File
void FileInterface::CreateBMPFile_Start(){
    outFile.open("Output.bmp", std::ios::out | std::ios::binary );
    if(!resolution){resolution = 256;}
    if(resolution < 256){std::cerr << "Error : Resolution is too small (Must be equal or greater than 256)";return;}
    if((resolution&(resolution-1)) != 0){std::cerr << "Warning : Resolution is not a power of 2";}
    outFile.seekp(BMPHEAD);
}

// Close BMP File
void FileInterface::CreateBMPFile_End(){
    CreateBMPHeader();
    outFile.seekp(0);
    outFile.write(bmpHeader,BMPHEAD);
    outFile.close();
}

// Write ColorBuffer To BMP File
void FileInterface::CreateBMPFileWithBuffer(ColorBuffer buffer){
    std::cout << "CreateBMP\n"; 
    CreateBMPFile_Start();
    outFile.seekp(BMPHEAD);
    outFile.write(buffer.c,buffer.length);
    CreateBMPFile_End();
    //freeColorBuffer(buffer);
}

void FileInterface::CreateBMPFile(){
    CreateBMPFile_Start();
    int size = resolution*resolution;
    for(int i = 0; i < size; i++){
        outFile << (char)0 << (char)0 << (char)0 << (char)255;
    }
    CreateBMPFile_End();
}


void FileInterface::CloseBMPFile(){
    CreateBMPFile_End();
}

void FileInterface::SubmitColorBufferAsRow(ColorBuffer &buffer, unsigned int row){
    outFile.seekp(BMPHEAD + 4 * resolution * row);
    outFile.write(buffer.c, 4 * resolution);
}

void FileInterface::PrepBMPFileWithSize(unsigned int size){
    CreateBMPFile_Start();
    outFile.seekp(BMPHEAD);
}

void FileInterface::SubmitCharBufferAsRow(char *buffer, unsigned int length, unsigned int row){
    outFile.seekp(BMPHEAD + 4 * resolution * row);
    outFile.write(buffer, length);
}