#include "FileInterface.h"


FileInterface::~FileInterface(){
    if(outFile.is_open()){outFile.close();}
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

// Open BMP File
bool FileInterface::OpenImageFile(){
    outFile.open("Output.bmp", std::ios::out | std::ios::binary );
    if(!outFile.is_open()){
        std::cerr << "Error : FileInterface : Could not open file\n";
        return false;
    }
    if(!resolution){resolution = 256;}
    if(resolution < 256){std::cerr << "Error : FileInterface : Resolution is too small (Must be equal or greater than 256)";return false;}
    if((resolution&(resolution-1)) != 0){std::cerr << "Warning : FileInterface : Resolution is not a power of 2";}
    outFile.seekp(BMPHEAD);
    return true;
}

// Close BMP File
void FileInterface::CloseImageFile(){
    CreateBMPHeader();
    outFile.seekp(0);
    outFile.write(bmpHeader,BMPHEAD);
    outFile.close();
}

void FileInterface::SubmitCharBufferAsRow(char *buffer, unsigned int length, unsigned int row){
    outFile.seekp(BMPHEAD + 4 * resolution * row);
    outFile.write(buffer, length);
}

// --- Non-Class Functions --- //

bool GetObjectFile(std::ifstream &fileStream, std::string fileName){
    fileStream.open(fileName, std::ios::in);
    if(!fileStream.is_open()){
        std::cerr << "Error : FileInterface : Could not open file\n";
        return false;
    }
    return true;
}