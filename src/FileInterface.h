#include <iostream>
#include <fstream>
#include "RenderTypes.h"

#ifndef FileInterface_h
#define FileInterface_h

#define BMPHEAD 138

#ifdef __cplusplus
   extern "C"
   {
#endif
   class FileInterface{
      private:
      std::ofstream outFile;
      char bmpHeader[BMPHEAD];
      void CreateBMPHeader();
      public:
      ~FileInterface();
      int resolution;
      bool OpenImageFile();
      void SubmitCharBufferAsRow(char *buffer, unsigned int length, unsigned int row);
      void CloseImageFile();
   };

   bool GetObjectFile(std::ifstream &fileStream, std::string fileName);

#ifdef __cplusplus
   } // extern "C"
#endif

#endif