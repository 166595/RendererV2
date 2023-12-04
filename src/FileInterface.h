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
      void CreateBMPFile_Start();
      void CreateBMPFile_End();
      void CreateBMPHeader();
      public:
      ~FileInterface();
      int resolution;
      void CreateBMPFile();
      void CreateBMPFileWithBuffer(ColorBuffer buffer);
      void LoadFrameToBuffer(ColorBuffer *buffer, unsigned int frame);
      void PrepBMPFileWithSize(unsigned int size);
      void SubmitColorBufferAsRow(ColorBuffer &buffer, unsigned int row);
      void SubmitCharBufferAsRow(char *buffer, unsigned int length, unsigned int row);
      void CloseBMPFile();
   };

#ifdef __cplusplus
   } // extern "C"
#endif

#endif