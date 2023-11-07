
#include <iostream>

#ifndef MINIFVALUE_H
#define MINIFVALUE_H

    #define FOURTEEN_BIT 16383
    #define FIFTEEN_INV 0.000030517578125
    #define SEVEN_BIT 127

    #define EIGHT_BIT_TO_FIVE_BIT 0.121568627451

    #define FIVE_BIT_TO_EIGHT_BIT 8.22580645161

    typedef short MiniFValue;
    namespace MFV{
        MiniFValue FloatToMiniFValue(float f);
        float MiniFValueToFloat(MiniFValue value);
        char MiniFValueToChar(MiniFValue value);

        MiniFValue Add(MiniFValue a, MiniFValue b);
        MiniFValue Add(MiniFValue a, float b);

        MiniFValue Mult(MiniFValue a, MiniFValue b);
        MiniFValue Mult(MiniFValue a, float b);
    }
    

#endif