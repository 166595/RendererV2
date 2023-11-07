
#include "MiniFValue.h"

/*
16 bits
First bit is normalised : 1 = Unnormalised
Second bit is sign : 1 = Negative
Last 14 bits are value;

// normal (1 bit) // sign (1 bit) // value (14 bit) //

// normal // sign // value //

// unnormal // signed char // bicimal (7 bit) //

// --- Normalised --- //
Normalised values are 0.xxxxxxxxxxxxxx
Positive and Negative values have identical absolute values

0.00000000000000 = 0
0.00000000000001 = 0.00006103515625
0.10000000000000 = 0.5
0.11111111111111 = 0.999938964844

// --- Unnormalised --- //
Unnormalised values are xxxxxxxx.xxxxxx
Basically a unsigned char and a 6 bit bicimal
Positive and Negative values have identical absolute values

Bits 2 to 9 are a signed char
Remaining 7 bits are a bicimal

.0000000 = 0
.0000001 = 0.0078125
.1000000 = 0.5
.1111111 = 0.9921875
*/

/* MiniFValue Functions -------------------------------------------------------------------- */

MiniFValue MFV::Add(MiniFValue a, MiniFValue b){
    bool a_unnorm = (a >> 15)&1;
    bool b_unnorm = (b >> 15)&1;
    if(a_unnorm != b_unnorm){return 0;}

    MiniFValue sum = 0;
    bool a_neg = (a >> 14)&1;
    bool b_neg = (b >> 14)&1;
    bool neg = a_neg ^ b_neg;

    sum = (a + b)&FOURTEEN_BIT;
    if(neg){
        sum = (a + b)&FOURTEEN_BIT;
    }
    else{
        sum = (a - b)&FOURTEEN_BIT;
    }
    sum |= (neg >> 14);
    sum |= (a_unnorm >> 14);
    return sum;
}

MiniFValue MFV::Add(MiniFValue a, float b){
    MiniFValue b_mfv = FloatToMiniFValue(b);
    return Add(a,b_mfv);
}

MiniFValue MFV::Mult(MiniFValue a, float b){
    bool a_unnorm = (a >> 15)&1; 
    MiniFValue product = 0;
    bool a_neg = (a >> 14)&1;
    bool b_neg = (b < 0);
    bool neg = a_neg ^ b_neg;

    if(a_unnorm){
        product = ((short)(a * b))&FOURTEEN_BIT;
    }
    else{
        product = ((short)(a * b))&SEVEN_BIT;
    }

    product |= (neg >> 14);
    product |= (a_unnorm >> 14);
    return product;
}

MiniFValue MFV::Mult(MiniFValue a, MiniFValue b){
    bool a_unnorm = (a >> 15)&1;
    bool b_unnorm = (b >> 15)&1;
    if(a_unnorm != b_unnorm){return 0;}

    MiniFValue product = 0;
    bool a_neg = (a >> 14)&1;
    bool b_neg = (b >> 14)&1;
    bool neg = a_neg ^ b_neg;

    if(a_unnorm){
        product = (a * b)&FOURTEEN_BIT;
    }
    else{
        product = (a * b)&SEVEN_BIT;
    }

    product |= (neg >> 14);
    product |= (a_unnorm >> 14);
    return product;

}

MiniFValue MFV::FloatToMiniFValue(float f){

    bool unnorm = (abs(f) >= 1);
    bool negative = (f < 0);
    short value = 0;

    if(negative){
        f = -f;
        value = value | (1 << 14); // negative
    }

    if(unnorm){
        value = value | FOURTEEN_BIT & (int) (f * SEVEN_BIT);
        value = value | (1 << 15); // unnorm
    } 

    else {
        value = value | FOURTEEN_BIT & (int) (f * FOURTEEN_BIT);
    }

    return value;
}

float MFV::MiniFValueToFloat(MiniFValue value){
    bool unnorm = (value >> 15)&1;
    bool negative = (value >> 14)&1;
    float f = 0;

    if(unnorm){
        f = (float) (value & FOURTEEN_BIT) / (float) SEVEN_BIT;
    } 

    else {
        f = (float) (value & FOURTEEN_BIT) / (float) FOURTEEN_BIT;

        if(f != 0){ // Preserve 0
            f += FIFTEEN_INV; // Ajustment for loss in precision
        }
    }

    if(negative){
        f = -f;
    }

    return f;
}

char MFV::MiniFValueToChar(MiniFValue value){

    bool unnorm = (value >> 15)&1;

    if(!unnorm){return (char)0;}

    bool neg = (value >> 14)&1;

    char out = (char)((value >> 7)&255);

    if(neg){
        return 127 - out;
    }

    return out;
}