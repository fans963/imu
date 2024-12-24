#pragma once

#include <cmath>

#ifdef __IAR_SYSTEMS_ICC__
# include <stdint.h>

typedef union _float2int {
    uint32_t u32_val;
    float float_val;
} float2int;
#endif

typedef struct LpMatrix3x3f {
    float data[3][3];
} LpMatrix3x3f;

typedef struct LpMatrix3x4f {
    float data[3][4];
} LpMatrix3x4f;

typedef struct LpMatrix4x3f {
    float data[4][3];
} LpMatrix4x3f;

typedef struct LpMatrix4x4f {
    float data[4][4];
} LpMatrix4x4f;

typedef struct LpVector3f {
    float data[3];
} LpVector3f;

typedef struct LpVector4f {
    float data[4];
} LpVector4f;

#ifdef __IAR_SYSTEMS_ICC__
typedef struct _LpVector3i {
    int16_t data[3];
} LpVector3i;

typedef struct _LpVector4i {
    int16_t data[4];
} LpVector4i;
#endif

void matZero3x3(LpMatrix3x3f* dest);
void matZero3x4(LpMatrix3x4f* dest);
void matZero4x3(LpMatrix4x3f* dest);
void matZero4x4(LpMatrix4x4f* dest);
void vectZero3x1(LpVector3f* dest);
void vectZero4x1(LpVector4f* dest);
void createIdentity3x3(LpMatrix3x3f* dest);
void createIdentity4x4(LpMatrix4x4f* dest);

#ifdef __WIN32
void print4x4(LpMatrix4x4f m);
#endif