#pragma once
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
#include <utility>

#define FORMAT_SPACE 0
#define FORMAT_CSV   1
#define FORMAT_YAML  2

union float2char {
    float float_val;
    uint8_t c[4];
};

union double2char {
    double double_val;
    uint8_t c[8];
};

union uint2char {
    uint32_t int_val;
    uint8_t c[4];
};

union uint162char {
    uint16_t int_val;
    uint8_t c[2];
};

union int2char {
    int int_val;
    uint8_t c[4];
};

union cArray2intArray {
    int16_t int_val[5];
    uint8_t c[10];
};

union floatArray2char {
    float float_val[6];
    uint32_t uint32_val[6];
    uint8_t c[24];
};

// void logd(std::string tag, const char* str, ...);

const std::string currentDateTime(const char* format);
int currentDateTimeInt();

struct MyException : public std::exception {
    std::string s;
    explicit MyException(std::string ss)
        : s(std::move(ss)) {}
    ~MyException() throw() {} // Updated
    const char* what() const throw() { return s.c_str(); }
};

std::string trimString(std::string s);
