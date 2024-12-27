#pragma once
class MicroMeasure {
public:
    MicroMeasure();
    void reset(void);
    long long measure(void);
    void sleep(long long s_t);

public:
    long long start_time;
    long long tpm;
};
