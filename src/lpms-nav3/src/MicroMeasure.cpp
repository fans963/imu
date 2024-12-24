#include "LPMS-NAV3/MicroMeasure.hh"

#include <sys/time.h>

#include <cstddef>
#include <cassert>

static const unsigned usec_per_sec = 1000000;
static const unsigned usec_per_msec = 1000;

bool QueryPerformanceFrequency(long long *frequency)
{
    *frequency = (long long)usec_per_sec;

    return true;
}

bool QueryPerformanceCounter(long long *performance_count)
{
    struct timeval time;

    assert(performance_count != NULL);

    gettimeofday(&time, NULL);
    *performance_count = time.tv_usec + time.tv_sec * (long long)usec_per_sec;

    return true;
}

MicroMeasure::MicroMeasure()
{
    long long ticksPerSecond;

    QueryPerformanceFrequency(&ticksPerSecond);
    tpm = ticksPerSecond;
    start_time = 0LL;
}

void MicroMeasure::reset(void)
{
    long long tick;

    QueryPerformanceCounter(&tick);
    start_time = tick;
}

long long MicroMeasure::measure(void)
{
    long long tick;

    QueryPerformanceCounter(&tick);

    return (((tick - start_time) * 1000LL * 1000LL) / tpm);
}

void MicroMeasure::sleep(long long s_t)
{
    reset();

    while (measure() < s_t)
    {
    }
}
