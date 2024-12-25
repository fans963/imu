#pragma once

#include <cstdio>
#include <cstdarg>
#include <string>
#include <ctime>
#include "util/logging.hpp"

// Verbose level
enum {
    VERBOSE_NONE,
    VERBOSE_INFO,
    VERBOSE_DEBUG
};

class LpLog
{
    int verboseLevel;

    public:
        static LpLog& getInstance()
        {
            static LpLog instance; // Guaranteed to be destroyed.
            return instance;
        }
        
    private:
        LpLog() {
            verboseLevel = VERBOSE_INFO;
        };

    public:
        LpLog(LpLog const&)   = delete;
        void operator=(LpLog const&)  = delete;

        void setVerbose(int level)
        {
            verboseLevel = level;
        }

        void d(const std::string& tag, const char* str, ...) const
        {
            if (verboseLevel < VERBOSE_DEBUG)
                return;
            va_list a_list;
            va_start(a_list, str);
            if (!tag.empty())
                LOG_WARN("[%s]: ", tag.c_str());
            vprintf(str, a_list);
            va_end(a_list);
        } 

        void i(const std::string& tag, const char* str, ...) const
        {
            if (verboseLevel < VERBOSE_INFO)
                return;
            va_list a_list;
            va_start(a_list, str);
            if (!tag.empty())
                LOG_INFO("[%s]: ", tag.c_str());
            vprintf(str, a_list);
            va_end(a_list);
        } 

        static void e(const std::string& tag, const char* str, ...)
        {
            va_list a_list;
            va_start(a_list, str);
            if (!tag.empty())
                LOG_ERROR("[%s]: ", tag.c_str());
            vprintf(str, a_list);
            va_end(a_list);
        } 
};
