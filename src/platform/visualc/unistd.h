/*
 * This file is part of the Mingw32 package.
 *
 * unistd.h maps (roughly) to io.h
 */

#ifndef __UNISTD_INTERFACE__
#define __UNISTD_INTERFACE__

#ifndef __STRICT_ANSI__
#include <io.h>
#endif
#include <windows.h>

inline void usleep(int waitTime) {
    __int64 time1 = 0, time2 = 0, freq = 0;

    QueryPerformanceCounter((LARGE_INTEGER *)&time1);
    QueryPerformanceFrequency((LARGE_INTEGER *)&freq);

    do {
        QueryPerformanceCounter((LARGE_INTEGER *)&time2);
    } while ((time2 - time1) < waitTime);
}

#endif /* __UNISTD_INTERFACE__ */