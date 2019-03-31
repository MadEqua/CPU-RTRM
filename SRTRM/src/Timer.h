#pragma once

#include <chrono>
#include "Types.h"

class Timer
{
public:
    void start();

    float getElapsedSeconds();
    uint64 getElapsedMiliseconds();

private:
    bool started;
    std::chrono::time_point<std::chrono::steady_clock> startTime;
};

