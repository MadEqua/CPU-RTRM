#include "Timer.h"

using namespace std::chrono;

void Timer::start() {
    started = true;
    startTime = steady_clock::now();
}

float Timer::getElapsedSeconds() {
    return static_cast<float>(getElapsedMiliseconds()) / 1000.0f;
}

uint64 Timer::getElapsedMiliseconds() {
    if(started) {
        time_point<steady_clock> now = steady_clock::now();
        return duration_cast<milliseconds>(now - startTime).count();
    }
    return 0;
}