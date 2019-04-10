#pragma once

#include <mutex>

#include <SFML/Graphics.hpp>

#include "Types.h"
#include "Timer.h"

struct RenderSettings;

class Window
{
public:
    Window(uint32 widthPx, uint32 heightPx);

    void notifyUpdate(const byte * const buffer, float dt);

    bool isOpen() const { return window.isOpen(); }

private:
    //Runs on the internal thread
    void internalUpdate(uint32 widthPx, uint32 heightPx);

    bool needsRedraw = false;
    const byte *buffer;
    std::mutex mutex;

    float lastFrameTime = 0.0f;

    Timer showFpsTimer;

    sf::RectangleShape rect;
    sf::RenderWindow window;
    sf::Texture texture;

    const sf::String TITLE = "Somewhat Real-Time RayMarcher - Frame time: ";
};

