#pragma once

#include <mutex>

#include <SFML/Graphics.hpp>

#include "Types.h"
#include "Timer.h"

struct RenderSettings;

class Window
{
public:
    Window(const RenderSettings &renderSettings);

    void notifyUpdate(const byte * const buffer, float dt);

    bool isOpen() const { return window.isOpen(); }

private:
    //Runs on the internal thread
    void internalUpdate(const RenderSettings &renderSettings);

    bool needsRedraw = false;
    const byte *buffer;
    std::mutex mutex;

    float frameTimeSum = 0.0f;
    uint32 frameCount = 0;

    Timer showFpsTimer;

    sf::RectangleShape rect;
    sf::RenderWindow window;
    sf::Texture texture;

    const sf::String TITLE = "Somewhat Real-Time Raymarcher - Avg frame time: ";
};

