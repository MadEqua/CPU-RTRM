#pragma once

#include <mutex>
#include <chrono>

#include <SFML/Graphics.hpp>

#include "Types.h"

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
    std::chrono::time_point<std::chrono::steady_clock> lastRedrawTime;

    sf::RectangleShape rect;
    sf::RenderWindow window;
    sf::Texture texture;

    const sf::String TITLE = "Somewhat Real-Time Raytracer [Frame time: ";
};

