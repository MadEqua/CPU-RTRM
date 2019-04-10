#include "Window.h"

#include <thread>
#include <chrono>

#include "RenderSettings.h"


Window::Window(uint32 widthPx, uint32 heightPx) :
    buffer(buffer) {

    std::thread thread(&Window::internalUpdate, this, widthPx, heightPx);
    thread.detach();
}

void Window::notifyUpdate(const byte * const buffer, float dt) {
    this->buffer = buffer;

    mutex.lock();
    needsRedraw = true;
    mutex.unlock();

    if(dt > 0.0f) {
        lastFrameTime = dt;
    }
}

void Window::internalUpdate(uint32 widthPx, uint32 heightPx) {
    window.create(sf::VideoMode(widthPx, heightPx), TITLE);
    texture.create(widthPx, heightPx);

    rect.setSize(sf::Vector2f(static_cast<float>(widthPx), static_cast<float>(heightPx)));
    rect.setTexture(&texture);
    texture.setSrgb(true); //enable conversion to srgb. TODO: change when we do this manually

    showFpsTimer.start();
    
    while(true) {
        sf::Event event;
        while(window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                window.close();
                return;
            }
        }

        if(needsRedraw) {
            mutex.lock();
            needsRedraw = false;
            mutex.unlock();

            texture.update(buffer);
            window.draw(rect);
            window.display();

            uint64 showFpsTime = showFpsTimer.getElapsedMiliseconds();
            if(showFpsTime > 100) {
                uint32 lastFrameTimeMs = static_cast<uint32>(lastFrameTime * 1000.0f);
                sf::String title = TITLE + std::to_string(lastFrameTimeMs) + " ms. FPS: " + std::to_string(1.0f / lastFrameTime);
                window.setTitle(title);

                showFpsTimer.start();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
