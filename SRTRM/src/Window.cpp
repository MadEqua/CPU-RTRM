#include "Window.h"

#include <thread>
#include <chrono>

#include "RenderSettings.h"


Window::Window(const RenderSettings &renderSettings) :
    buffer(buffer) {

    std::thread thread(&Window::internalUpdate, this, renderSettings);
    thread.detach();
}

void Window::notifyUpdate(const byte * const buffer, float dt) {
    this->buffer = buffer;

    mutex.lock();
    needsRedraw = true;
    mutex.unlock();

    if(dt > 0.0f) {
        frameTimeSum += dt;
        frameCount++;
    }
}

void Window::internalUpdate(const RenderSettings &renderSettings) {
    window.create(sf::VideoMode(renderSettings.width, renderSettings.height), TITLE);
    texture.create(renderSettings.width, renderSettings.height);

    rect.setSize(sf::Vector2f(static_cast<float>(renderSettings.width), static_cast<float>(renderSettings.height)));
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
            if(showFpsTime > 200) {
                float avgFrameTime = frameTimeSum / static_cast<float>(frameCount);
                uint32 avgFrameTimeMs = static_cast<uint32>(avgFrameTime * 1000.0f);
                sf::String title = TITLE + std::to_string(avgFrameTimeMs) + " ms. Avg FPS: " + std::to_string(1.0f / avgFrameTime);
                window.setTitle(title);

                showFpsTimer.start();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
