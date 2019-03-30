#include "Window.h"

#include <thread>
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
}

void Window::internalUpdate(const RenderSettings &renderSettings) {
    window.create(sf::VideoMode(renderSettings.width, renderSettings.height), TITLE);
    texture.create(renderSettings.width, renderSettings.height);

    rect.setSize(sf::Vector2f(static_cast<float>(renderSettings.width), static_cast<float>(renderSettings.height)));
    rect.setTexture(&texture);
    texture.setSrgb(true); //enable conversion to srgb. TODO: change when we do this manually

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

            using namespace std::chrono;
            time_point<steady_clock> now = steady_clock::now();
            uint64 dt = duration_cast<milliseconds>(now - lastRedrawTime).count();
            lastRedrawTime = steady_clock::now();
            sf::String title = TITLE + std::to_string(dt) + " ms. FPS:  " + std::to_string(1000.0f / dt);
            window.setTitle(title);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
