#include <SDL2/SDL.h>

#include "Window.hpp"
#include "Game.hpp"

int main() {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_TIMER);
    Window w("Pendulum", 1280, 720);
    Game(&w).run();
    SDL_Quit();
    return 0;
}
