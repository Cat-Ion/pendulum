#include "Window.hpp"
#include <math.h>

Window::Window(char const *title, int w, int h) {
    sdl_window = SDL_CreateWindow(title, 0, 0, w, h, 0);
    sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
}

void Window::clear() {
    SDL_SetRenderDrawColor(sdl_renderer, clear_color.r, clear_color.g, clear_color.b, clear_color.a);
    SDL_RenderClear(sdl_renderer);
    SDL_SetRenderDrawColor(sdl_renderer, draw_color.r, draw_color.g, draw_color.b, draw_color.a);
}

void Window::draw() {
    SDL_RenderPresent(sdl_renderer);
}

void Window::draw_circle(int x, int y, int r) {
    int n = r * 3;

    int x1 = r, y1 = 0, x2, y2;
    for(int i = 1; i <= n; i++) {
        x2 = r * cos(2 * M_PI * i / n);
        y2 = r * sin(2 * M_PI * i / n);
        draw_line(x+x1, y+y1, x+x2, y+y2);
        x1 = x2;
        y1 = y2;
    }
}

void Window::draw_line(int x1, int y1, int x2, int y2)
{
    SDL_RenderDrawLine(sdl_renderer, x1, y1, x2, y2);
}

void Window::draw_rectangle(int x1, int y1, int x2, int y2) {
    draw_line(x1, y1, x2, y1);
    draw_line(x2, y1, x2, y2);
    draw_line(x2, y2, x1, y2);
    draw_line(x1, y2, x1, y1);
}

void Window::set_clear_color(uint8_t r, uint8_t g, uint8_t b) {
    clear_color.r = r;
    clear_color.g = g;
    clear_color.b = b;
    clear_color.a = 255;
}

void Window::set_draw_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    draw_color.r = r;
    draw_color.g = g;
    draw_color.b = b;
    draw_color.a = 255;
    SDL_SetRenderDrawColor(sdl_renderer, draw_color.r, draw_color.g, draw_color.b, draw_color.a);
}

int Window::width() const {
    int w;
    SDL_GetWindowSize(sdl_window, &w, NULL);
    return w;
}

int Window::height() const {
    int h;
    SDL_GetWindowSize(sdl_window, NULL, &h);
    return h;
}
