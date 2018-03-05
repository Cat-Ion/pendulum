#ifndef WINDOW_HPP
#define WINDOW_HPP
#include <SDL2/SDL.h>

class Window {
public:
    Window(char const *title, int w, int h);
    void clear();
    void draw();
    void set_clear_color(uint8_t r, uint8_t g, uint8_t b);
    void set_draw_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

    void draw_circle(int x, int y, int r);
    void draw_line(int x1, int y1, int x2, int y2);
    void draw_rectangle(int x1, int y1, int x2, int y2);

    int width() const;
    int height() const;
protected:
    struct Color {
        uint8_t r, g, b, a;
    };

    SDL_Window *sdl_window;
    SDL_Renderer *sdl_renderer;
    Color clear_color;
    Color draw_color;
};

#endif // WINDOW_HPP
