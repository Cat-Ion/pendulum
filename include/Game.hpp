#ifndef GAME_HPP
#define GAME_HPP
#include "Controller.hpp"
#include "Physics.hpp"
#include "Window.hpp"

class Game {
public:
    Game(Window *w);

    void run();

protected:
    Window *window;
    Physics physics;
    Controller controller;

    bool pressing_left, pressing_right;
    bool controller_on;
    bool running;
    bool center;
    bool quit;

    void handle_input();
    Physics::Position scale_position(Physics::Position const &p, double scale);
};

#endif // GAME_HPP
