#include "Game.hpp"

Game::Game(Window *w)
    : window(w),
      physics(),
      pressing_left(false),
      pressing_right(false),
      controller_on(true),
      running(false),
      center(false)
{
}

void Game::run() {
    quit = false;
    double const max_force = 100;
    while(!quit) {
        handle_input();
        if (running) {
            double f = 0;
            if(controller_on) {
                f = controller.force(
                        physics.ballAngle(0), physics.ballAngularVelocity(0),
                        physics.ballAngle(1), physics.ballAngularVelocity(1),
                        physics.ballAngle(2), physics.ballAngularVelocity(2),
                        physics.sledX(), physics.sledVelocity() );
                if(f > max_force) f = max_force;
                if(f < -max_force) f = -max_force;
            }
            if(pressing_left && !pressing_right) {
                physics.setForce(f-10);
            } else if(pressing_right && !pressing_left) {
                physics.setForce(f+10);
            } else {
                physics.setForce(f);
            }

            physics.tick(1./60);
        }
        window->clear();
        window->draw_line(0, window->height()/2, window->width(), window->height()/2);

        double scale = 50;
        double ballRadius = 0.1;
        double cartLength = 0.1;
        if(!center) {
            while (fabs(physics.sledX() * scale) >= window->width()/2) {
                scale *= 0.5;
            }

            window->draw_line( window->width()/2, window->height()/2-10, window->width()/2, window->height()/2+10);
            for(int i = 10; i * scale < window->width(); i += 10) {
                int x0 = window->width() / 2;
                int x = scale * i;
                int y = window->height() / 2;
                window->draw_line(x0+x, y-5, x0+x, y+5);
                window->draw_line(x0-x, y-5, x0-x, y+5);
            }
        }

        Physics::Position sledPosition = physics.sledPosition();
        Physics::Position s0 = physics.ballPosition(0);
        Physics::Position s1 = physics.ballPosition(1);
        Physics::Position s2 = physics.ballPosition(2);
        if(center) {
            s0.x -= sledPosition.x;
            s0.y -= sledPosition.y;

            s1.x -= sledPosition.x;
            s1.y -= sledPosition.y;

            s2.x -= sledPosition.x;
            s2.y -= sledPosition.y;

            sledPosition.x = sledPosition.y = 0;
        }

        sledPosition = scale_position(sledPosition, scale);
        s0 = scale_position(s0, scale);
        s1 = scale_position(s1, scale);
        s2 = scale_position(s2, scale);

        window->draw_rectangle(sledPosition.x - cartLength/2*scale, sledPosition.y - cartLength/2*scale + 100, sledPosition.x + cartLength/2*scale, sledPosition.y + cartLength/2*scale + 100);

        window->draw_rectangle(sledPosition.x - cartLength/2*scale, sledPosition.y - cartLength/2*scale, sledPosition.x + cartLength/2*scale, sledPosition.y + cartLength/2*scale);
        window->draw_circle(s0.x, s0.y, ballRadius*scale);
        window->draw_circle(s1.x, s1.y, ballRadius*scale);
        window->draw_circle(s2.x, s2.y, ballRadius*scale);

        window->draw_line(sledPosition.x, sledPosition.y, s0.x, s0.y);
        window->draw_line(s1.x, s1.y, s0.x, s0.y);
        window->draw_line(s1.x, s1.y, s2.x, s2.y);
        window->draw();
    }
}

void Game::handle_input() {
    SDL_Event e;
    while(SDL_PollEvent(&e)) {
        if(e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
            switch(e.key.keysym.sym) {
            case SDLK_LEFT:
                pressing_left = e.key.state == SDL_PRESSED;
                break;
            case SDLK_RIGHT:
                pressing_right = e.key.state == SDL_PRESSED;
                break;
            case SDLK_c:
                if(!e.key.repeat && e.key.state == SDL_PRESSED) {
                    center = !center;
                }
                break;
            case SDLK_SPACE:
                if(!e.key.repeat && e.key.state == SDL_PRESSED) {
                    controller_on = !controller_on;
                    printf("Controller on: %d", controller_on ? 1 : 0);
                    fflush(stdout);
                }
                break;
            case SDLK_p:
                if(!e.key.repeat && e.key.state == SDL_PRESSED) {
                    running = !running;
                    printf("Running: %d", running ? 1 : 0);
                    fflush(stdout);
                }
                break;
            case SDLK_ESCAPE:
                quit = true;
                break;
            }
        }
    }
}

Physics::Position Game::scale_position(Physics::Position const &p, double scale) {
    Physics::Position retval;
    retval.x =  p.x * scale + window->width()/2;
    retval.y = -p.y * scale + window->height()/2;
    return retval;
}
