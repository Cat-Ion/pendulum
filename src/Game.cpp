#include "Game.hpp"

Game::Game(Window *w)
    : window(w),
      physics(),
      pressing_left(false),
      pressing_right(false),
      controller_on(true),
      running(true),
      center(true)
{
}

void Game::run() {
    quit = false;
    while(!quit) {
        handle_input();
        if (running) {
            double f = controller.force( physics.ballAngle(), physics.ballAngularVelocity(), physics.sledX(), physics.sledVelocity() );
            if(pressing_left && !pressing_right) {
                physics.setForce(-5);
            } else if(pressing_right && !pressing_left) {
                physics.setForce(5);
            } else if(controller_on){
                physics.setForce( f );
            } else {
                physics.setForce(0);
            }

            physics.tick(1./60);
        }
        window->clear();
        window->draw_line(0, window->height()/2, window->width(), window->height()/2);

        double scale = 5;
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
        Physics::Position s = physics.ballPosition();
        if(center) {
            s.x -= sledPosition.x;
            s.y -= sledPosition.y;
            sledPosition.x = sledPosition.y = 0;
        }

        sledPosition = scale_position(sledPosition, scale);
        s = scale_position(s, scale);

        window->draw_rectangle(sledPosition.x - 2*scale, sledPosition.y - 2*scale, sledPosition.x + 2*scale, sledPosition.y + 2*scale);
        window->draw_circle(s.x, s.y, 1*scale);

        window->draw_line(sledPosition.x, sledPosition.y, s.x, s.y);
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
    retval.x = p.x * scale + window->width()/2;
    retval.y = p.y * scale + window->height()/2;
    return retval;
}
