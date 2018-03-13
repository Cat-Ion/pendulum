#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include <stdio.h>
#include "PID.hpp"

class Controller {
public:
  Controller()
  {}

  double force(double phi, double dphi, double x, double dx) {
      double phimod = fmod(phi, 2*M_PI);
      if(phimod < 0) {
          phimod += 2*M_PI;
      }
      phimod -= M_PI;
      double u = x + 7.09 * dx - 342.965 * (phi-M_PI) - 420 * dphi;
      printf("x:%6.3f dx:%6.3f phi:%6.3f dphi:%6.3f f:%10.3f\n", x, dx, phimod, dphi, u);
      fflush(stdout);
      return u;
  }
};

#endif // CONTROLLER_HPP
