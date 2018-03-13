#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include <stdio.h>

class Controller {
public:
  Controller()
  {}

  double force(double p1, double dp1, double p2, double dp2, double x, double dx) {
      p1 = fmod(p1, 2*M_PI);
      p2 = fmod(p2, 2*M_PI);

      double weights[6]={-0.316,-2.975,-1409,-165.0,1441,303.9};
      double u = weights[0]*x + weights[1]*dx + weights[2]*p1 + weights[3]*dp1 + weights[4]*p2 + weights[5]*dp2;
      printf("x:%6.3f dx:%6.3f phi:%6.3f dphi:%6.3f f:%10.3f\n", x, dx, p1, dp1, u);
      fflush(stdout);
      return u;
  }
};

#endif // CONTROLLER_HPP
