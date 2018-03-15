#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include <stdio.h>

class Controller {
public:
  Controller()
  {}

  double force(double p1, double dp1, double p2, double dp2, double p3, double dp3, double x, double dx) {
      p1 = angular_modulo(p1);
      p2 = angular_modulo(p2);
      p3 = angular_modulo(p3);

      double weights[8]={0.316228, 3.16086, -260.349, -211.846, 4158.39, 88.6172, -6425.55, \
                         -1226.81};
      double u = weights[0]*x + weights[1]*dx + weights[2]*p1 + weights[3]*dp1 + weights[4]*p2 + weights[5]*dp2 + weights[6]*p3 + weights[7]*dp3;
      printf( "x:%10.4g dx:%10.4g ", x, dx);
      printf("p1:%10.4g dp1:%10.4g ", p1, dp1);
      printf("p2:%10.4g dp2:%10.4g ", p2, dp2);
      printf("p3:%10.4g dp3:%10.4g ", p3, dp3);
      printf("u:%6.3f\n", u);
      fflush(stdout);
      return u;
  }

  static double angular_modulo(double a) {
      a = fmod(a, 2*M_PI);
      if( a > M_PI ) {
          a -= 2*M_PI;
      } else if(a < -M_PI) {
          a += 2*M_PI;
      }
      return a;
  }
};

#endif // CONTROLLER_HPP
