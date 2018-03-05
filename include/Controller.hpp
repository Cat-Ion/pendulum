#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include <stdio.h>
#include "PID.hpp"

class Controller {
public:
  Controller()
      : tickLength(1./60),
        angleSetpoint          ( 0.01, 0.0005, 0.0001, tickLength, M_PI - M_PI/8, M_PI + M_PI/8, 0),
        angularVelocitySetpoint( 15.00,  5.0,  0.00, tickLength, -1,   1, M_PI),
        forceSetpointAboveSled ( 10.00,  5.0,  0.00, tickLength, -30., 30., 0),
        forceSetpointBelowSled (-10.00, -5.0, -0.00, tickLength, -30., 30., 0)
  {}

  double force(double phi, double dphi, double x, double dx) {
      if (fabs(x) > 1) {
          angleSetpoint.step(x);
          angularVelocitySetpoint.set_setpoint(angleSetpoint.output());
      } else {
          angleSetpoint.step(x*x*x);
          angularVelocitySetpoint.set_setpoint(M_PI);
      }

      double phimod = fmod(phi, 2*M_PI);
      angularVelocitySetpoint.step(phimod);
      double retval = 0;
      PID<double> *forceSetpoint = &forceSetpointBelowSled;
      if(phimod >= M_PI/2 && phimod <= 3*M_PI/2 ) {
          forceSetpoint = &forceSetpointAboveSled;
      }
      forceSetpoint->set_setpoint(angularVelocitySetpoint.output());
      forceSetpoint->step(dphi);


      printf("x:%6.3f phi SP:%6.3f phi:%6.3f dphi SP:%6.3f dphi:%6.3f f SP:%10.3f\n", x, angleSetpoint.output(), phi, angularVelocitySetpoint.output(), dphi, forceSetpoint->output());
      fflush(stdout);

      return forceSetpoint->output();
  }

protected:
  double tickLength;
  PID<double> angleSetpoint;
  PID<double> angularVelocitySetpoint;
  PID<double> forceSetpointAboveSled;
  PID<double> forceSetpointBelowSled;

  bool pendulum_above_sled(double phi) {
      double phimod = fmod(phi, 2*M_PI);
      if(phimod < 0) {
          phimod += 2*M_PI;
      }
      return phimod >= M_PI_2 && phimod <= 3*M_PI_2;
  }
};

#endif // CONTROLLER_HPP
