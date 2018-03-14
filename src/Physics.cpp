#include <math.h>
#include <stdio.h>

#include "Physics.hpp"
#include "RK4.hpp"

Physics::Physics()
    : t(0)
{
    setSled(0, 0);

    setBall(0, 0.0, 0);
    setRadius(0, 1);
    setBallMass(0, 0.3);

    setBall(1, 0, 0);
    setRadius(1, 1);
    setBallMass(1, 0.3);

    setBall(2, 0, 0);
    setRadius(2, 1);
    setBallMass(2, 0.3);

    setSledMass(10);
}

void Physics::tick(double dt) {
    RK4<VarCount, velocities>::calc(t, dt, params, &position[0], &position[0]);
    t += dt;
}

Physics::Position Physics::ballPosition(int index) const {
    Position p = sledPosition();
    double angle = position[Ball1Angle];
    p.x -= sin(angle) * params[Radius1];
    p.y += cos(angle) * params[Radius1];
    if(index >= 1) {
        angle += position[Ball2Angle];
        p.x -= sin(angle) * params[Radius2];
        p.y += cos(angle) * params[Radius2];
    }
    if(index >= 2) {
        angle += position[Ball3Angle];
        p.x -= sin(angle) * params[Radius3];
        p.y += cos(angle) * params[Radius3];
    }
    return p;
}

double Physics::ballAngle(int index) const {
    switch(index){
    case 0:
        return position[Ball1Angle];
    case 1:
        return position[Ball2Angle];
    case 2:
        return position[Ball3Angle];
    }
}

double Physics::ballAngularVelocity(int index) const {
    switch(index){
    case 0:
        return position[Ball1Velocity];
    case 1:
        return position[Ball2Velocity];
    case 2:
        return position[Ball3Velocity];
    }
}

Physics::Position Physics::sledPosition() const {
    Position p;
    p.x = position[SledPosition];
    p.y = 0;
    return p;
}

double Physics::sledX() const {
    return position[SledPosition];
}

double Physics::sledVelocity() const {
    return position[SledVelocity];
}

double Physics::energy() const {
    return 0;
}

double Physics::time() const {
    return t;
}

void Physics::setBall(int index, double angle, double velocity) {
    switch(index) {
    case 0:
        this->position[Ball1Angle] = angle;
        this->position[Ball1Velocity] = velocity;
        break;
    case 1:
        this->position[Ball2Angle] = angle;
        this->position[Ball2Velocity] = velocity;
        break;
    case 2:
        this->position[Ball3Angle] = angle;
        this->position[Ball3Velocity] = velocity;
        break;
    }
}

void Physics::setRadius(int index, double radius) {
    switch(index) {
    case 0:
        this->params[Radius1] = radius;
        break;
    case 1:
        this->params[Radius2] = radius;
        break;
    case 2:
        this->params[Radius3] = radius;
        break;
    }

}

void Physics::setBallMass(int index, double m) {
    switch(index) {
    case 0:
        this->params[Ball1Mass] = m;
        break;
    case 1:
        this->params[Ball2Mass] = m;
        break;
    case 2:
        this->params[Ball3Mass] = m;
        break;
    }
}

void Physics::setSledMass(double m) {
    this->params[SledMass] = m;
}

void Physics::setSled(double position, double velocity) {
    this->position[SledPosition] = position;
    this->position[SledVelocity] = velocity;
}

void Physics::setForce(double f) {
    this->params[ForceX] = f;
}

void Physics::velocities(double t, double const *params, double const *positions, double *out) {
    double M = params[SledMass];
    double m1 = params[Ball1Mass];
    double m2 = params[Ball2Mass];
    double m3 = params[Ball3Mass];

    double l1 = params[Radius1];
    double l2 = params[Radius2];
    double l3 = params[Radius3];

    double u = params[ForceX];

    double x = positions[SledPosition];
    double dx = positions[SledVelocity];

    double p1 = positions[Ball1Angle];
    double dp1 = positions[Ball1Velocity];

    double p2 = positions[Ball2Angle];
    double dp2 = positions[Ball2Velocity];

    double p3 = positions[Ball3Angle];
    double dp3 = positions[Ball3Velocity];

    out[SledPosition] = dx;
    out[Ball1Angle] = dp1;
    out[Ball2Angle] = dp2;
    out[Ball3Angle] = dp3;

    out[SledVelocity]  = -(pow(8*M*m1*m2 + 4*M*m1*m3 + 4*M*m2*m3 + 4*m1*m2*m3 -
                               2*m1*(2*m2*(m2 + m3) + m1*(2*m2 + m3))*cos(2*p1) -
                               4*M*m2*(m2 + m3)*cos(2*p2) - 4*M*m1*m3*cos(2*p3) + 4*m2*pow(m1,2) +
                               2*m3*pow(m1,2) + m3*cos(2*(p1 - p3))*pow(m1,2) - 2*m3*cos(2*p3)*pow(m1,2) +
                               m3*cos(2*(p1 + p3))*pow(m1,2) + 4*M*pow(m2,2) + 4*m1*pow(m2,2),-1)*
                             (-8*m1*m2*u - 4*m1*m3*u - 7*m2*m3*u + m2*(m2 + m3)*u*cos(2*p2) +
                               4*m1*m3*u*cos(2*p3) - 7*u*pow(m2,2) + 6*m2*(m2 + m3)*u*pow(cos(p2),2) +
                               8*l1*m1*m2*m3*pow(dp1,2)*sin(p1) +
                               8*l2*m1*m2*cos(p2)*((m2 + m3)*pow(dp1 + dp2,2) +
                                  m3*cos(p3)*pow(dp1 + dp2 + dp3,2))*sin(p1) +
                               8*l1*m2*pow(dp1,2)*pow(m1,2)*sin(p1) +
                               4*l1*m3*pow(dp1,2)*pow(m1,2)*sin(p1) +
                               8*l1*m1*pow(dp1,2)*pow(m2,2)*sin(p1) - 4*g*m1*m2*m3*sin(2*p1) -
                               4*g*m2*pow(m1,2)*sin(2*p1) - 2*g*m3*pow(m1,2)*sin(2*p1) -
                               4*g*m1*pow(m2,2)*sin(2*p1) - 2*l1*m3*pow(dp1,2)*pow(m1,2)*sin(p1 - 2*p3) +
                               g*m3*pow(m1,2)*sin(2*(p1 - p3)) + g*m3*pow(m1,2)*sin(2*(p1 + p3)) -
                               2*l1*m3*pow(dp1,2)*pow(m1,2)*sin(p1 + 2*p3)));

    out[Ball1Velocity] = pow(l1,-1)*pow(8*M*m1*m2 + 4*M*m1*m3 + 4*M*m2*m3 + 4*m1*m2*m3 -
                                        2*m1*(2*m2*(m2 + m3) + m1*(2*m2 + m3))*cos(2*p1) -
                                        4*M*m2*(m2 + m3)*cos(2*p2) - 4*M*m1*m3*cos(2*p3) + 4*m2*pow(m1,2) +
                                        2*m3*pow(m1,2) + m3*cos(2*(p1 - p3))*pow(m1,2) - 2*m3*cos(2*p3)*pow(m1,2) +
                                        m3*cos(2*(p1 + p3))*pow(m1,2) + 4*M*pow(m2,2) + 4*m1*pow(m2,2),-1)*
                                      (4*(m2*(m2 + m3) + m1*(2*m2 + m3))*u*cos(p1) -
                                        4*m2*(m2 + m3)*u*cos(p1 + 2*p2) - 2*m1*m3*u*cos(p1 - 2*p3) -
                                        2*m1*m3*u*cos(p1 + 2*p3) + 8*g*M*m1*m2*sin(p1) + 4*g*M*m1*m3*sin(p1) +
                                        4*g*M*m2*m3*sin(p1) + 8*g*m1*m2*m3*sin(p1) + 8*g*m2*pow(m1,2)*sin(p1) +
                                        4*g*m3*pow(m1,2)*sin(p1) + 4*g*M*pow(m2,2)*sin(p1) +
                                        8*g*m1*pow(m2,2)*sin(p1) - 4*l1*m1*m2*m3*pow(dp1,2)*sin(2*p1) -
                                        4*l1*m2*pow(dp1,2)*pow(m1,2)*sin(2*p1) -
                                        2*l1*m3*pow(dp1,2)*pow(m1,2)*sin(2*p1) -
                                        4*l1*m1*pow(dp1,2)*pow(m2,2)*sin(2*p1) + 16*dp1*dp2*l2*M*m2*m3*sin(p2) +
                                        8*dp1*dp2*l2*m1*m2*m3*sin(p2) + 8*l2*M*m2*m3*pow(dp1,2)*sin(p2) +
                                        4*l2*m1*m2*m3*pow(dp1,2)*sin(p2) + 8*l2*M*m2*m3*pow(dp2,2)*sin(p2) +
                                        4*l2*m1*m2*m3*pow(dp2,2)*sin(p2) + 16*dp1*dp2*l2*M*pow(m2,2)*sin(p2) +
                                        8*dp1*dp2*l2*m1*pow(m2,2)*sin(p2) + 8*l2*M*pow(dp1,2)*pow(m2,2)*sin(p2) +
                                        4*l2*m1*pow(dp1,2)*pow(m2,2)*sin(p2) + 8*l2*M*pow(dp2,2)*pow(m2,2)*sin(p2) +
                                        4*l2*m1*pow(dp2,2)*pow(m2,2)*sin(p2) + 4*l1*M*m2*m3*pow(dp1,2)*sin(2*p2) +
                                        4*l1*M*pow(dp1,2)*pow(m2,2)*sin(2*p2) -
                                        8*dp1*dp2*l2*m1*m2*m3*sin(2*p1 + p2) -
                                        4*l2*m1*m2*m3*pow(dp1,2)*sin(2*p1 + p2) -
                                        4*l2*m1*m2*m3*pow(dp2,2)*sin(2*p1 + p2) -
                                        8*dp1*dp2*l2*m1*pow(m2,2)*sin(2*p1 + p2) -
                                        4*l2*m1*pow(dp1,2)*pow(m2,2)*sin(2*p1 + p2) -
                                        4*l2*m1*pow(dp2,2)*pow(m2,2)*sin(2*p1 + p2) - 4*g*M*m2*m3*sin(p1 + 2*p2) -
                                        4*g*M*pow(m2,2)*sin(p1 + 2*p2) - 2*g*M*m1*m3*sin(p1 - 2*p3) -
                                        2*g*m3*pow(m1,2)*sin(p1 - 2*p3) +
                                        l1*m3*pow(dp1,2)*pow(m1,2)*sin(2*(p1 - p3)) +
                                        8*dp1*dp2*l2*M*m2*m3*sin(p2 - p3) + 8*dp1*dp3*l2*M*m2*m3*sin(p2 - p3) +
                                        8*dp2*dp3*l2*M*m2*m3*sin(p2 - p3) + 4*dp1*dp2*l2*m1*m2*m3*sin(p2 - p3) +
                                        4*dp1*dp3*l2*m1*m2*m3*sin(p2 - p3) + 4*dp2*dp3*l2*m1*m2*m3*sin(p2 - p3) +
                                        4*l2*M*m2*m3*pow(dp1,2)*sin(p2 - p3) +
                                        2*l2*m1*m2*m3*pow(dp1,2)*sin(p2 - p3) +
                                        4*l2*M*m2*m3*pow(dp2,2)*sin(p2 - p3) +
                                        2*l2*m1*m2*m3*pow(dp2,2)*sin(p2 - p3) +
                                        4*l2*M*m2*m3*pow(dp3,2)*sin(p2 - p3) +
                                        2*l2*m1*m2*m3*pow(dp3,2)*sin(p2 - p3) -
                                        4*dp1*dp2*l2*m1*m2*m3*sin(2*p1 + p2 - p3) -
                                        4*dp1*dp3*l2*m1*m2*m3*sin(2*p1 + p2 - p3) -
                                        4*dp2*dp3*l2*m1*m2*m3*sin(2*p1 + p2 - p3) -
                                        2*l2*m1*m2*m3*pow(dp1,2)*sin(2*p1 + p2 - p3) -
                                        2*l2*m1*m2*m3*pow(dp2,2)*sin(2*p1 + p2 - p3) -
                                        2*l2*m1*m2*m3*pow(dp3,2)*sin(2*p1 + p2 - p3) +
                                        l1*m3*pow(dp1,2)*pow(m1,2)*sin(2*(p1 + p3)) +
                                        8*dp1*dp2*l2*M*m2*m3*sin(p2 + p3) + 8*dp1*dp3*l2*M*m2*m3*sin(p2 + p3) +
                                        8*dp2*dp3*l2*M*m2*m3*sin(p2 + p3) + 4*dp1*dp2*l2*m1*m2*m3*sin(p2 + p3) +
                                        4*dp1*dp3*l2*m1*m2*m3*sin(p2 + p3) + 4*dp2*dp3*l2*m1*m2*m3*sin(p2 + p3) +
                                        4*l2*M*m2*m3*pow(dp1,2)*sin(p2 + p3) +
                                        2*l2*m1*m2*m3*pow(dp1,2)*sin(p2 + p3) +
                                        4*l2*M*m2*m3*pow(dp2,2)*sin(p2 + p3) +
                                        2*l2*m1*m2*m3*pow(dp2,2)*sin(p2 + p3) +
                                        4*l2*M*m2*m3*pow(dp3,2)*sin(p2 + p3) +
                                        2*l2*m1*m2*m3*pow(dp3,2)*sin(p2 + p3) -
                                        4*dp1*dp2*l2*m1*m2*m3*sin(2*p1 + p2 + p3) -
                                        4*dp1*dp3*l2*m1*m2*m3*sin(2*p1 + p2 + p3) -
                                        4*dp2*dp3*l2*m1*m2*m3*sin(2*p1 + p2 + p3) -
                                        2*l2*m1*m2*m3*pow(dp1,2)*sin(2*p1 + p2 + p3) -
                                        2*l2*m1*m2*m3*pow(dp2,2)*sin(2*p1 + p2 + p3) -
                                        2*l2*m1*m2*m3*pow(dp3,2)*sin(2*p1 + p2 + p3) - 2*g*M*m1*m3*sin(p1 + 2*p3) -
                                        2*g*m3*pow(m1,2)*sin(p1 + 2*p3));
    out[Ball2Velocity] = 2*pow(l1,-2)*pow(l2,-1)*pow(8*M*m1*m2 + 4*M*m1*m3 + 4*M*m2*m3 + 4*m1*m2*m3 -
                                                     2*m1*(2*m2*(m2 + m3) + m1*(2*m2 + m3))*cos(2*p1) -
                                                     4*M*m2*(m2 + m3)*cos(2*p2) - 4*M*m1*m3*cos(2*p3) + 4*m2*pow(m1,2) +
                                                     2*m3*pow(m1,2) + m3*cos(2*(p1 - p3))*pow(m1,2) - 2*m3*cos(2*p3)*pow(m1,2) +
                                                     m3*cos(2*(p1 + p3))*pow(m1,2) + 4*M*pow(m2,2) + 4*m1*pow(m2,2),-1)*
                                                   (-4*m3*((l1*(m2 + m3)*cos(p2) + l2*(m2 + 2*m3 + 2*m3*cos(p3)) +
                                                           l1*m3*cos(p2 + p3))*((M + m1 + m2 + m3)*
                                                            (l2 + l2*cos(p3) + l1*cos(p2 + p3)) +
                                                           cos(p1 + p2 + p3)*(-(l1*(m1 + m2 + m3)*cos(p1)) -
                                                              l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3)))) +
                                                        ((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3))*
                                                         ((l2 + l2*cos(p3) + l1*cos(p2 + p3))*
                                                            (-(l1*(m1 + m2 + m3)*cos(p1)) -
                                                              l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3))) +
                                                           cos(p1 + p2 + p3)*(2*l1*l2*(m2 + m3)*cos(p2) +
                                                              2*l2*m3*(l2*cos(p3) + l1*cos(p2 + p3)) + (m1 + m2 + m3)*pow(l1,2) +
                                                              (m2 + 2*m3)*pow(l2,2))) -
                                                        (1 + cos(p3))*((M + m1 + m2 + m3)*
                                                            (2*l1*l2*(m2 + m3)*cos(p2) + 2*l2*m3*(l2*cos(p3) + l1*cos(p2 + p3)) +
                                                              (m1 + m2 + m3)*pow(l1,2) + (m2 + 2*m3)*pow(l2,2)) -
                                                           pow(l1*(m1 + m2 + m3)*cos(p1) +
                                                             l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3)),2)))*
                                                      (l1*cos(p3)*pow(dp1,2)*sin(p2) +
                                                        (l1*cos(p2)*pow(dp1,2) + l2*pow(dp1 + dp2,2))*sin(p3) - g*sin(p1 + p2 + p3)
                                                        ) + 4*(-(m3*(l2 + l2*cos(p3) + l1*cos(p2 + p3))*
                                                           ((M + m1 + m2 + m3)*(l2 + l2*cos(p3) + l1*cos(p2 + p3)) +
                                                             cos(p1 + p2 + p3)*(-(l1*(m1 + m2 + m3)*cos(p1)) -
                                                                l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3))))) +
                                                        (M + m1 + m2 + m3)*(2*l1*l2*(m2 + m3)*cos(p2) +
                                                           2*l2*m3*(l2*cos(p3) + l1*cos(p2 + p3)) + (m1 + m2 + m3)*pow(l1,2) +
                                                           (m2 + 2*m3)*pow(l2,2)) -
                                                        m3*cos(p1 + p2 + p3)*((l2 + l2*cos(p3) + l1*cos(p2 + p3))*
                                                            (-(l1*(m1 + m2 + m3)*cos(p1)) -
                                                              l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3))) +
                                                           cos(p1 + p2 + p3)*(2*l1*l2*(m2 + m3)*cos(p2) +
                                                              2*l2*m3*(l2*cos(p3) + l1*cos(p2 + p3)) + (m1 + m2 + m3)*pow(l1,2) +
                                                              (m2 + 2*m3)*pow(l2,2))) -
                                                        pow(l1*(m1 + m2 + m3)*cos(p1) +
                                                          l2*((m2 + m3)*cos(p1 + p2) + m3*cos(p1 + p2 + p3)),2))*
                                                      (-(l1*(m2 + m3)*pow(dp1,2)*sin(p2)) + g*(m2 + m3)*sin(p1 + p2) +
                                                        m3*(dp3*(2*dp1 + 2*dp2 + dp3)*l2*sin(p3) - l1*pow(dp1,2)*sin(p2 + p3) +
                                                           g*sin(p1 + p2 + p3))) +
                                                     (-4*l2*M*m2 - 4*l2*m1*m2 - 2*l2*M*m3 - 2*l2*m1*m3 - 2*l2*m2*m3 -
                                                        l1*(2*m2*(m2 + m3) + 2*M*(2*m2 + m3) + m1*(2*m2 + m3))*cos(p2) +
                                                        2*l2*m2*(m2 + m3)*cos(2*(p1 + p2)) + 2*l1*m1*m2*cos(2*p1 + p2) +
                                                        l1*m1*m3*cos(2*p1 + p2) + 2*l1*m2*m3*cos(2*p1 + p2) +
                                                        2*l2*M*m3*cos(2*p3) + 2*l2*m1*m3*cos(2*p3) + 2*l1*M*m3*cos(p2 + 2*p3) +
                                                        l1*m1*m3*cos(p2 + 2*p3) - l1*m1*m3*cos(2*p1 + p2 + 2*p3) -
                                                        2*l2*pow(m2,2) + 2*l1*cos(2*p1 + p2)*pow(m2,2))*
                                                      (g*l1*(m1 + m2 + m3)*sin(p1) +
                                                        l2*(dp2*(2*dp1 + dp2)*l1*(m2 + m3)*sin(p2) + g*(m2 + m3)*sin(p1 + p2) +
                                                           m3*(dp3*(2*dp1 + 2*dp2 + dp3)*l2*sin(p3) +
                                                              (dp2 + dp3)*(2*dp1 + dp2 + dp3)*l1*sin(p2 + p3) + g*sin(p1 + p2 + p3)
                                                              ))) + 2*l1*(-u + l1*(m1 + m2 + m3)*pow(dp1,2)*sin(p1) +
                                                        l2*(m2 + m3)*pow(dp1 + dp2,2)*sin(p1 + p2) +
                                                        2*dp1*dp2*l2*m3*sin(p1 + p2 + p3) + 2*dp1*dp3*l2*m3*sin(p1 + p2 + p3) +
                                                        2*dp2*dp3*l2*m3*sin(p1 + p2 + p3) + l2*m3*pow(dp1,2)*sin(p1 + p2 + p3) +
                                                        l2*m3*pow(dp2,2)*sin(p1 + p2 + p3) + l2*m3*pow(dp3,2)*sin(p1 + p2 + p3))*
                                                      (l2*cos(p1)*(2*m1*m2 + m1*m3 + m2*m3 - m2*(m2 + m3)*cos(2*p2) -
                                                           m1*m3*cos(2*p3) + pow(m2,2)) +
                                                        sin(p1)*(l1*(2*m2*(m2 + m3) + m1*(2*m2 + m3))*sin(p2) +
                                                           l2*m2*(m2 + m3)*sin(2*p2) - l1*m1*m3*sin(p2 + 2*p3))));
    out[Ball3Velocity] = 4*pow(l2,-1)*pow(8*M*m1*m2 + 4*M*m1*m3 + 4*M*m2*m3 + 4*m1*m2*m3 -
                                          2*m1*(2*m2*(m2 + m3) + m1*(2*m2 + m3))*cos(2*p1) -
                                          4*M*m2*(m2 + m3)*cos(2*p2) - 4*M*m1*m3*cos(2*p3) + 4*m2*pow(m1,2) +
                                          2*m3*pow(m1,2) + m3*cos(2*(p1 - p3))*pow(m1,2) - 2*m3*cos(2*p3)*pow(m1,2) +
                                          m3*cos(2*(p1 + p3))*pow(m1,2) + 4*M*pow(m2,2) + 4*m1*pow(m2,2),-1)*
                                        ((4*l2*M*m2*cos(p2)*(dp1*dp2*(m2 + m3) +
                                                (dp2*dp3 + dp1*(dp2 + dp3))*m3*cos(p3)) -
                                             g*M*cos(p1)*(2*m2*(m2 + m3) + m1*(2*m2 + m3) - m1*m3*cos(2*p3)) +
                                             (2*m2*(m2 + m3) + m1*(2*m2 + m3) - m1*m3*cos(2*p3))*
                                              (l1*M*pow(dp1,2) + u*sin(p1)))*sin(p2) +
                                          l2*M*m2*((m2 + m3)*(pow(dp1,2) + pow(dp2,2)) +
                                             m3*cos(p3)*(pow(dp1,2) + pow(dp2,2) + pow(dp3,2)))*sin(2*p2) -
                                          l2*M*m2*m3*pow(dp1 + dp2 + dp3,2)*pow(sin(p2),2)*sin(p3) -
                                          ((-2*l2*m1*cos(2*p1)*(2*dp2*dp3*(m1 + m2)*m3 +
                                                  2*dp1*(dp3*(m1 + m2)*m3 + dp2*(m1*m2 + 2*m1*m3 + m2*m3)) +
                                                  (m2*m3 + m1*(m2 + 2*m3))*pow(dp1,2) +
                                                  (m1*m2 + 2*m1*m3 + m2*m3)*pow(dp2,2) + (m1 + m2)*m3*pow(dp3,2)) -
                                               l2*M*m2*m3*pow(dp1 + dp2 + dp3,2)*pow(cos(p2),2) +
                                               l2*(8*dp1*dp2*M*m1*m2 + 16*dp1*dp2*M*m1*m3 + 8*dp1*dp3*M*m1*m3 +
                                                  8*dp2*dp3*M*m1*m3 + 2*dp1*dp2*M*m2*m3 + 2*dp1*dp3*M*m2*m3 +
                                                  2*dp2*dp3*M*m2*m3 + 4*dp1*dp2*m1*m2*m3 + 4*dp1*dp3*m1*m2*m3 +
                                                  4*dp2*dp3*m1*m2*m3 + 4*M*m1*m2*pow(dp1,2) + 8*M*m1*m3*pow(dp1,2) +
                                                  M*m2*m3*pow(dp1,2) + 2*m1*m2*m3*pow(dp1,2) + 4*M*m1*m2*pow(dp2,2) +
                                                  8*M*m1*m3*pow(dp2,2) + M*m2*m3*pow(dp2,2) + 2*m1*m2*m3*pow(dp2,2) +
                                                  4*M*m1*m3*pow(dp3,2) + M*m2*m3*pow(dp3,2) + 2*m1*m2*m3*pow(dp3,2) +
                                                  2*m1*m3*(2*M + m1 - m1*cos(2*p1))*cos(p3)*
                                                   (2*dp2*dp3 + 2*dp1*(2*dp2 + dp3) + 2*pow(dp1,2) + 2*pow(dp2,2) +
                                                     pow(dp3,2)) + 4*dp1*dp2*m2*pow(m1,2) + 8*dp1*dp2*m3*pow(m1,2) +
                                                  4*dp1*dp3*m3*pow(m1,2) + 4*dp2*dp3*m3*pow(m1,2) +
                                                  2*m2*pow(dp1,2)*pow(m1,2) + 4*m3*pow(dp1,2)*pow(m1,2) +
                                                  2*m2*pow(dp2,2)*pow(m1,2) + 4*m3*pow(dp2,2)*pow(m1,2) +
                                                  2*m3*pow(dp3,2)*pow(m1,2) +
                                                  M*m2*m3*pow(dp1 + dp2 + dp3,2)*pow(sin(p2),2)) +
                                               4*m1*cos(p2)*(m2 + m3 + m3*cos(p3))*
                                                (-(g*M*cos(p1)) + l1*M*pow(dp1,2) + u*sin(p1)))*sin(p3))/2.);
}
