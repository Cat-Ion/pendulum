#include <math.h>
#include <stdio.h>

#include "Physics.hpp"
#include "RK4.hpp"

Physics::Physics()
    : t(0)
{
    setSled(0, 0);

    setBall(0, 0., 0);
    setRadius(0, 1);
    setBallMass(0, 0.6);

    setBall(1, 0, 0);
    setRadius(1, 1);
    setBallMass(1, 0.3);

    setSledMass(10);
}

void Physics::tick(double dt) {
    RK4<VarCount, velocities>::calc(t, dt, params, &position[0], &position[0]);
    t += dt;
}

Physics::Position Physics::ballPosition(int index) const {
    Position p = sledPosition();
    p.x -= sin(position[Ball1Angle]) * params[Radius1];
    p.y += cos(position[Ball1Angle]) * params[Radius1];
    if(index == 1) {
        p.x -= sin(position[Ball2Angle]) * params[Radius2];
        p.y += cos(position[Ball2Angle]) * params[Radius2];
    }
    return p;
}

double Physics::ballAngle(int index) const {
    switch(index){
    case 0:
        return position[Ball1Angle];
    case 1:
        return position[Ball2Angle];
    }
}

double Physics::ballAngularVelocity(int index) const {
    switch(index){
    case 0:
        return position[Ball1Velocity];
    case 1:
        return position[Ball2Velocity];
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
    }
}

void Physics::setRadius(int index, double radius) {
    switch(index) {
    case 0:
        this->params[Radius1] = radius;
        break;
    case 1:
        this->params[Radius2] = radius;
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

    double l1 = params[Radius1];
    double l2 = params[Radius2];

    double u = params[ForceX];

    double x = positions[SledPosition];
    double dx = positions[SledVelocity];

    double p1 = positions[Ball1Angle];
    double dp1 = positions[Ball1Velocity];

    double p2 = positions[Ball2Angle];
    double dp2 = positions[Ball2Velocity];

    out[SledPosition] = dx;
    out[Ball1Angle] = dp1;
    out[Ball2Angle] = dp2;

    double d = -(m1 * (2*M+m1) + m2 * (M+m1) - m1 * (m1+m2) * cos(2*p1) - M*m2*cos(2*(p1-p2)));

    out[SledVelocity]  = (-(2*m1+m2)*u + m2*u*cos(2*(p1-p2)) - g*m1*(m1+m2)*sin(2*p1) + 2*m1*sin(p1)*(l1*(m1+m2)*dp1*dp1 + l2*m2*cos(p1-p2)*dp2*dp2)) / d;
    out[Ball1Velocity] = (-(2*m1+m2)*u*cos(p1) + m2*u*cos(p1-2*p2) - g*(2*m1*(m1+m2)+M*(2*m1+m2))*sin(p1) - g*M*m2*sin(p1-2*p2) + l1*(m1*(m1+m2)*sin(2*p1) + M*m2*sin(2*(p1-p2)))*dp1*dp1 + 2*l2*m2*((M+m1)*cos(p2)*sin(p1) - M*cos(p1)*sin(p2))*dp2*dp2) / l1 / d;
    out[Ball2Velocity] = (2*sin(p1-p2) * ((m1+m2)*(g*M*cos(p1) - u*sin(p1) - l1*M*dp1*dp1) - l2*M*m2*cos(p1-p2)*dp2*dp2)) / l2 / d;
}
