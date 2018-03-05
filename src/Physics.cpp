#include <math.h>
#include <stdio.h>

#include "Physics.hpp"
#include "RK4.hpp"

Physics::Physics()
    : t(0)
{
    setSled(0, 0);
    setBall(M_PI, -0.1);
    setRadius(15);
    setBallMass(0.3);
    setSledMass(1);
}

void Physics::tick(double dt) {
    RK4<VarCount, velocities>::calc(t, dt, params, &position[0], &position[0]);
    t += dt;
}

Physics::Position Physics::ballPosition() const {
    Position p = sledPosition();
    p.x += sin(position[BallAngle]) * params[Radius];
    p.y += cos(position[BallAngle]) * params[Radius];
    return p;
}

double Physics::ballAngle() const {
    return position[BallAngle];
}

double Physics::ballAngularVelocity() const {
    return position[BallVelocity];
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

void Physics::setBall(double angle, double velocity) {
    this->position[BallAngle] = angle;
    this->position[BallVelocity] = velocity;
}

void Physics::setRadius(double radius) {
    this->params[Radius] = radius;
}

void Physics::setBallMass(double m) {
    this->params[BallMass] = m;
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
    double phi = positions[BallAngle];
    double dphi = positions[BallVelocity];

    double mb = params[BallMass];
    double ms = params[SledMass];
    double r = params[Radius];
    double fx = params[ForceX];

    double cp = cos(phi);
    double sp = sin(phi);

    out[SledPosition] = positions[SledVelocity];
    out[BallAngle] = positions[BallVelocity];

    out[SledVelocity] = - (fx + cp * g * mb * sp - dphi*dphi * r * mb * sp) / (-mb + cp*cp * mb - ms);
    out[BallVelocity] = - (-cp * fx - g * mb * sp + cp * dphi*dphi * r * mb * sp - g * ms * sp) / (r * (-mb + cp*cp * mb - ms));
}
