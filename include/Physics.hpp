#ifndef PHYSICS_HPP
#define PHYSICS_HPP


class Physics
{
public:
    Physics();

    struct Position {
        double x, y;
    };

    enum {
        Radius1,
        Ball1Mass,
        Radius2,
        Ball2Mass,
        Radius3,
        Ball3Mass,
        SledMass,
        ForceX,
        ParCount
    };
    enum {
        Ball1Angle,
        Ball1Velocity,
        Ball2Angle,
        Ball2Velocity,
        Ball3Angle,
        Ball3Velocity,
        SledPosition,
        SledVelocity,
        VarCount
    };

    void setRadius(int index, double radius);
    void setSled(double position, double velocity);
    void setBall(int index, double angle, double velocity);

    void setBallMass(int index, double m);
    void setSledMass(double m);

    void setForce(double f);

    Position ballPosition(int index) const;
    double ballAngle(int index) const;
    double ballAngularVelocity(int index) const;
    Position sledPosition() const;
    double sledX() const;
    double sledVelocity() const;
    double energy() const;
    double time() const;

    void tick(double dt);

protected:
    static double constexpr g = 9.81;
    double t;
    double params[ParCount];
    double position[VarCount];

    static void velocities(double t, double const *params, double const *positions, double *out);
};

#endif // PHYSICS_HPP
