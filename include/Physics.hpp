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
        Radius,
        BallMass,
        SledMass,
        ForceX,
        ParCount
    };
    enum {
        BallAngle,
        BallVelocity,
        SledPosition,
        SledVelocity,
        VarCount
    };

    void setRadius(double radius);
    void setSled(double position, double velocity);
    void setBall(double angle, double velocity);

    void setBallMass(double m);
    void setSledMass(double m);

    void setForce(double f);

    Position ballPosition() const;
    double ballAngle() const;
    double ballAngularVelocity() const;
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
