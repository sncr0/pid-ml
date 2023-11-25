#ifndef FORCES_H
#define FORCES_H
//#include "main.h"

#include <vector>
#include <fstream>
#include <iostream>

class Force {
public:
    virtual double calculate(double position, double dt) = 0;
    virtual void printPreamble(std::ofstream& file) = 0;
    virtual void printStatusHeader()  = 0;
    virtual void printStatus() = 0;
    virtual ~Force() = default;
};

class ForceVector : public Force {
public:
    std::vector<Force*> forces_vector;

    ForceVector(std::vector<Force*>& _forces_vector);  // Corrected constructor declaration

    double calculate(double position, double dt) override;

    void printPreamble(std::ofstream& file) override;

    void printStatusHeader() override;

    void printStatus() override;
};


class GravityForce : public Force {
public:
    double acceleration;

    GravityForce(double _acceleration);
    double calculate(double position, double dt) override;
    void printPreamble(std::ofstream& file) override;
    void printStatusHeader() override;
    void printStatus() override;
};

class PIDForce : public Force {
public:
    double Kp, Ki, Kd;
    double target;
    mutable double error, integral, derivative;

    PIDForce(double _Kp, double _Ki, double _Kd, double _target);

    double calculate(double position, double dt) override;

    void printPreamble(std::ofstream& file) override;

    void printStatusHeader() override;

    void printStatus() override;
};


#endif // FORCES_H