#include "forces.h"


ForceVector::ForceVector(std::vector<Force*>& _forces_vector) : forces_vector(_forces_vector) {};

    double ForceVector::calculate(double position, double dt) {
        double combinedForce = 0.0;
        for (auto& force : forces_vector) {
            combinedForce += force->calculate(position, dt);
        }
        return combinedForce;
    }

    void ForceVector::printPreamble(std::ofstream& file) {
        for (auto& force : forces_vector) {
            force->printPreamble(file);
        }
    }

    void ForceVector::printStatusHeader() {
        // No specific status header for ForceVectorClass
    }

    void ForceVector::printStatus() {
        for (auto& force : forces_vector) {
            force->printStatus();
        }
    }

GravityForce::GravityForce(double _acceleration) : acceleration(_acceleration) {}

    double GravityForce::calculate(double position, double dt) {
        return acceleration;
    }

    void GravityForce::printPreamble(std::ofstream& file) {
        file << "## Gravity force = " << acceleration << "\n";
    }

    void GravityForce::printStatus() {
        // Do nothing for GravityForce
    }

    void GravityForce::printStatusHeader() {
        // Do nothing for GravityForce
    }


PIDForce::PIDForce(double _Kp, double _Ki, double _Kd, double _target)
    : Kp(_Kp), Ki(_Ki), Kd(_Kd), target(_target), error(0), integral(0), derivative(0) {}

    double PIDForce::calculate(double position, double dt) {
        double previous_error = error;
        error = target - position;
        integral += error * dt;
        derivative = (error - previous_error) / dt;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    void PIDForce::printPreamble(std::ofstream& file) {
        file << "## PID controller = Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << "\n";
    }

    void PIDForce::printStatusHeader() {
        std::cout << "error integral derivative";
    }

    void PIDForce::printStatus() {
        std::cout << error << " " << integral << " " << derivative;
    }


