#ifndef SIMULATION_H
#define SIMULATION_H

#include "forces.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

extern bool print_time;
extern bool print_position;
extern bool print_velocity;
extern bool print_acceleration;
extern bool print_force;

class System {
private:
    double mass;
    double originalPosition, originalVelocity;
    double position, velocity, acceleration;
    double diff_p_t, diff_p_t_sq, diff_p_t_sq_cumulative;
    double dt;

    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> times;
    std::vector<double> diff_p_t_vector;
    std::vector<double> diff_p_t_sq_cumulative_vector;

    ForceVector& force_vector;

public:
    System(double _mass, double _originalPosition, double _originalVelocity, double _dt, ForceVector& _force_vector);

    void simulate(double simulationDuration);

    void saveResultsToFile(const std::string& filename) const;

    void plotResults() const;
};

#endif // SIMULATION_H