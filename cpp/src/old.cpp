#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>  // Include the header for std::setprecision
#include <vector>
#include <getopt.h>
#include <iostream>
#include <vector>
//#include "pid.h"

bool print_time = true;
bool print_position = true;
bool print_velocity = false;
bool print_acceleration = false;
bool print_force = false;

// ForceClasses
class Force {
public:
    virtual double calculate(double position, double dt) = 0;
    virtual void printPreamble(std::ofstream& file) = 0;
    virtual void printStatusHeader()  = 0;
    virtual void printStatus() = 0;
    virtual ~Force() = default;
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

// Gravity Force
class GravityForce : public Force {
public:
    double acceleration = 9.81;
    GravityForce(double _acceleration) : acceleration(_acceleration) {}
    double calculate(double position, double dt) override {
        return acceleration; 
    }
    void printPreamble(std::ofstream& file) override {
        file << "## Gravity force = " << acceleration << "\n";
    }
    void printStatus() override {};
    void printStatusHeader() override {};
};



class PIDForce : public Force {
public:
    double Kp, Ki, Kd;
    double target;
    mutable double error, integral, derivative;
    PIDForce(double _Kp, double _Ki, double _Kd, double _target)
        : Kp(_Kp), Ki(_Ki), Kd(_Kd), target(_target), error(0), integral(0), derivative(0) {}

    double calculate(double position, double dt) override {
        double previous_error = error;
        error = target - position;
        integral += error * dt;
        derivative = (error - previous_error) / dt;
        return Kp * error + Ki * integral + Kd * derivative;
    }
    
    void printPreamble(std::ofstream& file) override {
        file << "## PID controller = Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << "\n";
    }
    void printStatusHeader() override {
        std::cout << "error integral derivative";
    }
    void printStatus() override {
        std::cout << error << " " << integral << " " << derivative;
    }
    
};

class Force {
public:
    virtual double calculate(double position, double dt)  = 0;
    virtual void printPreamble(std::ofstream& file)  = 0;
    virtual void printStatusHeader()  = 0;
    virtual void printStatus()  = 0;
    virtual ~Force() = default;
};

class ForceVectorClass : public Force {
public:
    std::vector<Force*> forces_vector;

    ForceVectorClass( std::vector<Force*>& _forces_vector) : forces_vector(_forces_vector) {}

    double calculate(double position, double dt)  override {
        double combinedForce = 0.0;
        for ( auto& force : forces_vector) {
            combinedForce += force->calculate(position, dt);
        }
        return combinedForce;
    }

    void printPreamble(std::ofstream& file)  override {
        for (auto& force : forces_vector) {
            force->printPreamble(file);
        }
    }

    void printStatusHeader()  override {
        // No specific status header for ForceVectorClass
    }

    void printStatus()  override {
        for (auto& force : forces_vector) {
            force->printStatus();
        }
    }
};

class GravitySimulator {
private:
    double mass;
    double originalPosition, originalVelocity;
    double position, velocity, acceleration;
    double dt;

    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> times;

    Force& force;

public:
    GravitySimulator(double _mass, double _originalPosition, double _originalVelocity, double _dt, Force& _force)
        : mass(_mass),
          originalPosition(_originalPosition),
          originalVelocity(_originalVelocity),
          position(_originalPosition),
          velocity(_originalVelocity),
          dt(_dt),
          force(_force) {}

    void simulate(double simulationDuration) {
        for (double time = 0; time < simulationDuration; time += dt) {
            double forceValue = force.calculate(position, dt);

            // Forces calculation using F = m * a
            acceleration = forceValue / mass;

            // Verlet integration method
            double positionNext = 2 * position - originalPosition + acceleration * dt * dt;
            velocity = (positionNext - originalPosition) / (2 * dt);

            // Update position and reset originalPosition for the next iteration
            originalPosition = position;
            position = positionNext;

            // Append current position and time values for recording
            positions.push_back(position);
            velocities.push_back(velocity);
            accelerations.push_back(acceleration);
            times.push_back(time);
        }
    }

    void saveResultsToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (file.is_open()) {
            force.printPreamble(file);

            if (print_time)
                file << "time ";
            if (print_position)
                file << "position ";
            if (print_velocity) 
                file << " velocity ";   
            if (print_acceleration) 
                file << " acceleration ";
            if (print_force)    
                force.printStatusHeader();
            
            "# time position velocity acceleration\n";

            // Set precision for output
            file << std::fixed << std::setprecision(8);

            for (size_t i = 0; i < times.size(); ++i) {
                if (print_time)
                    file << times[i] << " ";
                if (print_position)
                    file << positions[i] << " ";
                if (print_velocity) 
                    file << velocities[i] << " ";   
                if (print_acceleration) 
                    file << accelerations[i] << " ";
                if (print_force)    
                    force.printStatus();

                file << "\n";



                //file << times[i] << " " << positions[i] << " " <<  velocities[i] << " " << accelerations[i] << "\n";
            }
            file.close();
        } else {
            std::cerr << "Error: Unable to open file for writing." << std::endl;
        }
    }

    void plotResults() const {
        std::cout << "Plotting using gnuplot...\n";
        std::ofstream tempData("temp_data.dat");
        for (size_t i = 0; i < times.size(); ++i) {
            tempData << times[i] << " " << positions[i] << "\n";
        }
        tempData.close();
        std::string plotCommand = "plot 'temp_data.dat' with lines title 'Position vs. Time'";
        system(("gnuplot -e \"" + plotCommand + "\" -p").c_str());
    }
};

int main(int argc, char **argv) {

    int c;


    while ((c = getopt(argc,argv,"tpvaf")) != -1 ){
        switch(c) {
            case 't':
                print_time = true;
                break;
            case 'p':
                print_position = true;
                break;
            case 'v':
                print_velocity = true;
                break;
            case 'e':
                print_acceleration = true;
                break;
            case 'f':
                print_force = true;
                break;
        }
    }


    // Set simulation parameters
    std::cout << "Starting simulation...\n";
    double mass = 1.0;
    double originalPosition = 0.0;
    double originalVelocity = 0.0;
    double dt = 0.0001;  // Smaller time step for increased accuracy
    double simulationDuration = 10.0;


    std::vector<double> Kp_list = {200}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
    std::vector<double> Ki_list = {5}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
    std::vector<double> Kd_list = {0}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
    std::vector<double> target_list = {0}; //{-100, -50, -10, -5, -2, -1, -0.1, 0, 0.1, 1, 2, 5, 10, 50, 100};

    // Total number of iterations
    int total = Ki_list.size() * Kd_list.size() * Kp_list.size() * target_list.size();
    int count = 0;
    
    // Create GravityForce instance
    GravityForce gravityForce(-9.81);

    for (auto Ki : Ki_list) {
        for (auto Kd : Kd_list) {
            for (auto Kp : Kp_list) {
                for (auto target : target_list) {
                    // Create PIDForce instance
                    PIDForce pidForce(Kp, Ki, Kd, target);

                    // Create CombinedForce instance with both gravity and PID forces
                    // std::vector<Force*>& _forces_vector
                    std::vector<Force*> forces_vector = {&gravityForce, &pidForce};
                    ForceVectorClass force_vector(forces_vector);

                    // Create GravitySimulator instance with combined force
                    GravitySimulator combinedSimulator(mass, originalPosition, originalVelocity, dt, force_vector);

                    // Run simulation
                    combinedSimulator.simulate(simulationDuration);

                    // Save results to a file: use format "res_KI_KD_KP_target.txt" where you replace KI with actual Ki etc. also save to folder /results
                    std::string filename = "results/res_Kp_" + std::to_string(Kp) + "_Ki_" + std::to_string(Ki) + "_Kd_" + std::to_string(Kd) + "_target_" + std::to_string(target) + ".txt";
                    combinedSimulator.saveResultsToFile(filename);
                    count++;

                    // Print progress, check how many total and print percent of that for every 5 percent
                    std::cout << "\r" << "Progress: " << count << "/" << total << std::flush;

                }
            }
        }
    }
    std::cout << "\n" << "Simulation finished!\n";
    return 0;
}