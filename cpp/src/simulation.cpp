#include "simulation.h"

System::System(double _mass, double _originalPosition, double _originalVelocity, double _dt, ForceVector& _force_vector)
    : mass(_mass),
      originalPosition(_originalPosition),
      originalVelocity(_originalVelocity),
      position(_originalPosition),
      velocity(_originalVelocity),
      dt(_dt),
      force_vector(_force_vector) {}

void System::simulate(double simulationDuration) {
    diff_p_t_sq_cumulative = 0;
    for (double time = 0; time < simulationDuration; time += dt) {
        double forceValue = force_vector.calculate(position, dt);

        // Forces calculation using F = m * a
        acceleration = forceValue / mass;

        // Verlet integration method
        double positionNext = 2 * position - originalPosition + acceleration * dt * dt;
        velocity = (positionNext - originalPosition) / (2 * dt);

        // Update position and reset originalPosition for the next iteration
        originalPosition = position;
        position = positionNext;

        // 
        diff_p_t = velocity - position;
        diff_p_t_sq = std::pow(diff_p_t,2);
        diff_p_t_sq_cumulative += diff_p_t_sq;


        // Append current position and time values for recording
        positions.push_back(position);
        velocities.push_back(velocity);
        accelerations.push_back(acceleration);
        times.push_back(time);
        diff_p_t_vector.push_back(diff_p_t);
        diff_p_t_sq_cumulative_vector.push_back(diff_p_t_sq_cumulative);
    }
}

void System::saveResultsToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (file.is_open()) {
        force_vector.printPreamble(file);

        file << "# ";
        if (print_time)
            file << "time ";
        if (print_position)
            file << "position ";
        if (print_velocity) 
            file << " velocity ";   
        if (print_acceleration) 
            file << " acceleration ";
        file << " diff_t_p diff_t_p_sq";
        if (print_force)    
            force_vector.printStatusHeader();
        file << "\n";
        
        //"# time position velocity acceleration\n";

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
            file << diff_p_t_vector[i] << " " << diff_p_t_sq_cumulative_vector[i] << " ";
            if (print_force)    
                force_vector.printStatus();
            
            // int diff_t_p = velocities[i] - positions[i];
            // int diff_t_p_sq = std::pow(diff_t_p,2);

            file << "\n";
        }
        file.close();
    } else {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
    }
}

void System::plotResults() const {
    std::cout << "Plotting using gnuplot...\n";
    std::ofstream tempData("temp_data.dat");
    for (size_t i = 0; i < times.size(); ++i) {
        tempData << times[i] << " " << positions[i] << "\n";
    }
    tempData.close();
    std::string plotCommand = "plot 'temp_data.dat' with lines title 'Position vs. Time'";
    system(("gnuplot -e \"" + plotCommand + "\" -p").c_str());
}
