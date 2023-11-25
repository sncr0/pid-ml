#include "main.h"
int main(int argc, char **argv) {

    int c;


    while ((c = getopt(argc,argv,"htpvafx")) != -1 ){
        switch(c) {
            case 'h':
                std::cout << "Usage: " << argv[0] << " [-htpvaf]\n";
                std::cout << "Options:\n";
                std::cout << "  -h: print this help message\n";
                std::cout << "  -t: print time\n";
                std::cout << "  -p: print position\n";
                std::cout << "  -v: print velocity\n";
                std::cout << "  -a: print acceleration\n";
                std::cout << "  -f: print force\n";
                return 0;
            case 'x':
                printf("No one is ever holy without suffering.\n");
                exit(0);
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
    double dt = 0.001;  // Smaller time step for increased accuracy
    double simulationDuration = 1.0;


    std::vector<double> Kp_list = {0, 10, 20, 50, 100, 200}; //{200}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
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
                    ForceVector force_vector(forces_vector);

                    System system(mass, originalPosition, originalVelocity, dt, force_vector);

                    // Run simulation
                    system.simulate(simulationDuration);

                    // Save results to a file: use format "res_KI_KD_KP_target.txt" where you replace KI with actual Ki etc. also save to folder /results
                    std::string filename = "results/res_Kp_" + std::to_string(Kp) + "_Ki_" + std::to_string(Ki) + "_Kd_" + std::to_string(Kd) + "_target_" + std::to_string(target) + ".csv";
                    system.saveResultsToFile(filename);
                    count++;

                    // Print progress, check how many total and print percent of that for every 5 percent
                    std::cout << "\r" << "Progress: " << count << "/" << total << std::flush;

                }
            }
        }
    }



    std::cout << "\n" << "Simulation finished!\n";


    // // Load the dataset.
    // arma::mat X;  // Feature matrix.
    // arma::rowvec y;  // Response variable.

    // // Load your data into X and y.
    // // For example:
    // // mlpack::data::Load("your_data.csv", X, true);
    // // mlpack::data::Load("your_labels.csv", y, true);

    // // Perform linear regression.
    // mlpack::LinearRegression linear_model(X, y);

    // // Get the parameters (coefficients) of the linear model.
    // arma::rowvec parameters = linear_model.Parameters();

    // // Print the parameters.
    // std::cout << "Parameters (Coefficients): " << parameters << std::endl;

    // // Make predictions.
    // arma::rowvec predictions;
    // linear_model.Predict(X, predictions);

    // // Print the predictions.
    // std::cout << "Predictions: " << predictions << std::endl;


    return 0;
}