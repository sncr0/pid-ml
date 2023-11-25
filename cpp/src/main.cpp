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


    std::vector<double> Kp_list = {0, 0.1, 1, 2, 5, 10, 20, 50, 100, 200}; //{200}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
    std::vector<double> Ki_list = {0, 0.1, 1, 2, 5, 10, 50, 100}; // {5}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
    std::vector<double> Kd_list = {0, 0.1, 1, 2, 5, 10, 50, 100}; // {0}; //{0, 0.1, 1, 2, 5, 10, 50, 100};
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

    double Kp, Ki, Kd, target;
    double time, position, velocity, acceleration, diff_t_p, diff_t_p_sq;
    std::vector<std::vector<double>> input_vector;
    std::vector<std::vector<double>> output_vector;
    std::string folderPath = "/home/samuel/pid-ml/cpp/src/results"; 
    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {
            std::string filePath = entry.path().string();
            
            // Open the file
            std::ifstream file(filePath);
            if (!file.is_open()) {
                std::cerr << "Error opening file: " << filePath << std::endl;
                continue;
            }

            // Extract a line
            std::string line;
            if (std::getline(file, line)) {
                // ## PID controller = Kp: 0 Ki: 5 Kd: 0
                // results/res_Kp_0.000000_Ki_5.000000_Kd_0.000000_target_0.000000.csv
                printf("%s", filePath.c_str());
                sscanf(filePath.c_str(), "/home/samuel/pid-ml/cpp/src/results/res_Kp_%lf_Ki_%lf_Kd_%lf_target_%lf.csv", &Kp, &Ki, &Kd, &target);

                //sscanf(filePath.c_str(), "## PID controller = Kp: %lf Ki: %lf Kd: %lf", &Kp, &Ki, &Kd);
                //sscanf(line.c_str(), "## PID controller = Kp: %lf Ki: %lf Kd: %lf", &Kp, &Ki, &Kd);
                printf("Kp: %lf, Ki: %lf, Kd: %lf\n", Kp, Ki, Kd);
                //sscanf(line.c_str(), "# Kp: %lf, Ki: %lf, Kd: %lf, target: %lf", &Kp, &Ki, &Kd, &target);


            while(std::getline(file, line)) {
                if (line[0] != '#') {
                    // time position diff_t_p diff_t_p_sq
                    sscanf(line.c_str(), "%lf %lf %lf %lf", &time, &position, &diff_t_p, &diff_t_p_sq);
                    //std::cout << "Read values: time=" << time << ", position=" << position
                    //        << ", diff_t_p=" << diff_t_p << ", diff_t_p_sq=" << diff_t_p_sq << std::endl;
                }
            }
            std::vector<double> input = std::vector<double>{Kp, Ki, Kd, target};
            std::vector<double> output = std::vector<double>{diff_t_p_sq};

            input_vector.push_back(input);
            output_vector.push_back(output);
            



            } else {
                std::cerr << "Error reading line from file: " << filePath << std::endl;
            }
        }
    }

    // Save input and output vectors to files
    std::ofstream input_file("input.csv");
        if (input_file.is_open()) {

            input_file << std::fixed << std::setprecision(8);

            for (size_t i = 0; i < input_vector.size(); ++i) {
                for (size_t j = 0; j < input_vector[i].size(); ++j) {
                    input_file << input_vector[i][j];
                    if (j != (input_vector[i].size()-1)){
                        input_file << ", ";
                    }
                }
                input_file << "\n";
            }
        input_file.close();
    } else {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
    }

    std::ofstream output_file("output.csv");
        if (output_file.is_open()) {

            output_file << std::fixed << std::setprecision(8);

            for (size_t i = 0; i < output_vector.size(); ++i) {
                for (size_t j = 0; j < output_vector[i].size(); ++j) {
                    output_file << output_vector[i][j];
                    if (j != (output_vector[i].size()-1)){
                        output_file << ", ";
                    }
                }
                output_file << "\n";
            }
        output_file.close();
    } 
    else {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
    }



    arma::mat X; // Input matrix
    arma::mat y; // Output matrix

    // Load your data from CSV files.
    mlpack::data::Load("input.csv", X, true);
    mlpack::data::Load("output.csv", y, true);

    // Create the linear regression model.
    mlpack::regression::LinearRegression lr(X, y);

    // Train the model.
    lr.Train(X, y);

    // Make predictions.
    arma::rowvec predictions;
    lr.Predict(X, predictions);

    // Print the elements of predictions and x side by side
    for (size_t i = 0; i < predictions.n_elem; ++i) {
        std::cout << predictions[i] << " " << y[i] << std::endl;
    }
    // Print the coefficients.
    std::cout << "Coefficients: " << lr.Parameters() << std::endl;

    // Return the mean squared error. rsquared



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