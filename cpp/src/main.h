#ifndef MAIN_H
#define MAIN_H

#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <vector>
#include <getopt.h>
#include <iostream>
#include <vector>


#include <mlpack/core.hpp>
#include <mlpack/methods/linear_regression/linear_regression.hpp>



//#include "forces.h"
#include "simulation.h"
#include "forces.h"

bool print_time = true;
bool print_position = true;
bool print_velocity = false;
bool print_acceleration = false;
bool print_force = false;

#endif // MAIN_H