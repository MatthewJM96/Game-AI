#include <iostream>

const size_t MAP_DIM   = 8;
const size_t MAX_STEPS = 30;

#include "aco/naive.h"

const char MAP_1[MAP_DIM * MAP_DIM] = {
    '0', 'e', '0', '0', '0', '0', '0', '0',
    '0', '0', 'x', 'x', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', 'x', 'x', 'x', 'x', '0', '0',
    '0', 's', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0'
};

int main() {
    std::cout << "Hello, world!" << std::endl;

    const size_t ant_count = 30;

    const float pheromone_increment   = 30.0f; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation = 0.02f; // Global decrement on each node per timestep

    aco::naive::do_simulation<MAP_DIM, 30>(&MAP_1[0], ant_count, pheromone_increment, pheromone_evaporation);
}
