#include <iostream>
#include <fstream>
#include <string>

#include "aco/naive.h"
#include "aco/halo.h"

int main() {
    std::cout << "Hello, world!" << std::endl;

    const size_t map_dim   = 51;
    const size_t max_steps = 400;
    const size_t ant_count = 5000;

    const float pheromone_increment   = map_dim * map_dim; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation = 0.01f; // Global decrement on each node per timestep

    for (size_t i = 0; i < 20; ++i) {
        std::string idx = std::to_string(i);

        std::array<char, map_dim * map_dim> map;
        std::array<char, (map_dim + 2) * (map_dim + 2)> halo_map;

        {
            std::ifstream map_file("maps/25." + idx + ".unsolved.map");

            size_t row = 0;
            std::string line;
            while (std::getline(map_file, line)) {
                halo_map[row * map_dim]           = WALL_TILE;
                halo_map[(row + 1) * map_dim - 1] = WALL_TILE;

                for (size_t col = 0; col < map_dim; ++col) {
                    size_t idx = row * map_dim + col;

                         map[idx]     = line[col];
                    halo_map[idx + 1] = line[col];
                }

                ++row;
            }
        }

        // for (size_t i = 0; i < map_dim; ++i) {
        //     for (size_t j = 0; j < map_dim; ++j) {
        //         size_t idx = i * map_dim + j;

        //         std::cout << map[idx] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        aco::naive::do_simulation<map_dim, max_steps>(idx, 2000, &map[0], ant_count, pheromone_increment, pheromone_evaporation);
         aco::halo::do_simulation<map_dim, max_steps>(idx, 2000, &halo_map[0], ant_count, pheromone_increment, pheromone_evaporation);
    }
}
