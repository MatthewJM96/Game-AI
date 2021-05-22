#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "constants.h"

#include "map/maze2d.h"

#include "aco/acs_new.h"

int main() {
    std::cout << "Hello, world!" << std::endl;

    const size_t map_dim          =  31;
    const size_t max_steps        = 300;
    const size_t ant_count        =  50;
    const size_t iterations       = 200;
    const aco::acs_new::ACSOptions::OutputFreq output_frequency = {
        10, 1
    };

    const float global_pheromone_increment   = 1.0f; // Global increment (best ant in round or all rounds).
    const float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    const float pheromone_increment          = 1.0f / (float)map_dim; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    const float exploitation_factor = 0.9f;
    const float cost_exponent = 2.0f;

    for (size_t i = 0; i < 1; ++i) {
        std::string idx = std::to_string(i);

        // map::maze2d::Map<map_dim> map;
        map::maze2d::Map<map_dim + 2> halo_map;

        // map = map::maze2d::load_map<map_dim>("maps/15." + idx + ".unsolved.map");
        halo_map = map::maze2d::load_map_with_halo<map_dim>("maps/15." + idx + ".unsolved.map");

        // map::maze2d::print_map<map_dim>(map);
        map::maze2d::print_map<map_dim + 2>(halo_map);

        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph<>(halo_map, 1.0f);

        // auto graph_out = std::ofstream("graph_out.txt");
        // boost::write_graphviz(graph_out, graph_map.graph);

        aco::acs_new::ACSOptions options {
            idx,
            iterations,
            {
                map_dim + 2,
                map_dim + 2
            },
            max_steps,
            ant_count,
            exploitation_factor,
            cost_exponent,
            {
                pheromone_increment,
                pheromone_evaporation
            },
            {
                global_pheromone_increment,
                global_pheromone_evaporation
            },
            output_frequency
        };

        aco::acs_new::do_simulation(graph_map, options);
    }
}
