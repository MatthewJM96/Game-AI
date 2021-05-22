#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "constants.h"

#include "map/maze2d.h"

#include "aco/acs.h"

void do_iteration_count_test() {
    const size_t map_dim_15       =   31;
    const size_t map_dim_25       =   51;
    const size_t max_steps_15     =  200;
    const size_t max_steps_25     = 1000;
    const size_t ant_count        =  200;
    const size_t iterations       = 1000;
    const aco::acs::ACSOptions::OutputFreq output_frequency = {
        10, 1
    };

    const float global_pheromone_increment   = 1.0f; // Global increment (best ant in round or all rounds).
    const float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    const float pheromone_increment_15       = 1.0f / (float)map_dim_15; // Local increment (per ant per node) per timestep.
    const float pheromone_increment_25       = 1.0f / (float)map_dim_25; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    const float exploitation_factor = 0.9f;
    const float cost_exponent = 2.0f;

    const aco::acs::ACSOptions DEFAULT_OPTIONS {
        "",
        iterations,
        {
            0,
            0
        },
        0,
        ant_count,
        exploitation_factor,
        cost_exponent,
        {
            0,
            pheromone_evaporation
        },
        {
            global_pheromone_increment,
            global_pheromone_evaporation
        },
        false,
        output_frequency,
        0
    };

    for (size_t i = 0; i < 2; ++i) {
        std::string idx = std::to_string(i);

        map::maze2d::Map<map_dim_15 + 2> halo_map;

        halo_map = map::maze2d::load_map_with_halo<map_dim_15>("maps/15." + idx + ".solved.map");

        std::cout << "Map " << i + 1 << " of dim 15 maps, with ideal solution length " << halo_map.solution_length << ":" << std::endl;

        // map::maze2d::print_map<map_dim_15 + 2>(halo_map);

        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph<>(halo_map, 1.0f);

        aco::acs::ACSOptions options = DEFAULT_OPTIONS;
        options.tag = idx;
        options.map_dimensions = {
            map_dim_15 + 2, map_dim_15 + 2
        };
        options.max_steps = max_steps_15;
        options.target_best_path_length = graph_map.solution_length;
        options.local.increment = pheromone_increment_15;

        size_t iterations_to_ideal_solution = aco::acs::do_simulation(graph_map, options);

        std::cout << "...ideal solution took " << iterations_to_ideal_solution << " iterations to be obtained." << std::endl;
    }

    for (size_t i = 0; i < 20; ++i) {
        std::string idx = std::to_string(i);

        map::maze2d::Map<map_dim_25 + 2> halo_map;

        halo_map = map::maze2d::load_map_with_halo<map_dim_25>("maps/25." + idx + ".solved.map");

        std::cout << "Map " << i + 1 << " of dim 25 maps, with ideal solution length " << halo_map.solution_length << ":" << std::endl;

        // map::maze2d::print_map<map_dim_25 + 2>(halo_map);

        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph<>(halo_map, 1.0f);

        aco::acs::ACSOptions options = DEFAULT_OPTIONS;
        options.tag = idx;
        options.map_dimensions = {
            map_dim_25 + 2, map_dim_25 + 2
        };
        options.max_steps = max_steps_25;
        options.target_best_path_length = graph_map.solution_length;
        options.local.increment = pheromone_increment_25;

        size_t iterations_to_ideal_solution = aco::acs::do_simulation(graph_map, options);

        std::cout << "...ideal solution took " << iterations_to_ideal_solution << " iterations to be obtained." << std::endl;
    }
}

void do_map_25_test(size_t map_idx) {
    const size_t map_dim       =  51;
    const size_t max_steps     = 300;
    const size_t ant_count     =  50;
    const size_t iterations    =  20;
    const aco::acs::ACSOptions::OutputFreq output_frequency = {
        2, 1
    };

    const float global_pheromone_increment   = 1.0f; // Global increment (best ant in round or all rounds).
    const float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    const float pheromone_increment          = 1.0f / (float)map_dim; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    const float exploitation_factor = 0.9f;
    const float cost_exponent = 2.0f;

    std::string idx_str = std::to_string(map_idx);

    map::maze2d::Map<map_dim + 2> halo_map;

    halo_map = map::maze2d::load_map_with_halo<map_dim>("maps/25." + idx_str + ".solved.map");

    std::cout << "Map " << map_idx + 1 << " of dim 25 maps, with ideal solution length " << halo_map.solution_length << ":\n" << std::endl;

    map::maze2d::print_map<map_dim + 2>(halo_map);

    map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph<>(halo_map, 1.0f);

    const aco::acs::ACSOptions options {
        idx_str,
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
        true,
        output_frequency,
        0
    };

    size_t iterations_to_ideal_solution = aco::acs::do_simulation(graph_map, options);
}

int main() {
    std::cout << "Hello, world!" << std::endl;

    do_map_25_test(0);

    // do_iteration_count_test();
}
