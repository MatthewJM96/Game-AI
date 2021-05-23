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
    const size_t ant_count        =   10;
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
        0,
        nullptr,
        true
    };

    for (size_t i = 0; i < 2; ++i) {
        std::string idx = std::to_string(i);

        map::maze2d::Map halo_map;

        halo_map = map::maze2d::load_map_with_halo("maps/15." + idx + ".solved.map", {map_dim_15, map_dim_15});

        std::cout << "Map " << i + 1 << " of dim 15 maps, with ideal solution length " << halo_map.solution_length << ":" << std::endl;

        // map::maze2d::print_map(halo_map);

        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph(halo_map, 1.0f);

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

        map::maze2d::Map halo_map;

        halo_map = map::maze2d::load_map_with_halo("maps/25." + idx + ".solved.map", {map_dim_25, map_dim_25});

        std::cout << "Map " << i + 1 << " of dim 25 maps, with ideal solution length " << halo_map.solution_length << ":" << std::endl;

        // map::maze2d::print_map<map_dim_25 + 2>(halo_map);

        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph(halo_map, 1.0f);

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

void do_map_test(
    size_t map_dim,
    size_t map_idx,
    bool quit_on_ideal_path = false,
    bool do_output = true,
    size_t iterations = 1000,
    aco::acs::ACSOptions::OutputFreq output_frequency = {
        50, 1
    }
) {
    size_t ant_count     =   10;

    float global_pheromone_increment   = 1.0f; // Global increment (best ant in round or all rounds).
    float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    float pheromone_increment          = 1.0f / (float)map_dim; // Local increment (per ant per node) per timestep.
    float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    float exploitation_factor = 0.9f;
    float cost_exponent = 2.0f;

    std::string idx_str = std::to_string(map_idx);

    map::maze2d::Map halo_map;

    size_t half_dim = ((map_dim - 1) / 2);
    halo_map = map::maze2d::load_map_with_halo("maps/" + std::to_string(half_dim) + "." + idx_str + ".solved.map", {map_dim, map_dim});

    std::cout << "Map " << map_idx + 1 << " of dim " << half_dim << " maps, with ideal solution length " << halo_map.solution_length << ":\n" << std::endl;

    map::maze2d::print_map(halo_map);

    map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph(halo_map, 1.0f);

    size_t num_vertices = boost::num_vertices(graph_map.graph);
    std::cout << "Num Vertices: " << num_vertices << std::endl;

    const aco::acs::ACSOptions options {
        idx_str,
        iterations,
        halo_map.dims,
        num_vertices,
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
        do_output,
        output_frequency,
        quit_on_ideal_path ? graph_map.solution_length : 0,
        nullptr,
        true
    };

    size_t iterations_to_ideal_solution = aco::acs::do_simulation(graph_map, options);

    if (quit_on_ideal_path)
        std::cout << "Achieved ideal solution after " << iterations_to_ideal_solution << " iterations." << std::endl;
}

int main() {
    std::cout << "Hello, world!" << std::endl;

    // do_map_test(31, 1);

    // do_map_test(51, 17);

    do_map_test(51, 17, true, false);

    do_map_test(101, 0, true, false);

    // do_iteration_count_test();
}
