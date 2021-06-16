#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include <iterator>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "constants.h"

#include "map/maze2d.h"

#include "aco/acs.h"
#include "aco/acs_dynamic_exploitation.h"
#include "aco/acs_mean_filtering.h"

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
        "",
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
        "pngs",
        output_frequency,
        quit_on_ideal_path ? graph_map.solution_length : 0,
        nullptr,
        true
    };

    size_t iterations_to_ideal_solution = aco::acs::do_simulation(graph_map, options);

    if (quit_on_ideal_path)
        std::cout << "Achieved ideal solution after " << iterations_to_ideal_solution << " iterations." << std::endl;
}

struct TestResults {
    struct ForMethod {
        std::vector<size_t> values;
        size_t total, avg, stdev, max, min;
    } acs, acs_dynamic_exploitation, acs_mean_filtering;
};
struct IntermediateTestResults {
    size_t acs, acs_dynamic_exploitation, acs_mean_filtering;
};
IntermediateTestResults do_acs_comparisons(
    size_t map_dim,
    size_t map_idx,
    std::string output_dir,
    bool quit_on_ideal_path   = false,
    bool do_output            = true,
    size_t iterations         = 1000,
    size_t coarse_output_freq = 2,
    size_t fine_output_freq   = 1
) {
    size_t ant_count     =   10;

    float global_pheromone_increment   = 1.0f; // Global increment (best ant in round or all rounds).
    float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    float pheromone_increment          = 1.0f / (float)map_dim; // Local increment (per ant per node) per timestep.
    float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    float exploitation_factor = 0.9f;
    float cost_exponent = 2.0f;

    float exploitation_exponent = 2.0f;

    size_t mean_filtering_order   = 1;
    float  mean_filtering_trigger = 0.5f;


    std::string idx_str = std::to_string(map_idx);

    map::maze2d::Map halo_map;

    std::string half_dim = std::to_string((map_dim - 1) / 2);
    halo_map = map::maze2d::load_map_with_halo("maps/" + half_dim + "." + idx_str + ".solved.map", {map_dim, map_dim});

    std::filesystem::create_directories(output_dir + "/acs/" + half_dim + "." + idx_str);
    std::filesystem::create_directories(output_dir + "/acs_de/" + half_dim + "." + idx_str);
    std::filesystem::create_directories(output_dir + "/acs_mf/" + half_dim + "." + idx_str);

    // std::cout << "Map " << map_idx + 1 << " of dim " << half_dim << " maps, with ideal solution length " << halo_map.solution_length << ":\n" << std::endl;

    // map::maze2d::print_map(halo_map);

    // Need a copy for each method - they change the state inside and lets be sure there's no accidental overlap.
    map::maze2d::GraphMap graph_map_1 = map::maze2d::map_to_graph(halo_map, 1.0f);
    map::maze2d::GraphMap graph_map_2 = map::maze2d::map_to_graph(halo_map, 1.0f);
    map::maze2d::GraphMap graph_map_3 = map::maze2d::map_to_graph(halo_map, 1.0f);

    size_t num_vertices = boost::num_vertices(graph_map_1.graph);
    // std::cout << "Num Vertices: " << num_vertices << std::endl;

    const aco::acs::ACSOptions options_1 {
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
        output_dir + "/acs/" + half_dim + "." + idx_str,
        {
            coarse_output_freq,
            fine_output_freq
        },
        quit_on_ideal_path ? graph_map_1.solution_length : 0,
        nullptr,
        true
    };
    const aco::acs_dynamic_exploitation::ACSOptions options_2 {
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
        output_dir + "/acs_de/" + half_dim + "." + idx_str,
        {
            coarse_output_freq,
            fine_output_freq
        },
        quit_on_ideal_path ? graph_map_2.solution_length : 0,
        nullptr,
        true,
        exploitation_exponent
    };
    const aco::acs_mean_filtering::ACSOptions options_3 {
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
        output_dir + "/acs_mf/" + half_dim + "." + idx_str,
        {
            coarse_output_freq,
            fine_output_freq
        },
        quit_on_ideal_path ? graph_map_3.solution_length : 0,
        nullptr,
        true,
        mean_filtering_order,
        mean_filtering_trigger
    };

    size_t iterations_to_ideal_solution_1 = aco::acs::do_simulation(graph_map_1, options_1);
    size_t iterations_to_ideal_solution_2 = aco::acs_dynamic_exploitation::do_simulation(graph_map_2, options_2);
    size_t iterations_to_ideal_solution_3 = aco::acs_mean_filtering::do_simulation(graph_map_3, options_3);

    return {
        iterations_to_ideal_solution_1,
        iterations_to_ideal_solution_2,
        iterations_to_ideal_solution_3
    };
        // std::cout << "ACS achieved ideal solution after " << iterations_to_ideal_solution_1 << " iterations." << std::endl;
        // std::cout << "ACS with dynamic exploitation achieved ideal solution after " << iterations_to_ideal_solution_2 << " iterations." << std::endl;
        // std::cout << "ACS with mean filtering achieved ideal solution after " << iterations_to_ideal_solution_3 << " iterations." << std::endl;
}

int main() {
    std::cout << "Hello, world!" << std::endl;

    // do_map_test(31, 1);

    // do_map_test(51, 17);

    // do_map_test(51, 17, true, false);

    const std::string output_dir = "/media/data/matthewm/Workspace_Data/Game-AI";

    std::ofstream overall_results = std::ofstream(output_dir + "/results.csv");

    auto run_test = [output_dir, &overall_results](size_t map_dim, size_t map_idx, size_t iterations = 100) {
        TestResults results = {
            {
                std::vector<size_t>(iterations, 0),
                0, 0, 0,
                std::numeric_limits<size_t>::min(),
                std::numeric_limits<size_t>::max()
            },
            {
                std::vector<size_t>(iterations, 0),
                0, 0, 0,
                std::numeric_limits<size_t>::min(),
                std::numeric_limits<size_t>::max()
            },
            {
                std::vector<size_t>(iterations, 0),
                0, 0, 0,
                std::numeric_limits<size_t>::min(),
                std::numeric_limits<size_t>::max()
            }
        };
        for (size_t i = 0; i < iterations; ++i) {
            // Only output one of the iterations as a representative case.
            // IntermediateTestResults it_result = do_acs_comparisons(map_dim,  map_idx, output_dir, true, i == 0);
            IntermediateTestResults it_result = do_acs_comparisons(map_dim,  map_idx, output_dir, true, false);

            results.acs.values[i]                       = it_result.acs;
            results.acs.total                          += it_result.acs;
            results.acs_dynamic_exploitation.values[i]  = it_result.acs_dynamic_exploitation;
            results.acs_dynamic_exploitation.total     += it_result.acs_dynamic_exploitation;
            results.acs_mean_filtering.values[i]        = it_result.acs_mean_filtering;
            results.acs_mean_filtering.total           += it_result.acs_mean_filtering;

            if (results.acs.max < it_result.acs) {
                results.acs.max = it_result.acs;
            }
            if (results.acs_dynamic_exploitation.max < it_result.acs_dynamic_exploitation) {
                results.acs_dynamic_exploitation.max = it_result.acs_dynamic_exploitation;
            }
            if (results.acs_mean_filtering.max < it_result.acs_mean_filtering) {
                results.acs_mean_filtering.max = it_result.acs_mean_filtering;
            }

            if (results.acs.min > it_result.acs) {
                results.acs.min = it_result.acs;
            }
            if (results.acs_dynamic_exploitation.min > it_result.acs_dynamic_exploitation) {
                results.acs_dynamic_exploitation.min = it_result.acs_dynamic_exploitation;
            }
            if (results.acs_mean_filtering.min > it_result.acs_mean_filtering) {
                results.acs_mean_filtering.min = it_result.acs_mean_filtering;
            }
        }

        results.acs.avg                      = results.acs.total                      / iterations;
        results.acs_dynamic_exploitation.avg = results.acs_dynamic_exploitation.total / iterations;
        results.acs_mean_filtering.avg       = results.acs_mean_filtering.total       / iterations;

        const auto calc_stdev = [](TestResults::ForMethod& results) {
            size_t avg = results.avg;

            std::vector<size_t> diff(results.values.size());
            std::transform(
                results.values.begin(),
                results.values.end(),
                diff.begin(),
                [avg](size_t val) { return val - avg; }
            );
            size_t sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0);

            results.stdev = std::sqrt(sq_sum / results.values.size());
        };

        calc_stdev(results.acs);
        calc_stdev(results.acs_dynamic_exploitation);
        calc_stdev(results.acs_mean_filtering);

        const auto output_results = [&overall_results](const TestResults::ForMethod& results, std::string method) {
            std::cout << method << " achieved an ideal solution with:\n"
                      << "    an average of:      " << results.avg   << " iterations;\n"
                      << "    a std deviation of: " << results.stdev << " iterations;\n"
                      << "    a max of:           " << results.max   << " iterations; and,\n"
                      << "    a min of:           " << results.min   << " iterations." << std::endl;

            overall_results << results.avg << ", " << results.stdev << ", " << results.max << ", " << results.min;
            for (size_t value : results.values) {
                overall_results << ", " << value;
            }
            overall_results << "\n";
        };

        output_results(results.acs, "ACS");
        output_results(results.acs_dynamic_exploitation, "ACS with dynamic exploitation");
        output_results(results.acs_mean_filtering, "ACS with mean filtering");
    };

    run_test(51,  0);
    run_test(51,  1);
    run_test(51,  2);
    run_test(51,  3);
    run_test(51,  4);
    run_test(51,  5);
    run_test(51,  6);
    run_test(51,  7);
    run_test(51,  8);
    run_test(51,  9);
    run_test(51, 10);
    run_test(51, 11);
    run_test(51, 12);
    run_test(51, 13);
    run_test(51, 14);
    run_test(51, 15);
    run_test(51, 16);
    run_test(51, 17);
    run_test(51, 18);
    run_test(51, 19);
    // run_test(101, 0);

    // do_map_test(101, 0, true, false);

    // do_iteration_count_test();
}
