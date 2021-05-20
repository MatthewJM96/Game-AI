#ifndef __aco_acs_graph_h
#define __aco_acs_graph_h

#pragma once

#include <array>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <random>

#include "dimension.hpp"
#include "map/maze2d.h"

namespace aco {
    namespace acs_graph {
        using namespace map::maze2d;

        std::default_random_engine generator;

        template <size_t MapSize>
        constexpr std::array<float, MapSize> initialise_pheromone_map();

        template <size_t MapSize>
        constexpr std::array<uint16_t, MapSize> initialise_ant_count_map();

        template <size_t MapSize>
        struct AntColony {
            GraphMap                      actual_map;
            std::array<uint16_t, MapSize> ant_count_map;
            std::array<float,    MapSize> pheromone_map;
        };

        template <size_t MapSize, size_t MaxSteps>
        struct Ant {
            AntColony<MapSize>* colony;

            bool found_food = false;
            bool has_food   = false;
            bool returned   = false;

            // TODO: This shouldn't really be a fixed amount, needs to be somehow
            //       set dynamically. If we want to keep it non-dynamic, then we
            //       would either have to have a pool that ants share and the ant
            //       count be made dynamic, or subdivide the space being searched.
            std::array<size_t, MaxSteps> previous_node_indices;
            size_t current_node_idx;
            size_t steps_taken = 0;
            size_t path_length = 0;
        };

        struct ACSOptions {
            size_t iterations;
            size_t ant_count;
            float  exploitation_factor;
            float  cost_exponent;
            struct {
                float increment;
                float evaporation;
            } local, global;
        };

        template <size_t MapSize, size_t MaxSteps>
        void prepare_ants(Ant<MapSize, MaxSteps>* ants, size_t ant_count, AntColony<MapSize>* ant_colony);

        template <size_t MapSize>
        void print_to_file(std::ofstream& file, AntColony<MapSize>* ant_colony, float (*value_for_idx)(size_t idx, AntColony<MapSize>* ant_colony));

        template <size_t MapDim, size_t MaxSteps>
        size_t choose_next_node(Ant<dimension::dim2d_to_padded_size(MapDim), MaxSteps>* ant, float exploitation_factor, float(*to_node_cost)(VertexDescriptor initial, VertexDescriptor final), float cost_exponent);

        template <size_t MapDim, size_t MaxSteps>
        void do_simulation(
            std::string tag,
            GraphMap    actual_map,
            ACSOptions  options
        );
    };
};

#include "acs_graph.inl"

#endif // __aco_acs_graph_h
