#ifndef __aco_acs_h
#define __aco_acs_h

#pragma once

#include <array>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <random>

#include <libheatmap/heatmap.h>

#include "dimension.hpp"
#include "map/maze2d.h"

namespace aco {
    namespace acs {
        using namespace map::maze2d;

        std::default_random_engine generator;
        template <size_t MapSize, size_t MaxSteps>
        struct Ant;

        template <size_t MapSize, size_t MaxSteps>
        struct AntColony {
            GraphMap                actual_map;
            Ant<MapSize, MaxSteps>* ants;
            size_t                  ant_count;
        };

        template <size_t MapSize, size_t MaxSteps>
        struct Ant {
            AntColony<MapSize, MaxSteps>* colony;

            bool found_food = false;
            bool has_food   = false;
            bool returned   = false;

            std::array<size_t, MaxSteps> previous_node_indices;
            std::array<bool,   MapSize>  visited_nodes; // Encodes no more info than stored in previous, but is faster for checking if node should be considered.
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
            size_t output_frequency;
        };

        template <size_t MapSize, size_t MaxSteps>
        void prepare_ants(Ant<MapSize, MaxSteps>* ants, size_t ant_count, AntColony<MapSize, MaxSteps>* ant_colony);

        template <size_t MapX, size_t MapY, size_t MaxSteps>
        void create_pheromone_heatmap_frame(std::string tag, AntColony<MapX * MapY, MaxSteps>& ant_colony);

        template <size_t MapX, size_t MapY, size_t MaxSteps>
        void create_ant_count_heatmap_frame(std::string tag, AntColony<MapX * MapY, MaxSteps>& ant_colony);

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

#include "acs.inl"

#endif // __aco_acs_h
