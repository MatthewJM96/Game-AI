#ifndef __aco_acs_mean_filtering_h
#define __aco_acs_mean_filtering_h

#pragma once

#include "dimension.hpp"
#include "map/maze2d.h"

namespace aco {
    namespace acs_mean_filtering {
        using namespace dimension;
        using namespace map::maze2d;

        struct Ant;

        struct ACSOptions {
            using EdgeCostFunc = float(*)(EdgeDescriptor edge);

            std::string     tag;
            size_t          iterations;
            Map2DDimensions map_dimensions;
            size_t          max_steps;
            size_t          ant_count;
            float           exploitation_factor;
            float           cost_exponent;
            struct {
                float increment;
                float evaporation;
            } local, global;
            bool do_output;
            struct OutputFreq {
                size_t coarse, fine;
            } output_frequency;
            size_t       target_best_path_length;
            EdgeCostFunc edge_cost_func;
            bool         prefer_to_get_closer_to_dest;
            size_t       mean_filtering_order; // The "neighbour of" order to go out to to calculate mean.
        };

        struct AntColony {
            GraphMap   map;
            Ant*       ants;
            ACSOptions options;
            struct ShortestPath {
                VertexDescriptor* steps;
                size_t            length;
            } shortest_path;
        };

        struct Ant {
            AntColony* colony;

            bool has_food   = false;
            bool returned   = false;

            VertexDescriptor* previous_vertices;
            bool* visited_vertices; // Encodes no more info than stored in previous, but is faster for checking if node should be considered.

            VertexDescriptor current_vertex;

            size_t steps_taken = 0;
            size_t path_length = 0;

            size_t back_step_counter = 0;

            size_t path_group = 0;
        };

        using Ants             = std::vector<Ant*>;
        using AntPathTracker   = std::unordered_map<size_t, Ants>;
        using PathGroupCursors = std::vector<size_t>;

        size_t do_simulation(GraphMap map, ACSOptions options);

        namespace impl {
            inline float rand(float min, float max);

            inline void initialise_pheromones(AntColony& ant_colony);

            inline void initialise_ants(AntColony& ant_colony);

            inline void reset_ants(AntColony& ant_colony);

            inline void destroy_ants(AntColony& ant_colony);

            inline void create_pheromone_heatmap_frame(std::string filename, AntColony& ant_colony);

            inline void create_ant_count_heatmap_frame(std::string filename, AntColony& ant_colony);

            inline void set_new_best_path(Ant& ant, AntColony& ant_colony);

            inline VertexDescriptor choose_next_vertex(size_t iteration, Ant& ant, AntColony& ant_colony);

            inline bool do_ant_next_step(
                size_t iteration,
                Ant& ant,
                AntColony& ant_colony,
                AntPathTracker& ant_path_tracker_old,
                AntPathTracker& ant_path_tracker_new,
                PathGroupCursors& path_group_cursors
            );

            inline void do_iteration(size_t iteration, AntColony& ant_colony);
        };
    };
};

#endif // __aco_acs_mean_filtering_h
