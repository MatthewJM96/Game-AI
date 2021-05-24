#include "aco/acs.h"

#include <iostream>
#include <limits>
#include <random>

#include <libheatmap/heatmap.h>

#include "image.h"

static float SQUARE_STAMP_DATA[] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static heatmap_stamp_t SQUARE_STAMP = {
    SQUARE_STAMP_DATA, 9, 9
};

float aco::acs::impl::rand(float min, float max) {
    static std::default_random_engine            generator;
    static std::uniform_real_distribution<float> distrib(0.0f, 1.0f);

    return min + max * distrib(generator);
}

void aco::acs::impl::initialise_pheromones(AntColony& ant_colony) {
    for (auto edge : boost::make_iterator_range(boost::edges(ant_colony.map.graph)))
        ant_colony.map.edge_weight_map[edge] = ant_colony.options.local.increment;
}

void aco::acs::impl::initialise_ants(AntColony& ant_colony) {
    size_t ant_count = ant_colony.options.ant_count;
    size_t max_steps = ant_colony.options.max_steps;
    size_t vertex_count = boost::num_vertices(ant_colony.map.graph);

    // At least keep some memory well-packed.
    VertexDescriptor* previous_vertices = new VertexDescriptor[ant_count * max_steps];
    bool* visited_vertices = new bool[ant_count * vertex_count];

    for (size_t ant_idx = 0; ant_idx < ant_count; ++ant_idx) {
        Ant& ant = ant_colony.ants[ant_idx];

        ant.previous_vertices = &previous_vertices[ant_idx * max_steps];
        ant.visited_vertices = &visited_vertices[ant_idx * vertex_count];

        ant.current_vertex = ant_colony.map.start_vertex;
    }
}

void aco::acs::impl::reset_ants(AntColony& ant_colony) {
    size_t ant_count = ant_colony.options.ant_count;
    size_t vertex_count = boost::num_vertices(ant_colony.map.graph);

    // Quicker to just do whole array at once.
    std::fill_n(ant_colony.ants[0].visited_vertices, ant_count * vertex_count, false);

    for (size_t ant_idx = 0; ant_idx < ant_colony.options.ant_count; ++ant_idx) {
        Ant& ant = ant_colony.ants[ant_idx];

        ant.has_food = false;
        ant.returned = false;

        ant.current_vertex = ant_colony.map.start_vertex;
        ant.visited_vertices[ant_colony.map.start_vertex] = true;

        ant.steps_taken = 0;
        ant.path_length = 0;

        ant.back_step_counter = 0;
    }
}

void aco::acs::impl::destroy_ants(AntColony& ant_colony) {
    delete[] ant_colony.ants[0].previous_vertices;
    delete[] ant_colony.ants[0].visited_vertices;
}

void aco::acs::impl::create_pheromone_heatmap_frame(std::string filename, AntColony& ant_colony) {
    size_t dim_x     = ant_colony.options.map_dimensions.x;
    size_t dim_y     = ant_colony.options.map_dimensions.y;
    size_t ant_count = ant_colony.options.ant_count;

    // * 10 per dimension to give something less tiny as heatmap.
    heatmap_t* heatmap = heatmap_new(dim_x * 10, dim_y * 10);

    map::maze2d::GraphMap& map = ant_colony.map;

    float saturation_point = 0.0f;
    for (auto vertex: boost::make_iterator_range(boost::vertices(map.graph))) {
        float net_pheromone_into_vertex = 0.0f;

        for (auto edge: boost::make_iterator_range(boost::in_edges(vertex, map.graph)))
            net_pheromone_into_vertex += map.edge_weight_map[edge];

        // If we have a new record pheromone "in" value, then update the saturation point.
        if (net_pheromone_into_vertex > saturation_point) saturation_point = net_pheromone_into_vertex;

        size_t idx = map.vertex_to_map_idx_map[vertex];

        size_t x_coord = 10 *      (idx % dim_x);
        size_t y_coord = 10 * floor(idx / dim_x);

        heatmap_add_weighted_point(heatmap, x_coord, y_coord, net_pheromone_into_vertex);
    }

    for (size_t idx = 0; idx < (dim_x * dim_y); ++idx) {
        if (ant_colony.map.map_idx_to_vertex_map.find(idx) == ant_colony.map.map_idx_to_vertex_map.end()) {
            size_t x_coord = 10 *      (idx % dim_x);
            size_t y_coord = 10 * floor(idx / dim_x);

            heatmap_add_weighted_point_with_stamp(heatmap, x_coord, y_coord, saturation_point, &SQUARE_STAMP);
        }
    }

    uint8_t* image_data = new uint8_t[dim_x * 10 * dim_y * 10 * 4];
    heatmap_render_saturated_to(heatmap, heatmap_cs_default, saturation_point, image_data);

    heatmap_free(heatmap);

    image::writepng("results/" + filename + ".png", image_data, dim_x * 10, dim_y * 10);
}

void aco::acs::impl::create_ant_count_heatmap_frame(std::string filename, AntColony& ant_colony) {
    size_t dim_x     = ant_colony.options.map_dimensions.x;
    size_t dim_y     = ant_colony.options.map_dimensions.y;
    size_t ant_count = ant_colony.options.ant_count;

    // * 10 per dimension to give something less tiny as heatmap.
    heatmap_t* heatmap = heatmap_new(dim_x * 10, dim_y * 10);

    map::maze2d::GraphMap& map = ant_colony.map;

    for (size_t idx = 0; idx < (dim_x * dim_y); ++idx) {
        if (ant_colony.map.map_idx_to_vertex_map.find(idx) == ant_colony.map.map_idx_to_vertex_map.end()) {
            size_t x_coord = 10 *      (idx % dim_x);
            size_t y_coord = 10 * floor(idx / dim_x);

            heatmap_add_weighted_point_with_stamp(heatmap, x_coord, y_coord, (float)ant_count, &SQUARE_STAMP);
        }
    }

    for (size_t i = 0; i < ant_count; ++i) {
        Ant& ant = ant_colony.ants[i];

        // If this ant has returned without food, don't show it on the heatmap.
        if (ant.returned && !ant.has_food) continue;

        size_t idx = ant_colony.map.vertex_to_map_idx_map[ant.current_vertex];

        size_t x_coord = 10 *      (idx % dim_x);
        size_t y_coord = 10 * floor(idx / dim_x);

        heatmap_add_point(heatmap, x_coord, y_coord);
    }

    uint8_t* image_data = new uint8_t[dim_x * 10 * dim_y * 10 * 4];
    heatmap_render_saturated_to(heatmap, heatmap_cs_default, (float)ant_count, image_data);

    heatmap_free(heatmap);

    image::writepng("results/" + filename + ".png", image_data, dim_x * 10, dim_y * 10);
}

void aco::acs::impl::set_new_best_path(Ant& ant, AntColony& ant_colony) {
    std::cout << "Found new shortest path of length " << std::to_string(ant.path_length) << "." << std::endl;

    AntColony::ShortestPath& best_path = ant_colony.shortest_path;

    /**
     * For each edge in old best path, set it as no longer in graph's best path.
     *
     * Note, we have to check if there actually is a previous best before trying
     * to remove it.
     */
    if (best_path.length < ant_colony.options.max_steps) {
        for (size_t path_idx = 1; path_idx < best_path.length; ++path_idx) {
            /**
             * Get edge in old best path.
             */
            VertexDescriptor source_vertex = best_path.steps[path_idx - 1];
            VertexDescriptor target_vertex = best_path.steps[path_idx];
            EdgeDescriptor edge; bool edge_exists;
            std::tie(edge, edge_exists) = boost::edge(source_vertex, target_vertex, ant_colony.map.graph);

            ant_colony.map.edge_in_path_map[edge] = false;
        }
    }

    /**
     * Copy previous vertices, and then add final one on the end.
     */
    std::memcpy(best_path.steps, ant.previous_vertices, sizeof(VertexDescriptor) * ant.path_length);
    best_path.steps[ant.path_length] = ant.current_vertex;

    /**
     * For each edge in new best path, set it as in graph's best path.
     */
    for (size_t path_idx = 1; path_idx < ant.path_length; ++path_idx) {
        /**
         * Get edge in new best path.
         */
        VertexDescriptor source_vertex = best_path.steps[path_idx - 1];
        VertexDescriptor target_vertex = best_path.steps[path_idx];
        EdgeDescriptor edge; bool edge_exists;
        std::tie(edge, edge_exists) = boost::edge(source_vertex, target_vertex, ant_colony.map.graph);

        ant_colony.map.edge_in_path_map[edge] = true;
    }

    ant_colony.shortest_path.length = ant.path_length;
}

map::maze2d::VertexDescriptor aco::acs::impl::choose_next_vertex(size_t iteration, Ant& ant, AntColony& ant_colony) {
    struct {
        VertexDescriptor vertex = 0;
        float score = std::numeric_limits<float>::lowest();
    } best_option;

    size_t num_candidates = 0;
    float total_score = 0.0f;
    float cumulative_scores[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    VertexDescriptor vertices[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    for (auto edge : boost::make_iterator_range(boost::out_edges(ant.current_vertex, ant_colony.map.graph))) {
        VertexDescriptor candidate_vertex = boost::target(edge, ant_colony.map.graph);

        /**
         * If ant has already visited the candidate vertex, then reject it as a candidate.
         */
        if (ant.visited_vertices[candidate_vertex]) continue;

        // TODO(Matthew): No edge "length" right now. We will have to introduce this eventually.
        //                When we do, it also will need to be applied to pheromone update calcs
        //                and "best path" length.
        float score = ant_colony.map.edge_weight_map[edge];

        if (ant_colony.options.prefer_to_get_closer_to_dest) {
            size_t dim_x     = ant_colony.options.map_dimensions.x;
            size_t dim_y     = ant_colony.options.map_dimensions.y;

            size_t current_tile_idx   = ant_colony.map.vertex_to_map_idx_map[ant.current_vertex];
            size_t candidate_tile_idx = ant_colony.map.vertex_to_map_idx_map[candidate_vertex];
            size_t dest_tile_idx      = ant_colony.map.vertex_to_map_idx_map[ant_colony.map.finish_vertex];

            float current_x_coord   = (float)      (current_tile_idx   % dim_x);
            float current_y_coord   = (float) floor(current_tile_idx   / dim_x);
            float candidate_x_coord = (float)      (candidate_tile_idx % dim_x);
            float candidate_y_coord = (float) floor(candidate_tile_idx / dim_x);
            float dest_x_coord      = (float)      (dest_tile_idx      % dim_x);
            float dest_y_coord      = (float) floor(dest_tile_idx      / dim_x);

            float dist_ratio = sqrt(
                                    (pow((candidate_x_coord - dest_x_coord), 2.0f) + pow((candidate_y_coord - dest_y_coord), 2.0f))
                                    / (pow((current_x_coord - dest_x_coord), 2.0f) + pow((current_y_coord - dest_y_coord), 2.0f))
                               );

            score /= dist_ratio;
        }

        /**
         * If this node has the best score so far, set it as best option.
         */
        if (score > best_option.score) {
            best_option.vertex = candidate_vertex;
            best_option.score = score;
        }

        /**
         * Increment total pheromone level of all candidates, add new cumulative
         * and add vertex to list of candidate vertices.
         */
        total_score += score;
        cumulative_scores[num_candidates] = total_score;
        vertices[num_candidates] = candidate_vertex;

        /**
         * We have found a new candidate, increment count.
         */
        num_candidates += 1;
    }

    /**
     * If no candidates are found, then just send the ant back to where it came from.
     */
    if (num_candidates == 0) {
        // If backstep counter is equal to steps taken, the ant can't
        // possibly step any further back.
        if (ant.back_step_counter == ant.steps_taken) {
            return ant.current_vertex;
        }
        // Increment backstep counter and do the backstep.
        //      Increment must be by 2 as steps_taken will be incremented too.
        //      Backstep is +1 on counter.
        size_t step_back = ant.back_step_counter + 1;
        ant.back_step_counter += 2;
        return ant.previous_vertices[ant.steps_taken - step_back];
    }

    // TODO(Matthew): On the first iteration, we do no exploitation. We may want
    //                to go even further and consider an adaptive exploitation_factor
    //                a la Bayesian Optimisation.
    /**
     * Decide if we should "exploit" (i.e., take best possible path according to scores of edges).
     *
     * Note that on first iteration we do no exploitation.
     */
    float exploitation_val = rand(0.0f, 1.0f);
    if (exploitation_val < (iteration > 0 ? ant_colony.options.exploitation_factor : 0.0f)) {
        // Reset backstep counter and return best choice.
        ant.back_step_counter = 0;
        return best_option.vertex;
    }

    // If we get here, then this ant is exploring.

    /**
     * Choose which of the candidates to send ant to with probability in
     * proportion to the score of each of the candidates. If we somehow
     * (BUG!) can't make a choice, send ant back to where it came from.
     */
    float choice_val = rand(0.0f, total_score);
    for (size_t choice_idx = 0; choice_idx < num_candidates; ++choice_idx)
        if (choice_val <= cumulative_scores[choice_idx]) {
            // Reset backstep counter and return choice.
            ant.back_step_counter = 0;
            return vertices[choice_idx];
        }

    std::cout << "Error: could not decide where to send ant, check maths of choose_next_vertex!" << std::endl;

    // If backstep counter is equal to steps taken, the ant can't
    // possibly step any further back.
    if (ant.back_step_counter == ant.steps_taken) {
        return ant.current_vertex;
    }
    // Increment backstep counter and do the backstep.
    //      Increment must be by 2 as steps_taken will be incremented too.
    //      Backstep is +1 on counter.
    size_t step_back = ant.back_step_counter + 1;
    ant.back_step_counter += 2;
    return ant.previous_vertices[ant.steps_taken - step_back];
}

bool aco::acs::impl::do_ant_next_step(size_t iteration, Ant& ant, AntColony& ant_colony) {
    /**
     * Nothing to do if ant has done its tour.
     */
    if (ant.returned) return false;

    /**
     * Check if ant has taken its last allowed searching step.
     *
     * If it has, then mark it returned and let outer loop know
     * that is has returned.
     */
    if (ant.steps_taken >= ant_colony.options.max_steps) {
        ant.returned = true;

        return true;
    }

    // TODO(Matthew): The local application of pheromone ONLY on return journey is
    //                not as per the ACS paper, but as we are interested only in shortest
    //                path from an initial vertex to a final, it feels like it ought
    //                to behave well.
    //                   The difference isn't too drastic though, as in principle an ant
    //                   would be applying the same pheromone to the same edges. In this
    //                   case it is only communicating to "exploring" ants in this round
    //                   and to ants in the next round - with max steps this works to avoid
    //                   ants that never make it muddying the waters.
    /**
     * Different handling if ant is searching for food
     * or if it has found food and is headed home.
     */
    if (ant.has_food) {
        /**
         * Next vertex is step one before most recent.
         */
        ant.steps_taken -= 1;
        VertexDescriptor next_vertex = ant.previous_vertices[ant.steps_taken];

        /**
         * Get edge going from where ant is about to be, to where it is going.
         */
        EdgeDescriptor edge; bool edge_exists;
        // Must use these in reverse order to get the correct edge - ant is headed backwards.
        std::tie(edge, edge_exists) = boost::edge(next_vertex, ant.current_vertex, ant_colony.map.graph);

        /**
         * Apply local pheromone updating rule to edge.
         */
        ant_colony.map.edge_weight_map[edge] =
              (1.0f - ant_colony.options.local.evaporation) * ant_colony.map.edge_weight_map[edge]
            +     ant_colony.options.local.evaporation      * ant_colony.options.local.increment;

        /**
         * Tell ant where it is now.
         */
        ant.current_vertex = next_vertex;

        /**
         * If ant has got home, mark it as returned.
         */
        if (next_vertex == ant_colony.map.start_vertex) {
            ant.returned = true;
        }
    } else {
        /**
         * Search for next vertex.
         */
        VertexDescriptor next_vertex = choose_next_vertex(iteration, ant, ant_colony);

        /**
         * Update ant with next vertex.
         */
        ant.previous_vertices[ant.steps_taken] = ant.current_vertex;
        ant.visited_vertices[next_vertex]      = true;
        ant.steps_taken                       += 1;
        ant.current_vertex = next_vertex;

        /**
         * If this next vertex is the food source, then
         * update ant to know its found that food and see
         * if it has found the best path so far.
         */
        if (next_vertex == ant_colony.map.finish_vertex) {
            /**
             * Let ant know it has found food.
             */
            ant.has_food = true;
            ant.path_length = ant.steps_taken;

            /**
             * If this ant has found the best path, update
             * the ant colony's knowledge of the best path found.
             */
            if (ant.path_length < ant_colony.shortest_path.length)
                set_new_best_path(ant, ant_colony);
        }
    }

    /**
     * Given prior early exits, the returned flag
     * of this ant indicates whether it has returned
     * home in this last step or not.
     */
    return ant.returned;
}

void aco::acs::impl::do_iteration(size_t iteration, AntColony& ant_colony) {
    /**
     * Reset ants.
     */
    reset_ants(ant_colony);

    /**
     * Calculate steps taken by ants in this iteration.
     */
    size_t ants_returned = 0;
    for (size_t step = 0; step < (2 * ant_colony.options.max_steps); ++step) {
        /**
         * Break out early if ants are done for this iteration.
         */
        if (ants_returned >= ant_colony.options.ant_count) break;

        /**
         * Do some periodic output of state.
         */
        if (                ant_colony.options.do_output
             && iteration % ant_colony.options.output_frequency.coarse == 0
                  && step % ant_colony.options.output_frequency.fine == 0
        ) {
            std::string numeric_code = std::to_string(iteration * (2 * ant_colony.options.max_steps) + step);
            if (numeric_code.size() < 10)
                numeric_code.insert(0, 10 - numeric_code.size(), '0');
            std::string file_part = "pngs/" + numeric_code + "." + ant_colony.options.tag + "." + std::to_string(iteration + 1) + "." + std::to_string(step + 1) + ".acs";

            create_pheromone_heatmap_frame(file_part + ".pheromone", ant_colony);
            create_ant_count_heatmap_frame(file_part + ".ant_count", ant_colony);
        }

        /**
         * Loop ants and calculate their next step.
         *
         * If do_ant_next_step returns true, then that ant has just returned.
         */
        for (size_t ant_idx = 0; ant_idx < ant_colony.options.ant_count; ++ant_idx)
            ants_returned += do_ant_next_step(iteration, ant_colony.ants[ant_idx], ant_colony) ? 1 : 0;
    }

    /**
     * Apply global updating rule.
     */
    for (auto edge : boost::make_iterator_range(boost::edges(ant_colony.map.graph))) {
        ant_colony.map.edge_weight_map[edge] *= (1.0f - ant_colony.options.global.evaporation);

        /**
         * If edge is in shortest path, we must apply the inverse
         * length component of the global updating rule.
         */
        if (ant_colony.map.edge_in_path_map[edge])
            ant_colony.map.edge_weight_map[edge] +=
                ant_colony.options.global.evaporation
                    * (ant_colony.options.global.increment / ant_colony.shortest_path.length);
    }
}

size_t aco::acs::do_simulation(GraphMap map, ACSOptions options) {
    /**
     * Critical data points for simulation.
     */
    Ant* ants = new Ant[options.ant_count];
    AntColony ant_colony = { 
        map,
        ants,
        options,
        {
            new VertexDescriptor[options.max_steps],
            std::numeric_limits<size_t>::max()
        }
    };

    /**
     * Initialise ants & pheromones.
     */
    impl::initialise_ants(ant_colony);
    impl::initialise_pheromones(ant_colony);

    /**
     * Main simulation loop.
     */
    size_t iteration = 0;
    for (; iteration < options.iterations; ++iteration) {
        impl::do_iteration(iteration, ant_colony);

        if (ant_colony.shortest_path.length <= options.target_best_path_length)
            break;
    }

    /**
     * Clean up.
     */
    impl::destroy_ants(ant_colony);
    delete[] ants;
    delete[] ant_colony.shortest_path.steps;

    /**
     * Return number of iterations taken to reach target.
     *
     * Will be equal to configured total iterations if target
     * is set impossibly low (e.g. 0).
     */
    return iteration + 1;
}
