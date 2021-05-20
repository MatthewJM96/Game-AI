#include <fstream>
#include <thread>

#include "constants.h"
#include "dimension.hpp"

template <size_t MapSize>
constexpr std::array<float, MapSize> aco::acs_graph::initialise_pheromone_map() {
    std::array<float, MapSize> arr;

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 0.0f;

    return arr;
}

template <size_t MapSize>
constexpr std::array<uint16_t, MapSize> aco::acs_graph::initialise_ant_count_map() {
    std::array<uint16_t, MapSize> arr;

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 0;

    return arr;
}

template <size_t MapSize, size_t MaxSteps>
void aco::acs_graph::prepare_ants(Ant<MapSize, MaxSteps>* ants, size_t ant_count, AntColony<MapSize>* ant_colony) {
    for (size_t i = 0; i < ant_count; ++i) {
        ants[i].colony           = ant_colony;
        ants[i].steps_taken      = 0;
        ants[i].current_node_idx = ant_colony->actual_map.start_idx;
    }
}

template <size_t MapSize>
void aco::acs_graph::print_to_file(std::ofstream& file, AntColony<MapSize>* ant_colony, float (*value_for_idx)(size_t idx, AntColony<MapSize>* ant_colony)) {
    for (size_t i = 0; i < MapSize; ++i) {
        if (ant_colony->actual_map.vertex_to_tile_char_map[ant_colony->actual_map.map_idx_to_vertex_map[i]] != WALL_TILE) {
            file << value_for_idx(i, ant_colony);
        } else {
            file << 0.0f;
        }

        if (i != MapSize - 1) file << ", ";
    }
    file << "\n";
}

template <size_t MapDim, size_t MaxSteps>
size_t aco::acs_graph::choose_next_node(Ant<dimension::dim2d_to_padded_size(MapDim), MaxSteps>* ant, float exploitation_factor, float(*to_node_cost)(VertexDescriptor initial, VertexDescriptor final), float cost_exponent) {
    size_t padded_dim = dimension::dim2d_to_padded_size(MapDim);

    VertexDescriptor best_option       = 0;
    float            best_option_score = -1000000.0f;

    float total_score = 0.0f;
    float cumulative_scores[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    size_t indices[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    float*    pheromone_map = &ant->colony->pheromone_map[0];
    GraphMap* actual_map    = &ant->colony->actual_map;

    size_t current_node_idx = ant->current_node_idx;

    size_t row_idx = std::floor((float)current_node_idx / (float)padded_dim);
    size_t col_idx = current_node_idx % padded_dim;

    size_t cumulation_idx = 0;

    auto do_next_node_check = [&](EdgeDescriptor edge, VertexDescriptor initial_vertex, VertexDescriptor candidate_vertex) {
        // If node is not actually valid to step onto, then it is not a valid candidate.
        if (actual_map->vertex_to_tile_char_map[candidate_vertex] == WALL_TILE) return;

        // Given node is valid to step to, add as a candidate.
        float score = actual_map->edge_weight_map[edge] / std::pow(to_node_cost(initial_vertex, candidate_vertex), cost_exponent);

        // If this node has the best score so far, set it as best option.
        if (score > best_option_score) {
            best_option       = candidate_vertex;
            best_option_score = score;
        }

        // Increment total pheromone level of all candidates.
        total_score += score;

        // Add cumulation entry for pheromone level of new candidate.
        cumulative_scores[cumulation_idx] = total_score;

        // Add index entry for the candidate node.
        indices[cumulation_idx] = actual_map->vertex_to_map_idx_map[candidate_vertex];

        // Increment cumulation index - its final value being number of candidates found.
        ++cumulation_idx;
    };

    // Iterate out edges from the vertex the ant is presently at.
    // Consider each vertex connecting to those edges as candidate
    // next vertices for the ant to move to.
    VertexDescriptor current_vertex = actual_map->map_idx_to_vertex_map[current_node_idx];
    for (EdgeDescriptor edge : boost::make_iterator_range(boost::out_edges(current_vertex, actual_map->graph))) {
        VertexDescriptor candidate_vertex = boost::target(edge, actual_map->graph);

        do_next_node_check(edge, current_vertex, candidate_vertex);
    }

    // If no candidates are found, then just go back to where we were.
    if (cumulation_idx == 0) return ant->previous_node_indices[ant->steps_taken - 1];

    // Determine if we exploit or explore.
    std::uniform_real_distribution<float> exploitation_distribution(0.0f, 1.0f);
    float exploitation_val = exploitation_distribution(generator);

    // If we are to exploit, then this ant will be choosing the best option of its next steps.
    if (exploitation_val < exploitation_factor) return actual_map->vertex_to_map_idx_map[best_option];

    // If we get here, then this ant is exploring.

    // Generate a choice score to use to select next step.
    std::uniform_real_distribution<float> score_distribution(0.0f, total_score);
    float choice_val = score_distribution(generator);

    // std::cout << cumulation_idx << " candidates for ant at " << ant->current_node_idx << std::endl;
    // std::cout << "Total score = " << total_score << std::endl;
    // std::cout << "Choice val = " << choice_val << std::endl;
    // std::cout << "Index by score levels:" << std::endl;
    // for (size_t i = 0; i < cumulation_idx; ++i) std::cout << "    " << indices[i] << "  -  " << cumulative_scores[i] << std::endl;

    // Identify index of next step based on choice val.
    size_t choice_idx = 0;
    for (size_t choice_idx = 0; choice_idx < cumulation_idx; ++choice_idx) {
        if (choice_val <= cumulative_scores[choice_idx]) return indices[choice_idx];
    }
    return ant->previous_node_indices[ant->steps_taken - 1];
}

template <size_t MapDim, size_t MaxSteps>
void aco::acs_graph::do_simulation(
    std::string tag,
    GraphMap    actual_map,
    ACSOptions  options
) {
    const size_t map_size = dimension::dim2d_to_padded_size(MapDim);

    /**
     * Critical data points for simulation.
     */
    AntColony<map_size> ant_colony = {
        actual_map,
        initialise_ant_count_map<map_size>(),
        initialise_pheromone_map<map_size>()
    };
    Ant<map_size, MaxSteps>* ants = new Ant<map_size, MaxSteps>[options.ant_count];

    /**
     * Set initial ant count.
     */
    ant_colony.ant_count_map[ant_colony.actual_map.start_idx] = options.ant_count;

    /**
     * Prepare ants for simulation.
     */
    prepare_ants<map_size, MaxSteps>(ants, options.ant_count, &ant_colony);

    /**
     * Walk vertices and update pheromone map with in_edge degree.
     */
    for (auto vertex : boost::make_iterator_range(boost::vertices(ant_colony.actual_map.graph))) {
        size_t num_edges = boost::in_degree(vertex, ant_colony.actual_map.graph);
        size_t map_idx   = ant_colony.actual_map.vertex_to_map_idx_map[vertex];

        ant_colony.pheromone_map[map_idx] = (float)num_edges;
    }

    std::ofstream pheromone_output;
    std::ofstream ants_output;
    pheromone_output.open("results/" + tag + ".acs_graph.pheromone_result.csv");
    ants_output.open("results/" + tag + ".acs_graph.ants_result.csv");

    std::array<size_t, MaxSteps> shortest_path        = {};
    long long                    shortest_path_length = -1;

    size_t iteration = options.iterations;
    while (iteration > 0) {
        // if (iteration != options.iterations) {
        //     std::cout << "Iteration " << options.iterations - iteration << ":\n";
        //     for (auto edge : boost::make_iterator_range(boost::out_edges(ant_colony.actual_map.map_idx_to_vertex_map[ant_colony.actual_map.start_idx], ant_colony.actual_map.graph))) {
        //         VertexDescriptor source = boost::source(edge, ant_colony.actual_map.graph);
        //         VertexDescriptor target = boost::target(edge, ant_colony.actual_map.graph);

        //         std::cout << "    Edge weight (" << source << ", " << target << "): " << ant_colony.actual_map.edge_weight_map[edge] << "\n";
        //     }
        //     std::cout << std::endl;
        // }

        for (size_t i = 0; i < options.ant_count; ++i) {
            --ant_colony.ant_count_map[ants[i].current_node_idx];
            ++ant_colony.ant_count_map[ant_colony.actual_map.start_idx];

            ants[i].steps_taken      = 0;
            ants[i].path_length      = 0;
            ants[i].returned         = false;
            ants[i].found_food       = false;
            ants[i].has_food         = false;
            ants[i].current_node_idx = ant_colony.actual_map.start_idx;
        }

        size_t ants_returned = 0;
        for (size_t step = 0; step < 2 * MaxSteps; ++step) {
            if (ants_returned >= options.ant_count) break;

            print_to_file<map_size>(pheromone_output, &ant_colony, [](size_t idx, AntColony<map_size>* ant_colony) {
                return ant_colony->pheromone_map[idx] < 0.005f ? 0.0f : ant_colony->pheromone_map[idx];
            });

            print_to_file<map_size>(ants_output, &ant_colony, [](size_t idx, AntColony<map_size>* ant_colony) {
                return (float)ant_colony->ant_count_map[idx];
            });

            for (size_t ant_idx = 0; ant_idx < options.ant_count; ++ant_idx) {
                Ant<map_size, MaxSteps>* ant = &ants[ant_idx];

                if (ant->returned) continue;

                if (ant->steps_taken >= MaxSteps) {
                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[ant_colony.actual_map.start_idx];

                    ant->steps_taken = 0;
                    ant->current_node_idx = ant_colony.actual_map.start_idx;
                    ant->found_food = false;
                    ant->has_food   = false;
                    ant->returned   = true;

                    ++ants_returned;
                }

                if (ant->has_food) {
                    size_t next_node_idx = ant->previous_node_indices[ant->steps_taken - 1];
                    ant->steps_taken -= 1;

                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[next_node_idx];

                    // Must use these in reverse order to get the correct edge - ant is headed backwards.
                    VertexDescriptor current_vertex = ant_colony.actual_map.map_idx_to_vertex_map[ant->current_node_idx];
                    VertexDescriptor next_vertex    = ant_colony.actual_map.map_idx_to_vertex_map[next_node_idx];
                    EdgeDescriptor edge;
                    bool edge_exists;
                    std::tie(edge, edge_exists) = boost::edge(next_vertex, current_vertex, ant_colony.actual_map.graph);

                    float increment_by = options.local.increment / ant->path_length;
                    ant_colony.actual_map.edge_weight_map[edge] += increment_by;
                    ant_colony.pheromone_map[current_vertex] += increment_by;

                    ant->current_node_idx = next_node_idx;

                    if (next_node_idx == ant_colony.actual_map.start_idx) {
                        ant->has_food    = false;
                        ant->returned    = true;
                        ant->steps_taken = 0;

                        ++ants_returned;
                    }
                } else {
                    size_t next_node_idx = choose_next_node<MapDim, MaxSteps>(ant, iteration == options.iterations ? 0.0f : options.exploitation_factor, [](VertexDescriptor initial, VertexDescriptor end) {
                        return 1.0f;
                    }, options.cost_exponent);

                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[next_node_idx];

                    ant->previous_node_indices[ant->steps_taken] = ant->current_node_idx;
                    ant->current_node_idx = next_node_idx;
                    ant->steps_taken += 1;

                    if (next_node_idx == ant_colony.actual_map.finish_idx) {
                        ant->found_food  = true;
                        ant->has_food    = true;
                        ant->path_length = ant->steps_taken;

                        if (shortest_path_length < 0 || ant->path_length < shortest_path_length) {
                            std::memcpy(&shortest_path[0], &ant->previous_node_indices[0], sizeof(size_t) * ant->path_length);
                            shortest_path_length = ant->path_length;
                        }
                    }
                }
            }

            for (auto edge : boost::make_iterator_range(boost::edges(ant_colony.actual_map.graph)))
                ant_colony.actual_map.edge_weight_map[edge] *= (1.0f - options.local.evaporation);
            for (size_t i = 0; i < map_size; ++i)
                ant_colony.pheromone_map[i] *= (1.0f - options.local.evaporation);
        }

        if (shortest_path_length > 0) {
            for (size_t i = 0; i < shortest_path_length; ++i) {
                float increment_by = options.global.increment / shortest_path_length;
                if (i > 0) {
                    VertexDescriptor source_vertex = ant_colony.actual_map.map_idx_to_vertex_map[shortest_path[i - 1]];
                    VertexDescriptor target_vertex = ant_colony.actual_map.map_idx_to_vertex_map[shortest_path[i]];
                    EdgeDescriptor edge;
                    bool edge_exists;
                    std::tie(edge, edge_exists) = boost::edge(source_vertex, target_vertex, ant_colony.actual_map.graph);

                    ant_colony.actual_map.edge_weight_map[edge] += increment_by;
                }
                ant_colony.pheromone_map[shortest_path[i]] += increment_by;
            }
        }

        for (auto edge : boost::make_iterator_range(boost::edges(ant_colony.actual_map.graph)))
            ant_colony.actual_map.edge_weight_map[edge] *= (1.0f - options.global.evaporation);
        for (size_t i = 0; i < map_size; ++i)
            ant_colony.pheromone_map[i] *= (1.0f - options.global.evaporation);

        --iteration;
    }

    pheromone_output.close();
    ants_output.close();
}
