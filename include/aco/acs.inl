#include <fstream>
#include <thread>

#include "constants.h"

constexpr size_t aco::acs::dim_to_padded_dim(size_t map_dim) {
    return map_dim + 2;
}
constexpr size_t aco::acs::dim_to_size(size_t map_dim) {
    return dim_to_padded_dim(map_dim) * dim_to_padded_dim(map_dim);
}

template <size_t MapSize>
constexpr std::array<float, MapSize> aco::acs::initialise_pheromone_map() {
    std::array<float, MapSize> arr;

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 1.0f;

    return arr;
}

template <size_t MapSize>
constexpr std::array<uint16_t, MapSize> aco::acs::initialise_ant_count_map() {
    std::array<uint16_t, MapSize> arr;

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 0;

    return arr;
}

template <size_t MapSize>
void aco::acs::get_start_and_end_locations(AntColony<MapSize>* ant_colony, size_t ant_count) {
    for (size_t i = 0; i < MapSize; ++i) {
        if (ant_colony->actual_map[i] == START_TILE) {
            ant_colony->start_idx = i;

            ant_colony->ant_count_map[i] = ant_count;
        }

        if (ant_colony->actual_map[i] == END_TILE) ant_colony->end_idx = i;
    }
}

template <size_t MapSize, size_t MaxSteps>
void aco::acs::prepare_ants(Ant<MapSize, MaxSteps>* ants, size_t ant_count, AntColony<MapSize>* ant_colony) {
    for (size_t i = 0; i < ant_count; ++i) {
        ants[i].colony           = ant_colony;
        ants[i].steps_taken      = 0;
        ants[i].current_node_idx = ant_colony->start_idx;
    }
}

template <size_t MapSize>
void aco::acs::print_to_file(std::ofstream& file, AntColony<MapSize>* ant_colony, float (*value_for_idx)(size_t idx, AntColony<MapSize>* ant_colony)) {
    for (size_t i = 0; i < MapSize; ++i) {
        if (ant_colony->actual_map[i] != WALL_TILE) {
            file << value_for_idx(i, ant_colony);
        } else {
            file << 0.0f;
        }

        if (i != MapSize - 1) file << ", ";
    }
    file << "\n";
}

template <size_t MapDim, size_t MaxSteps>
size_t aco::acs::choose_next_node(Ant<dim_to_size(MapDim), MaxSteps>* ant, float exploitation_factor, float(*to_node_cost)(size_t initial, size_t final), float cost_exponent) {
    size_t padded_dim = dim_to_padded_dim(MapDim);

    size_t best_option = 0;
    float  best_option_score = -1000000.0f;

    float total_score = 0.0f;
    float cumulative_scores[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    size_t indices[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    float* pheromone_map    = &ant->colony->pheromone_map[0];
    char*  actual_map       = &ant->colony->actual_map[0];
    size_t current_node_idx = ant->current_node_idx;

    size_t row_idx = std::floor((float)current_node_idx / (float)padded_dim);
    size_t col_idx = current_node_idx % padded_dim;

    size_t cumulation_idx = 0;

    auto do_next_node_check = [&](size_t idx) {
        // If the node being checked was where the ant came from, it is not a valid candidate.
        if (ant->steps_taken > 0 && idx == ant->previous_node_indices[ant->steps_taken - 1]) return;

        // If node is not actually valid to step onto, then it is not a valid candidate.
        if (actual_map[idx] == WALL_TILE) return;

        // Given node is valid to step to, add as a candidate.
        float score = pheromone_map[idx] / std::pow(to_node_cost(current_node_idx, idx), cost_exponent);

        // If this node has the best score so far, set it as best option.
        if (score > best_option_score) {
            best_option = idx;
            best_option_score = score;
        }

        // Increment total pheromone level of all candidates.
        total_score += score;

        // Add cumulation entry for pheromone level of new candidate.
        cumulative_scores[cumulation_idx] = score;
        if (cumulation_idx > 0) cumulative_scores[cumulation_idx] += cumulative_scores[cumulation_idx - 1];

        // Add index entry for the candidate node.
        indices[cumulation_idx] = idx;

        // Increment cumulation index - its final value being number of candidates found.
        ++cumulation_idx;
    };

    // Some special handling for borders of map, but generally build candidate
    // list of next nodes to step to.
    do_next_node_check((row_idx - 1) * padded_dim + col_idx - 1);
    do_next_node_check((row_idx - 1) * padded_dim + col_idx);
    do_next_node_check((row_idx - 1) * padded_dim + col_idx + 1);
    do_next_node_check(row_idx * padded_dim + col_idx + 1);
    do_next_node_check((row_idx + 1) * padded_dim + col_idx + 1);
    do_next_node_check((row_idx + 1) * padded_dim + col_idx);
    do_next_node_check((row_idx + 1) * padded_dim + col_idx - 1);
    do_next_node_check(row_idx * padded_dim + col_idx - 1);

    // If no candidates are found, then just go back to where we were.
    if (cumulation_idx == 0) return ant->previous_node_indices[ant->steps_taken - 1];

    // Determine if we exploit or explore.
    std::uniform_real_distribution<float> exploitation_distribution(0.0f, 1.0f);
    float exploitation_val = exploitation_distribution(generator);

    // If we are to exploit, then this ant will be choosing the best option of its next steps.
    if (exploitation_val < exploitation_factor) return best_option;

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
void aco::acs::do_simulation(
    std::string tag,
    const char* actual_map_ptr,
    ACSOptions  options
) {
    const size_t MapSize = dim_to_size(MapDim);

    /**
     * Critical data points for simulation.
     */
    AntColony<MapSize> ant_colony;
    Ant<MapSize, MaxSteps>* ants = new Ant<MapSize, MaxSteps>[options.ant_count];

    /**
     * Copy in map (next: bidrectional asymmetric graph).
     */
    std::memcpy(&ant_colony.actual_map[0], actual_map_ptr, MapSize);

    /**
     * Get start and end locations of map.
     */
    get_start_and_end_locations(&ant_colony, options.ant_count);

    /**
     * Prepare ants for simulation.
     */
    prepare_ants<MapSize, MaxSteps>(ants, options.ant_count, &ant_colony);

    std::ofstream pheromone_output;
    std::ofstream ants_output;
    pheromone_output.open("results/" + tag + ".acs.pheromone_result.csv");
    ants_output.open("results/" + tag + ".acs.ants_result.csv");

    std::array<size_t, MaxSteps> shortest_path        = {};
    long long                    shortest_path_length = -1;

    size_t iteration = options.iterations;
    while (iteration > 0) {
        Ant<MapSize, MaxSteps>* ant_with_shortest_path = nullptr;

        for (size_t i = 0; i < options.ant_count; ++i) {
            --ant_colony.ant_count_map[ants[i].current_node_idx];
            ++ant_colony.ant_count_map[ant_colony.start_idx];

            ants[i].steps_taken      = 0;
            ants[i].path_length      = 0;
            ants[i].returned         = false;
            ants[i].found_food       = false;
            ants[i].has_food         = false;
            ants[i].current_node_idx = ant_colony.start_idx;
        }

        size_t ants_returned = 0;
        for (size_t step = 0; step < 2 * MaxSteps; ++step) {
            if (ants_returned >= options.ant_count) break;

            print_to_file<MapSize>(pheromone_output, &ant_colony, [](size_t idx, AntColony<MapSize>* ant_colony) {
                return ant_colony->pheromone_map[idx] < 0.005f ? 0.0f : ant_colony->pheromone_map[idx];
            });

            print_to_file<MapSize>(ants_output, &ant_colony, [](size_t idx, AntColony<MapSize>* ant_colony) {
                return (float)ant_colony->ant_count_map[idx];
            });

            for (size_t ant_idx = 0; ant_idx < options.ant_count; ++ant_idx) {
                Ant<MapSize, MaxSteps>* ant = &ants[ant_idx];

                if (ant->returned) continue;

                if (ant->steps_taken >= MaxSteps) {
                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[ant_colony.start_idx];

                    ant->steps_taken = 0;
                    ant->current_node_idx = ant_colony.start_idx;
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

                    ant->current_node_idx = next_node_idx;

                    ant_colony.pheromone_map[next_node_idx] += options.local.increment / ant->path_length;

                    if (ant_colony.actual_map[next_node_idx] == START_TILE) {
                        ant->has_food    = false;
                        ant->returned    = true;
                        ant->steps_taken = 0;

                        ++ants_returned;
                    }
                } else {
                    size_t next_node_idx = choose_next_node<MapDim, MaxSteps>(ant, iteration == options.iterations ? 0.0f : options.exploitation_factor, [](size_t initial, size_t end) {
                        return 1.0f;
                    }, options.cost_exponent);

                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[next_node_idx];

                    ant->previous_node_indices[ant->steps_taken] = ant->current_node_idx;
                    ant->current_node_idx = next_node_idx;
                    ant->steps_taken += 1;

                    if (ant_colony.actual_map[next_node_idx] == END_TILE) {
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

            for (size_t i = 0; i < MapSize; ++i) ant_colony.pheromone_map[i] *= (1.0f - options.local.evaporation);
        }

        if (shortest_path_length > 0) {
            for (size_t i = 0; i < shortest_path_length; ++i) ant_colony.pheromone_map[shortest_path[i]] += options.global.increment / shortest_path_length;
        }

        for (size_t i = 0; i < MapSize; ++i) ant_colony.pheromone_map[i] *= (1.0f - options.global.evaporation);

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        --iteration;
    }

    pheromone_output.close();
    ants_output.close();
}
