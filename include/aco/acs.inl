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

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 0.0f;

    return arr;
}

template <size_t MapSize>
constexpr std::array<uint16_t, MapSize> aco::acs::initialise_ant_count_map() {
    std::array<uint16_t, MapSize> arr;

    for (size_t i = 0; i < MapSize; ++i) arr[i] = 0;

    return arr;
}

template <size_t MapDim>
constexpr void aco::acs::zero_halo_of_pheromone_map(AntColony<dim_to_size(MapDim)>* ant_colony) {
    size_t padded_dim = dim_to_padded_dim(MapDim);
    for (size_t i = 0; i < padded_dim; ++i) {
        ant_colony->pheromone_map[i] = 0.0f;

        if (i != 0)          ant_colony->pheromone_map[(i * padded_dim) - 1] = 0.0f;
        if (i != padded_dim) ant_colony->pheromone_map[(i * padded_dim)]     = 0.0f;

        ant_colony->pheromone_map[padded_dim * (padded_dim - 1) + i] = 0.0f;
    }
}

template <size_t MapDim, size_t MaxSteps>
size_t aco::acs::choose_next_node(Ant<dim_to_size(MapDim), MaxSteps>* ant) {
    size_t padded_dim = dim_to_padded_dim(MapDim);

    float total_pheromones = 0.0f;
    float cumulative_pheromones[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
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
        if (actual_map[idx] != WALL_TILE) {
            // Given node is valid to step to, add as a candidate.

            // Increment total pheromone level of all candidates.
            total_pheromones += pheromone_map[idx] + 0.1f;

            // Add cumulation entry for pheromone level of new candidate.
            cumulative_pheromones[cumulation_idx] = pheromone_map[idx] + 0.1f;
            if (cumulation_idx > 0) cumulative_pheromones[cumulation_idx] += cumulative_pheromones[cumulation_idx - 1];

            // Add index entry for the candidate node.
            indices[cumulation_idx] = idx;

            // Increment cumulation index - its final value being number of candidates found.
            ++cumulation_idx;
        }
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

    // Generate a choice pheromone "level" to use to select next step.
    std::uniform_real_distribution<float> distribution(0.0f, total_pheromones);
    float choice_val = distribution(generator);

    // std::cout << cumulation_idx << " candidates for ant at " << ant->current_node_idx << std::endl;
    // std::cout << "Total pheromones = " << total_pheromones << std::endl;
    // std::cout << "Choice val = " << choice_val << std::endl;
    // std::cout << "Index by pheromone levels:" << std::endl;
    // for (size_t i = 0; i < cumulation_idx; ++i) std::cout << "    " << indices[i] << "  -  " << cumulative_pheromones[i] << std::endl;

    // Identify index of next step based on choice val.
    size_t choice_idx = 0;
    for (size_t choice_idx = 0; choice_idx < cumulation_idx; ++choice_idx) {
        if (choice_val <= cumulative_pheromones[choice_idx]) return indices[choice_idx];
    }
    return ant->previous_node_indices[ant->steps_taken - 1];
}

template <size_t MapDim, size_t MaxSteps>
void aco::acs::do_simulation(
    std::string tag,
    size_t iterations,
    const char* actual_map_ptr,
    size_t ant_count,
    float local_pheromone_increment,
    float global_pheromone_increment,
    float pheromone_evaporation
) {
    const size_t MapSize = dim_to_size(MapDim);

    AntColony<MapSize>     ant_colony;
    Ant<MapSize, MaxSteps>* ants = new Ant<MapSize, MaxSteps>[ant_count];

    std::memcpy(&ant_colony.actual_map[0], actual_map_ptr, MapSize);

    for (size_t i = 0; i < MapSize; ++i) {
        if (ant_colony.actual_map[i] == START_TILE) {
            ant_colony.start_idx = i;

            ant_colony.ant_count_map[i] = ant_count;
        }

        if (ant_colony.actual_map[i] == END_TILE) ant_colony.end_idx = i;
    }

    for (size_t i = 0; i < ant_count; ++i) {
        ants[i].colony           = &ant_colony;
        ants[i].steps_taken      = 0;
        ants[i].current_node_idx = ant_colony.start_idx;
    }

    std::ofstream pheromone_output;
    std::ofstream ants_output;
    pheromone_output.open("results/" + tag + ".acs.pheromone_result.csv");
    ants_output.open("results/" + tag + ".acs.ants_result.csv");

    while (iterations-- > 0) {
        Ant<MapSize, MaxSteps>* ant_with_shortest_path = nullptr;

        for (size_t i = 0; i < ant_count; ++i) {
            ants[i].steps_taken      = 0;
            ants[i].path_length      = 0;
            ants[i].returned         = false;
            ants[i].found_food       = false;
            ants[i].has_food         = false;
            ants[i].current_node_idx = ant_colony.start_idx;
        }

        size_t ants_returned = 0;
        for (size_t i = 0; i < 2 * MaxSteps; ++i) {
            if (ants_returned >= ant_count) break;

            for (size_t i = 0; i < MapSize; ++i) {
                if (ant_colony.actual_map[i] != WALL_TILE) {
                    pheromone_output << ant_colony.pheromone_map[i];
                } else {
                    pheromone_output << 0.0f;
                }

                if (i != MapSize - 1) pheromone_output << ", ";
            }
            pheromone_output << std::endl;

            for (size_t i = 0; i < MapSize; ++i) {
                if (ant_colony.actual_map[i] != WALL_TILE) {
                    ants_output << ant_colony.ant_count_map[i];
                } else {
                    ants_output << 0;
                }

                if (i != MapSize - 1) ants_output << ", ";
            }
            ants_output << std::endl;

            for (size_t i = 0; i < ant_count; ++i) {
                Ant<MapSize, MaxSteps>* ant = &ants[i];

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

                    ant_colony.pheromone_map[next_node_idx] += local_pheromone_increment / ant->path_length;

                    if (ant_colony.actual_map[next_node_idx] == START_TILE) {
                        ant->has_food    = false;
                        ant->returned    = true;
                        ant->steps_taken = 0;

                        ++ants_returned;
                    }
                } else {
                    size_t next_node_idx = choose_next_node<MapDim, MaxSteps>(ant);

                    --ant_colony.ant_count_map[ant->current_node_idx];
                    ++ant_colony.ant_count_map[next_node_idx];

                    ant->previous_node_indices[ant->steps_taken] = ant->current_node_idx;
                    ant->current_node_idx = next_node_idx;
                    ant->steps_taken += 1;

                    if (ant_colony.actual_map[next_node_idx] == END_TILE) {
                        ant->found_food  = true;
                        ant->has_food    = true;
                        ant->path_length = ant->steps_taken;

                        if (ant_with_shortest_path == nullptr) {
                            ant_with_shortest_path = ant;
                        }
                    }
                }
            }

            for (size_t i = 0; i < MapSize; ++i) {
                ant_colony.pheromone_map[i] *= (1.0f - pheromone_evaporation);

                if (ant_colony.pheromone_map[i] < 0.0f) ant_colony.pheromone_map[i] = 0.0f;
            }
        }

        if (ant_with_shortest_path) {
            for (size_t i = 0; i < ant_with_shortest_path->path_length; ++i) {
                ant_colony.pheromone_map[ant_with_shortest_path->previous_node_indices[i]] += global_pheromone_increment / ant_with_shortest_path->path_length;
            }
        }

        zero_halo_of_pheromone_map<MapDim>(&ant_colony);

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    pheromone_output.close();
    ants_output.close();
}
