#include <iostream>
#include <fstream>
#include <cstring>
#include <random>
#include <cmath>
#include <thread>

const size_t MAP_DIM   = 8;
const size_t MAX_STEPS = 30;

std::default_random_engine generator;

const char MAP_1[MAP_DIM * MAP_DIM] = {
    '0', 'e', '0', '0', '0', '0', '0', '0',
    '0', '0', 'x', 'x', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', 'x', 'x', 'x', 'x', '0', '0',
    '0', 's', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0'
};

struct AntColony {
    float pheromone_map[MAP_DIM * MAP_DIM] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    char actual_map[MAP_DIM * MAP_DIM];

    unsigned int ant_count_map[MAP_DIM * MAP_DIM] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    };

    size_t start_idx;
    size_t end_idx;
};

struct Ant {
    AntColony* colony;
    bool has_food = false;
    size_t current_node_idx;
    // TODO: This shouldn't really be a fixed amount, needs to be somehow
    //       set dynamically. If we want to keep it non-dynamic, then we
    //       would either have to have a pool that ants share and the ant
    //       count be made dynamic, or subdivide the space being searched.
    size_t previous_node_indices[MAX_STEPS];
    size_t steps_taken = 0;
    size_t path_length = 0;
};

size_t choose_next_node(Ant* ant) {
    float total_pheromones = 0.0f;
    float cumulative_pheromones[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    size_t indices[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    float* pheromone_map    = ant->colony->pheromone_map;
    char*  actual_map       = ant->colony->actual_map;
    size_t current_node_idx = ant->current_node_idx;

    size_t row_idx = std::floor((float)current_node_idx / (float)MAP_DIM);
    size_t col_idx = current_node_idx % MAP_DIM;

    size_t cumulation_idx = 0;

    auto do_next_node_check = [&](size_t idx) {
        // If the node being checked was where the ant came from, it is not a valid candidate.
        if (ant->steps_taken > 0 && idx == ant->previous_node_indices[ant->steps_taken - 1]) return;

        // If node is not actually valid to step onto, then it is not a valid candidate.
        if (actual_map[idx] != 'x') {
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
    if (current_node_idx < MAP_DIM) {
        if (col_idx == 0) {
            do_next_node_check(1);
            do_next_node_check(MAP_DIM + 1);
            do_next_node_check(MAP_DIM);
        } else if (col_idx == 7) {
            do_next_node_check(2 * MAP_DIM - 1);
            do_next_node_check(2 * MAP_DIM - 2);
            do_next_node_check(6);
        } else {
            do_next_node_check(col_idx + 1);
            do_next_node_check(MAP_DIM + col_idx + 1);
            do_next_node_check(MAP_DIM + col_idx);
            do_next_node_check(MAP_DIM + col_idx - 1);
            do_next_node_check(col_idx - 1);
        }
    } else if (current_node_idx > (MAP_DIM * (MAP_DIM - 1) - 1)) {
        if (col_idx == 0) {
            do_next_node_check(MAP_DIM * (MAP_DIM - 2));
            do_next_node_check(MAP_DIM * (MAP_DIM - 2) + 1);
            do_next_node_check(MAP_DIM * (MAP_DIM - 1) + 1);
        } else if (col_idx == 7) {
            do_next_node_check(MAP_DIM * (MAP_DIM - 1) - 2);
            do_next_node_check(MAP_DIM * (MAP_DIM - 1) - 1);
            do_next_node_check(MAP_DIM * MAP_DIM - 2);
        } else {
            do_next_node_check(MAP_DIM * (MAP_DIM - 2) + col_idx - 1);
            do_next_node_check(MAP_DIM * (MAP_DIM - 2) + col_idx);
            do_next_node_check(MAP_DIM * (MAP_DIM - 2) + col_idx + 1);
            do_next_node_check(MAP_DIM * (MAP_DIM - 1) + col_idx + 1);
            do_next_node_check(MAP_DIM * (MAP_DIM - 1) + col_idx - 1);
        }
    } else {
        if (col_idx == 0) {
            do_next_node_check((row_idx - 1) * MAP_DIM);
            do_next_node_check((row_idx - 1) * MAP_DIM + 1);
            do_next_node_check(row_idx * MAP_DIM + 1);
            do_next_node_check((row_idx + 1) * MAP_DIM + 1);
            do_next_node_check((row_idx + 1) * MAP_DIM);
        } else if (col_idx == 7) {
            do_next_node_check((row_idx - 1) * MAP_DIM + MAP_DIM - 2);
            do_next_node_check((row_idx - 1) * MAP_DIM + MAP_DIM - 1);
            do_next_node_check((row_idx + 1) * MAP_DIM + MAP_DIM - 1);
            do_next_node_check((row_idx + 1) * MAP_DIM + MAP_DIM - 2);
            do_next_node_check(row_idx * MAP_DIM + MAP_DIM - 2);
        } else {
            do_next_node_check((row_idx - 1) * MAP_DIM + col_idx - 1);
            do_next_node_check((row_idx - 1) * MAP_DIM + col_idx);
            do_next_node_check((row_idx - 1) * MAP_DIM + col_idx + 1);
            do_next_node_check(row_idx * MAP_DIM + col_idx + 1);
            do_next_node_check((row_idx + 1) * MAP_DIM + col_idx + 1);
            do_next_node_check((row_idx + 1) * MAP_DIM + col_idx);
            do_next_node_check((row_idx + 1) * MAP_DIM + col_idx - 1);
            do_next_node_check(row_idx * MAP_DIM + col_idx - 1);
        }
    }

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
    return indices[cumulation_idx - 1];
}

int main() {
    std::cout << "Hello, world!" << std::endl;

    const size_t ant_count = 30;

    const float pheromone_increment    = 30.0f;  // Local increment (per ant per node) per timestep.
    const float pheromone_evapouration = 0.02f; // Global decrement on each node per timestep

    AntColony ant_colony;
    Ant       ants[ant_count];

    std::memcpy(ant_colony.actual_map, MAP_1, MAP_DIM * MAP_DIM);

    for (size_t i = 0; i < MAP_DIM * MAP_DIM; ++i) {
        if (ant_colony.actual_map[i] == 's') {
            ant_colony.start_idx = i;

            ant_colony.ant_count_map[i] = ant_count;
        }

        if (ant_colony.actual_map[i] == 'e') ant_colony.end_idx = i;
    }

    for (size_t i = 0; i < ant_count; ++i) {
        ants[i].colony = &ant_colony;
        ants[i].current_node_idx  = ant_colony.start_idx;
    }

    std::ofstream pheromone_output;
    std::ofstream ants_output;
    pheromone_output.open("results/pheromone_result.csv");
    ants_output.open("results/ants_result.csv");

    size_t iterations = 1000;
    while (iterations-- > 0) {
        // std::cout << std::endl << std::endl;
        // for (size_t i = 0; i < MAP_DIM * MAP_DIM; ++i) {
        //     if (i % MAP_DIM == 0) std::cout << std::endl;

        //     if (ant_colony.actual_map[i] != 'x') {
        //         std::cout << ant_colony.pheromone_map[i] << " ";
        //     } else {
        //         std::cout << "x ";
        //     }

        //     // if (ant_colony.ant_count_map[i] > 0) {
        //     //     std::cout << ant_colony.ant_count_map[i] << " ";
        //     // } else {
        //     //     std::cout << ant_colony.actual_map[i] << " ";
        //     // }
        // }

        for (size_t i = 0; i < MAP_DIM * MAP_DIM; ++i) {
            if (ant_colony.actual_map[i] != 'x') {
                pheromone_output << ant_colony.pheromone_map[i];
            } else {
                pheromone_output << 0.0f;
            }

            if (i != MAP_DIM * MAP_DIM - 1) pheromone_output << ", ";
        }
        pheromone_output << std::endl;

        for (size_t i = 0; i < MAP_DIM * MAP_DIM; ++i) {
            if (ant_colony.actual_map[i] != 'x') {
                ants_output << ant_colony.ant_count_map[i];
            } else {
                ants_output << 0;
            }

            if (i != MAP_DIM * MAP_DIM - 1) ants_output << ", ";
        }
        ants_output << std::endl;

        for (size_t i = 0; i < ant_count; ++i) {
            Ant* ant = &ants[i];

            if (ant->steps_taken > MAX_STEPS) {
                --ant_colony.ant_count_map[ant->current_node_idx];
                ++ant_colony.ant_count_map[ant_colony.start_idx];

                ant->steps_taken = 0;
                ant->current_node_idx = ant_colony.start_idx;
                ant->has_food = false;
            }

            if (ant->has_food) {
                size_t next_node_idx = ant->previous_node_indices[ant->steps_taken - 1];
                ant->steps_taken -= 1;

                --ant_colony.ant_count_map[ant->current_node_idx];
                ++ant_colony.ant_count_map[next_node_idx];

                ant->current_node_idx = next_node_idx;

                ant_colony.pheromone_map[next_node_idx] += pheromone_increment / ant->path_length;

                if (ant_colony.actual_map[next_node_idx] == 's') {
                    ant->has_food = false;
                    ant->steps_taken = 0;
                }
            } else {
                size_t next_node_idx = choose_next_node(ant);

                // std::cout << "Ant " << i << " goes to " << next_node_idx << std::endl;

                --ant_colony.ant_count_map[ant->current_node_idx];
                ++ant_colony.ant_count_map[next_node_idx];

                ant->previous_node_indices[ant->steps_taken] = ant->current_node_idx;
                ant->current_node_idx = next_node_idx;
                ant->steps_taken += 1;

                if (ant_colony.actual_map[next_node_idx] == 'e') {
                    ant->has_food = true;
                    ant->path_length = ant->steps_taken;
                }
            }
        }

        for (size_t i = 0; i < MAP_DIM * MAP_DIM; ++i) {
            ant_colony.pheromone_map[i] *= (1.0f - pheromone_evapouration);

            if (ant_colony.pheromone_map[i] < 0.0f) ant_colony.pheromone_map[i] = 0.0f;
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    pheromone_output.close();
    ants_output.close();
}
