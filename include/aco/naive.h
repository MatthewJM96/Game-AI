#include <array>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <random>

namespace aco {
    namespace naive {
        std::default_random_engine generator;

        template <size_t MapSize>
        constexpr std::array<float, MapSize> initialisePheromoneMap();

        template <size_t MapSize>
        constexpr std::array<uint16_t, MapSize> initialiseAntCountMap();

        template <size_t MapSize>
        struct AntColony {
            std::array<float, MapSize>    pheromone_map = initialisePheromoneMap<MapSize>();
            std::array<char, MapSize>     actual_map;
            std::array<uint16_t, MapSize> ant_count_map = initialiseAntCountMap<MapSize>();

            size_t start_idx;
            size_t end_idx;
        };
        
        template <size_t MapSize, size_t MaxSteps>
        struct Ant {
            AntColony<MapSize>* colony;

            bool has_food = false;

            // TODO: This shouldn't really be a fixed amount, needs to be somehow
            //       set dynamically. If we want to keep it non-dynamic, then we
            //       would either have to have a pool that ants share and the ant
            //       count be made dynamic, or subdivide the space being searched.
            std::array<size_t, MaxSteps> previous_node_indices;
            size_t current_node_idx;
            size_t steps_taken = 0;
            size_t path_length = 0;
        };

        template <size_t MapDim, size_t MaxSteps>
        size_t choose_next_node(Ant<MapDim * MapDim, MaxSteps>* ant);

        template <size_t MapDim, size_t MaxSteps>
        void do_simulation(const char* actual_map_ptr, size_t ant_count, float pheromone_increment, float pheromone_evaporation);
    };
};

#include "naive.inl"
