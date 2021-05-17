#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "constants.h"

#include "map/maze2d.h"

#include "aco/naive.h"
#include "aco/halo.h"
#include "aco/acs.h"
#include "aco/acs_graph.h"

int main() {
    std::cout << "Hello, world!" << std::endl;

    const size_t map_dim   = 51;
    const size_t max_steps = 200;
    const size_t ant_count = 800;

    const float global_pheromone_increment   = map_dim * map_dim; // Global increment (best ant in round or all rounds).
    const float global_pheromone_evaporation = 0.1f; // Global decrement on each node per round.
    const float pheromone_increment          = map_dim * map_dim; // Local increment (per ant per node) per timestep.
    const float pheromone_evaporation        = 0.1f; // Global decrement on each node per timestep.

    const float exploitation_factor = 0.9f;
    const float cost_exponent = 2.0f;

    for (size_t i = 0; i < 1; ++i) {
        std::string idx = std::to_string(i);

        map::maze2d::Map<map_dim> map;
        map::maze2d::Map<map_dim + 2> halo_map;

        map      = map::maze2d::load_map<map_dim>("maps/25." + idx + ".unsolved.map");
        halo_map = map::maze2d::load_map_with_halo<map_dim>("maps/25." + idx + ".unsolved.map");

        map::maze2d::print_map<map_dim>(map);
        map::maze2d::print_map<map_dim + 2>(halo_map);

        // aco::naive::do_simulation<map_dim, max_steps>(idx, 2000, &map[0],      ant_count, pheromone_increment, pheromone_evaporation);
        // aco::halo:: do_simulation<map_dim, max_steps>(idx, 2000, &halo_map[0], ant_count, pheromone_increment, pheromone_evaporation);

        using Graph = boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, boost::property<vertex_map_idx_t, size_t>, boost::property<boost::edge_weight_t, float>>;
        using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;
        using EdgeDescriptor = boost::graph_traits<Graph>::edge_descriptor;
        using Edge = std::pair<size_t, size_t>;

        size_t map_indices[] = { 6, 1, 2, 3, 4 };
        float weights[] = { 0.5f, 3.0f, 1.0f, 3.0f, 2.0f, 5.0f, 0.2f, 1.3f, 2.4f, 3.1f};
        Edge edge_array[] = {
            Edge(0, 2), Edge(2, 0), Edge(1, 1), Edge(1, 3), Edge(1, 4), Edge(2, 1), Edge(2, 3), Edge(3, 4), Edge(4, 0), Edge(4, 1)
        };
        Graph g(edge_array, edge_array + 10, weights, 5);

        boost::property_map<Graph, boost::edge_weight_t>::type weight_map = get(boost::edge_weight, g);
        boost::property_map<Graph, vertex_map_idx_t>::type vertex_map_idx_map = get(vertex_map_idx, g);

        for (size_t vertex_idx = 0; vertex_idx < 5; ++vertex_idx) vertex_map_idx_map[vertex_idx] = map_indices[vertex_idx];

        auto e_it = boost::out_edges(0, g);
        for (; e_it.first != e_it.second; ++e_it.first) {
            std::cout << "(" 
                        << vertex_map_idx_map[boost::source(*e_it.first, g)]
                        << ", "
                        << vertex_map_idx_map[boost::target(*e_it.first, g)]
                        << ") weight: "
                        << weight_map[*e_it.first]
                        << std::endl;
        }
        auto e_it_2 = boost::in_edges(0, g);
        for (; e_it_2.first != e_it_2.second; ++e_it_2.first) {
            std::cout << "(" 
                        << vertex_map_idx_map[boost::source(*e_it_2.first, g)]
                        << ", "
                        << vertex_map_idx_map[boost::target(*e_it_2.first, g)]
                        << ") weight: "
                        << weight_map[*e_it_2.first]
                        << std::endl;
        }


        map::maze2d::GraphMap graph_map = map::maze2d::map_to_graph<>(halo_map, 1.0f);

        auto graph_out = std::ofstream("graph_out.txt");
        // boost::write_graphviz(graph_out, graph_map.graph);


        // std::pair<EdgeDescriptor, bool> e = boost::edge(0, 2, g);
        // std::pair<EdgeDescriptor, bool> e2 = boost::edge(2, 0, g);

        // weightmap[e.first] += 0.1f;

        // std::cout << "Graph edge (0, 2) score: " << weightmap[e.first] << std::endl;
        // std::cout << "Graph edge (2, 0) score: " << weightmap[e2.first] << std::endl;

        aco::acs_graph::ACSOptions options {
            10,
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
            }
        };

        aco::acs_graph::do_simulation<map_dim, max_steps>(idx, graph_map, options);

        // aco::acs::do_simulation<map_dim, max_steps>(idx, &halo_map[0], options);
    }
}
