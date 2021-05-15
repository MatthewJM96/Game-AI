#ifndef __map_maze2d_h
#define __map_maze2d_h

#pragma once

#include <array>

#include "dimension.hpp"

namespace map {
    namespace maze2d {
        using Graph = boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, boost::no_property, boost::property<boost::edge_weight_t, float>>;
        using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;
        using EdgeDescriptor = boost::graph_traits<Graph>::edge_descriptor;

        template <size_t MapX, size_t MapY = MapX>
        struct Map {
            std::array<char, MapX * MapY> map;
            size_t start_idx;
            size_t finish_idx;
        };

        template <size_t MapX, size_t MapY = MapX>
        Map<MapX, MapY> load_map(std::string map_filepath);

        template <size_t MapX, size_t MapY = MapX>
        Map<
            dimension::dim_to_padded_dim(MapX),
            dimension::dim_to_padded_dim(MapY)
        > load_map_with_halo(std::string map_filepath);

        template <size_t MapX, size_t MapY = MapX>
        void print_map(const Map<MapX, MapY>& map);

        template <size_t MapX, size_t MapY = MapX>
        Graph load_map_as_graph(std::string map_filepath);

        template <size_t MapX, size_t MapY = MapX, bool MapHasHalo = true>
        Graph map_to_graph(Map<MapX, MapY> map);

        namespace impl {
            template <size_t MapX, size_t MapY>
            Graph halo_map_to_graph(Map<MapX, MapY> map);

            template <size_t MapX, size_t MapY>
            Graph non_halo_map_to_graph(Map<MapX, MapY> map);
        };
    };
};

#include "maze2d.inl"

#endif // __map_maze2d_h
