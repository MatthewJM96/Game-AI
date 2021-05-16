#ifndef __map_maze2d_h
#define __map_maze2d_h

#pragma once

#include <array>
#include <unordered_map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "dimension.hpp"

enum vertex_map_idx_t {
    vertex_map_idx
};

namespace boost {
BOOST_INSTALL_PROPERTY(vertex, map_idx);
};

namespace map {
    namespace maze2d {
        using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, boost::property<vertex_map_idx_t, size_t>, boost::property<boost::edge_weight_t, float>>;

        using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;
        using EdgeDescriptor   = boost::graph_traits<Graph>::edge_descriptor;

        using EdgeWeightMap     = boost::property_map<Graph, boost::edge_weight_t>::type;
        using VertexToMapIdxMap = boost::property_map<Graph, vertex_map_idx_t>::type;
        using MapIdxToVertexMap = std::unordered_map<size_t, VertexDescriptor>;

        template <size_t MapX, size_t MapY = MapX>
        struct Map {
            std::array<char, MapX * MapY> map;
            size_t start_idx;
            size_t finish_idx;
        };

        struct GraphMap {
            Graph             graph;
            EdgeWeightMap     edge_weight_map;
            VertexToMapIdxMap vertex_to_map_idx_map;
            MapIdxToVertexMap map_idx_to_vertex_map;
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
        GraphMap map_to_graph(Map<MapX, MapY> map, float initial_weight);

        namespace impl {
            template <size_t MapX, size_t MapY>
            GraphMap halo_map_to_graph(Map<MapX, MapY> map, float initial_weight);

            template <size_t MapX, size_t MapY>
            GraphMap non_halo_map_to_graph(Map<MapX, MapY> map, float initial_weight);
        };
    };
};

#include "maze2d.inl"

#endif // __map_maze2d_h
