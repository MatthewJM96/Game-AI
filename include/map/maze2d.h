#ifndef __map_maze2d_h
#define __map_maze2d_h

#pragma once

#include <array>
#include <unordered_map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "dimension.hpp"

enum edge_in_path_t {
    edge_in_path
};
enum vertex_map_idx_t {
    vertex_map_idx
};
enum vertex_tile_char_t {
    vertex_tile_char
};

namespace boost {
BOOST_INSTALL_PROPERTY(edge, in_path);
BOOST_INSTALL_PROPERTY(vertex, map_idx);
BOOST_INSTALL_PROPERTY(vertex, tile_char);
};

namespace map {
    namespace maze2d {
        using VertexProperties = boost::property<vertex_map_idx_t, size_t, boost::property<vertex_tile_char_t, char>>;
        using EdgeProperties   = boost::property<boost::edge_weight_t, float, boost::property<edge_in_path_t, bool>>;

        using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperties, EdgeProperties>;

        using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;
        using EdgeDescriptor   = boost::graph_traits<Graph>::edge_descriptor;

        using EdgeWeightMap       = boost::property_map<Graph, boost::edge_weight_t>::type;
        using EdgeInPathMap       = boost::property_map<Graph, edge_in_path_t>::type;
        using VertexToMapIdxMap   = boost::property_map<Graph, vertex_map_idx_t>::type;
        using VertexToTileCharMap = boost::property_map<Graph, vertex_tile_char_t>::type;
        using MapIdxToVertexMap   = std::unordered_map<size_t, VertexDescriptor>;

        template <size_t MapX, size_t MapY = MapX>
        struct Map {
            std::array<char, MapX * MapY> map;
            size_t start_idx;
            size_t finish_idx;
            size_t solution_length;
        };

        struct GraphMap {
            Graph               graph;
            EdgeWeightMap       edge_weight_map;
            EdgeInPathMap       edge_in_path_map;
            VertexToTileCharMap vertex_to_tile_char_map;
            VertexToMapIdxMap   vertex_to_map_idx_map;
            MapIdxToVertexMap   map_idx_to_vertex_map;
            VertexDescriptor    start_vertex;
            VertexDescriptor    finish_vertex;
            size_t              solution_length;
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
