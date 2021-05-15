#include <fstream>

template <size_t MapX, size_t MapY>
map::maze2d::Map<MapX, MapY> map::maze2d::load_map(std::string map_filepath) {
    Map<MapX, MapY> map;

    std::ifstream map_file(map_filepath);

    size_t row = 0;
    std::string line;
    while (std::getline(map_file, line)) {
        for (size_t col = 0; col < MapX; ++col) {
            size_t idx = row * MapX + col;

            map.map[idx] = line[col];

            if (line[col] == START_TILE) {
                map.start_idx = idx;
            } else if (line[col] == END_TILE) {
                map.finish_idx = idx;
            }
        }

        ++row;
    }

    return std::move(map);
}

template <size_t MapX, size_t MapY>
map::maze2d::Map<
    dimension::dim_to_padded_dim(MapX),
    dimension::dim_to_padded_dim(MapY)
> map::maze2d::load_map_with_halo(std::string map_filepath) {
    const size_t padded_dim_x = dimension::dim_to_padded_dim(MapX);
    const size_t padded_dim_y = dimension::dim_to_padded_dim(MapY);

    Map<
        dimension::dim_to_padded_dim(MapX),
        dimension::dim_to_padded_dim(MapY)
    > halo_map;

    // Initialise first and last row of halo map.
    for (size_t i = 0; i < padded_dim_x; ++i) {
        halo_map.map[i] = '#';
        halo_map.map[padded_dim_x * (padded_dim_y - 1) + i] = '#';
    }

    std::ifstream map_file(map_filepath);

    // For each line in the map file, copy in its contents,
    // padding with a wall tile on each side.
    size_t row = 0;
    std::string line;
    while (std::getline(map_file, line)) {
        halo_map.map[(row + 1) * padded_dim_x] = WALL_TILE;
        halo_map.map[(row + 2) * padded_dim_x - 1] = WALL_TILE;

        // Iterate over the "inner" (non-halo) width - that is,
        // the part we actually load from the map file.
        for (size_t col = 0; col < MapX; ++col) {
            size_t halo_idx = (row + 1) * padded_dim_x + col + 1;

            halo_map.map[halo_idx] = line[col];

            if (line[col] == START_TILE) {
                halo_map.start_idx = halo_idx;
            } else if (line[col] == END_TILE) {
                halo_map.finish_idx = halo_idx;
            }
        }

        ++row;
    }

    return std::move(halo_map);
}

template <size_t MapX, size_t MapY>
void map::maze2d::print_map(const Map<MapX, MapY>& map) {
    for (size_t i = 0; i < MapY; ++i) {
        for (size_t j = 0; j < MapX; ++j) {
            size_t idx = i * MapX + j;

            std::cout << map.map[idx] << " ";
        }
        std::cout << "\n";
    }

    std::cout << std::endl;
}

template <size_t MapX, size_t MapY = MapX, bool MapHasHalo>
map::maze2d::Graph map::maze2d::map_to_graph(Map<MapX, MapY> map) {
    if constexpr (MapHasHalo) {
        return impl::halo_map_to_graph(map);
    } else {
        return impl::non_halo_map_to_graph(map);
    }
}

template <size_t MapX, size_t MapY>
map::maze2d::Graph map::maze2d::impl::halo_map_to_graph(Map<MapX, MapY> map) {
    std::vector<size_t> nodes_visiting;
    nodes_visiting.reserve(8);


}

template <size_t MapX, size_t MapY>
map::maze2d::Graph map::maze2d::impl::non_halo_map_to_graph(Map<MapX, MapY> map) {
    // TODO
}
