#include "heatmap.h"

#include <cstring>

#include "constants.h"

static float SQUARE_STAMP_DATA[] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

static heatmap_stamp_t SQUARE_STAMP = {
    SQUARE_STAMP_DATA, 9, 9
};

void heatmap::create_protoheatmap_from_map(HeatmapData& data, const map::maze2d::Map& map) {
    data.protoheatmap = heatmap_new(10 * map.dims.x, 10 * map.dims.y);

    for (size_t i = 0; i < map.dims.x; ++i) {
        for (size_t j = 0; j < map.dims.y; ++j) {
            size_t idx = j * map.dims.x + i;
            if (map.map[idx] == WALL_TILE) {
                heatmap_add_weighted_point_with_stamp(data.protoheatmap, 10 * i, 10 * j, 9999.0f, &SQUARE_STAMP);
            }
        }
    }
}


void heatmap::apply_weighted_point(HeatmapData& data, size_t idx, size_t x_coord, size_t y_coord, float weight) {
    heatmap_t* target_heatmap = impl::get_heatmap(data, idx);

    heatmap_add_weighted_point(target_heatmap, x_coord, y_coord, weight);
}

heatmap_t* heatmap::impl::get_heatmap(HeatmapData& data, size_t idx) {
    heatmap_t* obtained_heatmap = nullptr;

    try {
        obtained_heatmap = data.heatmaps.at(idx);
    } catch (std::exception&) {
        obtained_heatmap = make_new_heatmap(data.protoheatmap);

        data.heatmaps[idx] = obtained_heatmap;
    }

    return obtained_heatmap;
}

void heatmap::impl::overlay_walls(heatmap_t* protoheatmap, heatmap_t* target_heatmap) {
    for (size_t i = 0; i < protoheatmap->w; ++i) {
        for (size_t j = 0; j < protoheatmap->h; ++j) {
            size_t idx = j * protoheatmap->w + i;

            if (protoheatmap->buf[idx] != 0.0f)
                target_heatmap->buf[idx] = target_heatmap->max;
        }
    }
}

heatmap_t* heatmap::impl::make_new_heatmap(heatmap_t* protoheatmap) {
    heatmap_t* new_heatmap = heatmap_new(protoheatmap->w, protoheatmap->h);

    return new_heatmap;
}
