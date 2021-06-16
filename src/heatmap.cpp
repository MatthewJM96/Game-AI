#include "heatmap.h"

#include <cstring>

#include "constants.h"
#include "image.h"

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

void heatmap::create_protoheatmap_from_map(HeatmapData& heatmap_data, const map::maze2d::Map& map) {
    heatmap_data.protoheatmap = heatmap_new(10 * map.dims.x, 10 * map.dims.y);

    for (size_t i = 0; i < map.dims.x; ++i) {
        for (size_t j = 0; j < map.dims.y; ++j) {
            size_t idx = j * map.dims.x + i;
            if (map.map[idx] == WALL_TILE) {
                heatmap_add_weighted_point_with_stamp(heatmap_data.protoheatmap, 10 * i, 10 * j, 1.0f, &SQUARE_STAMP);
            }
        }
    }
}

void heatmap::apply_point(HeatmapData& heatmap_data, size_t idx, size_t x_coord, size_t y_coord) {
    heatmap_t* target_heatmap = impl::get_heatmap(heatmap_data, idx);

    heatmap_add_point(target_heatmap, x_coord, y_coord);
}

void heatmap::apply_weighted_point(HeatmapData& heatmap_data, size_t idx, size_t x_coord, size_t y_coord, float weight) {
    heatmap_t* target_heatmap = impl::get_heatmap(heatmap_data, idx);

    heatmap_add_weighted_point(target_heatmap, x_coord, y_coord, weight);
}

void heatmap::print_heatmaps(HeatmapData& heatmap_data, std::string output_dir, std::string tag) {
    uint8_t* image_buffer = new uint8_t[heatmap_data.protoheatmap->w * heatmap_data.protoheatmap->h * 4];

    float saturation = 0.0f;

    size_t     idx;
    heatmap_t* target_heatmap;

    // Get saturation point.
    for (auto& heatmap : heatmap_data.heatmaps) {
        std::tie(idx, target_heatmap) = heatmap;

        if (saturation < target_heatmap->max)
            saturation = target_heatmap->max;
    }

    // Add wals to heatmaps & print to pngs.
    for (auto& heatmap : heatmap_data.heatmaps) {
        std::tie(idx, target_heatmap) = heatmap;

        impl::overlay_walls(heatmap_data.protoheatmap, target_heatmap, saturation);

        heatmap_render_saturated_to(target_heatmap, heatmap_cs_default, saturation, image_buffer);

        std::string numeric_code = std::to_string(idx);
        if (numeric_code.size() < 10)
            numeric_code.insert(0, 10 - numeric_code.size(), '0');

        image::writepng(
            output_dir + "/" + tag + "." + numeric_code + ".png",
            image_buffer,
            heatmap_data.protoheatmap->w,
            heatmap_data.protoheatmap->h
        );
    }

    delete[] image_buffer;
}

void heatmap::free_heatmaps(HeatmapData& heatmap_data) {
    size_t     idx;
    heatmap_t* target_heatmap;
    for (auto& heatmap : heatmap_data.heatmaps) {
        std::tie(idx, target_heatmap) = heatmap;

        heatmap_free(target_heatmap);
    }

    heatmap_free(heatmap_data.protoheatmap);
}

heatmap_t* heatmap::impl::get_heatmap(HeatmapData& heatmap_data, size_t idx) {
    heatmap_t* obtained_heatmap = nullptr;

    try {
        obtained_heatmap = heatmap_data.heatmaps.at(idx);
    } catch (std::exception&) {
        obtained_heatmap = make_new_heatmap(heatmap_data.protoheatmap);

        heatmap_data.heatmaps[idx] = obtained_heatmap;
    }

    return obtained_heatmap;
}

void heatmap::impl::overlay_walls(heatmap_t* protoheatmap, heatmap_t* target_heatmap, float saturation) {
    for (size_t i = 0; i < protoheatmap->w; ++i) {
        for (size_t j = 0; j < protoheatmap->h; ++j) {
            size_t idx = j * protoheatmap->w + i;

            if (protoheatmap->buf[idx] != 0.0f)
                target_heatmap->buf[idx] = saturation;
        }
    }
}

heatmap_t* heatmap::impl::make_new_heatmap(heatmap_t* protoheatmap) {
    heatmap_t* new_heatmap = heatmap_new(protoheatmap->w, protoheatmap->h);

    return new_heatmap;
}
