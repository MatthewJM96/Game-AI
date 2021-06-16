#ifndef __heatmap_hpp
#define __heatmap_hpp

#pragma once

#include <unordered_map>

#include <libheatmap/heatmap.h>

#include "map/maze2d.h"

namespace heatmap {
    using Heatmaps = std::unordered_map<size_t, heatmap_t*>;
    struct HeatmapData {
        heatmap_t* protoheatmap;
        Heatmaps   heatmaps;
    };

    void create_protoheatmap_from_map(HeatmapData& data, const map::maze2d::Map& map);

    void apply_weighted_point(HeatmapData& data, size_t idx, size_t x_coord, size_t y_coord, float weight);

    namespace impl {
        heatmap_t* get_heatmap(HeatmapData& data, size_t idx);

        heatmap_t* make_new_heatmap(heatmap_t* protoheatmap);

        void overlay_walls(heatmap_t* protoheatmap, heatmap_t* target_heatmap);
    };
};

#endif // __heatmap_hpp
