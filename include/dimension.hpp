#ifndef __dimension_h
#define __dimension_h

#pragma once

#include <array>
#include <fstream>

namespace dimension {
    constexpr size_t dim_to_padded_dim(size_t dim) {
        return dim + 2;
    }

    constexpr size_t dim2d_to_padded_size(size_t dim) {
        return dim_to_padded_dim(dim) * dim_to_padded_dim(dim);
    }

    constexpr size_t dim2d_to_padded_size(size_t dim_x, size_t dim_y) {
        return dim_to_padded_dim(dim_x) * dim_to_padded_dim(dim_y);
    }

    constexpr size_t dim3d_to_padded_size(size_t dim) {
        return dim_to_padded_dim(dim) * dim_to_padded_dim(dim) * dim_to_padded_dim(dim);
    }

    constexpr size_t dim3d_to_padded_size(size_t dim_x, size_t dim_y, size_t dim_z) {
        return dim_to_padded_dim(dim_x) * dim_to_padded_dim(dim_y) * dim_to_padded_dim(dim_z);
    }
};

#endif // __dimension_h
