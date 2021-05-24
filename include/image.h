#ifndef __image_hpp
#define __image_hpp

#pragma once

#include <cstdint>
#include <string>

namespace image {
    uint8_t writepng(const std::string& filename, uint8_t* data, size_t width, size_t height);
};

#endif // __image_hpp
