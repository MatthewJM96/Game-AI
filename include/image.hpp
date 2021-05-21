#ifndef __image_hpp
#define __image_hpp

#pragma once

#include <cerrno>
#include <cstring>
#include <iostream>

#include <libpng/png.h>

namespace image {
    uint8_t writepng(const std::string& filename, uint8_t* data, size_t width, size_t height) {
        png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
        if (!png_ptr) {
            std::cerr << "Error initialising libpng write struct." << std::endl;
            return 2;
        }

        png_infop info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr) {
            std::cerr << "Error initialising libpng info struct." << std::endl;
            png_destroy_write_struct(&png_ptr, (png_infopp)nullptr);
            return 3;
        }

        if (setjmp(png_jmpbuf(png_ptr))) {
            std::cerr << "Error in setjmp!?" << std::endl;
            png_destroy_write_struct(&png_ptr, &info_ptr);
            return 4;
        }

        FILE *fp = fopen(filename.c_str(), "wb");
        if (!fp) {
            std::cerr << "Error writing " << filename << ": " << strerror(errno) << std::endl;
            png_destroy_write_struct(&png_ptr, &info_ptr);
            return 5;
        }
        png_init_io(png_ptr, fp);

        // Turn on or off filtering, and/or choose specific filters.
        png_set_filter(png_ptr, 0, PNG_FILTER_NONE  | PNG_FILTER_VALUE_NONE);
        //                             | PNG_FILTER_SUB   | PNG_FILTER_VALUE_SUB
        //                             | PNG_FILTER_UP    | PNG_FILTER_VALUE_UP
        //                             | PNG_FILTER_AVE   | PNG_FILTER_VALUE_AVE
        //                             | PNG_FILTER_PAETH | PNG_FILTER_VALUE_PAETH
        //                             | PNG_ALL_FILTERS

        // Set the zlib compression level.
        // 1 = fast but not much compression, 9 = slow but much compression.
        png_set_compression_level(png_ptr, 1);

        static const int bit_depth = 8;
        static const int color_type = PNG_COLOR_TYPE_RGB_ALPHA;
        static const int interlace_type = PNG_INTERLACE_ADAM7; // or PNG_INTERLACE_NONE
        png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, interlace_type, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

        png_bytep* row_pointers = (png_byte**)png_malloc(png_ptr, height * sizeof(png_bytep));
        for(size_t y = 0; y < height; ++y) {
            row_pointers[y] = const_cast<png_bytep>(data + y * width * 4);
        }
        png_set_rows(png_ptr, info_ptr, row_pointers);

        png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, nullptr);

        // Cleanup
        png_free(png_ptr, row_pointers);
        fclose(fp);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return 0;
    }
};

#endif // __image_hpp
