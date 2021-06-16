cmake_minimum_required(VERSION 3.10)

project(Game_AI)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

add_executable(Game_AI
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
    "${PROJECT_SOURCE_DIR}/src/heatmap.cpp"
    "${PROJECT_SOURCE_DIR}/src/image.cpp"
    "${PROJECT_SOURCE_DIR}/src/map/maze2d.cpp"
    "${PROJECT_SOURCE_DIR}/src/aco/acs.cpp"
    "${PROJECT_SOURCE_DIR}/src/aco/acs_mean_filtering.cpp"
    "${PROJECT_SOURCE_DIR}/src/aco/acs_dynamic_exploitation.cpp"
)

target_include_directories(Game_AI
    SYSTEM
    PUBLIC
    "${PROJECT_SOURCE_DIR}/deps"
)

target_include_directories(Game_AI
    PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(Game_AI
    "${PROJECT_SOURCE_DIR}/deps/libheatmap/libheatmap.a"
    "${PROJECT_SOURCE_DIR}/deps/libpng/lib/libpng.a"
    z
)
