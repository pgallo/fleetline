cmake_minimum_required(VERSION 3.20)
project(fleetline_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(FLEETLINE_BUILD_TESTS "Build the C++ unit tests" ON)

find_package(Threads REQUIRED)

add_library(fleetline_core STATIC
    src/geo.cpp
    src/filter.cpp
    src/geofence.cpp
    src/trips.cpp
    src/eta.cpp
    src/stream.cpp
)
target_include_directories(fleetline_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(fleetline_core PUBLIC Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(fleetline_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(FLEETLINE_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_executable(fleetline_tests
        tests/test_geo.cpp
        tests/test_filter.cpp
        tests/test_geofence.cpp
        tests/test_trips.cpp
        tests/test_eta.cpp
        tests/test_stream.cpp
    )
    target_link_libraries(fleetline_tests PRIVATE fleetline_core GTest::gtest GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(fleetline_tests)
endif()
