cmake_minimum_required(VERSION 3.20)
project(fleetline LANGUAGES CXX)

# Python extension build (driven by scikit-build-core from pyproject.toml).
# The C++ core and its unit tests build on their own from core/.

set(FLEETLINE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(core)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core bindings/module.cpp)
target_link_libraries(_core PRIVATE fleetline_core)

install(TARGETS _core LIBRARY DESTINATION fleetline)
