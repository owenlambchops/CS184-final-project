cmake_minimum_required(VERSION 3.15)
project(droplet_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Force Homebrew's Boost, bypassing Anaconda's outdated version ─────────────
set(BOOST_ROOT "/opt/homebrew")
set(Boost_NO_SYSTEM_PATHS OFF)

# ── Prevent CMake from picking up Anaconda headers/libs accidentally ──────────
list(REMOVE_ITEM CMAKE_PREFIX_PATH "/opt/anaconda3")
list(REMOVE_ITEM CMAKE_SYSTEM_PREFIX_PATH "/opt/anaconda3")

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(CGAL   REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# ── Extension module (bindings.cpp #includes water_sim_basic.cpp) ─────────────
pybind11_add_module(droplet_cpp bindings.cpp)

target_link_libraries(droplet_cpp PRIVATE CGAL::CGAL)

# ── Make sure Homebrew headers come before Anaconda headers ───────────────────
target_include_directories(droplet_cpp BEFORE PRIVATE
    "/opt/homebrew/include"
)

# ── Explicitly exclude Anaconda's Boost from the include search ───────────────
target_compile_options(droplet_cpp PRIVATE
    -isystem /opt/homebrew/include
)

set_target_properties(droplet_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})