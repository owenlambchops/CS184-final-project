cmake_minimum_required(VERSION 3.20)
project(WaterDropletSimulatorSkeleton LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_library(wd_core
    src/core/scene.cpp
    src/surface/plane_surface.cpp
    src/surface/heightfield_surface.cpp
    src/forces/iforce_field.cpp
    src/sim/droplet_template.cpp
    src/sim/droplet.cpp
    src/sim/droplet_factory.cpp
    src/sim/operators.cpp
    src/sim/single_droplet_solver.cpp
    src/sim/merge_split_controller.cpp
    src/sim/simulation_system.cpp
    src/render/gpu_mesh_buffer.cpp
    src/render/droplet_gpu_cache.cpp
    src/render/refractive_renderer.cpp
    src/interaction/input_router.cpp
    src/interaction/picker.cpp
    src/interaction/drag_interactor.cpp
    src/ui/ui_controller.cpp
    src/experiments/experiment_logger.cpp
    src/app/app.cpp
)

target_include_directories(wd_core PUBLIC include)
target_link_libraries(wd_core PUBLIC Eigen3::Eigen)

target_compile_options(wd_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(wd_skeleton src/main.cpp)
target_link_libraries(wd_skeleton PRIVATE wd_core)
