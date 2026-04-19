cmake_minimum_required(VERSION 3.20)
project(WaterDropletSimulatorSkeleton LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Prefer the project's local vcpkg installation tree when it exists, so the
# repo can configure without passing the toolchain file every time.
if(NOT DEFINED VCPKG_TARGET_TRIPLET)
    if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
        set(VCPKG_TARGET_TRIPLET arm64-osx CACHE STRING "vcpkg target triplet")
    elseif(APPLE)
        set(VCPKG_TARGET_TRIPLET x64-osx CACHE STRING "vcpkg target triplet")
    endif()
endif()

if(DEFINED VCPKG_TARGET_TRIPLET)
    set(WD_LOCAL_VCPKG_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}")
    if(EXISTS "${WD_LOCAL_VCPKG_PREFIX}")
        list(PREPEND CMAKE_PREFIX_PATH "${WD_LOCAL_VCPKG_PREFIX}")
        message(STATUS "Using local vcpkg packages from ${WD_LOCAL_VCPKG_PREFIX}")
    endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(libigl CONFIG REQUIRED)

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
target_link_libraries(wd_core PUBLIC
    Eigen3::Eigen
    glad::glad
    glfw
    imgui::imgui
    igl::igl_core
    igl::igl_opengl
    igl::igl_glfw
    igl::igl_imgui
)

target_compile_options(wd_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(wd_skeleton src/main.cpp)
target_link_libraries(wd_skeleton PRIVATE wd_core)
