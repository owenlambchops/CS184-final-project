cmake_minimum_required(VERSION 3.10)
project(PhysicsSim CXX)

# Require C++17 as used in our simulation code (e.g., for structured bindings)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find the libmesh-config executable in your system PATH
find_program(LIBMESH_CONFIG libmesh-config)

if(NOT LIBMESH_CONFIG)
    message(FATAL_ERROR "libmesh-config not found! Please ensure libMesh is installed and added to your PATH.")
endif()

# 2. Extract flags and libraries using libmesh-config
execute_process(COMMAND ${LIBMESH_CONFIG} --cxxflags OUTPUT_VARIABLE LIBMESH_CXXFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${LIBMESH_CONFIG} --include OUTPUT_VARIABLE LIBMESH_INCLUDE OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${LIBMESH_CONFIG} --ldflags OUTPUT_VARIABLE LIBMESH_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${LIBMESH_CONFIG} --libs OUTPUT_VARIABLE LIBMESH_LIBS OUTPUT_STRIP_TRAILING_WHITESPACE)

# 3. Append libMesh includes and CXX flags to the global CMake flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBMESH_CXXFLAGS} ${LIBMESH_INCLUDE}")

# 4. Create the executable from your Canvas code
add_executable(physics_sim main.cpp)

# 5. Convert the space-separated output from libmesh-config into CMake lists
separate_arguments(LIBMESH_LDFLAGS_LIST UNIX_COMMAND ${LIBMESH_LDFLAGS})
separate_arguments(LIBMESH_LIBS_LIST UNIX_COMMAND ${LIBMESH_LIBS})

# 6. Link the executable against libMesh
target_link_libraries(physics_sim PRIVATE ${LIBMESH_LDFLAGS_LIST} ${LIBMESH_LIBS_LIST})