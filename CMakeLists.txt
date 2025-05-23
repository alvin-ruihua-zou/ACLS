cmake_minimum_required(VERSION 3.16)

# Set the project name
project(ACLS)

# Enable C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)
include_directories(include/planners) # Add the planners folder to the include paths
include_directories(include/utils) # Add the utils folder to the include paths
include_directories(include/domains) # Add the domains folder to the include paths
include_directories(include/heap) # Add the heap folder to the include paths


# Add source files
file(GLOB DOMAIN_SOURCES src/domains/*.cpp)
file(GLOB PLANNER_SOURCES src/planners/*.cpp)
file(GLOB UTILS_SOURCES src/utils/*.cpp)
set(SOURCES ${DOMAIN_SOURCES} ${PLANNER_SOURCES} ${UTILS_SOURCES})

# Add header files
file(GLOB DOMAIN_HEADERS include/domains/*.h)
file(GLOB PLANNER_HEADERS include/planners/*.h)
file(GLOB UTILS_HEADERS include/utils/*.h)
file(GLOB HEAP_HEADERS include/heap/*.h include/heap/details/*.hpp) # Include heap headers
set(HEADERS ${DOMAIN_HEADERS} ${PLANNER_HEADERS} ${UTILS_HEADERS} ${HEAP_HEADERS})

# Add executables for tests
file(GLOB TEST_SOURCES src/tests/*.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE} ${SOURCES} ${HEADERS})
endforeach()