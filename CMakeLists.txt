project(K-CBS)

cmake_minimum_required (VERSION 3.23)
set (CMAKE_CXX_STANDARD 20)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

include_directories("include")
file(GLOB SOURCES "src/*.cpp" "src/Spaces/*.cpp" 
    "src/StateValidityCheckers/*.cpp" "src/StatePropogators/*.cpp" 
    "src/Planners/*.cpp" "src/Goals/*.cpp" "src/OptimizationObjectives/*.cpp"
    "src/Mergers/*.cpp" "src/PlanValidityCheckers/*.cpp" "src/Constraints/*.cpp"
    "src/ConstraintValidityCheckers/*.cpp")
add_executable(K-CBS ${SOURCES})

# Find Required packages
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

if (UNIX)
    # os specific library importing
    add_library(ompl SHARED IMPORTED)
    set_target_properties(ompl PROPERTIES
      IMPORTED_LOCATION "/opt/homebrew/lib/libompl.dylib"
      INTERFACE_INCLUDE_DIRECTORIES "/opt/homebrew/include/ompl-1.6"
    )
endif (UNIX)

target_link_libraries(K-CBS ${Boost_LIBRARIES} ompl Eigen3::Eigen)
