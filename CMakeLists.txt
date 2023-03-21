project(K-CBS)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(ompl REQUIRED)

include_directories(
    include 
    ${EIGEN3_INCLUDE_DIRS} 
    ${BOOST_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp" "src/Spaces/*.cpp" "src/utils/*.cpp"
    "src/StateValidityCheckers/*.cpp" "src/StatePropogators/*.cpp" 
    "src/Planners/*.cpp" "src/Goals/*.cpp" "src/OptimizationObjectives/*.cpp"
    "src/Mergers/*.cpp" "src/PlanValidityCheckers/*.cpp" "src/Constraints/*.cpp"
    "src/ConstraintValidityCheckers/*.cpp")

add_library (multi-agent-ompl ${SOURCES})

add_executable(K-CBS demos/main.cpp)

target_link_libraries (K-CBS
    multi-agent-ompl
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
)

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")