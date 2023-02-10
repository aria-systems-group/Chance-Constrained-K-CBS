project(K-CBS)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

# Find Required packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(ompl REQUIRED)

file(GLOB SOURCES "src/*.cpp" "src/Spaces/*.cpp" 
    "src/StateValidityCheckers/*.cpp" "src/StatePropogators/*.cpp" 
    "src/Planners/*.cpp" "src/Goals/*.cpp" "src/OptimizationObjectives/*.cpp"
    "src/Mergers/*.cpp" "src/PlanValidityCheckers/*.cpp" "src/Constraints/*.cpp"
    "src/ConstraintValidityCheckers/*.cpp")
add_executable(K-CBS ${SOURCES})

target_include_directories(K-CBS
    PUBLIC
    include
    ${BOOST_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
)

target_link_libraries (K-CBS
    ${Boost_LIBRARIES}
    ompl 
    Eigen3::Eigen
)
