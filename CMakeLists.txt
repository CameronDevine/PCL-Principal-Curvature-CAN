cmake_minimum_required(VERSION 3.0.0)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include/)

add_library(principal_curvatures_can SHARED src/principle_curvatures_can.cpp)
# target_link_libraries(principal_curvatures_can ${PCL_LIBRARIES})

add_executable(example example/example.cpp)
target_link_libraries(example ${PCL_LIBRARIES} $<TARGET_FILE:principal_curvatures_can>)
add_dependencies(example principal_curvatures_can)
