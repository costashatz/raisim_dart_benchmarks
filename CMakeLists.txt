cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(dart_raisim_benchmarks)

find_package(DART 6.8.0 CONFIG REQUIRED COMPONENTS utils-urdf collision-bullet)
find_package(raisim CONFIG REQUIRED)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(anymal_benchmark anymal_benchmark.cpp)
target_link_libraries(anymal_benchmark PUBLIC raisim::raisim dart dart-utils-urdf)
target_include_directories(anymal_benchmark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(anymal_contacts_benchmark anymal_contacts_benchmark.cpp)
target_link_libraries(anymal_contacts_benchmark PUBLIC raisim::raisim dart dart-utils-urdf dart-collision-bullet)
target_include_directories(anymal_contacts_benchmark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(objects_benchmark objects_benchmark.cpp)
target_link_libraries(objects_benchmark PUBLIC raisim::raisim dart dart-utils-urdf dart-collision-bullet)
target_include_directories(objects_benchmark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(iiwa_benchmark iiwa_benchmark.cpp)
target_link_libraries(iiwa_benchmark PUBLIC raisim::raisim dart dart-utils-urdf)
target_include_directories(iiwa_benchmark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
