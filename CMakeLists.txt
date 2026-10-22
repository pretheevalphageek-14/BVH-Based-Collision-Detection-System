cmake_minimum_required(VERSION 3.20)
project(bvh_physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(SDL3 REQUIRED CONFIG)

add_executable(bvh_physics
    src/main.cpp
    src/collision.cpp
    src/physics.cpp
)

target_link_libraries(bvh_physics PRIVATE SDL3::SDL3)

target_include_directories(bvh_physics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)

enable_testing()

add_executable(test_physics
    tests/test_physics.cpp
    src/collision.cpp
    src/physics.cpp
)
target_include_directories(test_physics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_test(NAME test_physics COMMAND test_physics)

add_executable(test_correctness
    tests/test_correctness.cpp
    src/collision.cpp
    src/physics.cpp
)
target_include_directories(test_correctness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_test(NAME test_correctness COMMAND test_correctness)