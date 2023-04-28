cmake_minimum_required(VERSION 3.5)
project(sea_glider)

set(CMAKE_BUILD_TYPE Release)

include_directories(include)

# Add the executable
add_executable(glider_sbmpo src/main.cpp)
target_include_directories(glider_sbmpo PUBLIC include)
target_link_libraries(glider_sbmpo PUBLIC sbmpo stdc++ stdc++fs m)