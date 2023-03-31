cmake_minimum_required(VERSION 3.5)
project(my_project)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include)

# Add the executable
add_executable(my_executable src/main.cpp)
target_include_directories(my_executable PUBLIC include)
target_link_libraries(my_executable PUBLIC sbmpo sbmpo_models stdc++ stdc++fs m)