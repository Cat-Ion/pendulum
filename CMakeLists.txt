cmake_minimum_required(VERSION 3.7)

project(Pendulum)

find_package(SDL2 REQUIRED)
include_directories(${PROJECT_NAME} ${SDL2_INCLUDE_DIRS} "include" )


add_executable(${PROJECT_NAME} "src/main.cpp" "src/Window.cpp" "include/Window.hpp" "src/Game.cpp" "include/Game.hpp" "src/Physics.cpp" "include/Physics.hpp" "include/RK4.hpp" "include/Controller.hpp")

target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})
