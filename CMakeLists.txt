cmake_minimum_required(VERSION 3.20)
project(Elemancer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(OpenGL REQUIRED)
# Optional: the solver runs 480 steps/s, so the neighbour passes are worth
# parallelising. Without it the pragmas are simply ignored.
find_package(OpenMP)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)

# Solver core: no graphics dependency, so it can run headless under test.
add_library(elemancer_sim STATIC src/sim/Fluid.cpp)
target_include_directories(elemancer_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(elemancer_sim PUBLIC glm::glm)
if(OpenMP_CXX_FOUND)
  target_link_libraries(elemancer_sim PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(elemancer src/main.cpp src/render/FluidRenderer.cpp)
target_link_libraries(elemancer PRIVATE elemancer_sim OpenGL::GL glfw GLEW::GLEW)
target_compile_definitions(elemancer PRIVATE
  ELEMANCER_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

enable_testing()
add_executable(test_sim tests/test_sim.cpp)
target_link_libraries(test_sim PRIVATE elemancer_sim)
add_test(NAME sim_smoke COMMAND test_sim)
