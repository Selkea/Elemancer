cmake_minimum_required(VERSION 3.20)
project(Elemancer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Tune for the build machine's CPU. On by default so a local dev build gets the
# host's AVX2/FMA (measured ~6% off the settled SPH step on a 14900K, fidelity
# identical -- FMA only tightens rounding). The shareable package build sets this
# OFF (package.sh / CI) so the zip stays portable: -march=native bakes in
# instructions that would crash on an older CPU. Baseline x86-64 (SSE2) otherwise.
option(ELEMANCER_NATIVE "Optimise for the build machine's CPU (-march=native)" ON)
if(ELEMANCER_NATIVE AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
                         CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  add_compile_options(-march=native)
  message(STATUS "Elemancer: -march=native enabled (local build)")
endif()

# Version string baked into the build for the update check: the exact git tag
# when built at one (e.g. "v0.1.0"), else a describe like "v0.1.0-3-gabc1234", or
# the bare commit if there are no tags. Resolved at configure time, so re-run
# cmake after tagging. The updater compares this against the latest GitHub release.
execute_process(
  COMMAND git describe --tags --always --dirty
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE ELEMANCER_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT ELEMANCER_VERSION)
  set(ELEMANCER_VERSION "0.0.0")
endif()
message(STATUS "Elemancer version: ${ELEMANCER_VERSION}")

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

# Dear ImGui (vendored in external/imgui, v1.91.5) for the in-app pause menu and
# settings panel. Compiled from source into a static lib, so it adds no runtime
# DLL and the shareable build is unchanged. The OpenGL3 backend uses its own
# embedded GL loader, which coexists with GLEW.
add_library(imgui STATIC
  external/imgui/imgui.cpp
  external/imgui/imgui_draw.cpp
  external/imgui/imgui_tables.cpp
  external/imgui/imgui_widgets.cpp
  external/imgui/backends/imgui_impl_glfw.cpp
  external/imgui/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
  ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

add_executable(elemancer
  src/main.cpp src/render/FluidRenderer.cpp src/render/Hud.cpp src/update/Updater.cpp)
target_include_directories(elemancer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_link_libraries(elemancer PRIVATE elemancer_sim OpenGL::GL glfw GLEW::GLEW imgui)
if(WIN32)
  target_link_libraries(elemancer PRIVATE winhttp)  # update check / download
endif()
target_compile_definitions(elemancer PRIVATE
  ELEMANCER_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  ELEMANCER_VERSION="${ELEMANCER_VERSION}"
  ELEMANCER_REPO="Selkea/Elemancer")

enable_testing()
add_executable(test_sim tests/test_sim.cpp)
target_link_libraries(test_sim PRIVATE elemancer_sim)
add_test(NAME sim_smoke COMMAND test_sim)
