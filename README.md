# Elemancer

A cohesive liquid that gathers around your cursor. The cursor is a gravity
well; the fluid is a real SPH simulation, so it is held together by its own
pressure and viscosity rather than by any scripted motion.

## Build

Requires MSYS2 UCRT64 (`g++`, `cmake`, `ninja`) plus `glfw`, `glew`, `glm`:

```sh
pacman -S --needed mingw-w64-ucrt-x86_64-{glfw,glew,glm}
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run

```sh
./build/elemancer                    # interactive
./build/elemancer --shot out.bmp     # headless capture, for verification
./build/test_sim                     # headless solver check, prints PASS/FAIL
```

| Input | Effect |
| --- | --- |
| Mouse | Moves the gravity well |
| LMB | Pull harder |
| RMB | Repel |
| `G` | Toggle world gravity |
| `R` | Reset |
| `Esc` | Quit |

## Layout

- `src/sim/` — SPH solver. No graphics dependency, so it runs headless in tests.
- `src/main.cpp` — window, input, and the particle draw pass.
- `shaders/` — loaded from disk at runtime; edit and rerun without recompiling.
- `tests/` — headless solver checks.

## Verifying a change

`test_sim` asserts the solver stays finite, stays in bounds, holds near rest
density, and that the body actually migrates toward the well. `--shot` renders
one frame headlessly to a BMP and prints the blob centroid and mean radius, so
a visual change can be checked without a display.
