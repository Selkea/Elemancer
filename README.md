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
| `[` `]` | Surface tension — how tightly droplets hold together |
| `-` `=` | Well strength |
| `,` `.` | Viscosity |
| `;` `'` | Drag |
| `G` | Toggle world gravity |
| `R` | Reset |
| `Esc` | Quit |

The tuning keys scale their value while held, and the current values are shown
in the title bar so a setting that feels right can be read straight off and
made the default in `FluidParams`.

The domain tracks the window: its half-extents are the view frustum measured at
the near face of the tank, so the liquid is walled in by exactly what is on
screen, at any window size.

## Running from VS Code

`Ctrl+Shift+B` builds. `F5` builds and launches under gdb. Other targets are
under **Terminal > Run Task**: `Elemancer: Run`, `Elemancer: Test (headless)`,
`Elemancer: Shot (headless capture)`.

Every task prepends `C:\msys64\ucrt64\bin` to `PATH` — the `glfw3`, `glew32`
and `libstdc++` DLLs live there and the executable will not start without it.

Note that VS Code only reads `.vscode/` from the **workspace root**. The copies
here apply when this folder is opened directly; opening a parent folder as the
workspace needs the same files at that parent's root.

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
