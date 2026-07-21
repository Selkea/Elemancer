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

# Override tunables to reproduce a tuning pathology headlessly, and time it
./build/elemancer --shot out.bmp --frames 150 --tension 3 --well 80

# Whip the well back and forth at a peak speed, in world units/s, and report
# the worst stretch reached. Prints WHIP peakMeanRadius / peakStrays.
./build/elemancer --shot out.bmp --sweepspeed 8
```

Rest-packed `meanRadius` is 0.216, so a whip that leaves it at 0.216 with 0%
strays did not deform the body at all. As of now it stays intact through a
peak cursor speed of 5, begins to shed at 6, and tears properly at 8.

| Input | Effect |
| --- | --- |
| Mouse | Moves the gravity well |
| LMB | Pull harder |
| RMB | Repel |
| `[` `]` | Surface tension — how tightly droplets hold together |
| `-` `=` | Well strength |
| `9` `0` | Hold radius — the cursor speed at which the liquid starts to tear |
| `O` `P` | Clarity — `O` toward clear glass, `P` toward deeper colour |
| `K` `L` | Spin — how fast the body rotates about its own centroid |
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

## Techniques and references

- **Screen-space fluid surface** — Green (NVIDIA), "Screen Space Fluid Rendering".
- **Spray** — Ihmsen et al. 2012, "Unified Spray, Foam and Bubbles for
  Particle-Based Fluids". Diffuse particles are a post-process over the SPH
  state with no forces between them, generated from a trapped-air potential
  (relative-velocity based) gated by kinetic energy. Only spray is modelled;
  foam and bubbles need a gravity direction this scene does not have.
- **Refraction / Fresnel** — de Greve 2006, "Reflections and Refractions in
  Ray Tracing": Snell's law in vector form for the background bend, Schlick
  reflectance with R0 = 0.02 for the air/water interface.
- **Environment** — a procedural sky (gradient, drifting fbm clouds, sun) over
  a checkered floor plane with distance fog. Shared by the background pass and
  the liquid's reflections, and gives the clear water detail to lens and mirror
  (`shaders/common_env.glsl`).

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
