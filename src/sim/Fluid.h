#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace elem {

// Tunables for the SPH solver. Defaults are aimed at droplet-scale liquid
// gathering around a cursor gravity well.
struct FluidParams {
    float h           = 0.050f;   // smoothing radius; particle spacing is h * 0.6
    float restDensity = 1000.0f;

    // Stiffness sets the speed of sound, which sets the CFL limit on dt.
    // c ~ sqrt(stiffness), and dt must stay under ~0.4 * h / c. Raising this
    // without shrinking dt makes the solver explode.
    float stiffness   = 60.0f;

    float viscosity   = 5.0f;
    float mass        = 0.0f;     // 0 => derived from the initial packing
    float drag        = 0.35f;    // linear velocity damping, per second
    float restitution = 0.30f;    // wall bounce
    float maxSpeed    = 12.0f;    // clamps keep a stiff solver from exploding
    float maxAccel    = 3000.0f;

    // Half-extents of the domain, per axis. Driven by the window so the liquid
    // is bounded by what you can actually see.
    glm::vec3 boundsHalf{2.6f, 1.6f, 0.7f};

    glm::vec3 gravity = glm::vec3(0.0f);  // world gravity, off by default

    // Akinci cohesion, expressed as an acceleration (gamma * m_j * C), so the
    // value is independent of h and particle spacing. Getting this wrong by
    // dividing through density instead of mass makes it silently scale with
    // spacing^3, so a change of scale quietly destroys surface tension.
    float surfaceTension = 0.45f;

    // Hard floor on particle separation.
    //
    // Both restoring terms vanish as r -> 0: the spiky pressure gradient is
    // finite at zero, and the cohesion spline carries an r^3 factor. So once
    // cohesion and the well together beat peak pressure repulsion, nothing
    // stops the body collapsing to a point. Measured at tension 3 / well 80:
    // 4000 particles crushed to meanRadius 0.027, about 500x rest density.
    // That is fatal twice over -- the liquid freezes into a dot, and at that
    // spacing every particle is a neighbour of every other, so the uniform
    // grid degenerates to O(n^2) and the frame rate falls off a cliff.
    //
    // This term diverges as r -> 0, which bounds density, which bounds cell
    // occupancy, which bounds the neighbour cost. It also makes the liquid
    // behave far more incompressibly, which water does anyway.
    float antiClump = 600.0f;
    float antiClumpRadius = 0.5f;  // as a fraction of h; rest spacing is 0.6h

    // Cursor gravity well, with Plummer softening. The softening length must
    // be on the order of the body radius: any smaller and the well becomes a
    // singularity buried inside the liquid that slingshots particles out
    // through the far side.
    // Inverse-square falls off hard across a window-sized domain: a droplet
    // 2.5 units out feels only ~1.2 u/s^2 at G=8, so strays take far too long
    // to come home. Hence the stronger default.
    float attractG    = 12.0f;
    float attractSoft = 0.30f;
};

// Muller-2003 SPH with a uniform grid for neighbour lookup. Deliberately free
// of any graphics dependency so the solver can be exercised headlessly.
class Fluid {
public:
    void init(int count, std::uint32_t seed = 1337u);
    void step(float dt);

    // Resizes the domain and rebuilds the grid. Safe to call every frame.
    void setBounds(const glm::vec3& half);

    void setAttractor(const glm::vec3& p, bool enabled);
    const glm::vec3& attractor() const { return attractor_; }

    std::size_t size() const { return pos_.size(); }
    const std::vector<glm::vec3>& positions() const { return pos_; }
    const std::vector<glm::vec3>& velocities() const { return vel_; }
    const std::vector<float>& densities() const { return density_; }

    const FluidParams& params() const { return params_; }
    FluidParams& params() { return params_; }

private:
    void rebuildGrid();
    void buildGrid();
    int cellIndex(const glm::vec3& p) const;

    FluidParams params_;

    std::vector<glm::vec3> pos_, vel_, accel_;
    std::vector<float> density_, pressure_;

    // Uniform grid stored as one intrusive linked list per cell.
    std::vector<int> cellHead_;
    std::vector<int> nextInCell_;
    glm::vec3 gridMin_{0.0f};
    glm::ivec3 gridDim_{1};
    float cellSize_ = 1.0f;

    glm::vec3 attractor_{0.0f};
    bool attractOn_ = false;
};

}  // namespace elem
