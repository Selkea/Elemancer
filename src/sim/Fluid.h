#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace elem {

// Tunables for the SPH solver. The defaults are aimed at a cohesive blob that
// gathers around a cursor gravity well inside a cube of half-extent boundHalf.
struct FluidParams {
    float h           = 0.14f;    // smoothing radius
    float restDensity = 1000.0f;
    float stiffness   = 180.0f;   // pressure gas constant
    float viscosity   = 5.0f;
    float mass        = 0.0f;     // 0 => derived from the initial packing
    float drag        = 0.35f;    // linear velocity damping, per second
    float boundHalf   = 1.2f;     // half-extent of the cubic domain

    // Akinci-style cohesion. This is what makes the body behave like a liquid
    // rather than a cloud: it rounds the surface, lets it neck and pinch, and
    // pulls stray droplets back in.
    //
    // It has to be strong. Pressure is clamped non-negative, so an expanded
    // body has no restoring force of its own, and cohesion falls off with
    // distance -- too little and expansion runs away into a diffuse cloud.
    float surfaceTension = 400.0f;
    float restitution = 0.30f;    // wall bounce
    float maxSpeed    = 12.0f;    // clamps keep a stiff solver from exploding
    float maxAccel    = 3000.0f;

    glm::vec3 gravity = glm::vec3(0.0f);  // world gravity, off by default

    // Cursor gravity well: a softened inverse-square pull toward the attractor.
    //
    // Plummer softening, and the length matters enormously. It must be on the
    // order of the body's own radius: any smaller and the well becomes a
    // singularity buried inside the liquid that slingshots particles straight
    // out through the far side. At this length the well is harmonic within the
    // body and only falls off as inverse-square outside it.
    float attractG    = 8.0f;
    float attractSoft = 0.45f;
};

// Muller-2003 SPH with a uniform grid for neighbour lookup. Deliberately free
// of any graphics dependency so the solver can be exercised headlessly.
class Fluid {
public:
    void init(int count, std::uint32_t seed = 1337u);
    void step(float dt);

    void setAttractor(const glm::vec3& p, bool enabled);
    const glm::vec3& attractor() const { return attractor_; }

    std::size_t size() const { return pos_.size(); }
    const std::vector<glm::vec3>& positions() const { return pos_; }
    const std::vector<glm::vec3>& velocities() const { return vel_; }
    const std::vector<float>& densities() const { return density_; }

    const FluidParams& params() const { return params_; }
    FluidParams& params() { return params_; }

private:
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
