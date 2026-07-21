#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
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
    // Must stay above any cursor speed we expect to track, or this clamp
    // becomes the thing that causes breakup instead of the well cap.
    float maxSpeed    = 30.0f;
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

    // Cursor well: a saturating spring, not an inverse-square attractor.
    //
    // Inverse-square is backwards for cursor-following. Its pull weakens with
    // distance, so a particle that falls behind can never catch up and the
    // body shears apart at any speed. A spring pulls stragglers proportionally
    // harder, so the liquid tracks the cursor as a single body.
    //
    // But a pure spring is a converging field: it only ever compacts the body,
    // so on its own it never breaks apart, at any speed. Clamping the
    // magnitude does not help either -- that makes the force uniform across
    // the body, and a uniform body force (gravity on a falling drop) tears
    // nothing.
    //
    // So the pull holds like a spring within wellHoldRadius and decays as 1/r
    // beyond it. Ordinary cursor speeds keep the whole body inside that
    // radius and it tracks as one. Move fast enough that the tail lags past
    // it and the tail is pulled progressively less, falls further behind, and
    // detaches. The hold radius is therefore the speed threshold: the body
    // lags roughly wellDamping/wellStiffness * cursorSpeed.
    // Split into two terms, because following and sloshing otherwise fight
    // over one constant. A per-particle spring strong enough to track the
    // cursor is also a strong confining field: it squeezes the liquid into a
    // rigid ball that never deforms.
    //
    // wellStiffness acts on the centroid and is applied uniformly to every
    // particle, so it translates the body without squeezing it and the shape
    // stays free to slosh. wellLocal is the much weaker per-particle pull that
    // gathers strays, and it is the one carrying the hold-radius falloff, so
    // it is what decides when the tail tears away.
    // The local term has to carry most of the pull. Bulk force is uniform, so
    // it produces no differential across the body: too much of it and the
    // liquid translates as a rigid ball that never deforms, for the same
    // reason a drop falling under gravity does not deform. Bulk is here only
    // to guarantee the body keeps up; local is what makes it slosh and tear.
    float wellStiffness = 22.0f;   // bulk, on the centroid
    float wellLocal = 34.0f;       // per-particle
    float wellHoldRadius = 1.1f;
    float wellMaxAccel = 400.0f;   // safety clamp only; should not shape behaviour

    // Damps the body's bulk drift so it settles onto the cursor instead of
    // orbiting it. Uses the mean velocity rather than each particle's own, so
    // it never damps internal motion and the slosh survives.
    float wellDamping   = 8.0f;

    // Imposed spin about the body's own centroid. Each particle is driven
    // toward the rigid-rotation velocity field w x r, firmly enough to keep
    // turning against drag but gently enough that the liquid still sloshes and
    // deforms rather than turning as a solid. The bulk-relative velocity is
    // used, so this rotation is invisible to wellDamping and does not fight
    // the body following the cursor.
    float spinRate     = 2.6f;                    // rad/s
    float spinStrength = 3.0f;                    // 1/s, drive toward that field
    glm::vec3 spinAxis = glm::vec3(0.18f, 1.0f, 0.12f);  // normalised in use
};

// Diffuse material, after Ihmsen et al. 2012, "Unified Spray, Foam and Bubbles
// for Particle-Based Fluids". Generated as a post-process over the SPH state
// with no forces between diffuse particles, so large counts stay cheap.
//
// Only the spray class is modelled. Foam and bubbles both need a gravity
// direction to mean anything -- foam floats, bubbles rise -- and this scene
// has no world gravity by default.
struct DiffuseParams {
    bool enabled = true;

    // Trapped air is estimated from *relative* velocity rather than curl:
    // curl misses head-on impacts, which are exactly where air gets dragged in.
    float trappedAirMin = 2.0f;
    float trappedAirMax = 20.0f;

    // Kinetic energy gates the whole thing, so spray appears when the liquid
    // is actually being thrown around and not while it sits on the cursor.
    // Particle mass is ~0.027 at h=0.05, so Ek = 0.0135 v^2. These thresholds
    // put the onset near v=4 and saturation near v=17.
    float kineticMin = 0.25f;
    float kineticMax = 4.0f;

    // Measured: at the tear (~speed 8) trapped air peaks near 87 and Ek near
    // 3.5, but that is a brief instant over few particles, so the rate has to
    // be high to put visible spray on screen.
    float spawnRate = 900.0f;      // max samples per fluid particle per second
    float lifeMin = 0.35f;
    float lifeMax = 1.20f;
    float drag = 0.8f;
    int maxCount = 60000;

    // A spray particle back inside the body has rejoined it, so it is removed
    // rather than drawn over the surface.
    int reabsorbNeighbours = 14;
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

    // Scales the well. Negative repels.
    void setWellScale(float s) { wellScale_ = s; }

    std::size_t size() const { return pos_.size(); }
    const std::vector<glm::vec3>& positions() const { return pos_; }
    const std::vector<glm::vec3>& velocities() const { return vel_; }
    const std::vector<float>& densities() const { return density_; }

    const FluidParams& params() const { return params_; }
    FluidParams& params() { return params_; }

    DiffuseParams& diffuse() { return diffuse_; }
    const DiffuseParams& diffuse() const { return diffuse_; }

    // Spray, as position + remaining life. Life is exposed so the renderer can
    // fade a droplet out rather than popping it.
    const std::vector<glm::vec3>& sprayPositions() const { return sprayPos_; }
    const std::vector<float>& sprayLife() const { return sprayLife_; }
    const std::vector<float>& trappedAir() const { return trappedAir_; }
    std::size_t sprayCount() const { return sprayPos_.size(); }

private:
    void rebuildGrid();
    void buildGrid();
    int cellIndex(const glm::vec3& p) const;
    int countNeighbours(const glm::vec3& p) const;
    glm::vec3 wellAccel(const glm::vec3& p, const glm::vec3& velBulk,
                        const glm::vec3& posBulk) const;
    void stepDiffuse(float dt, const glm::vec3& velBulk, const glm::vec3& posBulk);

    FluidParams params_;
    DiffuseParams diffuse_;

    std::vector<glm::vec3> pos_, vel_, accel_;
    std::vector<float> density_, pressure_;
    std::vector<float> trappedAir_;

    std::vector<glm::vec3> sprayPos_, sprayVel_;
    std::vector<float> sprayLife_;
    std::mt19937 rng_{9271u};

    // Uniform grid stored as one intrusive linked list per cell.
    std::vector<int> cellHead_;
    std::vector<int> nextInCell_;
    glm::vec3 gridMin_{0.0f};
    glm::ivec3 gridDim_{1};
    float cellSize_ = 1.0f;

    glm::vec3 attractor_{0.0f};
    bool attractOn_ = false;
    float wellScale_ = 1.0f;
};

}  // namespace elem
