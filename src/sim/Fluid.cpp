#include "sim/Fluid.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace elem {
namespace {

constexpr float kPi = 3.14159265358979323846f;

glm::vec3 clampLength(const glm::vec3& v, float maxLen) {
    const float l2 = glm::dot(v, v);
    if (l2 > maxLen * maxLen && l2 > 0.0f) {
        return v * (maxLen / std::sqrt(l2));
    }
    return v;
}

bool finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Akinci et al. cohesion spline. It peaks at r = h/2 and returns to zero at
// both ends, so neighbours pull together without piling onto one another the
// way a plain attractive force would.
float cohesionSpline(float r, float h, float coef) {
    if (r <= 0.0f || r >= h) return 0.0f;

    const float hr = h - r;
    const float term = hr * hr * hr * r * r * r;
    if (2.0f * r > h) return coef * term;

    const float h3 = h * h * h;
    return coef * (2.0f * term - (h3 * h3) / 64.0f);
}

}  // namespace

void Fluid::init(int count, std::uint32_t seed) {
    const float spacing = params_.h * 0.60f;
    if (params_.mass <= 0.0f) {
        params_.mass = params_.restDensity * spacing * spacing * spacing;
    }

    pos_.clear();
    pos_.reserve(count);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> jitter(-0.15f, 0.15f);

    // Fill a centred lattice cube until we reach the requested count. Starting
    // near the rest packing keeps the first few steps from erupting.
    const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<float>(count))));
    const float origin = -0.5f * static_cast<float>(side - 1) * spacing;
    for (int z = 0; z < side && static_cast<int>(pos_.size()) < count; ++z) {
        for (int y = 0; y < side && static_cast<int>(pos_.size()) < count; ++y) {
            for (int x = 0; x < side && static_cast<int>(pos_.size()) < count; ++x) {
                glm::vec3 p(origin + static_cast<float>(x) * spacing,
                            origin + static_cast<float>(y) * spacing,
                            origin + static_cast<float>(z) * spacing);
                p += glm::vec3(jitter(rng), jitter(rng), jitter(rng)) * spacing;
                pos_.push_back(p);
            }
        }
    }

    const std::size_t n = pos_.size();
    vel_.assign(n, glm::vec3(0.0f));
    accel_.assign(n, glm::vec3(0.0f));
    density_.assign(n, 0.0f);
    pressure_.assign(n, 0.0f);
    nextInCell_.assign(n, -1);

    rebuildGrid();
}

void Fluid::setBounds(const glm::vec3& half) {
    const glm::vec3 clamped = glm::max(half, glm::vec3(params_.h * 2.0f));
    if (clamped == params_.boundsHalf && !cellHead_.empty()) return;
    params_.boundsHalf = clamped;
    rebuildGrid();
}

void Fluid::rebuildGrid() {
    cellSize_ = params_.h;
    const float pad = params_.h * 2.0f;

    gridMin_ = -(params_.boundsHalf + pad);
    const glm::vec3 extent = 2.0f * (params_.boundsHalf + pad);

    gridDim_ = glm::ivec3(std::max(1, static_cast<int>(std::ceil(extent.x / cellSize_))),
                          std::max(1, static_cast<int>(std::ceil(extent.y / cellSize_))),
                          std::max(1, static_cast<int>(std::ceil(extent.z / cellSize_))));

    cellHead_.assign(static_cast<std::size_t>(gridDim_.x) * gridDim_.y * gridDim_.z, -1);
}

void Fluid::setAttractor(const glm::vec3& p, bool enabled) {
    attractor_ = p;
    attractOn_ = enabled;
}

int Fluid::cellIndex(const glm::vec3& p) const {
    const glm::vec3 rel = (p - gridMin_) / cellSize_;
    const int cx = std::min(std::max(static_cast<int>(std::floor(rel.x)), 0), gridDim_.x - 1);
    const int cy = std::min(std::max(static_cast<int>(std::floor(rel.y)), 0), gridDim_.y - 1);
    const int cz = std::min(std::max(static_cast<int>(std::floor(rel.z)), 0), gridDim_.z - 1);
    return (cz * gridDim_.y + cy) * gridDim_.x + cx;
}

void Fluid::buildGrid() {
    std::fill(cellHead_.begin(), cellHead_.end(), -1);
    for (int i = 0; i < static_cast<int>(pos_.size()); ++i) {
        const int c = cellIndex(pos_[i]);
        nextInCell_[i] = cellHead_[c];
        cellHead_[c] = i;
    }
}

void Fluid::step(float dt) {
    const int n = static_cast<int>(pos_.size());
    if (n == 0) return;

    const FluidParams& P = params_;
    const float h = P.h;
    const float h2 = h * h;
    const float m = P.mass;

    const float poly6Coef = 315.0f / (64.0f * kPi * std::pow(h, 9.0f));
    const float spikyCoef = -45.0f / (kPi * std::pow(h, 6.0f));
    const float viscCoef = 45.0f / (kPi * std::pow(h, 6.0f));
    const float cohesionCoef = 32.0f / (kPi * std::pow(h, 9.0f));
    const float rMin = h * P.antiClumpRadius;

    buildGrid();

    // Pass 1: density, then equation-of-state pressure. Negative pressure is
    // clamped away; letting it go tensile makes particles clump and blow up.
#pragma omp parallel for schedule(static)
    for (int i = 0; i < n; ++i) {
        const glm::vec3 pi = pos_[i];
        const glm::vec3 rel = (pi - gridMin_) / cellSize_;
        const int bx = std::min(std::max(static_cast<int>(std::floor(rel.x)), 0), gridDim_.x - 1);
        const int by = std::min(std::max(static_cast<int>(std::floor(rel.y)), 0), gridDim_.y - 1);
        const int bz = std::min(std::max(static_cast<int>(std::floor(rel.z)), 0), gridDim_.z - 1);

        float rho = 0.0f;
        for (int dz = -1; dz <= 1; ++dz) {
            const int cz = bz + dz;
            if (cz < 0 || cz >= gridDim_.z) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                const int cy = by + dy;
                if (cy < 0 || cy >= gridDim_.y) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int cx = bx + dx;
                    if (cx < 0 || cx >= gridDim_.x) continue;
                    const int cell = (cz * gridDim_.y + cy) * gridDim_.x + cx;
                    for (int j = cellHead_[cell]; j != -1; j = nextInCell_[j]) {
                        const glm::vec3 d = pi - pos_[j];
                        const float r2 = glm::dot(d, d);
                        if (r2 < h2) {
                            const float q = h2 - r2;
                            rho += m * poly6Coef * q * q * q;
                        }
                    }
                }
            }
        }

        density_[i] = std::max(rho, 1e-6f);
        pressure_[i] = std::max(0.0f, P.stiffness * (density_[i] - P.restDensity));
    }

    // Pass 2: pressure, viscosity and cohesion, then body forces.
#pragma omp parallel for schedule(static)
    for (int i = 0; i < n; ++i) {
        const glm::vec3 pi = pos_[i];
        const glm::vec3 vi = vel_[i];
        const float pri = pressure_[i];

        const glm::vec3 rel = (pi - gridMin_) / cellSize_;
        const int bx = std::min(std::max(static_cast<int>(std::floor(rel.x)), 0), gridDim_.x - 1);
        const int by = std::min(std::max(static_cast<int>(std::floor(rel.y)), 0), gridDim_.y - 1);
        const int bz = std::min(std::max(static_cast<int>(std::floor(rel.z)), 0), gridDim_.z - 1);

        glm::vec3 fPress(0.0f);
        glm::vec3 fVisc(0.0f);
        glm::vec3 aCohesion(0.0f);

        for (int dz = -1; dz <= 1; ++dz) {
            const int cz = bz + dz;
            if (cz < 0 || cz >= gridDim_.z) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                const int cy = by + dy;
                if (cy < 0 || cy >= gridDim_.y) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int cx = bx + dx;
                    if (cx < 0 || cx >= gridDim_.x) continue;
                    const int cell = (cz * gridDim_.y + cy) * gridDim_.x + cx;
                    for (int j = cellHead_[cell]; j != -1; j = nextInCell_[j]) {
                        if (j == i) continue;
                        const glm::vec3 d = pi - pos_[j];
                        const float r2 = glm::dot(d, d);
                        if (r2 >= h2) continue;

                        const float r = std::sqrt(r2);
                        const float rhoj = density_[j];

                        if (r > 1e-6f) {
                            // grad W_spiky points from j toward i; spikyCoef is
                            // negative, so the net pressure term is repulsive.
                            const float gradMag = spikyCoef * (h - r) * (h - r);
                            fPress += -m * (pri + pressure_[j]) / (2.0f * rhoj) * gradMag * (d / r);

                            // Acceleration directly: F/m_i = gamma * m_j * C.
                            aCohesion += -P.surfaceTension * m *
                                         cohesionSpline(r, h, cohesionCoef) * (d / r);

                            // Separation floor. Ramp is capped so a pair that
                            // starts coincident cannot produce a huge impulse.
                            if (r < rMin) {
                                const float push = std::min(rMin / r - 1.0f, 8.0f);
                                aCohesion += P.antiClump * push * (d / r);
                            }
                        }
                        fVisc += P.viscosity * m * (vel_[j] - vi) / rhoj * (viscCoef * (h - r));
                    }
                }
            }
        }

        glm::vec3 a = (fPress + fVisc) / density_[i] + aCohesion + P.gravity;

        if (attractOn_) {
            const glm::vec3 d = attractor_ - pi;
            const float soft = P.attractSoft * P.attractSoft;
            const float inv = 1.0f / std::pow(glm::dot(d, d) + soft, 1.5f);
            a += P.attractG * d * inv;
        }

        accel_[i] = clampLength(a, P.maxAccel);
    }

    // Integrate (semi-implicit Euler) and resolve the domain walls.
    const float damp = std::max(0.0f, 1.0f - P.drag * dt);
    for (int i = 0; i < n; ++i) {
        vel_[i] = clampLength((vel_[i] + accel_[i] * dt) * damp, P.maxSpeed);
        pos_[i] += vel_[i] * dt;

        if (!finite(pos_[i]) || !finite(vel_[i])) {
            pos_[i] = glm::vec3(0.0f);
            vel_[i] = glm::vec3(0.0f);
            continue;
        }

        for (int k = 0; k < 3; ++k) {
            const float b = P.boundsHalf[k];
            if (pos_[i][k] < -b) {
                pos_[i][k] = -b;
                if (vel_[i][k] < 0.0f) vel_[i][k] *= -P.restitution;
            } else if (pos_[i][k] > b) {
                pos_[i][k] = b;
                if (vel_[i][k] > 0.0f) vel_[i][k] *= -P.restitution;
            }
        }
    }
}

}  // namespace elem
