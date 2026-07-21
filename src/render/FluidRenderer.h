#pragma once

#include <string>
#include <vector>

#include <GL/glew.h>

#include <glm/glm.hpp>

namespace elem {

// Screen-space fluid rendering, after Green (NVIDIA). Rather than meshing the
// fluid, the particles are rasterised into a view-space depth buffer, that
// depth is bilaterally smoothed into a continuous surface, and the surface is
// shaded with refraction, reflection and thickness-based absorption. The
// individual particles stop being visible as spheres and read as one body.
class FluidRenderer {
public:
    struct Settings {
        // Must exceed the solver's particle spacing (h * 0.6), or the spheres
        // never overlap and the surface reads as loose beads.
        float radius = 0.090f;        // particle radius, world units
        int blurIterations = 2;       // each iteration is one H + one V pass

        // The blur has to be wide enough to span a particle on screen or the
        // body still reads as beads. These spheres cover ~54px, so a 12-tap
        // kernel barely touched them.
        float blurRadius = 24.0f;     // taps either side
        float sigmaSpatial = 11.0f;
        // Must exceed the depth gap between neighbouring particles (~one
        // radius) to fuse them, while staying small enough to keep silhouettes.
        float sigmaDepth = 0.12f;

        float refractScale = 0.045f;
        float absorption = 0.55f;
        glm::vec3 liquidColor{0.42f, 0.70f, 0.92f};
        glm::vec3 lightDirWorld{0.45f, 0.75f, 0.50f};
    };

    bool init(const std::string& assetDir);
    void shutdown();

    void render(const std::vector<glm::vec3>& positions, const glm::mat4& view,
                const glm::mat4& proj, int fbWidth, int fbHeight);

    Settings& settings() { return settings_; }
    const Settings& settings() const { return settings_; }

private:
    bool buildPrograms();
    void ensureTargets(int w, int h);
    void releaseTargets();
    void drawQuad() const;

    Settings settings_;
    std::string assetDir_;

    GLuint progBackground_ = 0;
    GLuint progDepth_ = 0;
    GLuint progThickness_ = 0;
    GLuint progBlur_ = 0;
    GLuint progComposite_ = 0;

    GLuint fboScene_ = 0, fboDepth_ = 0, fboThickness_ = 0;
    GLuint fboBlur_[2] = {0, 0};
    GLuint texScene_ = 0, texDepth_ = 0, texThickness_ = 0;
    GLuint texBlur_[2] = {0, 0};
    GLuint rboDepth_ = 0;

    GLuint vaoParticles_ = 0, vboParticles_ = 0, vaoQuad_ = 0;
    GLsizeiptr vboCapacity_ = 0;

    int width_ = 0, height_ = 0;
};

}  // namespace elem
