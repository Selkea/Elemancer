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
        // Must exceed the solver's particle spacing (h * 0.6 = 0.03), or the
        // spheres never overlap and the surface reads as loose beads. At 1.5x
        // spacing a lone particle renders as a single water droplet.
        float radius = 0.045f;        // particle radius, world units
        int blurIterations = 2;       // each iteration is one H + one V pass

        // The blur has to be wide enough to span a particle on screen, but no
        // wider or separate droplets smear together. These spheres cover
        // ~17px, so the kernel is sized to match.
        float blurRadius = 10.0f;     // taps either side
        float sigmaSpatial = 4.5f;
        // Must exceed the depth gap between neighbouring particles (~one
        // radius) to fuse them, while staying small enough to keep silhouettes.
        float sigmaDepth = 0.05f;

        float refractScale = 0.045f;

        // How coloured the liquid is. absorption tints the light passing
        // through it (Beer-Lambert, absorbing 1 - liquidColor, i.e. mostly
        // red), and scatter adds the liquid's own colour back out of thick
        // regions. Both were high enough that a small droplet read as blue
        // jelly rather than near-clear water; kept low so colour only shows
        // where the body is genuinely thick.
        float absorption = 0.15f;
        float scatter = 0.05f;
        glm::vec3 liquidColor{0.86f, 0.91f, 0.94f};  // barely-there aqua
        glm::vec3 lightDirWorld{0.45f, 0.75f, 0.50f};

        // Diffuse spray, drawn additively over the surface.
        float sprayRadius = 0.011f;
        float sprayIntensity = 0.55f;
        float sprayLifeMax = 1.20f;  // must match DiffuseParams::lifeMax
        glm::vec3 sprayColor{0.86f, 0.94f, 1.0f};
    };

    bool init(const std::string& assetDir);
    void shutdown();

    void render(const std::vector<glm::vec3>& positions,
                const std::vector<glm::vec3>& sprayPositions,
                const std::vector<float>& sprayLife, const glm::mat4& view,
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
    GLuint progSpray_ = 0;

    GLuint fboScene_ = 0, fboDepth_ = 0, fboThickness_ = 0;
    GLuint fboBlur_[2] = {0, 0};
    GLuint texScene_ = 0, texDepth_ = 0, texThickness_ = 0;
    GLuint texBlur_[2] = {0, 0};
    GLuint rboDepth_ = 0;

    GLuint vaoParticles_ = 0, vboParticles_ = 0, vaoQuad_ = 0;
    GLuint vaoSpray_ = 0, vboSpray_ = 0;
    GLsizeiptr vboCapacity_ = 0;
    GLsizeiptr sprayCapacity_ = 0;
    std::vector<glm::vec4> sprayScratch_;  // xyz position, w life

    int width_ = 0, height_ = 0;
};

}  // namespace elem
