// Elemancer - a cohesive liquid that gathers around a cursor gravity well.
//
//   elemancer                       interactive
//   elemancer --shot out.bmp        headless capture, for self-verification

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "render/FluidRenderer.h"
#include "render/Hud.h"
#include "sim/Fluid.h"

namespace {

constexpr int kWidth = 1280;
constexpr int kHeight = 800;

// dt is bounded by the CFL condition, ~0.4 * h / sqrt(stiffness). At h = 0.044
// this leaves headroom; 1/640 over ten substeps advances ~1/64 s of simulation
// per frame and keeps the same feel as the finer scale.
constexpr int kSubSteps = 10;
constexpr float kDt = 1.0f / 640.0f;

constexpr float kFovDegrees = 45.0f;
constexpr float kCamDistance = 5.0f;
constexpr float kDepthHalf = 0.7f;

// Accumulated mouse-wheel movement, drained each frame to dolly the camera.
// GLFW scroll only arrives through a callback, so it lands here.
double g_scrollY = 0.0;
void scrollCallback(GLFWwindow*, double, double yOffset) { g_scrollY += yOffset; }

// 24-bit BMP. glReadPixels hands back bottom-up rows and BMP stores bottom-up,
// so the rows go out in the order they arrive.
bool saveBMP(const std::string& path, int w, int h, const std::vector<unsigned char>& rgb) {
    const int rowRaw = w * 3;
    const int pad = (4 - (rowRaw % 4)) % 4;
    const int rowSize = rowRaw + pad;
    const int dataSize = rowSize * h;

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    unsigned char hdr[54] = {0};
    hdr[0] = 'B';
    hdr[1] = 'M';
    const auto put32 = [&](int off, int v) {
        hdr[off] = v & 0xFF;
        hdr[off + 1] = (v >> 8) & 0xFF;
        hdr[off + 2] = (v >> 16) & 0xFF;
        hdr[off + 3] = (v >> 24) & 0xFF;
    };
    const auto put16 = [&](int off, int v) {
        hdr[off] = v & 0xFF;
        hdr[off + 1] = (v >> 8) & 0xFF;
    };
    put32(2, 54 + dataSize);
    put32(10, 54);
    put32(14, 40);
    put32(18, w);
    put32(22, h);
    put16(26, 1);
    put16(28, 24);
    put32(34, dataSize);
    f.write(reinterpret_cast<char*>(hdr), 54);

    std::vector<unsigned char> row(rowSize, 0);
    for (int y = 0; y < h; ++y) {
        const unsigned char* src = rgb.data() + static_cast<std::size_t>(y) * rowRaw;
        for (int x = 0; x < w; ++x) {
            row[x * 3 + 0] = src[x * 3 + 2];  // BGR
            row[x * 3 + 1] = src[x * 3 + 1];
            row[x * 3 + 2] = src[x * 3 + 0];
        }
        f.write(reinterpret_cast<char*>(row.data()), rowSize);
    }
    return true;
}

// The domain tracks the window: half-extents are the view frustum measured at
// the near face of the tank, so the liquid is walled in by exactly what is on
// screen regardless of how the window is resized.
glm::vec3 boundsForView(float aspect, float camDistance) {
    const float halfH =
        (camDistance - kDepthHalf) * std::tan(glm::radians(kFovDegrees) * 0.5f);
    return glm::vec3(halfH * aspect * 0.97f, halfH * 0.97f, kDepthHalf);
}

glm::mat4 viewFor(float camDistance) {
    return glm::lookAt(glm::vec3(0.0f, 0.0f, camDistance), glm::vec3(0.0f),
                       glm::vec3(0.0f, 1.0f, 0.0f));
}

// Unproject the cursor onto the z=0 plane, which is where the well lives.
glm::vec3 cursorOnPlane(GLFWwindow* win, const glm::mat4& view, const glm::mat4& proj) {
    double mx = 0.0, my = 0.0;
    glfwGetCursorPos(win, &mx, &my);
    int fw = 1, fh = 1;
    glfwGetFramebufferSize(win, &fw, &fh);

    const float ndcX = static_cast<float>(2.0 * mx / fw - 1.0);
    const float ndcY = static_cast<float>(1.0 - 2.0 * my / fh);

    const glm::mat4 inv = glm::inverse(proj * view);
    glm::vec4 pNear = inv * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 pFar = inv * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    pNear /= pNear.w;
    pFar /= pFar.w;

    const glm::vec3 o(pNear);
    const glm::vec3 d = glm::normalize(glm::vec3(pFar) - o);
    if (std::fabs(d.z) < 1e-6f) return glm::vec3(0.0f);
    return o + d * (-o.z / d.z);
}

// The live-tunable values, persisted between interactive sessions so a setting
// dialled in by feel survives a restart. Deliberately not loaded in --shot
// mode, which stays on the compiled defaults so headless verification is
// reproducible. The file is a plain "key value" list, hand-editable.
void saveConfig(const std::string& path, const elem::FluidParams& p, float clarity) {
    std::ofstream f(path, std::ios::trunc);
    if (!f) {
        std::fprintf(stderr, "[elemancer] could not write %s\n", path.c_str());
        return;
    }
    f << "tension " << p.surfaceTension << "\n"
      << "well " << p.wellStiffness << "\n"
      << "viscosity " << p.viscosity << "\n"
      << "drag " << p.drag << "\n"
      << "holdRadius " << p.wellHoldRadius << "\n"
      << "clarity " << clarity << "\n"
      << "spin " << p.spinRate << "\n";
}

bool loadConfig(const std::string& path, elem::FluidParams& p, float& clarity) {
    std::ifstream f(path);
    if (!f) return false;

    std::string key;
    float v = 0.0f;
    while (f >> key >> v) {
        if (key == "tension") p.surfaceTension = v;
        else if (key == "well") p.wellStiffness = v;
        else if (key == "viscosity") p.viscosity = v;
        else if (key == "drag") p.drag = v;
        else if (key == "holdRadius") p.wellHoldRadius = v;
        else if (key == "clarity") clarity = v;
        else if (key == "spin") p.spinRate = v;
    }
    return true;
}

std::vector<std::string> hudControlLines() {
    return {
        "ELEMANCER",
        "Mouse: move well    LMB: pull    RMB: repel    Scroll: zoom",
        "[ ] tension     - = well     , . viscosity     ; ' drag",
        "9 0 hold radius     O P clarity     K L spin",
        "G gravity     R reset     S save     Tab hide     Esc quit",
    };
}

// Hold-to-scale a tunable. Exponential so a value stays in the same relative
// ballpark whichever end of its range it is at.
void holdAdjust(GLFWwindow* win, int keyDown, int keyUp, float& value, float rate, float dt,
                float lo, float hi) {
    if (glfwGetKey(win, keyDown) == GLFW_PRESS) value *= std::exp(-rate * dt);
    if (glfwGetKey(win, keyUp) == GLFW_PRESS) value *= std::exp(rate * dt);
    value = std::clamp(value, lo, hi);
}

struct KeyEdge {
    bool down = false;
    bool justPressed(GLFWwindow* win, int key) {
        const bool now = glfwGetKey(win, key) == GLFW_PRESS;
        const bool fired = now && !down;
        down = now;
        return fired;
    }
};

}  // namespace

int main(int argc, char** argv) {
    bool shotMode = false;
    std::string shotPath = "elemancer_shot.bmp";
    int particleCount = 6000;
    int shotFrames = 420;
    float tensionOverride = -1.0f;
    float wellOverride = -1.0f;
    float sweepSpeed = 4.0f;  // world units/s the shot drags the well at
    bool drawHudShot = false;
    float distOverride = -1.0f;  // camera distance for the shot; scroll does this live
    bool measureJitter = false;  // report frame-to-frame pixel diff of a moving body
    bool noTemporal = false;     // disable temporal surface smoothing, for A/B
    float spinOverride = -1.0f;     // spin rate for the shot
    float clarityOverride = -1.0f;  // absorption for the shot, to match a live look
    float holdOverride = -1.0f;
    float viscOverride = -1.0f;
    float hOverride = -1.0f;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shot") {
            shotMode = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') shotPath = argv[++i];
        } else if (a == "--particles" && i + 1 < argc) {
            particleCount = std::atoi(argv[++i]);
        } else if (a == "--frames" && i + 1 < argc) {
            shotFrames = std::atoi(argv[++i]);
        } else if (a == "--tension" && i + 1 < argc) {
            tensionOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--well" && i + 1 < argc) {
            wellOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--sweepspeed" && i + 1 < argc) {
            sweepSpeed = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--hud") {
            drawHudShot = true;
        } else if (a == "--dist" && i + 1 < argc) {
            distOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--jitter") {
            measureJitter = true;
        } else if (a == "--notemporal") {
            noTemporal = true;
        } else if (a == "--spin" && i + 1 < argc) {
            spinOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--clarity" && i + 1 < argc) {
            clarityOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--hold" && i + 1 < argc) {
            holdOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--visc" && i + 1 < argc) {
            viscOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--h" && i + 1 < argc) {
            hOverride = static_cast<float>(std::atof(argv[++i]));
        }
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "[elemancer] glfwInit failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    if (shotMode) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    GLFWwindow* win = glfwCreateWindow(kWidth, kHeight, "Elemancer", nullptr, nullptr);
    if (!win) {
        std::fprintf(stderr, "[elemancer] window creation failed\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(win);

    glewExperimental = GL_TRUE;
    const GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK) {
        std::fprintf(stderr, "[elemancer] glewInit failed: %s\n", glewGetErrorString(glewErr));
        return 1;
    }
    glGetError();  // GLEW's loader leaves a benign INVALID_ENUM behind.
    glfwSwapInterval(1);
    glfwSetScrollCallback(win, scrollCallback);

    std::printf("[elemancer] GL %s | %s\n",
                reinterpret_cast<const char*>(glGetString(GL_VERSION)),
                reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

    elem::FluidRenderer renderer;
    if (!renderer.init(ELEMANCER_ASSET_DIR)) {
        std::fprintf(stderr, "[elemancer] renderer init failed\n");
        return 1;
    }
    if (noTemporal) renderer.settings().temporalBlend = 0.0f;
    if (clarityOverride >= 0.0f) renderer.settings().absorption = clarityOverride;

    elem::Fluid fluid;
    if (hOverride > 0.0f) fluid.params().h = hOverride;
    fluid.init(particleCount);
    if (tensionOverride > 0.0f) fluid.params().surfaceTension = tensionOverride;
    if (wellOverride > 0.0f) fluid.params().wellStiffness = wellOverride;
    if (spinOverride >= 0.0f) fluid.params().spinRate = spinOverride;
    if (holdOverride >= 0.0f) fluid.params().wellHoldRadius = holdOverride;
    if (viscOverride >= 0.0f) fluid.params().viscosity = viscOverride;
    std::printf("[elemancer] particles=%zu tension=%.3f well=%.1f\n", fluid.size(),
                fluid.params().surfaceTension, fluid.params().wellStiffness);

    const float shotDist = distOverride > 0.0f ? distOverride : kCamDistance;
    const glm::mat4 view = viewFor(shotDist);

    const auto projFor = [](int fbw, int fbh) {
        return glm::perspective(glm::radians(kFovDegrees),
                                static_cast<float>(fbw) / static_cast<float>(fbh), 0.05f, 100.0f);
    };

    if (shotMode) {
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        fluid.setBounds(
            boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh), shotDist));

        // Settle, then whip the well back and forth along x at a known peak
        // speed. It has to be a reversing path, not a circle: on a circular
        // sweep the body simply orbits inside the circle and cuts the corner,
        // so the lag is bounded by the sweep radius and no speed can ever
        // stretch it far enough to tear. Reversing lets the lag grow with
        // speed without bound, which is what actually happens when a cursor
        // is flicked.
        const float kWhipAmplitude = 1.8f;
        const float frameDur = static_cast<float>(kSubSteps) * kDt;

        const int settle = shotFrames / 2;
        const int sweep = shotFrames - settle;

        glm::vec3 wellPos(0.0f);
        fluid.setAttractor(wellPos, true);
        for (int i = 0; i < settle; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }

        // Sampling one instant makes the result depend on where in the
        // oscillation the capture lands, so track the worst stretch reached
        // across the whole sweep instead.
        const auto measure = [&fluid]() {
            const float n = static_cast<float>(fluid.size());
            glm::vec3 c(0.0f);
            for (const glm::vec3& p : fluid.positions()) c += p;
            c /= n;
            float sp = 0.0f;
            for (const glm::vec3& p : fluid.positions()) sp += glm::length(p - c);
            sp /= n;
            int stray = 0;
            for (const glm::vec3& p : fluid.positions()) {
                if (glm::length(p - c) > 2.5f * sp) ++stray;
            }
            return std::pair<float, float>{sp, 100.0f * stray / n};
        };

        float peakSpread = 0.0f;
        float peakStrays = 0.0f;
        // Peaks of the two quantities that gate spray generation, so the
        // thresholds can be set against measured ranges instead of guessed.
        float peakTrappedAir = 0.0f;
        float peakEk = 0.0f;
        std::size_t peakSpray = 0;

        const float omega = sweepSpeed / kWhipAmplitude;  // peak speed = A * omega
        for (int i = 0; i < sweep; ++i) {
            const float t = static_cast<float>(i) * frameDur;
            wellPos = glm::vec3(kWhipAmplitude * std::sin(omega * t), 0.0f, 0.0f);
            fluid.setAttractor(wellPos, true);
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);

            const auto [sp, st] = measure();
            peakSpread = std::max(peakSpread, sp);
            peakStrays = std::max(peakStrays, st);

            peakSpray = std::max(peakSpray, fluid.sprayCount());
            for (float ta : fluid.trappedAir()) peakTrappedAir = std::max(peakTrappedAir, ta);
            for (const glm::vec3& v : fluid.velocities()) {
                peakEk = std::max(peakEk, 0.5f * fluid.params().mass * glm::dot(v, v));
            }
        }
        std::printf("WHIP peakSpeed=%.1f peakMeanRadius=%.3f peakStrays=%.1f%%"
                    " peakSpray=%zu peakTrappedAir=%.2f peakEk=%.3f\n",
                    sweepSpeed, peakSpread, peakStrays, peakSpray, peakTrappedAir, peakEk);

        renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                        projFor(fbw, fbh), fbw, fbh, 6.0f);
        if (drawHudShot) {
            elem::Hud hud;
            hud.init();
            hud.draw(hudControlLines(), fbw, fbh);
            hud.shutdown();
        }
        glFinish();

        std::vector<unsigned char> pixels(static_cast<std::size_t>(fbw) * fbh * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        glm::vec3 centroid(0.0f);
        for (const glm::vec3& p : fluid.positions()) centroid += p;
        centroid /= static_cast<float>(fluid.size());
        float spread = 0.0f;
        for (const glm::vec3& p : fluid.positions()) spread += glm::length(p - centroid);
        spread /= static_cast<float>(fluid.size());

        // Measured angular velocity about the centroid, projected on the spin
        // axis. Least-squares fit of a rigid rotation to the relative
        // velocities: omega = (sum r x v_rel) / (sum |r|^2). A still frame
        // cannot show spin, so this is how the shot proves the body is turning.
        glm::vec3 velMean(0.0f);
        for (const glm::vec3& v : fluid.velocities()) velMean += v;
        velMean /= static_cast<float>(fluid.size());
        glm::vec3 angMom(0.0f);
        float inertia = 0.0f;
        for (std::size_t i = 0; i < fluid.size(); ++i) {
            const glm::vec3 r = fluid.positions()[i] - centroid;
            angMom += glm::cross(r, fluid.velocities()[i] - velMean);
            inertia += glm::dot(r, r);
        }
        const glm::vec3 bodyOmega = inertia > 0.0f ? angMom / inertia : glm::vec3(0.0f);
        const float omegaAxial = glm::dot(bodyOmega, glm::normalize(fluid.params().spinAxis));

        // Fraction of the body still travelling with the bulk: anything more
        // than a body-radius away from the centroid has torn off.
        int strays = 0;
        for (const glm::vec3& p : fluid.positions()) {
            if (glm::length(p - centroid) > 2.5f * spread) ++strays;
        }

        const bool ok = saveBMP(shotPath, fbw, fbh, pixels);
        std::printf("SHOT file=%s %dx%d sweepSpeed=%.1f centroid=(%.3f, %.3f, %.3f)"
                    " meanRadius=%.3f wellLag=%.3f strays=%.1f%% omegaAxial=%.2f saved=%d\n",
                    shotPath.c_str(), fbw, fbh, sweepSpeed, centroid.x, centroid.y, centroid.z,
                    spread, glm::length(centroid - wellPos),
                    100.0f * strays / static_cast<float>(fluid.size()), omegaAxial, ok ? 1 : 0);

        // Advance one frame and re-render, then report how much the image
        // changed. A boiling surface makes consecutive frames differ a lot even
        // when the body is only turning gently, so this is a headless proxy for
        // "smooth while rotating".
        if (measureJitter) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            projFor(fbw, fbh), fbw, fbh, 6.0f);
            glFinish();
            std::vector<unsigned char> pixels2(pixels.size());
            glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, pixels2.data());

            double sum = 0.0;
            for (std::size_t i = 0; i < pixels.size(); ++i) {
                sum += std::abs(static_cast<int>(pixels[i]) - static_cast<int>(pixels2[i]));
            }
            std::printf("JITTER temporal=%d meanAbsDiff=%.4f\n", noTemporal ? 0 : 1,
                        sum / static_cast<double>(pixels.size()));
        }

        renderer.shutdown();
        glfwDestroyWindow(win);
        glfwTerminate();
        return ok ? 0 : 1;
    }

    // Restore any values tuned in a previous session, then keep them saved.
    const std::string cfgPath = std::string(ELEMANCER_ASSET_DIR) + "/elemancer.cfg";
    if (loadConfig(cfgPath, fluid.params(), renderer.settings().absorption)) {
        std::printf("[elemancer] loaded settings from elemancer.cfg\n");
    }

    std::printf("[elemancer] mouse moves the well | LMB pull | RMB repel | scroll zoom\n");
    std::printf("[elemancer] [ ] tension | - = well | , . viscosity | ; ' drag\n");
    std::printf("[elemancer] 9 0 hold radius | O P clarity | K L spin\n");
    std::printf("[elemancer] G gravity | R reset | S save | Tab hide HUD | Esc quit\n");

    elem::Hud hud;
    if (!hud.init()) std::fprintf(stderr, "[elemancer] hud init failed\n");
    bool showHud = true;
    const std::vector<std::string> hudLines = hudControlLines();

    KeyEdge gravityKey, resetKey, saveKey, hudKey;

    // Camera dolly, driven by the scroll wheel.
    float camDistance = kCamDistance;

    auto last = std::chrono::high_resolution_clock::now();
    double titleTimer = 0.0;
    double fpsAccum = 0.0;
    int fpsFrames = 0;
    float fps = 0.0f;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) break;

        const auto now = std::chrono::high_resolution_clock::now();
        const float frameDt =
            std::min(0.1f, std::chrono::duration<float>(now - last).count());
        last = now;

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) continue;

        // Scroll to dolly the camera. Multiplicative so the zoom feels even at
        // any distance; scroll up zooms in.
        if (g_scrollY != 0.0) {
            camDistance = std::clamp(
                camDistance * std::exp(static_cast<float>(-g_scrollY) * 0.12f), 1.8f, 24.0f);
            g_scrollY = 0.0;
        }
        const glm::mat4 view = viewFor(camDistance);

        elem::FluidParams& P = fluid.params();
        holdAdjust(win, GLFW_KEY_LEFT_BRACKET, GLFW_KEY_RIGHT_BRACKET, P.surfaceTension, 1.5f,
                   frameDt, 0.005f, 2.5f);
        holdAdjust(win, GLFW_KEY_MINUS, GLFW_KEY_EQUAL, P.wellStiffness, 1.5f, frameDt, 3.0f,
                   200.0f);
        holdAdjust(win, GLFW_KEY_COMMA, GLFW_KEY_PERIOD, P.viscosity, 1.5f, frameDt, 0.1f, 60.0f);
        holdAdjust(win, GLFW_KEY_SEMICOLON, GLFW_KEY_APOSTROPHE, P.drag, 1.5f, frameDt, 0.01f,
                   5.0f);
        // Hold radius is the breakup threshold: raise it and the liquid stays
        // together through faster flicks.
        holdAdjust(win, GLFW_KEY_9, GLFW_KEY_0, P.wellHoldRadius, 1.5f, frameDt, 0.2f, 6.0f);
        // Clarity: absorption tints and darkens the light through the liquid,
        // so O clears it toward glass and P deepens the colour.
        holdAdjust(win, GLFW_KEY_O, GLFW_KEY_P, renderer.settings().absorption, 1.5f, frameDt,
                   0.02f, 2.5f);
        // Linear so it can reach exactly zero (exponential scaling never can).
        if (glfwGetKey(win, GLFW_KEY_K) == GLFW_PRESS) P.spinRate -= 5.0f * frameDt;
        if (glfwGetKey(win, GLFW_KEY_L) == GLFW_PRESS) P.spinRate += 5.0f * frameDt;
        P.spinRate = std::clamp(P.spinRate, 0.0f, 12.0f);

        if (resetKey.justPressed(win, GLFW_KEY_R)) fluid.init(particleCount);
        if (gravityKey.justPressed(win, GLFW_KEY_G)) {
            P.gravity = (P.gravity.y < -0.1f) ? glm::vec3(0.0f) : glm::vec3(0.0f, -4.0f, 0.0f);
        }
        if (hudKey.justPressed(win, GLFW_KEY_TAB)) showHud = !showHud;
        if (saveKey.justPressed(win, GLFW_KEY_S)) {
            saveConfig(cfgPath, P, renderer.settings().absorption);
            std::printf("[elemancer] saved settings to elemancer.cfg\n");
        }

        const glm::mat4 proj = projFor(fbw, fbh);
        fluid.setBounds(
            boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh), camDistance));
        fluid.setAttractor(cursorOnPlane(win, view, proj), true);

        fluid.setWellScale(
            glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ? -1.0f
            : glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ? 3.0f
                                                                            : 1.0f);
        for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);

        renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view, proj,
                        fbw, fbh, static_cast<float>(glfwGetTime()));
        if (showHud) hud.draw(hudLines, fbw, fbh);
        glfwSwapBuffers(win);

        fpsAccum += frameDt;
        if (++fpsFrames >= 30) {
            fps = static_cast<float>(fpsFrames / fpsAccum);
            fpsAccum = 0.0;
            fpsFrames = 0;
        }

        titleTimer += frameDt;
        if (titleTimer > 0.2) {
            titleTimer = 0.0;
            char title[320];
            std::snprintf(title, sizeof title,
                          "Elemancer  |  tension %.3f  |  well %.1f  |  visc %.1f  |  drag %.2f"
                          "  |  hold %.2f  |  clarity %.2f  |  spin %.1f"
                          "  |  %zu drops  |  %zu spray  |  %.0f fps",
                          P.surfaceTension, P.wellStiffness, P.viscosity, P.drag, P.wellHoldRadius,
                          renderer.settings().absorption, P.spinRate, fluid.size(),
                          fluid.sprayCount(), fps);
            glfwSetWindowTitle(win, title);
        }
    }

    // Persist whatever was tuned this session.
    saveConfig(cfgPath, fluid.params(), renderer.settings().absorption);
    std::printf("[elemancer] saved settings to elemancer.cfg\n");

    hud.shutdown();
    renderer.shutdown();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
