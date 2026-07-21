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
#include <vector>

#include "render/FluidRenderer.h"
#include "sim/Fluid.h"

namespace {

constexpr int kWidth = 1280;
constexpr int kHeight = 800;

// dt is bounded by the CFL condition, ~0.4 * h / sqrt(stiffness). At h = 0.05
// and stiffness 60 that is about 1/390, so 1/480 leaves headroom. Eight
// substeps then advances 1/60 s of simulation per frame.
constexpr int kSubSteps = 8;
constexpr float kDt = 1.0f / 480.0f;

constexpr float kFovDegrees = 45.0f;
constexpr float kCamDistance = 5.0f;
constexpr float kDepthHalf = 0.7f;

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
glm::vec3 boundsForView(float aspect) {
    const float halfH =
        (kCamDistance - kDepthHalf) * std::tan(glm::radians(kFovDegrees) * 0.5f);
    return glm::vec3(halfH * aspect * 0.97f, halfH * 0.97f, kDepthHalf);
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
    int particleCount = 4000;
    int shotFrames = 420;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shot") {
            shotMode = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') shotPath = argv[++i];
        } else if (a == "--particles" && i + 1 < argc) {
            particleCount = std::atoi(argv[++i]);
        } else if (a == "--frames" && i + 1 < argc) {
            shotFrames = std::atoi(argv[++i]);
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

    std::printf("[elemancer] GL %s | %s\n",
                reinterpret_cast<const char*>(glGetString(GL_VERSION)),
                reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

    elem::FluidRenderer renderer;
    if (!renderer.init(ELEMANCER_ASSET_DIR)) {
        std::fprintf(stderr, "[elemancer] renderer init failed\n");
        return 1;
    }

    elem::Fluid fluid;
    fluid.init(particleCount);
    std::printf("[elemancer] particles=%zu\n", fluid.size());

    const glm::vec3 eye(0.0f, 0.0f, kCamDistance);
    const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    const auto projFor = [](int fbw, int fbh) {
        return glm::perspective(glm::radians(kFovDegrees),
                                static_cast<float>(fbw) / static_cast<float>(fbh), 0.05f, 100.0f);
    };

    if (shotMode) {
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        fluid.setBounds(boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh)));

        // Settle, sweep the well in an arc, then snap it back to the centre and
        // capture while the body is still rushing in. A settled blob only
        // proves it is round; one caught mid-motion proves it is liquid,
        // because it lags, stretches and recoils.
        const int settle = shotFrames * 45 / 100;
        const int sweep = shotFrames * 40 / 100;
        const int recover = shotFrames - settle - sweep;

        fluid.setAttractor(glm::vec3(0.0f), true);
        for (int i = 0; i < settle; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }
        for (int i = 0; i < sweep; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(sweep);
            const float angle = t * 6.2831853f * 0.9f;
            fluid.setAttractor(glm::vec3(0.8f * std::cos(angle), 0.5f * std::sin(angle), 0.0f),
                               true);
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }
        fluid.setAttractor(glm::vec3(0.0f), true);
        for (int i = 0; i < recover; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }

        renderer.render(fluid.positions(), view, projFor(fbw, fbh), fbw, fbh);
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

        const bool ok = saveBMP(shotPath, fbw, fbh, pixels);
        std::printf("SHOT file=%s %dx%d centroid=(%.3f, %.3f, %.3f) meanRadius=%.3f"
                    " wellLag=%.3f saved=%d\n",
                    shotPath.c_str(), fbw, fbh, centroid.x, centroid.y, centroid.z, spread,
                    glm::length(centroid), ok ? 1 : 0);

        renderer.shutdown();
        glfwDestroyWindow(win);
        glfwTerminate();
        return ok ? 0 : 1;
    }

    std::printf("[elemancer] mouse moves the well | LMB pull | RMB repel\n");
    std::printf("[elemancer] [ ] tension | - = well | , . viscosity | ; ' drag\n");
    std::printf("[elemancer] G gravity | R reset | Esc quit\n");

    const float baseG = fluid.params().attractG;
    KeyEdge gravityKey, resetKey;

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

        elem::FluidParams& P = fluid.params();
        holdAdjust(win, GLFW_KEY_LEFT_BRACKET, GLFW_KEY_RIGHT_BRACKET, P.surfaceTension, 1.5f,
                   frameDt, 0.005f, 5.0f);
        holdAdjust(win, GLFW_KEY_MINUS, GLFW_KEY_EQUAL, P.attractG, 1.5f, frameDt, 0.2f, 80.0f);
        holdAdjust(win, GLFW_KEY_COMMA, GLFW_KEY_PERIOD, P.viscosity, 1.5f, frameDt, 0.1f, 60.0f);
        holdAdjust(win, GLFW_KEY_SEMICOLON, GLFW_KEY_APOSTROPHE, P.drag, 1.5f, frameDt, 0.01f,
                   5.0f);

        if (resetKey.justPressed(win, GLFW_KEY_R)) fluid.init(particleCount);
        if (gravityKey.justPressed(win, GLFW_KEY_G)) {
            P.gravity = (P.gravity.y < -0.1f) ? glm::vec3(0.0f) : glm::vec3(0.0f, -4.0f, 0.0f);
        }

        const glm::mat4 proj = projFor(fbw, fbh);
        fluid.setBounds(boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh)));
        fluid.setAttractor(cursorOnPlane(win, view, proj), true);

        const float wellScale =
            glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ? -1.0f
            : glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ? 3.0f
                                                                            : 1.0f;
        const float wellSetting = P.attractG;
        P.attractG = wellSetting * wellScale;
        for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        P.attractG = wellSetting;

        renderer.render(fluid.positions(), view, proj, fbw, fbh);
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
            char title[256];
            std::snprintf(title, sizeof title,
                          "Elemancer  |  tension %.3f  |  well %.1f  |  visc %.1f  |  drag %.2f"
                          "  |  %zu drops  |  %.0f fps",
                          P.surfaceTension, wellSetting, P.viscosity, P.drag, fluid.size(), fps);
            glfwSetWindowTitle(win, title);
        }
    }

    renderer.shutdown();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
