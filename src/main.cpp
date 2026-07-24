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
#include "update/Updater.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace {

// Directory of the running executable, so a shared build can find its shaders
// next to itself rather than at the compile-time source path.
std::string exeDir() {
#ifdef _WIN32
    char buf[MAX_PATH];
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    std::string p(buf, n);
    const auto slash = p.find_last_of("\\/");
    return slash == std::string::npos ? std::string() : p.substr(0, slash);
#else
    return {};
#endif
}

// Prefer a shaders/ folder sitting beside the executable (a packaged, portable
// build); fall back to the compiled-in source path for running in place during
// development. Shaders and the settings file are both taken from here.
std::string resolveAssetDir() {
    const std::string dir = exeDir();
    if (!dir.empty()) {
        std::ifstream probe(dir + "/shaders/common_env.glsl");
        if (probe.good()) return dir;
    }
    return ELEMANCER_ASSET_DIR;
}


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
constexpr float kGravity = 15.0f;  // downward pull in gravity mode

// Drop-count sizing. Changing the number of drops resizes each one so the total
// liquid volume is unchanged: the settled volume is ~ N * spacing^3 ~ N * h^3,
// so holding it fixed means h ~ N^(-1/3). The render radius scales with h too, so
// the surface stays as full (radius / spacing is constant). Anchored at the
// defaults: kDropsRef drops give h = kHRef and render radius kRadiusRef. The
// solver is written to be scale-independent, so this changes resolution, not feel.
constexpr int kDropsRef = 6000;
constexpr float kHRef = 0.044f;       // must match FluidParams::h default
constexpr float kRadiusRef = 0.040f;  // must match FluidRenderer::Settings::radius default
constexpr int kMinDrops = 1000;
constexpr int kMaxDrops = 16000;

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
void saveConfig(const std::string& path, const elem::FluidParams& p, float clarity, float spray,
                int drops) {
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
      << "spin " << p.spinRate << "\n"
      << "spray " << spray << "\n"
      << "drops " << drops << "\n";
}

bool loadConfig(const std::string& path, elem::FluidParams& p, float& clarity, float& spray,
                int& drops) {
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
        else if (key == "spray") spray = v;
        else if (key == "drops") drops = static_cast<int>(v);
    }
    return true;
}

std::vector<std::string> hudControlLines() {
    return {
        "ELEMANCER",
        "Mouse: move well    LMB: pull/grab    RMB: repel    Scroll: zoom",
        "[ ] tension     - = well     , . viscosity     ; ' drag",
        "9 0 hold radius     O P clarity     K L spin",
        "G gravity mode (LMB grabs)   R reset   S save   Tab hide   Esc menu",
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

// State of the Escape-triggered pause menu.
struct UiState {
    bool paused = false;
    bool showSettings = false;
    bool exitRequested = false;
    bool dropsChanged = false;  // the drop-count slider was committed; re-init
    bool applyUpdate = false;   // the Update & Restart button was clicked
    std::string updateError;    // shown under the button if applying failed
};

// One centered ImGui slider that keeps a HUD-hotkey range in sync. Ctrl+Click
// to type an exact value; AlwaysClamp keeps a typed value inside the range so a
// stray keystroke cannot detonate the solver.
void settingSlider(const char* label, float* v, float lo, float hi, const char* fmt) {
    ImGui::SliderFloat(label, v, lo, hi, fmt, ImGuiSliderFlags_AlwaysClamp);
}

// The Escape pause menu and its settings page, drawn with Dear ImGui over the
// frozen scene. Slider edits write straight into the live params, so they take
// effect the instant the sim resumes; the caller persists them to elemancer.cfg.
void drawPauseUI(UiState& ui, elem::FluidParams& P, float& absorption, elem::DiffuseParams& D,
                 int& drops, const elem::Updater::Status& upd) {
    ImGuiIO& io = ImGui::GetIO();

    // Dim the paused scene so the menu reads clearly on top of it.
    ImGui::GetBackgroundDrawList()->AddRectFilled(ImVec2(0, 0), io.DisplaySize,
                                                  IM_COL32(6, 10, 16, 160));

    const ImVec2 center(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));

    if (!ui.showSettings) {
        ImGui::Begin("##pause", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoTitleBar);

        const auto centeredText = [](const char* s) {
            ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(s).x) * 0.5f);
            ImGui::TextUnformatted(s);
        };
        centeredText("ELEMANCER");
        centeredText("Paused");
        ImGui::Dummy(ImVec2(0.0f, 10.0f));

        const ImVec2 b(220.0f, 36.0f);
        if (ImGui::Button("Resume", b)) { ui.paused = false; ui.showSettings = false; }
        if (ImGui::Button("Settings", b)) ui.showSettings = true;
        if (ImGui::Button("Exit Elemancer", b)) ui.exitRequested = true;

        // Update notice, shown only when a newer release was found.
        if (upd.available) {
            ImGui::Dummy(ImVec2(0.0f, 6.0f));
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0.0f, 2.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(150, 220, 150, 255));
            ImGui::TextUnformatted(("Update available: " + upd.latestVersion).c_str());
            ImGui::PopStyleColor();
            ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(46, 120, 60, 255));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(58, 150, 74, 255));
            const bool clicked = ImGui::Button("Update & Restart", b);
            ImGui::PopStyleColor(2);
            if (clicked) ui.applyUpdate = true;
            if (!ui.updateError.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(230, 150, 150, 255));
                ImGui::TextWrapped("%s", ui.updateError.c_str());
                ImGui::PopStyleColor();
            }
        }
        ImGui::End();
    } else {
        ImGui::SetNextWindowSize(ImVec2(440.0f, 0.0f), ImGuiCond_Always);
        ImGui::Begin("Settings", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoCollapse);
        ImGui::TextDisabled("Drag a slider, or Ctrl+Click to type a value.");
        ImGui::Spacing();
        // Drop count. Applied on release (a re-init), and each drop is resized to
        // keep the total volume the same. Ctrl+Click to type an exact count.
        ImGui::SliderInt("Drops", &drops, kMinDrops, kMaxDrops, "%d",
                         ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::IsItemDeactivatedAfterEdit()) ui.dropsChanged = true;
        settingSlider("Surface tension", &P.surfaceTension, 0.005f, 2.5f, "%.3f");
        settingSlider("Well strength", &P.wellStiffness, 3.0f, 200.0f, "%.1f");
        settingSlider("Viscosity", &P.viscosity, 0.1f, 60.0f, "%.1f");
        settingSlider("Drag", &P.drag, 0.01f, 5.0f, "%.2f");
        settingSlider("Hold radius", &P.wellHoldRadius, 0.2f, 6.0f, "%.2f");
        settingSlider("Clarity", &absorption, 0.02f, 2.5f, "%.2f");
        settingSlider("Spin", &P.spinRate, 0.0f, 12.0f, "%.1f");
        settingSlider("Spray amount", &D.spawnRate, 0.0f, 2000.0f, "%.0f");
        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Back", ImVec2(140.0f, 32.0f))) ui.showSettings = false;
        ImGui::End();
    }
}

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
    float gravityOverride = 0.0f;
    bool dropMode = false;
    float adhesionOverride = -1.0f;
    int benchFrames = 0;   // >0 => timing bench: measure ms/frame vs distance
    bool benchNoSpray = false;
    float benchHz = 1.0f;  // gesture frequency; higher = sharper flicks, more spray
    bool benchSprayFlood = false;  // force max spray generation, to measure its cost
    std::string menuShotPath;      // capture the pause menu headlessly, for verification
    bool menuShotSettings = false;  // capture the settings page instead of the buttons
    std::string moveShotPath;      // capture a mid-motion frame with accumulated history
    bool noReproject = false;      // disable temporal history reprojection, for the A/B
    bool checkUpdateOnly = false;  // headless: run the update check and print the result
    std::string checkUpdateVersion; // optional baseline version override for that check

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
        } else if (a == "--gravity" && i + 1 < argc) {
            gravityOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--drop") {
            dropMode = true;
        } else if (a == "--adhesion" && i + 1 < argc) {
            adhesionOverride = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--bench" && i + 1 < argc) {
            benchFrames = std::atoi(argv[++i]);
        } else if (a == "--nospray") {
            benchNoSpray = true;
        } else if (a == "--benchhz" && i + 1 < argc) {
            benchHz = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--sprayflood") {
            benchSprayFlood = true;
        } else if (a == "--menushot" && i + 1 < argc) {
            menuShotPath = argv[++i];
        } else if (a == "--menusettings") {
            menuShotSettings = true;
        } else if (a == "--moveshot" && i + 1 < argc) {
            moveShotPath = argv[++i];
        } else if (a == "--noreproject") {
            noReproject = true;
        } else if (a == "--checkupdate") {
            checkUpdateOnly = true;
            // Optional version override, to test detection against a real release
            // from an older baseline: --checkupdate v0.0.1
            if (i + 1 < argc && argv[i + 1][0] != '-') checkUpdateVersion = argv[++i];
        }
    }

    // Headless verification of the update check: hit the real GitHub API and
    // print what it finds (nothing, until a release exists), no window needed.
    if (checkUpdateOnly) {
        const std::string cur = checkUpdateVersion.empty() ? ELEMANCER_VERSION : checkUpdateVersion;
        elem::Updater u;
        u.checkAsync(ELEMANCER_REPO, cur, ".");
        for (int i = 0; i < 150 && !u.status().checked; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const elem::Updater::Status s = u.status();
        std::printf("CHECKUPDATE version=%s checked=%d available=%d latest=%s url=%s\n",
                    cur.c_str(), s.checked ? 1 : 0, s.available ? 1 : 0,
                    s.latestVersion.c_str(), s.downloadUrl.c_str());
        return 0;
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "[elemancer] glfwInit failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    if (shotMode || benchFrames > 0 || !menuShotPath.empty() || !moveShotPath.empty())
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

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

    // Dear ImGui for the pause menu / settings panel. Installed after our scroll
    // callback so its GLFW backend chains to ours rather than replacing it.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // no imgui.ini scribbled into the cwd
    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(1.15f);
    ImGui::GetIO().FontGlobalScale = 1.2f;
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::printf("[elemancer] GL %s | %s\n",
                reinterpret_cast<const char*>(glGetString(GL_VERSION)),
                reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

    const std::string assetDir = resolveAssetDir();

    elem::FluidRenderer renderer;
    if (!renderer.init(assetDir)) {
        std::fprintf(stderr, "[elemancer] renderer init failed\n");
        return 1;
    }
    if (noTemporal) renderer.settings().temporalBlend = 0.0f;
    if (noReproject) renderer.settings().temporalReproject = false;
    if (clarityOverride >= 0.0f) renderer.settings().absorption = clarityOverride;

    elem::Fluid fluid;
    if (hOverride > 0.0f) fluid.params().h = hOverride;
    fluid.init(particleCount);
    if (tensionOverride > 0.0f) fluid.params().surfaceTension = tensionOverride;
    if (wellOverride > 0.0f) fluid.params().wellStiffness = wellOverride;
    if (spinOverride >= 0.0f) fluid.params().spinRate = spinOverride;
    if (holdOverride >= 0.0f) fluid.params().wellHoldRadius = holdOverride;
    if (viscOverride >= 0.0f) fluid.params().viscosity = viscOverride;
    if (gravityOverride != 0.0f) fluid.params().gravity = glm::vec3(0.0f, -gravityOverride, 0.0f);
    if (adhesionOverride >= 0.0f) fluid.params().adhesion = adhesionOverride;
    std::printf("[elemancer] particles=%zu tension=%.3f well=%.1f\n", fluid.size(),
                fluid.params().surfaceTension, fluid.params().wellStiffness);

    const float shotDist = distOverride > 0.0f ? distOverride : kCamDistance;
    const glm::mat4 view = viewFor(shotDist);

    const auto projFor = [](int fbw, int fbh) {
        return glm::perspective(glm::radians(kFovDegrees),
                                static_cast<float>(fbw) / static_cast<float>(fbh), 0.05f, 100.0f);
    };

    // Headless capture of the pause menu (or its settings page) over a settled
    // body, so the overlay's layout and centering can be verified without a
    // display, the same way --shot verifies the fluid.
    if (!menuShotPath.empty()) {
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        fluid.setBounds(boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh), shotDist));
        fluid.setAttractor(glm::vec3(0.0f), true);
        for (int i = 0; i < 200; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }
        // Twice: an auto-resizing ImGui window is hidden on its first frame while
        // it measures its size, so a single frame would capture only the dimmed
        // scene. The second frame draws the menu at its settled size.
        UiState ui;
        ui.paused = true;
        ui.showSettings = menuShotSettings;
        for (int f = 0; f < 2; ++f) {
            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            projFor(fbw, fbh), fbw, fbh, 6.0f);
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            drawPauseUI(ui, fluid.params(), renderer.settings().absorption, fluid.diffuse(),
                        particleCount, elem::Updater::Status{});
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        glFinish();

        std::vector<unsigned char> px(static_cast<std::size_t>(fbw) * fbh * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
        const bool ok = saveBMP(menuShotPath, fbw, fbh, px);
        std::printf("MENUSHOT file=%s settings=%d %dx%d saved=%d\n", menuShotPath.c_str(),
                    menuShotSettings ? 1 : 0, fbw, fbh, ok ? 1 : 0);

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        renderer.shutdown();
        glfwDestroyWindow(win);
        glfwTerminate();
        return ok ? 0 : 1;
    }

    // Capture a mid-motion frame with the temporal history actually built up.
    // --shot renders once, so its history is empty and it cannot show what the
    // surface does while the body is dragged; this settles, then translates the
    // well steadily while rendering every frame, and saves a frame from the
    // middle of the motion. --sweepspeed sets the drag speed; --notemporal for an
    // A/B against the raw per-frame surface.
    if (!moveShotPath.empty()) {
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const glm::mat4 proj = projFor(fbw, fbh);
        fluid.setBounds(boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh), shotDist));
        fluid.diffuse().enabled = false;  // isolate the surface

        const float frameDur = static_cast<float>(kSubSteps) * kDt;
        glm::vec3 wellPos(-1.2f, 0.0f, 0.0f);
        fluid.setAttractor(wellPos, true);
        for (int i = 0; i < 150; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }
        // Translate while rendering every frame so the history accumulates.
        const int moveFrames = 40;
        for (int i = 0; i < moveFrames; ++i) {
            wellPos.x += sweepSpeed * frameDur;
            fluid.setAttractor(wellPos, true);
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            proj, fbw, fbh, static_cast<float>(i) * frameDur);
        }
        glFinish();

        std::vector<unsigned char> px(static_cast<std::size_t>(fbw) * fbh * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
        const bool ok = saveBMP(moveShotPath, fbw, fbh, px);
        glm::vec3 c(0.0f);
        for (const glm::vec3& p : fluid.positions()) c += p;
        c /= static_cast<float>(fluid.size());
        std::printf("MOVESHOT file=%s temporal=%d speed=%.1f centroid.x=%.3f saved=%d\n",
                    moveShotPath.c_str(), noTemporal ? 0 : 1, sweepSpeed, c.x, ok ? 1 : 0);

        renderer.shutdown();
        glfwDestroyWindow(win);
        glfwTerminate();
        return ok ? 0 : 1;
    }

    // Timing bench: settle the body, then whip the well across a fixed *fraction
    // of the screen* at a fixed gesture frequency, and time step() and render()
    // separately over N frames. The screen-fraction gesture is the point: the
    // visible width grows with camera distance, so the same mouse motion that a
    // user makes maps to a proportionally larger -- and faster -- world sweep
    // when zoomed out. That is what "the fps drops the further you go" actually
    // reproduces; a fixed world-space --sweepspeed would not. Prints ms/frame
    // split into sim vs render, plus the spray count and grid coarsening that
    // scale the cost, so the distance dependence can be read directly.
    if (benchFrames > 0) {
        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        fluid.setBounds(boundsForView(aspect, shotDist));
        if (benchNoSpray) fluid.diffuse().enabled = false;
        // Force spray to flood so its cost can be measured against a known count,
        // decoupled from the fact that ordinary cursor motion barely sheds any.
        if (benchSprayFlood) {
            elem::DiffuseParams& D = fluid.diffuse();
            D.trappedAirMin = 0.0f;
            D.kineticMin = 0.0f;
            D.spawnRate = 20000.0f;
        }

        const float frameDur = static_cast<float>(kSubSteps) * kDt;

        // Sweep amplitude tracks the visible half-width so the gesture covers the
        // same screen fraction at any zoom; a fixed 1 Hz back-and-forth then makes
        // peak cursor speed scale with distance, as a real mouse gesture does.
        const float visHalfW =
            (shotDist - kDepthHalf) * std::tan(glm::radians(kFovDegrees) * 0.5f) * aspect;
        const float amp = 0.6f * visHalfW;
        const float omega = 2.0f * 3.14159265f * benchHz;  // benchHz full sweeps/second
        const float peakCursorSpeed = amp * omega;

        glm::vec3 wellPos(0.0f);
        fluid.setAttractor(wellPos, true);
        for (int i = 0; i < 120; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }

        // A few warm-up rendered frames: build the FBO targets and seed the
        // temporal history so the timed loop measures steady state, not setup.
        for (int i = 0; i < 5; ++i) {
            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            projFor(fbw, fbh), fbw, fbh, 0.0f);
        }
        glFinish();

        using clock = std::chrono::high_resolution_clock;
        double stepMs = 0.0, renderMs = 0.0;
        std::size_t peakSpray = 0;
        float t = 0.0f;
        for (int i = 0; i < benchFrames; ++i) {
            t += frameDur;
            wellPos = glm::vec3(amp * std::sin(omega * t), 0.0f, 0.0f);
            fluid.setAttractor(wellPos, true);

            const auto t0 = clock::now();
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
            const auto t1 = clock::now();
            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            projFor(fbw, fbh), fbw, fbh, t);
            glFinish();
            const auto t2 = clock::now();

            stepMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
            renderMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
            peakSpray = std::max(peakSpray, fluid.sprayCount());
        }

        glm::vec3 lo = fluid.positions()[0], hi = fluid.positions()[0];
        for (const glm::vec3& p : fluid.positions()) {
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        const float bodySpan = glm::length(hi - lo);

        const double f = static_cast<double>(benchFrames);
        const double frameMs = (stepMs + renderMs) / f;
        std::printf("BENCH dist=%.1f spray=%d frames=%d peakCursorSpeed=%.1f"
                    " frame=%.2fms (sim=%.2f render=%.2f) fps=%.0f"
                    " sprayPeak=%zu bodySpan=%.2f gridCells=%zu cellSize=%.4f\n",
                    shotDist, benchNoSpray ? 0 : 1, benchFrames, peakCursorSpeed, frameMs,
                    stepMs / f, renderMs / f, 1000.0 / frameMs, peakSpray, bodySpan,
                    fluid.gridCells(), fluid.cellSize());

        renderer.shutdown();
        glfwDestroyWindow(win);
        glfwTerminate();
        return 0;
    }

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

        // --drop: release the well and let gravity pool the liquid on the floor,
        // which is what pressing G does interactively.
        if (dropMode) {
            const float g = gravityOverride != 0.0f ? gravityOverride : kGravity;
            fluid.params().gravity = glm::vec3(0.0f, -g, 0.0f);
            fluid.setAttractor(wellPos, false);
            for (int i = 0; i < shotFrames; ++i) {
                for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
            }

            renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view,
                            projFor(fbw, fbh), fbw, fbh, 6.0f);
            glFinish();
            std::vector<unsigned char> px(static_cast<std::size_t>(fbw) * fbh * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadBuffer(GL_BACK);
            glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            glm::vec3 c(0.0f);
            glm::vec3 vmean(0.0f);
            for (std::size_t i = 0; i < fluid.size(); ++i) {
                c += fluid.positions()[i];
                vmean += fluid.velocities()[i];
            }
            c /= static_cast<float>(fluid.size());
            vmean /= static_cast<float>(fluid.size());
            glm::vec3 angMom(0.0f);
            float inertia = 0.0f;
            for (std::size_t i = 0; i < fluid.size(); ++i) {
                const glm::vec3 r = fluid.positions()[i] - c;
                angMom += glm::cross(r, fluid.velocities()[i] - vmean);
                inertia += glm::dot(r, r);
            }
            const float spin = inertia > 0.0f ? glm::length(angMom / inertia) : 0.0f;

            // Self-levelling check: bin the pooled liquid by x and take the top
            // surface height per bin. A level puddle has a flat top (low stddev)
            // and spreads wide and thin; a beaded one mounds up (high stddev).
            constexpr int kBins = 24;
            float xlo = 1e9f, xhi = -1e9f;
            for (const glm::vec3& p : fluid.positions()) {
                xlo = std::min(xlo, p.x);
                xhi = std::max(xhi, p.x);
            }
            std::vector<float> top(kBins, -1e9f);
            std::vector<int> cnt(kBins, 0);
            const float binW = std::max(1e-4f, (xhi - xlo) / kBins);
            for (const glm::vec3& p : fluid.positions()) {
                int b = std::min(kBins - 1, static_cast<int>((p.x - xlo) / binW));
                top[b] = std::max(top[b], p.y);
                ++cnt[b];
            }
            float meanTop = 0.0f;
            int used = 0;
            for (int b = 0; b < kBins; ++b)
                if (cnt[b] > 3) {
                    meanTop += top[b];
                    ++used;
                }
            meanTop /= std::max(1, used);
            float var = 0.0f;
            for (int b = 0; b < kBins; ++b)
                if (cnt[b] > 3) var += (top[b] - meanTop) * (top[b] - meanTop);
            const float surfaceStddev = std::sqrt(var / std::max(1, used));
            const float thickness = meanTop - fluid.params().floorY;

            const bool okd = saveBMP(shotPath, fbw, fbh, px);
            std::printf("DROP file=%s centroid.y=%.3f residualSpin=%.3f width=%.2f thickness=%.3f"
                        " surfaceStddev=%.3f saved=%d\n",
                        shotPath.c_str(), c.y, spin, xhi - xlo, thickness, surfaceStddev,
                        okd ? 1 : 0);

            // Then grab: turn the well on above the pool and confirm it lifts
            // off the floor (adhesion must not pin it once the cursor pulls).
            fluid.setAttractor(glm::vec3(0.0f, 0.5f, 0.0f), true);
            fluid.setWellScale(1.5f);
            for (int i = 0; i < shotFrames / 2; ++i) {
                for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
            }
            float grabbedY = 0.0f;
            for (const glm::vec3& p : fluid.positions()) grabbedY += p.y;
            grabbedY /= static_cast<float>(fluid.size());
            std::printf("GRAB pooledY=%.3f -> grabbedY=%.3f (well at 0.5; should rise off floor)\n",
                        c.y, grabbedY);

            renderer.shutdown();
            glfwDestroyWindow(win);
            glfwTerminate();
            return okd ? 0 : 1;
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

    // Re-init the body at a given drop count, resizing each drop (via h and the
    // render radius) so the total liquid volume is unchanged. particleCount is
    // the live count; the Settings slider and the config both drive this.
    const auto applyDropCount = [&](int n) {
        particleCount = std::clamp(n, kMinDrops, kMaxDrops);
        const float h =
            kHRef * std::cbrt(static_cast<float>(kDropsRef) / static_cast<float>(particleCount));
        fluid.params().h = h;
        fluid.params().mass = 0.0f;  // re-derive from the new spacing
        fluid.init(particleCount);
        renderer.settings().radius = kRadiusRef * (h / kHRef);
    };

    // Restore any values tuned in a previous session, then keep them saved.
    const std::string cfgPath = assetDir + "/elemancer.cfg";
    if (loadConfig(cfgPath, fluid.params(), renderer.settings().absorption,
                   fluid.diffuse().spawnRate, particleCount)) {
        std::printf("[elemancer] loaded settings from elemancer.cfg\n");
    }
    applyDropCount(particleCount);  // size the drops to the (possibly restored) count

    std::printf("[elemancer] mouse moves the well | LMB pull/grab | RMB repel | scroll zoom\n");
    std::printf("[elemancer] [ ] tension | - = well | , . viscosity | ; ' drag\n");
    std::printf("[elemancer] 9 0 hold radius | O P clarity | K L spin\n");
    std::printf("[elemancer] G gravity mode (hold LMB to grab) | R reset | S save | Tab | Esc\n");

    // Check GitHub for a newer release, in the background so it never stalls
    // startup. Only for a packaged build (shaders found beside the exe): a dev
    // build run from the source tree updates via git, and must not have its
    // files swapped out from under it. The result surfaces in the pause menu.
    elem::Updater updater;
    const bool packaged = !exeDir().empty() && assetDir == exeDir();
    if (packaged) {
        std::printf("[elemancer] version %s | checking for updates...\n", ELEMANCER_VERSION);
        updater.checkAsync(ELEMANCER_REPO, ELEMANCER_VERSION, assetDir);
    }

    elem::Hud hud;
    if (!hud.init()) std::fprintf(stderr, "[elemancer] hud init failed\n");
    bool showHud = true;
    const std::vector<std::string> hudLines = hudControlLines();

    KeyEdge gravityKey, resetKey, saveKey, hudKey, escKey;
    UiState ui;
    bool wasPaused = false;  // to persist settings the moment the menu is dismissed

    // Camera dolly, driven by the scroll wheel.
    float camDistance = kCamDistance;
    bool gravityMode = false;  // toggled by G: liquid falls, LMB grabs it back

    auto last = std::chrono::high_resolution_clock::now();
    double titleTimer = 0.0;
    double fpsAccum = 0.0;
    int fpsFrames = 0;
    float fps = 0.0f;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();

        // Escape opens the pause menu (was: quit). From the settings page it steps
        // back to the buttons; from the buttons it resumes. Ignored while typing a
        // value into a slider, where ImGui already uses Escape to cancel the edit.
        const bool escPressed = escKey.justPressed(win, GLFW_KEY_ESCAPE);
        if (escPressed && !ImGui::GetIO().WantTextInput) {
            if (!ui.paused) { ui.paused = true; ui.showSettings = false; }
            else if (ui.showSettings) ui.showSettings = false;
            else ui.paused = false;
        }

        const auto now = std::chrono::high_resolution_clock::now();
        const float frameDt =
            std::min(0.1f, std::chrono::duration<float>(now - last).count());
        last = now;

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) continue;

        // Scroll to dolly the camera. Multiplicative so the zoom feels even at
        // any distance; scroll up zooms in. Drained even while paused so a wheel
        // nudge over the menu does not jump the zoom on resume.
        if (g_scrollY != 0.0) {
            if (!ui.paused) {
                camDistance = std::clamp(
                    camDistance * std::exp(static_cast<float>(-g_scrollY) * 0.12f), 1.8f, 24.0f);
            }
            g_scrollY = 0.0;
        }
        const glm::mat4 view = viewFor(camDistance);
        const glm::mat4 proj = projFor(fbw, fbh);
        elem::FluidParams& P = fluid.params();

        // The whole live simulation -- input, tuning hotkeys and stepping -- is
        // frozen while the pause menu is up; only rendering and the menu run.
        if (!ui.paused) {
            holdAdjust(win, GLFW_KEY_LEFT_BRACKET, GLFW_KEY_RIGHT_BRACKET, P.surfaceTension, 1.5f,
                       frameDt, 0.005f, 2.5f);
            holdAdjust(win, GLFW_KEY_MINUS, GLFW_KEY_EQUAL, P.wellStiffness, 1.5f, frameDt, 3.0f,
                       200.0f);
            holdAdjust(win, GLFW_KEY_COMMA, GLFW_KEY_PERIOD, P.viscosity, 1.5f, frameDt, 0.1f,
                       60.0f);
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
            if (gravityKey.justPressed(win, GLFW_KEY_G)) gravityMode = !gravityMode;
            if (hudKey.justPressed(win, GLFW_KEY_TAB)) showHud = !showHud;
            if (saveKey.justPressed(win, GLFW_KEY_S)) {
                saveConfig(cfgPath, P, renderer.settings().absorption, fluid.diffuse().spawnRate,
                           particleCount);
                std::printf("[elemancer] saved settings to elemancer.cfg\n");
            }

            fluid.setBounds(
                boundsForView(static_cast<float>(fbw) / static_cast<float>(fbh), camDistance));
            const glm::vec3 cursor = cursorOnPlane(win, view, proj);
            const bool lmb = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            const bool rmb = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

            if (gravityMode) {
                // Gravity mode: the liquid falls and pools on the floor. The well is
                // released, so hold LMB to grab it back up (RMB still repels).
                P.gravity = glm::vec3(0.0f, -kGravity, 0.0f);
                fluid.setAttractor(cursor, lmb || rmb);
                fluid.setWellScale(rmb ? -1.0f : 1.5f);
            } else {
                // Follow mode: the well always holds the liquid at the cursor.
                P.gravity = glm::vec3(0.0f);
                fluid.setAttractor(cursor, true);
                fluid.setWellScale(rmb ? -1.0f : lmb ? 3.0f : 1.0f);
            }

            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }

        renderer.render(fluid.positions(), fluid.sprayPositions(), fluid.sprayLife(), view, proj,
                        fbw, fbh, static_cast<float>(glfwGetTime()));
        if (showHud && !ui.paused) hud.draw(hudLines, fbw, fbh);

        // Pause menu / settings, over the frozen scene.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        if (ui.paused)
            drawPauseUI(ui, P, renderer.settings().absorption, fluid.diffuse(), particleCount,
                        updater.status());
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // A committed drop-count change re-inits the body at the new count, with
        // each drop resized to hold the volume. Done here (not mid-widget) so the
        // re-init happens once, after the slider is released or a value typed.
        if (ui.dropsChanged) {
            applyDropCount(particleCount);
            ui.dropsChanged = false;
        }

        // Update & Restart: persist settings first, then hand off to the helper
        // that swaps the files once we exit. On success we break out and quit.
        if (ui.applyUpdate) {
            ui.applyUpdate = false;
            saveConfig(cfgPath, P, renderer.settings().absorption, fluid.diffuse().spawnRate,
                       particleCount);
            std::string err;
            if (updater.applyAndRestart(err)) {
                std::printf("[elemancer] applying update, restarting...\n");
                break;
            }
            ui.updateError = "Update failed: " + err;
            std::fprintf(stderr, "[elemancer] update failed: %s\n", err.c_str());
        }

        // Persist the moment the menu is dismissed (Resume/Esc) or Exit is chosen,
        // so slider edits survive even a hard quit.
        if ((wasPaused && !ui.paused) || ui.exitRequested) {
            saveConfig(cfgPath, P, renderer.settings().absorption, fluid.diffuse().spawnRate,
                       particleCount);
        }
        wasPaused = ui.paused;
        if (ui.exitRequested) break;

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
    saveConfig(cfgPath, fluid.params(), renderer.settings().absorption, fluid.diffuse().spawnRate,
               particleCount);
    std::printf("[elemancer] saved settings to elemancer.cfg\n");

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    hud.shutdown();
    renderer.shutdown();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
