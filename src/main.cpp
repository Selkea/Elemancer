// Elemancer - a cohesive liquid that gathers around a cursor gravity well.
//
// Milestone 1: SPH solver on the CPU, drawn as lit sphere impostors. The
// screen-space surface pass that fuses the impostors into a single glossy
// body comes next; impostors are already enough to read the motion.
//
//   elemancer                       interactive
//   elemancer --shot out.bmp        headless capture, for self-verification

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sim/Fluid.h"

namespace {

constexpr int kWidth = 1280;
constexpr int kHeight = 800;
constexpr int kSubSteps = 2;
constexpr float kDt = 1.0f / 240.0f;

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "[elemancer] cannot open %s\n", path.c_str());
        return {};
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

GLuint compileShader(GLenum type, const std::string& src, const char* label) {
    const GLuint s = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(s, 1, &p, nullptr);
    glCompileShader(s);

    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::fprintf(stderr, "[elemancer] %s shader failed:\n%s\n", label, log);
    }
    return s;
}

GLuint linkProgram(const std::string& vsSrc, const std::string& fsSrc) {
    const GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc, "vertex");
    const GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc, "fragment");

    const GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(prog, sizeof log, nullptr, log);
        std::fprintf(stderr, "[elemancer] link failed:\n%s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

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

}  // namespace

int main(int argc, char** argv) {
    bool shotMode = false;
    std::string shotPath = "elemancer_shot.bmp";
    int particleCount = 2600;
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
    glfwWindowHint(GLFW_SAMPLES, 4);
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

    std::printf("[elemancer] GL %s | %s\n",
                reinterpret_cast<const char*>(glGetString(GL_VERSION)),
                reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

    const std::string assetDir = ELEMANCER_ASSET_DIR;
    const GLuint prog = linkProgram(readFile(assetDir + "/shaders/particle.vert"),
                                    readFile(assetDir + "/shaders/particle.frag"));

    const GLint uView = glGetUniformLocation(prog, "uView");
    const GLint uProj = glGetUniformLocation(prog, "uProj");
    const GLint uPointScale = glGetUniformLocation(prog, "uPointScale");
    const GLint uRadius = glGetUniformLocation(prog, "uRadius");
    const GLint uLightDir = glGetUniformLocation(prog, "uLightDir");
    const GLint uBaseColor = glGetUniformLocation(prog, "uBaseColor");

    elem::Fluid fluid;
    fluid.init(particleCount);
    std::printf("[elemancer] particles=%zu\n", fluid.size());

    GLuint vao = 0, vbo = 0;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(fluid.size() * sizeof(glm::vec3)),
                 nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);

    const float visualRadius = 0.055f;
    const glm::vec3 eye(0.0f, 0.0f, 3.0f);

    const auto drawScene = [&](int fbw, int fbh) {
        const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 proj = glm::perspective(
            glm::radians(45.0f), static_cast<float>(fbw) / static_cast<float>(fbh), 0.05f, 100.0f);

        glViewport(0, 0, fbw, fbh);
        glClearColor(0.03f, 0.04f, 0.06f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(fluid.size() * sizeof(glm::vec3)),
                        fluid.positions().data());

        glUseProgram(prog);
        glUniformMatrix4fv(uView, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(uProj, 1, GL_FALSE, glm::value_ptr(proj));
        glUniform1f(uPointScale, static_cast<float>(fbh) * proj[1][1]);
        glUniform1f(uRadius, visualRadius);
        glUniform3f(uLightDir, 0.4f, 0.7f, 0.6f);
        glUniform3f(uBaseColor, 0.16f, 0.42f, 0.72f);

        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(fluid.size()));

        return proj;
    };

    if (shotMode) {
        // Let the blob gather around a fixed well, then grab one frame.
        fluid.setAttractor(glm::vec3(0.30f, 0.15f, 0.0f), true);
        for (int i = 0; i < shotFrames; ++i) {
            for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        drawScene(fbw, fbh);
        glFinish();

        std::vector<unsigned char> pixels(static_cast<std::size_t>(fbw) * fbh * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        // Centroid + spread give a numeric read on the shot alongside the image.
        glm::vec3 centroid(0.0f);
        for (const glm::vec3& p : fluid.positions()) centroid += p;
        centroid /= static_cast<float>(fluid.size());
        float spread = 0.0f;
        for (const glm::vec3& p : fluid.positions()) spread += glm::length(p - centroid);
        spread /= static_cast<float>(fluid.size());

        const bool ok = saveBMP(shotPath, fbw, fbh, pixels);
        std::printf("SHOT file=%s %dx%d centroid=(%.3f, %.3f, %.3f) meanRadius=%.3f saved=%d\n",
                    shotPath.c_str(), fbw, fbh, centroid.x, centroid.y, centroid.z, spread,
                    ok ? 1 : 0);

        glfwDestroyWindow(win);
        glfwTerminate();
        return ok ? 0 : 1;
    }

    std::printf("[elemancer] LMB pull harder | RMB repel | G gravity | R reset | ESC quit\n");

    const float baseG = fluid.params().attractG;
    auto last = std::chrono::high_resolution_clock::now();
    double fpsAccum = 0.0;
    int fpsFrames = 0;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();

        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) break;
        if (glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS) fluid.init(particleCount);
        if (glfwGetKey(win, GLFW_KEY_G) == GLFW_PRESS) {
            fluid.params().gravity =
                (fluid.params().gravity.y < -0.1f) ? glm::vec3(0.0f) : glm::vec3(0.0f, -4.0f, 0.0f);
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) continue;

        const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 proj = glm::perspective(
            glm::radians(45.0f), static_cast<float>(fbw) / static_cast<float>(fbh), 0.05f, 100.0f);

        fluid.setAttractor(cursorOnPlane(win, view, proj), true);
        fluid.params().attractG =
            glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ? -baseG
            : glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ? baseG * 3.0f
                                                                            : baseG;

        for (int s = 0; s < kSubSteps; ++s) fluid.step(kDt);

        drawScene(fbw, fbh);
        glfwSwapBuffers(win);

        const auto now = std::chrono::high_resolution_clock::now();
        fpsAccum += std::chrono::duration<double>(now - last).count();
        last = now;
        if (++fpsFrames == 120) {
            std::printf("[elemancer] %.1f fps\n", fpsFrames / fpsAccum);
            fpsAccum = 0.0;
            fpsFrames = 0;
        }
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
