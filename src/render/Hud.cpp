#include "render/Hud.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>

#include <GL/glew.h>

#include "stb_easy_font.h"

namespace elem {
namespace {

constexpr float kScale = 2.0f;       // stb glyphs are tiny; scale up to read
constexpr float kLineHeight = 22.0f; // pixels between lines
constexpr float kOriginX = 16.0f;
constexpr float kOriginY = 14.0f;
constexpr float kPad = 10.0f;

const char* kVert = R"(#version 430 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
uniform vec2 uViewport;
out vec4 vColor;
void main() {
    vec2 ndc = vec2(aPos.x / uViewport.x * 2.0 - 1.0, 1.0 - aPos.y / uViewport.y * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    vColor = aColor;
}
)";

const char* kFrag = R"(#version 430 core
in vec4 vColor;
layout(location = 0) out vec4 outColor;
void main() { outColor = vColor; }
)";

// One vertex: pixel position + RGBA8. Matches the attribute layout below.
struct Vert {
    float x, y;
    unsigned char c[4];
};

GLuint compile(GLenum type, const char* src) {
    const GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::fprintf(stderr, "[elemancer] hud shader: %s\n", log);
    }
    return s;
}

void pushTri(std::vector<Vert>& out, const Vert& a, const Vert& b, const Vert& c) {
    out.push_back(a);
    out.push_back(b);
    out.push_back(c);
}

}  // namespace

bool Hud::init() {
    const GLuint vs = compile(GL_VERTEX_SHADER, kVert);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, kFrag);
    prog_ = glCreateProgram();
    glAttachShader(prog_, vs);
    glAttachShader(prog_, fs);
    glLinkProgram(prog_);
    glDeleteShader(vs);
    glDeleteShader(fs);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vert),
                          reinterpret_cast<void*>(offsetof(Vert, x)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vert),
                          reinterpret_cast<void*>(offsetof(Vert, c)));
    glBindVertexArray(0);
    return prog_ != 0;
}

void Hud::draw(const std::vector<std::string>& lines, int fbWidth, int fbHeight) {
    if (lines.empty() || prog_ == 0) return;

    std::vector<Vert> verts;

    // Translucent backing panel, so the text stays legible over a bright sky.
    const float panelH = kOriginY + kLineHeight * static_cast<float>(lines.size()) + kPad;
    const unsigned char panel[4] = {8, 10, 14, 170};
    const Vert p0{0.0f, 0.0f, {panel[0], panel[1], panel[2], panel[3]}};
    const Vert p1{static_cast<float>(fbWidth), 0.0f, {panel[0], panel[1], panel[2], panel[3]}};
    const Vert p2{static_cast<float>(fbWidth), panelH, {panel[0], panel[1], panel[2], panel[3]}};
    const Vert p3{0.0f, panelH, {panel[0], panel[1], panel[2], panel[3]}};
    pushTri(verts, p0, p1, p2);
    pushTri(verts, p0, p2, p3);

    // stb_easy_font writes 16-byte vertices (x,y,z float; rgba8) in quads.
    std::vector<char> scratch(1 << 16);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const unsigned char white[4] = {235, 240, 247, 255};
        const int quads = stb_easy_font_print(0.0f, 0.0f, const_cast<char*>(lines[i].c_str()),
                                              const_cast<unsigned char*>(white), scratch.data(),
                                              static_cast<int>(scratch.size()));
        const float baseY = kOriginY + static_cast<float>(i) * kLineHeight;

        for (int q = 0; q < quads; ++q) {
            Vert corner[4];
            for (int k = 0; k < 4; ++k) {
                const char* v = scratch.data() + (static_cast<std::size_t>(q) * 4 + k) * 16;
                float vx = 0.0f, vy = 0.0f;
                std::memcpy(&vx, v + 0, sizeof(float));
                std::memcpy(&vy, v + 4, sizeof(float));
                corner[k].x = kOriginX + vx * kScale;
                corner[k].y = baseY + vy * kScale;
                corner[k].c[0] = white[0];
                corner[k].c[1] = white[1];
                corner[k].c[2] = white[2];
                corner[k].c[3] = white[3];
            }
            pushTri(verts, corner[0], corner[1], corner[2]);
            pushTri(verts, corner[0], corner[2], corner[3]);
        }
    }

    const GLsizeiptr bytes = static_cast<GLsizeiptr>(verts.size() * sizeof(Vert));
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    if (bytes > capacityBytes_) {
        glBufferData(GL_ARRAY_BUFFER, bytes, nullptr, GL_STREAM_DRAW);
        capacityBytes_ = static_cast<int>(bytes);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, verts.data());

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(prog_);
    glUniform2f(glGetUniformLocation(prog_, "uViewport"), static_cast<float>(fbWidth),
                static_cast<float>(fbHeight));
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size()));

    glDisable(GL_BLEND);
}

void Hud::shutdown() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (prog_) glDeleteProgram(prog_);
}

}  // namespace elem
