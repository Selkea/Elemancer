#include "render/FluidRenderer.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace elem {
namespace {

constexpr float kBackgroundDepth = 1.0e6f;

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

// Splice shared GLSL in just after the #version line, since core GLSL has no
// #include. Keeps the environment function identical across shaders.
std::string spliceCommon(const std::string& src, const std::string& common) {
    if (common.empty()) return src;
    const std::size_t nl = src.find('\n');
    if (nl == std::string::npos) return src;
    return src.substr(0, nl + 1) + common + "\n" + src.substr(nl + 1);
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
        std::fprintf(stderr, "[elemancer] %s failed to compile:\n%s\n", label, log);
    }
    return s;
}

GLuint linkProgram(const std::string& vsSrc, const std::string& fsSrc, const char* label) {
    const GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc, label);
    const GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc, label);

    const GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(prog, sizeof log, nullptr, log);
        std::fprintf(stderr, "[elemancer] %s failed to link:\n%s\n", label, log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

void setMat4(GLuint prog, const char* name, const glm::mat4& m) {
    glUniformMatrix4fv(glGetUniformLocation(prog, name), 1, GL_FALSE, glm::value_ptr(m));
}
void setVec3(GLuint prog, const char* name, const glm::vec3& v) {
    glUniform3fv(glGetUniformLocation(prog, name), 1, glm::value_ptr(v));
}
void setVec2(GLuint prog, const char* name, const glm::vec2& v) {
    glUniform2fv(glGetUniformLocation(prog, name), 1, glm::value_ptr(v));
}
void setFloat(GLuint prog, const char* name, float v) {
    glUniform1f(glGetUniformLocation(prog, name), v);
}
void setInt(GLuint prog, const char* name, int v) {
    glUniform1i(glGetUniformLocation(prog, name), v);
}

GLuint makeColorTexture(int w, int h, GLenum internalFormat) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat), w, h, 0, GL_RED,
                 GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return tex;
}

GLuint makeRgbaTexture(int w, int h) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return tex;
}

GLuint makeFbo(GLuint colorTex, GLuint depthRbo) {
    GLuint fbo = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);
    if (depthRbo != 0) {
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRbo);
    }
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::fprintf(stderr, "[elemancer] incomplete framebuffer\n");
    }
    return fbo;
}

}  // namespace

bool FluidRenderer::init(const std::string& assetDir) {
    assetDir_ = assetDir;
    if (!buildPrograms()) return false;

    glGenVertexArrays(1, &vaoQuad_);

    glGenVertexArrays(1, &vaoParticles_);
    glGenBuffers(1, &vboParticles_);
    glBindVertexArray(vaoParticles_);
    glBindBuffer(GL_ARRAY_BUFFER, vboParticles_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glBindVertexArray(0);

    glGenVertexArrays(1, &vaoSpray_);
    glGenBuffers(1, &vboSpray_);
    glBindVertexArray(vaoSpray_);
    glBindBuffer(GL_ARRAY_BUFFER, vboSpray_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), nullptr);
    glBindVertexArray(0);

    glEnable(GL_PROGRAM_POINT_SIZE);
    return true;
}

bool FluidRenderer::buildPrograms() {
    const std::string common = readFile(assetDir_ + "/shaders/common_env.glsl");
    const std::string fullscreenVs = readFile(assetDir_ + "/shaders/fullscreen.vert");
    const std::string impostorVs = readFile(assetDir_ + "/shaders/impostor.vert");

    progBackground_ = linkProgram(
        fullscreenVs, spliceCommon(readFile(assetDir_ + "/shaders/background.frag"), common),
        "background");
    progDepth_ = linkProgram(impostorVs, readFile(assetDir_ + "/shaders/depth.frag"), "depth");
    progThickness_ =
        linkProgram(impostorVs, readFile(assetDir_ + "/shaders/thickness.frag"), "thickness");
    progBlur_ = linkProgram(fullscreenVs, readFile(assetDir_ + "/shaders/blur.frag"), "blur");
    progComposite_ = linkProgram(
        fullscreenVs, spliceCommon(readFile(assetDir_ + "/shaders/composite.frag"), common),
        "composite");
    progSpray_ = linkProgram(readFile(assetDir_ + "/shaders/spray.vert"),
                             readFile(assetDir_ + "/shaders/spray.frag"), "spray");
    progTemporal_ =
        linkProgram(fullscreenVs, readFile(assetDir_ + "/shaders/temporal.frag"), "temporal");

    return progBackground_ && progDepth_ && progThickness_ && progBlur_ && progComposite_ &&
           progSpray_ && progTemporal_;
}

void FluidRenderer::releaseTargets() {
    const GLuint fbos[] = {fboScene_,   fboDepth_,   fboThickness_, fboBlur_[0],
                           fboBlur_[1], fboHist_[0], fboHist_[1]};
    for (GLuint f : fbos) {
        if (f) glDeleteFramebuffers(1, &f);
    }
    const GLuint texs[] = {texScene_,   texDepth_,   texThickness_, texBlur_[0],
                           texBlur_[1], texHist_[0], texHist_[1]};
    for (GLuint t : texs) {
        if (t) glDeleteTextures(1, &t);
    }
    if (rboDepth_) glDeleteRenderbuffers(1, &rboDepth_);

    fboScene_ = fboDepth_ = fboThickness_ = fboBlur_[0] = fboBlur_[1] = 0;
    texScene_ = texDepth_ = texThickness_ = texBlur_[0] = texBlur_[1] = 0;
    fboHist_[0] = fboHist_[1] = texHist_[0] = texHist_[1] = 0;
    rboDepth_ = 0;
}

void FluidRenderer::ensureTargets(int w, int h) {
    if (w == width_ && h == height_ && fboScene_ != 0) return;
    releaseTargets();
    width_ = w;
    height_ = h;

    glGenRenderbuffers(1, &rboDepth_);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);

    texScene_ = makeRgbaTexture(w, h);
    texDepth_ = makeColorTexture(w, h, GL_R32F);
    texThickness_ = makeColorTexture(w, h, GL_R16F);
    texBlur_[0] = makeColorTexture(w, h, GL_R32F);
    texBlur_[1] = makeColorTexture(w, h, GL_R32F);
    texHist_[0] = makeColorTexture(w, h, GL_R32F);
    texHist_[1] = makeColorTexture(w, h, GL_R32F);

    fboScene_ = makeFbo(texScene_, 0);
    fboDepth_ = makeFbo(texDepth_, rboDepth_);
    fboThickness_ = makeFbo(texThickness_, 0);
    fboBlur_[0] = makeFbo(texBlur_[0], 0);
    fboBlur_[1] = makeFbo(texBlur_[1], 0);
    fboHist_[0] = makeFbo(texHist_[0], 0);
    fboHist_[1] = makeFbo(texHist_[1], 0);

    // Seed the history as empty (background) so the first frame is all "current".
    const float bg[4] = {kBackgroundDepth, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 2; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, fboHist_[i]);
        glClearBufferfv(GL_COLOR, 0, bg);
    }
    histIndex_ = 0;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FluidRenderer::drawQuad() const {
    glBindVertexArray(vaoQuad_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

void FluidRenderer::render(const std::vector<glm::vec3>& positions,
                           const std::vector<glm::vec3>& sprayPositions,
                           const std::vector<float>& sprayLife, const glm::mat4& view,
                           const glm::mat4& proj, int fbWidth, int fbHeight,
                           float timeSeconds) {
    if (fbWidth <= 0 || fbHeight <= 0) return;
    ensureTargets(fbWidth, fbHeight);

    const GLsizei count = static_cast<GLsizei>(positions.size());
    const GLsizeiptr bytes = static_cast<GLsizeiptr>(positions.size() * sizeof(glm::vec3));

    glBindBuffer(GL_ARRAY_BUFFER, vboParticles_);
    if (bytes > vboCapacity_) {
        glBufferData(GL_ARRAY_BUFFER, bytes, nullptr, GL_STREAM_DRAW);
        vboCapacity_ = bytes;
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, positions.data());

    const glm::mat4 invProj = glm::inverse(proj);
    const glm::mat4 invView = glm::inverse(view);
    const float pointScale = static_cast<float>(fbHeight) * proj[1][1];
    const glm::vec3 lightDirView(view * glm::vec4(settings_.lightDirWorld, 0.0f));

    // The depth blur and normal reconstruction are sized in screen texels, but
    // the body shrinks on screen as the camera dollies out. Left fixed, those
    // kernels come to span a large fraction of a distant body: the blur flattens
    // it and the normal baseline goes flat across the crown, so the grazing rim
    // collapses into a hard bright cap -- slivers, from the wrong scale. Scale
    // the kernels with the projected particle size (~ 1 / distance), anchored so
    // the near design view (distance ~5) is unchanged. Clamped so a very close or
    // very far body cannot drive the kernels to a degenerate size.
    const float camDist = glm::length(glm::vec3(invView[3]));
    constexpr float kRefDist = 5.0f;
    const float kernelScale = glm::clamp(kRefDist / glm::max(camDist, 0.5f), 0.35f, 1.5f);

    glViewport(0, 0, fbWidth, fbHeight);

    // 1. Environment behind the fluid, so refraction has something to bend.
    glBindFramebuffer(GL_FRAMEBUFFER, fboScene_);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(progBackground_);
    setMat4(progBackground_, "uInvProj", invProj);
    setMat4(progBackground_, "uInvView", invView);
    setVec3(progBackground_, "uLightDirWorld", settings_.lightDirWorld);
    setFloat(progBackground_, "uTime", timeSeconds);
    drawQuad();

    // 2. View-space depth of the nearest liquid surface.
    glBindFramebuffer(GL_FRAMEBUFFER, fboDepth_);
    const float clearDepth[4] = {kBackgroundDepth, 0.0f, 0.0f, 0.0f};
    glClearBufferfv(GL_COLOR, 0, clearDepth);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glUseProgram(progDepth_);
    setMat4(progDepth_, "uView", view);
    setMat4(progDepth_, "uProj", proj);
    setFloat(progDepth_, "uPointScale", pointScale);
    setFloat(progDepth_, "uRadius", settings_.radius);
    glBindVertexArray(vaoParticles_);
    glDrawArrays(GL_POINTS, 0, count);

    // 3. Bilateral smoothing, ping-ponging horizontal then vertical.
    glDisable(GL_DEPTH_TEST);
    glUseProgram(progBlur_);
    setFloat(progBlur_, "uRadius", settings_.blurRadius * kernelScale);
    setFloat(progBlur_, "uSigmaSpatial", settings_.sigmaSpatial * kernelScale);
    setFloat(progBlur_, "uSigmaDepth", settings_.sigmaDepth);
    setInt(progBlur_, "uDepth", 0);
    glActiveTexture(GL_TEXTURE0);

    GLuint source = texDepth_;
    const float texelX = 1.0f / static_cast<float>(fbWidth);
    const float texelY = 1.0f / static_cast<float>(fbHeight);
    for (int i = 0; i < settings_.blurIterations; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, fboBlur_[0]);
        glBindTexture(GL_TEXTURE_2D, source);
        setVec2(progBlur_, "uDir", glm::vec2(texelX, 0.0f));
        drawQuad();

        glBindFramebuffer(GL_FRAMEBUFFER, fboBlur_[1]);
        glBindTexture(GL_TEXTURE_2D, texBlur_[0]);
        setVec2(progBlur_, "uDir", glm::vec2(0.0f, texelY));
        drawQuad();

        source = texBlur_[1];
    }

    // 3b. Temporal resolve: blend the smoothed depth toward last frame's, which
    // stops the surface boiling while the body moves or spins. The result also
    // becomes next frame's history, ping-ponged between the two hist targets.
    if (settings_.temporalBlend > 0.0f) {
        const int cur = histIndex_;
        const int prev = 1 - histIndex_;

        glBindFramebuffer(GL_FRAMEBUFFER, fboHist_[cur]);
        glUseProgram(progTemporal_);
        setInt(progTemporal_, "uCurrent", 0);
        setInt(progTemporal_, "uHistory", 1);
        setFloat(progTemporal_, "uBlend", settings_.temporalBlend);
        setFloat(progTemporal_, "uMaxDelta", settings_.temporalMaxDelta);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, source);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texHist_[prev]);
        drawQuad();
        glActiveTexture(GL_TEXTURE0);

        source = texHist_[cur];
        histIndex_ = prev;
    }

    // 4. Thickness, accumulated additively with depth testing off.
    glBindFramebuffer(GL_FRAMEBUFFER, fboThickness_);
    const float clearZero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    glClearBufferfv(GL_COLOR, 0, clearZero);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glUseProgram(progThickness_);
    setMat4(progThickness_, "uView", view);
    setMat4(progThickness_, "uProj", proj);
    setFloat(progThickness_, "uPointScale", pointScale);
    setFloat(progThickness_, "uRadius", settings_.radius);
    glBindVertexArray(vaoParticles_);
    glDrawArrays(GL_POINTS, 0, count);
    glDisable(GL_BLEND);

    // 5. Shade the surface into the default framebuffer.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, fbWidth, fbHeight);
    glUseProgram(progComposite_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texThickness_);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, texScene_);
    setInt(progComposite_, "uDepth", 0);
    setInt(progComposite_, "uThickness", 1);
    setInt(progComposite_, "uScene", 2);
    setMat4(progComposite_, "uInvProj", invProj);
    setMat4(progComposite_, "uInvView", invView);
    setVec2(progComposite_, "uTexel", glm::vec2(texelX, texelY));
    setFloat(progComposite_, "uNormalBaseline", 8.0f * kernelScale);
    setVec3(progComposite_, "uLightDirView", glm::vec3(lightDirView));
    setVec3(progComposite_, "uLightDirWorld", settings_.lightDirWorld);
    setFloat(progComposite_, "uTime", timeSeconds);
    setVec3(progComposite_, "uLiquidColor", settings_.liquidColor);
    setFloat(progComposite_, "uRefractScale", settings_.refractScale);
    setFloat(progComposite_, "uAbsorption", settings_.absorption);
    setFloat(progComposite_, "uScatter", settings_.scatter);
    drawQuad();

    // 6. Diffuse spray, additively over the shaded surface. Deliberately not
    // depth tested: the droplets are sub-pixel scale and reading them as a
    // haze over the body looks better than popping them against its silhouette.
    if (!sprayPositions.empty() && progSpray_ != 0) {
        sprayScratch_.resize(sprayPositions.size());
        for (std::size_t i = 0; i < sprayPositions.size(); ++i) {
            const float life = i < sprayLife.size() ? sprayLife[i] : 0.0f;
            sprayScratch_[i] = glm::vec4(sprayPositions[i], life);
        }

        const GLsizeiptr sprayBytes =
            static_cast<GLsizeiptr>(sprayScratch_.size() * sizeof(glm::vec4));
        glBindBuffer(GL_ARRAY_BUFFER, vboSpray_);
        if (sprayBytes > sprayCapacity_) {
            glBufferData(GL_ARRAY_BUFFER, sprayBytes, nullptr, GL_STREAM_DRAW);
            sprayCapacity_ = sprayBytes;
        }
        glBufferSubData(GL_ARRAY_BUFFER, 0, sprayBytes, sprayScratch_.data());

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);  // premultiplied in the shader
        glDisable(GL_DEPTH_TEST);

        glUseProgram(progSpray_);
        setMat4(progSpray_, "uView", view);
        setMat4(progSpray_, "uProj", proj);
        setFloat(progSpray_, "uPointScale", pointScale);
        setFloat(progSpray_, "uRadius", settings_.sprayRadius);
        setFloat(progSpray_, "uLifeMax", settings_.sprayLifeMax);
        setFloat(progSpray_, "uIntensity", settings_.sprayIntensity);
        setVec3(progSpray_, "uColor", settings_.sprayColor);

        glBindVertexArray(vaoSpray_);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(sprayScratch_.size()));
        glDisable(GL_BLEND);
    }

    glActiveTexture(GL_TEXTURE0);
}

void FluidRenderer::shutdown() {
    releaseTargets();
    if (vboParticles_) glDeleteBuffers(1, &vboParticles_);
    if (vaoParticles_) glDeleteVertexArrays(1, &vaoParticles_);
    if (vboSpray_) glDeleteBuffers(1, &vboSpray_);
    if (vaoSpray_) glDeleteVertexArrays(1, &vaoSpray_);
    if (vaoQuad_) glDeleteVertexArrays(1, &vaoQuad_);

    const GLuint progs[] = {progBackground_, progDepth_, progThickness_, progBlur_,
                            progComposite_,  progSpray_, progTemporal_};
    for (GLuint p : progs) {
        if (p) glDeleteProgram(p);
    }
}

}  // namespace elem
