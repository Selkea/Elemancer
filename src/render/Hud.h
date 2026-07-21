#pragma once

#include <string>
#include <vector>

namespace elem {

// Minimal text overlay for the on-screen controls list. Uses stb_easy_font to
// turn strings into quad geometry on the CPU -- no font file, no texture -- and
// draws them in pixel space over the finished frame.
class Hud {
public:
    bool init();
    void shutdown();

    // Draws a translucent panel and the given lines at the top-left. Expects to
    // run last, over the default framebuffer.
    void draw(const std::vector<std::string>& lines, int fbWidth, int fbHeight);

private:
    unsigned int prog_ = 0;
    unsigned int vao_ = 0;
    unsigned int vbo_ = 0;
    int capacityBytes_ = 0;
};

}  // namespace elem
