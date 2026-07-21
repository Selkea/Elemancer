// Headless check on the solver: the blob must stay finite, stay inside the
// domain, and actually migrate toward the cursor gravity well.
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "sim/Fluid.h"

int main() {
    const glm::vec3 target(0.30f, 0.10f, 0.0f);

    elem::Fluid f;
    f.init(900);
    f.setAttractor(target, true);

    const float dt = 1.0f / 240.0f;
    for (int i = 0; i < 600; ++i) f.step(dt);

    int failures = 0;
    float meanDensity = 0.0f;
    glm::vec3 centroid(0.0f);
    const float bound = f.params().boundHalf + 1e-3f;

    for (std::size_t i = 0; i < f.size(); ++i) {
        const glm::vec3& p = f.positions()[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            ++failures;
            continue;
        }
        if (std::fabs(p.x) > bound || std::fabs(p.y) > bound || std::fabs(p.z) > bound) {
            ++failures;
        }
        centroid += p;
        meanDensity += f.densities()[i];
    }

    const float n = static_cast<float>(f.size());
    centroid /= n;
    meanDensity /= n;
    const float distToWell = glm::length(centroid - target);

    std::printf("SIMTEST particles=%zu meanDensity=%.1f centroid=(%.3f, %.3f, %.3f)"
                " distToWell=%.3f nonFiniteOrOOB=%d\n",
                f.size(), meanDensity, centroid.x, centroid.y, centroid.z,
                distToWell, failures);

    const bool ok = failures == 0 && meanDensity > 1.0f && distToWell < 0.20f;
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
