#include "wd/ui/ui_controller.h"
#include "wd/core/types.h"

#include <imgui.h>
#include <algorithm>
#include <cmath>

namespace {

constexpr const char* kDebugViewLabels[] = {
    "Final",
    "Scene Color",
    "Environment Map",
    "Scene Depth",
    "Droplet Depth",
    "Droplet Normal",
    "Thickness",
    "Wireframe",
    "Caustics",
    "Wireframe",
    "Caustics",
};

constexpr double kPi = 3.14159265358979323846;
inline double toRad(double deg) { return deg * kPi / 180.0; }

Vec3 normalFromAngles(double pitchDeg, double rollDeg) {   // ← PUT IT HERE
    double px = toRad(pitchDeg);
    double rz = toRad(rollDeg);
    double sinP = std::sin(px), cosP = std::cos(px);
    double sinR = std::sin(rz), cosR = std::cos(rz);
    Vec3 n(sinR * cosP, cosP * cosR, -sinP);
    double len = n.norm();
    if (len > 1e-12) return Vec3(n / len);
    return Vec3::UnitY();
}
 
} // namespace

void UiController::draw(SolverParams&, RenderParams&, MaterialParams&, Vec3&, MergeSplitController&) {
    // TODO: add Dear ImGui sliders here.
}
 
} // namespace wd
