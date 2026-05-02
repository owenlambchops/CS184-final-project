#include "wd/ui/ui_controller.h"
#include "wd/core/types.h"

#include <imgui.h>
#include <algorithm>
#include <cmath>

namespace wd {
 
namespace {

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
 
UiActions UiController::draw(SolverParams& solverParams,
                             RenderParams& renderParams,
                             MaterialParams& material,
                             const Vec3& gravityLikeForce,
                             MergeSplitController&) {
    UiActions actions;
 
    if (!gravityDraftInitialized_) {
        gravityDraft_ = gravityLikeForce;
        gravityDraftInitialized_ = true;
    }
 
    ImGui::Begin("Droplet Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
 
    const float w = ImGui::CalcTextSize("-000.000  -000.000  -000.000").x;
 
    // ── Spawn ────────────────────────────────────────────────────────────────
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalarN("Spawn XYZ", ImGuiDataType_Double,
                       spawnAnchor_.data(), 3, 0.01f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Create Droplet")) {
        actions.createDroplet = true;
        actions.spawnAnchor   = spawnAnchor_;
    }
    if (ImGui::Button("Save Screenshot")) {
        actions.saveScreenshot = true;
    }
 
    // ── Gravity ──────────────────────────────────────────────────────────────
    ImGui::Separator();
    ImGui::TextUnformatted("Gravity");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalarN("Gravity XYZ", ImGuiDataType_Double,
                       gravityDraft_.data(), 3, 0.05f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Apply Gravity")) {
        actions.applyGravity  = true;
        actions.gravityForce  = gravityDraft_;
    }
 
    // ── Plane Tilt ───────────────────────────────────────────────────────────
    ImGui::Separator();
    ImGui::TextUnformatted("Plane Tilt");
 
    bool tiltChanged = false;
    double tiltMin = kTiltMin;
    double tiltMax = kTiltMax;

    ImGui::SetNextItemWidth(w);
    tiltChanged |= ImGui::SliderScalar("Pitch (deg) away/toward",
                                       ImGuiDataType_Double, &pitchDeg_,
                                       &tiltMin, &tiltMax, "%.1f");
    ImGui::SetNextItemWidth(w);
    tiltChanged |= ImGui::SliderScalar("Roll  (deg) left/right",
                                       ImGuiDataType_Double, &rollDeg_,
                                       &tiltMin, &tiltMax, "%.1f");
 
    if (ImGui::Button("Reset Tilt")) {
        pitchDeg_   = 0.0;
        rollDeg_    = 0.0;
        tiltChanged = true;
    }
 
    if (tiltChanged) {
        actions.tiltPlane   = true;
        actions.planeNormal = normalFromAngles(pitchDeg_, rollDeg_);
    }
 
    // ── Render / Surface (unchanged from your original) ──────────────────────
    ImGui::Separator();
    ImGui::TextUnformatted("Render");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("IOR", ImGuiDataType_Double,
                      &renderParams.ior, 0.01f, nullptr, nullptr, "%.3f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Fresnel Bias", ImGuiDataType_Double,
                      &renderParams.fresnelBias, 0.01f, nullptr, nullptr, "%.3f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Fresnel Scale", ImGuiDataType_Double,
                      &renderParams.fresnelScale, 0.01f, nullptr, nullptr, "%.3f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Fresnel Power", ImGuiDataType_Double,
                      &renderParams.fresnelPower, 0.1f, nullptr, nullptr, "%.2f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Refraction Scale", ImGuiDataType_Double,
                      &renderParams.refractionScale, 0.001f, nullptr, nullptr, "%.4f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Specular Power", ImGuiDataType_Double,
                      &renderParams.specularPower, 1.0f, nullptr, nullptr, "%.1f");
    ImGui::Checkbox("Enable Thickness", &renderParams.enableThickness);
 
    // --- Glass Surface ---
    // ── Material (MaterialParams) ─────────────────────────────────────────────
    ImGui::Separator();
    ImGui::TextUnformatted("Material");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Surface Tension", ImGuiDataType_Double,
                      &material.surfaceTension, 0.01f, nullptr, nullptr, "%.3f");
    material.surfaceTension = std::max(material.surfaceTension, 0.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Friction", ImGuiDataType_Double,
                      &material.friction, 0.01f, nullptr, nullptr, "%.3f");
    material.friction = std::max(material.friction, 0.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Viscous Damping", ImGuiDataType_Double,
                      &material.viscousDamping, 0.01f, nullptr, nullptr, "%.3f");
    material.viscousDamping = std::max(material.viscousDamping, 0.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Receding Angle", ImGuiDataType_Double,
                      &material.recContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Advancing Angle", ImGuiDataType_Double,
                      &material.advContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
    material.recContactAngleDeg = std::clamp(material.recContactAngleDeg, 0.0, 180.0);
    material.advContactAngleDeg = std::clamp(material.advContactAngleDeg, 0.0, 180.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Contact Stiffness", ImGuiDataType_Double,
                      &material.contactStiffness, 0.01f, nullptr, nullptr, "%.3f");
    material.contactStiffness = std::max(material.contactStiffness, 0.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Volume Stiffness", ImGuiDataType_Double,
                      &material.volumeStiffness, 0.01f, nullptr, nullptr, "%.3f");
    material.volumeStiffness = std::max(material.volumeStiffness, 0.0);
 
    // ── Solver (SolverParams) ─────────────────────────────────────────────────
    ImGui::Separator();
    ImGui::TextUnformatted("Solver");
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Adhesion Distance", ImGuiDataType_Double,
                      &solverParams.adhesionDistance, 0.005f, nullptr, nullptr, "%.3f");
    solverParams.adhesionDistance = std::max(solverParams.adhesionDistance, 0.0);
    ImGui::SetNextItemWidth(w);
    ImGui::DragScalar("Max Velocity", ImGuiDataType_Double,
                      &solverParams.maxVelocity, 0.1f, nullptr, nullptr, "%.2f");
    solverParams.maxVelocity = std::max(solverParams.maxVelocity, 0.0);
    ImGui::Checkbox("Enable Collision",      &solverParams.enableCollision);
    ImGui::Checkbox("Enable Viscosity",      &solverParams.enableViscosity);
    ImGui::Checkbox("Enable Curvature Flow", &solverParams.enableCurvatureFlow);
    ImGui::Checkbox("Enable Contact Angle",  &solverParams.enableContactAngle);
    ImGui::Checkbox("Enable Volume Correct", &solverParams.enableVolumeCorrect);
 
    ImGui::End();
    return actions;
}
 
} // namespace wd
