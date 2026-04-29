#include "wd/ui/ui_controller.h"

#include <imgui.h>
#include <algorithm>

namespace wd {

namespace {

constexpr const char* kDebugViewLabels[] = {
    "Final",
    "Scene Color",
    "Environment Map",
    "Scene Depth",
    "Droplet Depth",
    "Droplet Normal",
    "Thickness",
};

} // namespace

UiActions UiController::draw(SolverParams&,
                             RenderParams& renderParams,
                             MaterialParams&,
                             SurfaceMaterialParams& surfaceMaterial,
                             SurfaceRenderParams& surfaceRender,
                             const Vec3& gravityLikeForce,
                             MergeSplitController&) {
    UiActions actions;

    if (!gravityDraftInitialized_) {
        gravityDraft_ = gravityLikeForce;
        gravityDraftInitialized_ = true;
    }

    ImGui::Begin("Droplet Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const float vectorInputWidth = ImGui::CalcTextSize("-000.000  -000.000  -000.000").x;
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalarN("Spawn XYZ", ImGuiDataType_Double, spawnAnchor_.data(), 3, 0.01f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Create Droplet")) {
        actions.createDroplet = true;
        actions.spawnAnchor = spawnAnchor_;
    }
    if (ImGui::Button("Save Screenshot")) {
        actions.saveScreenshot = true;
    }

    ImGui::Separator();
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalarN("Gravity XYZ", ImGuiDataType_Double, gravityDraft_.data(), 3, 0.05f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Apply Gravity")) {
        actions.applyGravity = true;
        actions.gravityForce = gravityDraft_;
    }

    ImGui::Separator();
    int debugView = static_cast<int>(renderParams.debugView);
    ImGui::SetNextItemWidth(vectorInputWidth);
    if (ImGui::Combo("Debug View", &debugView, kDebugViewLabels, IM_ARRAYSIZE(kDebugViewLabels))) {
        renderParams.debugView = static_cast<RenderDebugView>(debugView);
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Glass Surface");
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Glass IOR", ImGuiDataType_Double, &surfaceRender.ior, 0.01f, nullptr, nullptr, "%.3f");
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Opacity", ImGuiDataType_Double, &surfaceRender.opacity, 0.01f, nullptr, nullptr, "%.3f");
    surfaceRender.opacity = std::clamp(surfaceRender.opacity, 0.0, 1.0);

    float tint[3] = {
        static_cast<float>(surfaceRender.tintColor.x()),
        static_cast<float>(surfaceRender.tintColor.y()),
        static_cast<float>(surfaceRender.tintColor.z()),
    };
    ImGui::SetNextItemWidth(vectorInputWidth);
    if (ImGui::ColorEdit3("Tint", tint)) {
        surfaceRender.tintColor = Vec3(tint[0], tint[1], tint[2]);
    }

    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Surface Friction", ImGuiDataType_Double, &surfaceMaterial.friction, 0.01f, nullptr, nullptr, "%.3f");
    surfaceMaterial.friction = std::max(surfaceMaterial.friction, 0.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Receding Angle", ImGuiDataType_Double, &surfaceMaterial.recContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Advancing Angle", ImGuiDataType_Double, &surfaceMaterial.advContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
    surfaceMaterial.recContactAngleDeg = std::clamp(surfaceMaterial.recContactAngleDeg, 0.0, 180.0);
    surfaceMaterial.advContactAngleDeg = std::clamp(surfaceMaterial.advContactAngleDeg, 0.0, 180.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Contact Stiffness", ImGuiDataType_Double, &surfaceMaterial.contactStiffness, 0.01f, nullptr, nullptr, "%.3f");
    surfaceMaterial.contactStiffness = std::max(surfaceMaterial.contactStiffness, 0.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Adhesion Distance", ImGuiDataType_Double, &surfaceMaterial.adhesionDistance, 0.005f, nullptr, nullptr, "%.3f");
    surfaceMaterial.adhesionDistance = std::max(surfaceMaterial.adhesionDistance, 0.0);

    ImGui::End();

    return actions;
}

} // namespace wd
