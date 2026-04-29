#include "wd/ui/ui_controller.h"

#include <imgui.h>

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

    ImGui::End();

    return actions;
}

} // namespace wd
