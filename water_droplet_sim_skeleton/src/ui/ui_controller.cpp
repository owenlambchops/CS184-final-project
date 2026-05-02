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
    "Caustics",
};

} // namespace

UiActions UiController::draw(SolverParams&,
                             RenderParams& renderParams,
                             MaterialParams&,
                             SurfaceMaterialParams& surfaceMaterial,
                             SurfaceRenderParams& surfaceRender,
                             double planeSideLength,
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
    ImGui::TextUnformatted("Water Rendering");
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Water IOR", ImGuiDataType_Double, &renderParams.ior, 0.01f, nullptr, nullptr, "%.3f");
    renderParams.ior = std::max(renderParams.ior, 1.0001);
    ImGui::Checkbox("Enable Thickness", &renderParams.enableThickness);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Refraction Scale", ImGuiDataType_Double, &renderParams.refractionScale, 0.001f, nullptr, nullptr, "%.3f");
    renderParams.refractionScale = std::max(renderParams.refractionScale, 0.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Max Thickness", ImGuiDataType_Double, &renderParams.maxThickness, 0.01f, nullptr, nullptr, "%.3f");
    renderParams.maxThickness = std::max(renderParams.maxThickness, 0.001);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Absorption Strength", ImGuiDataType_Double, &renderParams.absorptionStrength, 0.01f, nullptr, nullptr, "%.3f");
    renderParams.absorptionStrength = std::max(renderParams.absorptionStrength, 0.0);

    float absorption[3] = {
        static_cast<float>(renderParams.absorptionColor.x()),
        static_cast<float>(renderParams.absorptionColor.y()),
        static_cast<float>(renderParams.absorptionColor.z()),
    };
    ImGui::SetNextItemWidth(vectorInputWidth);
    if (ImGui::ColorEdit3("Absorption Color", absorption)) {
        renderParams.absorptionColor = Vec3(absorption[0], absorption[1], absorption[2]);
    }

    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Fresnel Bias", ImGuiDataType_Double, &renderParams.fresnelBias, 0.005f, nullptr, nullptr, "%.3f");
    renderParams.fresnelBias = std::clamp(renderParams.fresnelBias, 0.0, 1.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Fresnel Scale", ImGuiDataType_Double, &renderParams.fresnelScale, 0.005f, nullptr, nullptr, "%.3f");
    renderParams.fresnelScale = std::clamp(renderParams.fresnelScale, 0.0, 2.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Fresnel Power", ImGuiDataType_Double, &renderParams.fresnelPower, 0.05f, nullptr, nullptr, "%.3f");
    renderParams.fresnelPower = std::max(renderParams.fresnelPower, 0.1);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Debug Depth Range", ImGuiDataType_Double, &renderParams.debugDepthRange, 0.1f, nullptr, nullptr, "%.3f");
    renderParams.debugDepthRange = std::max(renderParams.debugDepthRange, 0.001);
    ImGui::Checkbox("Enable Caustics", &renderParams.enableCaustics);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Caustic Strength", ImGuiDataType_Double, &renderParams.causticStrength, 0.01f, nullptr, nullptr, "%.3f");
    renderParams.causticStrength = std::max(renderParams.causticStrength, 0.0);
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalar("Caustic Point Size", ImGuiDataType_Double, &renderParams.causticPointSize, 0.05f, nullptr, nullptr, "%.3f");
    renderParams.causticPointSize = std::max(renderParams.causticPointSize, 0.1);

    ImGui::Separator();
    ImGui::TextUnformatted("Glass Surface");
    if (planeSideLength > 0.0) {
        double editedSideLength = planeSideLength;
        ImGui::SetNextItemWidth(vectorInputWidth);
        if (ImGui::DragScalar("Plane Side Length", ImGuiDataType_Double, &editedSideLength, 0.05f, nullptr, nullptr, "%.3f")) {
            actions.setPlaneSideLength = true;
            actions.planeSideLength = std::max(editedSideLength, 0.1);
        }
    }
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
