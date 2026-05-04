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
    "Wireframe",
    "Caustics",
};

} // namespace

UiActions UiController::draw(SolverParams& solverParams,
                             RenderParams& renderParams,
                             MaterialParams& defaultMaterial,
                             SurfaceMaterialParams& surfaceMaterial,
                             SurfaceRenderParams& surfaceRender,
                             double planeSideLength,
                             bool planeTiltEnabled,
                             double planeTiltMaxDeg,
                             double planeTiltResponsiveness,
                             double planeTiltAxisScaleX,
                             double planeTiltAxisScaleZ,
                             const Vec3& gravityLikeForce,
                             MergeSplitController&) {
    UiActions actions;

    if (!gravityDraftInitialized_) {
        gravityDraft_ = gravityLikeForce;
        gravityDraftInitialized_ = true;
    }

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const float vectorInputWidth = ImGui::CalcTextSize("-000.000  -000.000  -000.000").x;
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalarN("Spawn XYZ", ImGuiDataType_Double, spawnAnchor_.data(), 3, 0.01f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Spawn")) {
        actions.createDroplet = true;
        actions.spawnAnchor = spawnAnchor_;
    }
    ImGui::SameLine();
    if (ImGui::Button("Screenshot")) {
        actions.saveScreenshot = true;
    }

    ImGui::Separator();
    ImGui::SetNextItemWidth(vectorInputWidth);
    ImGui::DragScalarN("Gravity", ImGuiDataType_Double, gravityDraft_.data(), 3, 0.05f, nullptr, nullptr, "%.3f");
    if (ImGui::Button("Apply")) {
        actions.applyGravity = true;
        actions.gravityForce = gravityDraft_;
    }
    if (ImGui::CollapsingHeader("Water", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("IOR", ImGuiDataType_Double, &renderParams.ior, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.ior = std::max(renderParams.ior, 1.0001);
        ImGui::Checkbox("Thickness", &renderParams.enableThickness);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Refraction", ImGuiDataType_Double, &renderParams.refractionScale, 0.001f, nullptr, nullptr, "%.3f");
        renderParams.refractionScale = std::max(renderParams.refractionScale, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Absorption", ImGuiDataType_Double, &renderParams.absorptionStrength, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.absorptionStrength = std::max(renderParams.absorptionStrength, 0.0);
    }

    if (ImGui::CollapsingHeader("Rendering")) {
        ImGui::Checkbox("Caustics", &renderParams.enableCaustics);
        ImGui::Checkbox("Thickness Pass", &renderParams.enableThickness);

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
        ImGui::DragScalar("Max Thickness", ImGuiDataType_Double, &renderParams.maxThickness, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.maxThickness = std::max(renderParams.maxThickness, 0.001);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Caustic Strength", ImGuiDataType_Double, &renderParams.causticStrength, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.causticStrength = std::max(renderParams.causticStrength, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Caustic Point Size", ImGuiDataType_Double, &renderParams.causticPointSize, 0.05f, nullptr, nullptr, "%.3f");
        renderParams.causticPointSize = std::max(renderParams.causticPointSize, 0.1);

        float absorption[3] = {
            static_cast<float>(renderParams.absorptionColor.x()),
            static_cast<float>(renderParams.absorptionColor.y()),
            static_cast<float>(renderParams.absorptionColor.z()),
        };
        ImGui::SetNextItemWidth(vectorInputWidth);
        if (ImGui::ColorEdit3("Absorption Color", absorption)) {
            renderParams.absorptionColor = Vec3(absorption[0], absorption[1], absorption[2]);
        }
    }

    if (ImGui::CollapsingHeader("Surface")) {
        if (planeSideLength > 0.0) {
            double editedSideLength = planeSideLength;
            ImGui::SetNextItemWidth(vectorInputWidth);
            if (ImGui::DragScalar("Plane Size", ImGuiDataType_Double, &editedSideLength, 0.05f, nullptr, nullptr, "%.3f")) {
                actions.setPlaneSideLength = true;
                actions.planeSideLength = std::max(editedSideLength, 0.1);
            }
        }
        if (ImGui::Button("Reset Plane")) {
            actions.resetPlaneAndDisableInteraction = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Enable Tilt")) {
            actions.resetPlaneAndDisableInteraction = false;
        }
        if (ImGui::Button("Disable Tilt")) {
            actions.disable_tilt = true;
        }
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Friction", ImGuiDataType_Double, &surfaceMaterial.friction, 0.01f, nullptr, nullptr, "%.3f");
        surfaceMaterial.friction = std::max(surfaceMaterial.friction, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Adhesion", ImGuiDataType_Double, &solverParams.adhesionDistance, 0.005f, nullptr, nullptr, "%.3f");
        solverParams.adhesionDistance = std::max(solverParams.adhesionDistance, 0.0);
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(vectorInputWidth);
        if (ImGui::SliderInt("Mesh Points", &meshSubdivisions_, 0, 5)) {
            actions.setMeshSubdivisions = true;
            actions.meshSubdivisions = meshSubdivisions_;
        }
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Surface Tension", ImGuiDataType_Double, &defaultMaterial.surfaceTension, 0.01f, nullptr, nullptr, "%.3f");
        defaultMaterial.surfaceTension = std::max(defaultMaterial.surfaceTension, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Damping", ImGuiDataType_Double, &defaultMaterial.viscousDamping, 0.01f, nullptr, nullptr, "%.3f");
        defaultMaterial.viscousDamping = std::max(defaultMaterial.viscousDamping, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Viscosity", ImGuiDataType_Double, &defaultMaterial.laplacianViscosity, 0.01f, nullptr, nullptr, "%.3f");
        defaultMaterial.laplacianViscosity = std::max(defaultMaterial.laplacianViscosity, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Volume Stiffness", ImGuiDataType_Double, &defaultMaterial.volumeStiffness, 1.0f, nullptr, nullptr, "%.1f");
        defaultMaterial.volumeStiffness = std::max(defaultMaterial.volumeStiffness, 0.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Density", ImGuiDataType_Double, &defaultMaterial.density, 0.01f, nullptr, nullptr, "%.3f");
        defaultMaterial.density = std::max(defaultMaterial.density, 1e-6);
    }

    if (ImGui::CollapsingHeader("Advanced")) {
        int debugView = static_cast<int>(renderParams.debugView);
        ImGui::SetNextItemWidth(vectorInputWidth);
        if (ImGui::Combo("Debug View", &debugView, kDebugViewLabels, IM_ARRAYSIZE(kDebugViewLabels))) {
            renderParams.debugView = static_cast<RenderDebugView>(debugView);
        }
        ImGui::Checkbox("Contact Angle", &solverParams.enableContactAngle);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Receding", ImGuiDataType_Double, &surfaceMaterial.recContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Advancing", ImGuiDataType_Double, &surfaceMaterial.advContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
        surfaceMaterial.recContactAngleDeg = std::clamp(surfaceMaterial.recContactAngleDeg, 0.0, 180.0);
        surfaceMaterial.advContactAngleDeg = std::clamp(surfaceMaterial.advContactAngleDeg, 0.0, 180.0);
        ImGui::SetNextItemWidth(vectorInputWidth);
        ImGui::DragScalar("Contact Stiff.", ImGuiDataType_Double, &surfaceMaterial.contactStiffness, 0.01f, nullptr, nullptr, "%.3f");
        surfaceMaterial.contactStiffness = std::max(surfaceMaterial.contactStiffness, 0.0);
    }

    ImGui::End();

    return actions;
}

} // namespace wd
