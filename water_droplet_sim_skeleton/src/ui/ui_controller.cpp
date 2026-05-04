#include "wd/ui/ui_controller.h"

#include <algorithm>
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

    (void)planeTiltEnabled;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const float width = ImGui::CalcTextSize("-000.000  -000.000  -000.000").x;

    if (ImGui::CollapsingHeader("Droplet", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalarN("Spawn", ImGuiDataType_Double, spawnAnchor_.data(), 3, 0.01f, nullptr, nullptr, "%.3f");
        if (ImGui::Button("Spawn Droplet")) {
            actions.createDroplet = true;
            actions.spawnAnchor = spawnAnchor_;
        }
        ImGui::SameLine();
        if (ImGui::Button("Screenshot")) {
            actions.saveScreenshot = true;
        }

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalarN("Gravity", ImGuiDataType_Double, gravityDraft_.data(), 3, 0.05f, nullptr, nullptr, "%.3f");
        if (ImGui::Button("Apply Gravity")) {
            actions.applyGravity = true;
            actions.gravityForce = gravityDraft_;
        }

        int debugView = static_cast<int>(renderParams.debugView);
        ImGui::SetNextItemWidth(width);
        if (ImGui::Combo("Debug View", &debugView, kDebugViewLabels, IM_ARRAYSIZE(kDebugViewLabels))) {
            renderParams.debugView = static_cast<RenderDebugView>(debugView);
        }
    }

    if (ImGui::CollapsingHeader("Water", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("IOR", ImGuiDataType_Double, &renderParams.ior, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.ior = std::max(renderParams.ior, 1.0001);

        ImGui::Checkbox("Thickness", &renderParams.enableThickness);

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Refraction Scale", ImGuiDataType_Double, &renderParams.refractionScale, 0.001f, nullptr, nullptr, "%.3f");
        renderParams.refractionScale = std::max(renderParams.refractionScale, 0.0);

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Max Thickness", ImGuiDataType_Double, &renderParams.maxThickness, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.maxThickness = std::max(renderParams.maxThickness, 0.001);

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Absorption Strength", ImGuiDataType_Double, &renderParams.absorptionStrength, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.absorptionStrength = std::max(renderParams.absorptionStrength, 0.0);

        float absorption[3] = {
            static_cast<float>(renderParams.absorptionColor.x()),
            static_cast<float>(renderParams.absorptionColor.y()),
            static_cast<float>(renderParams.absorptionColor.z()),
        };
        ImGui::SetNextItemWidth(width);
        if (ImGui::ColorEdit3("Absorption Color", absorption)) {
            renderParams.absorptionColor = Vec3(absorption[0], absorption[1], absorption[2]);
        }

        ImGui::Checkbox("Caustics", &renderParams.enableCaustics);
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Caustic Strength", ImGuiDataType_Double, &renderParams.causticStrength, 0.01f, nullptr, nullptr, "%.3f");
        renderParams.causticStrength = std::max(renderParams.causticStrength, 0.0);

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Caustic Size", ImGuiDataType_Double, &renderParams.causticPointSize, 0.05f, nullptr, nullptr, "%.3f");
        renderParams.causticPointSize = std::max(renderParams.causticPointSize, 0.1);
    }

    if (ImGui::CollapsingHeader("Surface", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (planeSideLength > 0.0) {
            double editedSideLength = planeSideLength;
            ImGui::SetNextItemWidth(width);
            if (ImGui::DragScalar("Plane Size", ImGuiDataType_Double, &editedSideLength, 0.05f, nullptr, nullptr, "%.3f")) {
                actions.setPlaneSideLength = true;
                actions.planeSideLength = std::max(editedSideLength, 0.1);
            }
        }

        ImGui::TextUnformatted("Plane tilt is always active");

        double maxTilt = planeTiltMaxDeg;
        ImGui::SetNextItemWidth(width);
        if (ImGui::DragScalar("Max Tilt (deg)", ImGuiDataType_Double, &maxTilt, 0.25f, nullptr, nullptr, "%.2f")) {
            actions.setPlaneTiltMaxDeg = true;
            actions.planeTiltMaxDeg = std::clamp(maxTilt, 0.0, 30.0);
        }

        double responsiveness = planeTiltResponsiveness;
        ImGui::SetNextItemWidth(width);
        if (ImGui::DragScalar("Responsiveness", ImGuiDataType_Double, &responsiveness, 0.1f, nullptr, nullptr, "%.2f")) {
            actions.setPlaneTiltResponsiveness = true;
            actions.planeTiltResponsiveness = std::max(responsiveness, 0.0);
        }

        double axisScaleX = planeTiltAxisScaleX;
        ImGui::SetNextItemWidth(width);
        if (ImGui::DragScalar("Axis Scale X", ImGuiDataType_Double, &axisScaleX, 0.05f, nullptr, nullptr, "%.2f")) {
            actions.setPlaneTiltAxisScaleX = true;
            actions.planeTiltAxisScaleX = axisScaleX;
        }

        double axisScaleZ = planeTiltAxisScaleZ;
        ImGui::SetNextItemWidth(width);
        if (ImGui::DragScalar("Axis Scale Z", ImGuiDataType_Double, &axisScaleZ, 0.05f, nullptr, nullptr, "%.2f")) {
            actions.setPlaneTiltAxisScaleZ = true;
            actions.planeTiltAxisScaleZ = axisScaleZ;
        }

        if (ImGui::Button("Reset Plane")) {
            actions.resetPlaneAndDisableInteraction = true;
        }

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Surface IOR", ImGuiDataType_Double, &surfaceRender.ior, 0.01f, nullptr, nullptr, "%.3f");
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Opacity", ImGuiDataType_Double, &surfaceRender.opacity, 0.01f, nullptr, nullptr, "%.3f");
        surfaceRender.opacity = std::clamp(surfaceRender.opacity, 0.0, 1.0);

        float tint[3] = {
            static_cast<float>(surfaceRender.tintColor.x()),
            static_cast<float>(surfaceRender.tintColor.y()),
            static_cast<float>(surfaceRender.tintColor.z()),
        };
        ImGui::SetNextItemWidth(width);
        if (ImGui::ColorEdit3("Tint", tint)) {
            surfaceRender.tintColor = Vec3(tint[0], tint[1], tint[2]);
        }

        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Friction", ImGuiDataType_Double, &surfaceMaterial.friction, 0.01f, nullptr, nullptr, "%.3f");
        surfaceMaterial.friction = std::max(surfaceMaterial.friction, 0.0);

        ImGui::Checkbox("Contact Angle", &solverParams.enableContactAngle);
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Receding Angle", ImGuiDataType_Double, &surfaceMaterial.recContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Advancing Angle", ImGuiDataType_Double, &surfaceMaterial.advContactAngleDeg, 1.0f, nullptr, nullptr, "%.1f");
        surfaceMaterial.recContactAngleDeg = std::clamp(surfaceMaterial.recContactAngleDeg, 0.0, 180.0);
        surfaceMaterial.advContactAngleDeg = std::clamp(surfaceMaterial.advContactAngleDeg, 0.0, 180.0);
        ImGui::SetNextItemWidth(width);
        ImGui::DragScalar("Contact Stiffness", ImGuiDataType_Double, &surfaceMaterial.contactStiffness, 0.01f, nullptr, nullptr, "%.3f");
        surfaceMaterial.contactStiffness = std::max(surfaceMaterial.contactStiffness, 0.0);
    }

    ImGui::End();
    return actions;
}

} // namespace wd
