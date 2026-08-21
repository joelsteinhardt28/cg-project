#include "gui.hpp"

#include <filesystem>

#include "polyscope/messages.h"
#include "polyscope/curve_network.h"
#include "imgui.h"

#include <pmp/io/io.h>

#include "io.hpp"
#include "mesh_utils.hpp"
#include "kernel_gen.hpp"

namespace gui {

/**
 * Resets the polyscope application state by clearing all registered structures and resetting the AppState
 * fields to their default values.
 */
void reset(AppState& state) {
    print::info("Resetting application state...");
    polyscope::removeAllStructures();
    state.mesh = pmp::SurfaceMesh();
    state.kHat = pmp::SurfaceMesh();
    state.meshLoaded = false;
    state.oSMesh = nullptr;
    state.pc = nullptr;
    state.kSMesh = nullptr;
    state.bboxVertices.clear();
    state.supportPlanes.clear();
    state.exactSupportPlanes.clear();
    state.hasActiveCutPlane = false;
    state.isSteppingKernel = false;
    state.currentPlaneIdx = 0;
    state.lastComputeTime = 0.0;
    state.skippedCuts = 0;
    state.statusMessage.clear();
    state.statusMessageColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
 * Loads an OFF file by index, resets application state, and initializes properties and Polyscope structures.
 */
void loadOffFile(AppState& state, int fileIdx) {
    if (fileIdx < 0 || fileIdx >= static_cast<int>(state.offFiles.size())) return;

    reset(state);
    state.selectedOffFileIdx = fileIdx;
    std::filesystem::path selectedPath = std::filesystem::path(state.targetDir) / state.offFiles[fileIdx];

    try {
        pmp::SurfaceMesh tempMesh;
        print::info("Reading mesh from: " + selectedPath.string());
        pmp::read(tempMesh, selectedPath);

        // Properties for ipg
        auto exactPoints = tempMesh.add_vertex_property<ExactPoint>("v:exact_pos");
        auto exactPlanes = tempMesh.add_face_property<ExactPlane>("f:exact_plane");

        // Scale and snap vertex positions to integer grid and store in pmp property
        for (auto v : tempMesh.vertices()) {
            pmp::Point p = tempMesh.position(v);
            tg::ipos3 ipos(
                static_cast<int64_t>(p[0] * globalSettings::scaleFactor),
                static_cast<int64_t>(p[1] * globalSettings::scaleFactor),
                static_cast<int64_t>(p[2] * globalSettings::scaleFactor)
            );
            exactPoints[v] = ExactPoint(ipos);
        }

        for (auto face : tempMesh.faces()) {
            auto it = tempMesh.vertices(face).begin();
            ExactPoint vA = exactPoints[*it]; ++it;
            ExactPoint vB = exactPoints[*it]; ++it;
            ExactPoint vC = exactPoints[*it];

            tg::pos<3, ExactGeom::pos_scalar_t> pA(int64_t(vA.x), int64_t(vA.y), int64_t(vA.z));
            tg::pos<3, ExactGeom::pos_scalar_t> pB(int64_t(vB.x), int64_t(vB.y), int64_t(vB.z));
            tg::pos<3, ExactGeom::pos_scalar_t> pC(int64_t(vC.x), int64_t(vC.y), int64_t(vC.z));

            exactPlanes[face] = ExactPlane::from_points(pA, pB, pC);
        }

        state.mesh = std::move(tempMesh);
        print::info("Mesh loaded: " + std::to_string(state.mesh.n_vertices()) + " vertices, " + std::to_string(state.mesh.n_faces()) + " faces.");

        state.oSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::mesh), state.mesh);
        state.oSMesh->setSurfaceColor(constants::colors::mesh);
        state.oSMesh->setTransparency(constants::transparencies::mesh);
        state.pc = mesh_utils::register_pmp_pc(std::string(constants::polyNames::pc), state.mesh);
        if (!state.mesh.is_empty()) state.meshLoaded = true;

        mesh_utils::register_bbox(state);

        // Point cloud cosmetics
        state.pc->setEnabled(false);

        // Reset camera to fit the new mesh
        polyscope::view::resetCameraToHomeView();
    } catch (const std::exception& e) {
        print::error("Error while loading " + selectedPath.string() + ": " + std::string(e.what()));
        state.selectedOffFileIdx = -1;
        state.meshLoaded = false;
    }
}

/**
 * Helper to scan the OFF file directory and populate the list in AppState of available files
 */
void refreshOffFileList(AppState& state) {
    state.offFiles.clear();
    if (!std::filesystem::exists(state.targetDir)) return;

    for (const auto& entry : std::filesystem::directory_iterator(state.targetDir)) {
        if (entry.path().extension() == ".off") {
            state.offFiles.push_back(entry.path().filename().string());
        }
    }

    std::sort(state.offFiles.begin(), state.offFiles.end());
}    


void render(AppState& state) {
    if (state.offFiles.empty()) {
        refreshOffFileList(state);
    }

    // * Mesh Loading Section
    ImGui::TextColored(constants::colors::guiTitle, "=== MESH LOADING ===");
    ImGui::Text("OFF Files in Directory: %s", state.targetDir.c_str());

    // * Dropdown to select OFF file
    const char* preview = (state.selectedOffFileIdx >= 0) ? state.offFiles[state.selectedOffFileIdx].c_str() : "Select an OFF file";

    if (ImGui::BeginCombo("OFF File", preview)) {
        int nFiles = state.offFiles.size();
        for (int i = 0; i < nFiles; i++) {
            const bool isSelected = (state.selectedOffFileIdx == i);
            if (ImGui::Selectable(state.offFiles[i].c_str(), isSelected)) {
                loadOffFile(state, i);
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    // * Reset Button to reload the currently loaded mesh (at bottom of Mesh Loading section)
    if (!state.meshLoaded || state.selectedOffFileIdx < 0) {
        ImGui::BeginDisabled();
    }

    ImGui::PushStyleColor(ImGuiCol_Button, constants::colors::guiResetButton);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, constants::colors::guiResetButtonHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, constants::colors::guiResetButtonActive);
    
    if (ImGui::Button("Reset", constants::gui::buttonSize)) {
        loadOffFile(state, state.selectedOffFileIdx);
    }

    ImGui::PopStyleColor(3);  // For Reset Button

    if (!state.meshLoaded || state.selectedOffFileIdx < 0) {
        ImGui::EndDisabled();
    }


    if (state.meshLoaded) {
        if (ImGui::Button("Visualize Face Normals", constants::gui::buttonSize)) {
            mesh_utils::visualize_face_normals(state);
        }

        ImGui::SameLine();

        if (ImGui::Button("Identify Concave Faces", constants::gui::buttonSize)) {
            std::vector<bool> isConcave = mesh_utils::identify_concave_faces(state.mesh);
            std::vector<double> scalarVal(state.mesh.n_faces());
            for (size_t i = 0; i < isConcave.size(); ++i) {
                scalarVal[i] = isConcave[i] ? 1.0 : 0.0;
            }
            state.oSMesh->addFaceScalarQuantity("isConcave", scalarVal)->setEnabled(true);
        }

        ImGui::Separator();

        // * Mesh Kernel Generation Section
        ImGui::TextColored(constants::colors::guiTitle, "=== MESH KERNEL GENERATION ===");

        ImGui::PushStyleColor(ImGuiCol_Button, constants::colors::guiLimeButton);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, constants::colors::guiLimeButtonHovered);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, constants::colors::guiLimeButtonActive);

        if (ImGui::Button("Generate Kernel", constants::gui::buttonSize)) {
            auto start = std::chrono::high_resolution_clock::now();
            generate_kernel(state);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            state.lastComputeTime = elapsed.count();
        }

        ImGui::SameLine();

        if (ImGui::Button("Generate Kernel Parallel", constants::gui::buttonSize)) {
            generate_kernel_parallel(state);
        }

        ImGui::PopStyleColor(3);  // For Lime Button

        // * Toggle Cut Plane & Normal Visibility button (below Generation buttons)
        bool hasCutPlane = polyscope::hasSurfaceMesh(std::string(constants::polyNames::cutPlane));
        bool hasCutPlaneNormal = polyscope::hasCurveNetwork(std::string(constants::polyNames::cutPlaneNormal));
        bool bothCreated = hasCutPlane && hasCutPlaneNormal;

        if (!bothCreated) {
            ImGui::BeginDisabled();
        }

        bool currentlyVisible = false;
        if (bothCreated) {
            auto* planeMesh = polyscope::getSurfaceMesh(std::string(constants::polyNames::cutPlane));
            auto* normalCN = polyscope::getCurveNetwork(std::string(constants::polyNames::cutPlaneNormal));
            currentlyVisible = planeMesh->isEnabled() || normalCN->isEnabled();
        }

        std::string btnText = currentlyVisible ? "Hide Cut Plane & Normal" : "Show Cut Plane & Normal";
        if (ImGui::Button(btnText.c_str(), constants::gui::buttonSize)) {
            if (bothCreated) {
                auto* planeMesh = polyscope::getSurfaceMesh(std::string(constants::polyNames::cutPlane));
                auto* normalCN = polyscope::getCurveNetwork(std::string(constants::polyNames::cutPlaneNormal));
                bool nextState = !currentlyVisible;
                planeMesh->setEnabled(nextState);
                normalCN->setEnabled(nextState);
            }
        }

        if (!bothCreated) {
            ImGui::EndDisabled();
        }

        if (!state.statusMessage.empty()) {
            ImGui::TextColored(state.statusMessageColor, "%s", state.statusMessage.c_str());
        }

        if (state.lastComputeTime > 0.0) {
            ImGui::TextColored(constants::colors::guiInfo, "Last compute time: %.4f seconds", state.lastComputeTime);
        }

        ImGui::Separator();
        ImGui::TextColored(constants::colors::guiTitle, "=== KERNEL STEPPING ===");

        ImGui::Checkbox("Update Visuals During Stepping", &state.updateVisuals);

        if (!state.isSteppingKernel) {
            ImGui::PushStyleColor(ImGuiCol_Button, constants::colors::guiLimeButton);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, constants::colors::guiLimeButtonHovered);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, constants::colors::guiLimeButtonActive);
            
            if (ImGui::Button("Start Kernel Stepping", constants::gui::buttonSize)) {
                init_kernel_stepping(state);
            }

            ImGui::PopStyleColor(3);  // For Lime Button

            if (state.kSMesh) {
                ImGui::Separator();
                ImGui::Text("Kernel Generation Finished");
                ImGui::Text("Total Planes Processed: %zu", state.supportPlanes.size());
                ImGui::TextColored(constants::colors::guiInfo, "Cuts skipped (AABB Check): %d", state.skippedCuts);
            }
        } else {
            ImGui::Text("Step: %d / %zu", state.currentPlaneIdx, state.supportPlanes.size());
            ImGui::TextColored(constants::colors::guiInfo, "Cuts Skipped (AABB Check): %d", state.skippedCuts);
            if (ImGui::Button("Next Step")) {
                step_kernel(state, state.updateVisuals);
            }
            ImGui::SameLine();
            if (ImGui::Button("Finish Kernel")) {
                while (state.isSteppingKernel) {
                    step_kernel(state, state.updateVisuals);
                }

                // Final visual update
                if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
                state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
                state.kSMesh->setSurfaceColor(constants::colors::kernel);
                state.kSMesh->setTransparency(constants::transparencies::kernel);
            }
            if (ImGui::Button("Cancel Stepping")) {
                state.isSteppingKernel = false;
            }
        }
        
    }
}

} // namespace gui
