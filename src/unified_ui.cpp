#include "unified_ui.h"

#include <iostream>

#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui.h>

namespace hlk {

UnifiedUI::UnifiedUI(int rosy, std::string in_path, std::string out_path)
    : remesh_(rosy, std::move(in_path), std::move(out_path)) {
    plugin_name = "Unified";
}

void UnifiedUI::init(igl::opengl::glfw::Viewer* v) {
    // Order: our own ImGuiMenu init first, so the global imgui context
    // exists. Then init the children — they each call ImGuiMenu::init
    // again, which is idempotent (uses a static global context, GLFW
    // backend init with install_callbacks=false, etc.). Each child also
    // does its own viewer-state setup (RemeshingMenu appends three data
    // slots and installs a key callback; LabelingUI is lazy).
    igl::opengl::glfw::imgui::ImGuiMenu::init(v);
    remesh_.init(v);
    n_remesh_slots_ = (int)v->data_list.size();
    label_.init(v);
    apply_mode_visibility();
}

bool UnifiedUI::mouse_down(int button, int modifier) {
    return active().mouse_down(button, modifier);
}
bool UnifiedUI::mouse_up(int button, int modifier) {
    return active().mouse_up(button, modifier);
}
bool UnifiedUI::mouse_move(int mouse_x, int mouse_y) {
    return active().mouse_move(mouse_x, mouse_y);
}
bool UnifiedUI::mouse_scroll(float delta_y) {
    return active().mouse_scroll(delta_y);
}
bool UnifiedUI::key_down(int key, int modifiers) {
    return active().key_down(key, modifiers);
}
bool UnifiedUI::key_up(int key, int modifiers) {
    return active().key_up(key, modifiers);
}
bool UnifiedUI::key_pressed(unsigned int key, int modifiers) {
    return active().key_pressed(key, modifiers);
}

void UnifiedUI::draw_viewer_menu() {
    draw_mode_toolbar();
    active().draw_viewer_menu();
}

void UnifiedUI::draw_custom_window() {
    active().draw_custom_window();
}

void UnifiedUI::draw_mode_toolbar() {
    // Sits at the top of the (shared) viewer menu window; ImGuiMenu's
    // draw_viewer_window has already opened a Begin("Viewer") for us.
    if (ImGui::CollapsingHeader("Stage", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool is_remesh = (mode_ == Mode::Remesh);
        bool is_label  = (mode_ == Mode::Label);
        if (ImGui::RadioButton("Remesh", is_remesh) && !is_remesh) {
            mode_ = Mode::Remesh;
            apply_mode_visibility();
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Label", is_label) && !is_label) {
            mode_ = Mode::Label;
            apply_mode_visibility();
        }

        // From the remesh side, offer one-click promotion of the extracted
        // quad mesh into the labeling UI. The bundled imgui v1.69 predates
        // BeginDisabled, so we just suppress the click and tag the state.
        if (mode_ == Mode::Remesh) {
            const bool has_quads = remesh_.is_quad_meshed
                                && remesh_.quad_mesh.V.rows() > 0
                                && remesh_.quad_mesh.F_q.rows() > 0;
            const float w = ImGui::GetContentRegionAvailWidth();
            if (ImGui::Button("Send quad mesh -> Label", ImVec2(w, 0)) && has_quads) {
                send_to_label();
            }
            if (!has_quads) {
                ImGui::TextDisabled("(extract a quad mesh first)");
            }
        }
    }
    ImGui::Separator();
}

void UnifiedUI::send_to_label() {
    if (!viewer) return;
    if (remesh_.quad_mesh.V.rows() == 0 || remesh_.quad_mesh.F_q.rows() == 0) {
        std::cerr << "[hlk] Send to Label: no quad mesh available yet.\n";
        return;
    }

    // Make sure LabelingUI builds its own data slots fresh, after the
    // remeshing slots, instead of clobbering remesh_'s slot 0.
    if (!label_.has_loaded_mesh()) {
        viewer->selected_data_index = viewer->append_mesh();
    }

    label_.load_quad_mesh_in_memory(remesh_.quad_mesh.V, remesh_.quad_mesh.F_q);
    mode_ = Mode::Label;
    apply_mode_visibility();
}

void UnifiedUI::apply_mode_visibility() {
    if (!viewer) return;
    const bool show_remesh = (mode_ == Mode::Remesh);
    for (int i = 0; i < (int)viewer->data_list.size(); ++i) {
        const bool is_remesh_slot = (i < n_remesh_slots_);
        const bool show = is_remesh_slot ? show_remesh : !show_remesh;
        auto& d = viewer->data(i);
        d.show_faces = show;
        d.show_lines = show;
        d.show_overlay = show;
        d.show_overlay_depth = show;
        d.show_texture = show;
        d.show_vertid = show && d.show_vertid;
        d.show_faceid = show && d.show_faceid;
    }
}

} // namespace hlk
