#pragma once
//
// UnifiedUI hosts both the RemeshingMenu and the LabelingUI as a single
// libigl ImGuiMenu plugin so that hlk can expose a single 2-phase UI:
// design the cross/frame field and extract the quad mesh, then promote
// directly into labeling without quitting and re-launching.
//
// Only one mode is "active" at a time. The active mode owns the screen:
// its draw_viewer_menu() and draw_custom_window() are forwarded to,
// and its mouse/keyboard events are the only ones dispatched. Going
// from remesh to label hands the quad mesh over in-memory (no OBJ
// roundtrip); going back to remesh is a pure mode flip.
//

#include <string>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "remeshing_plugin.h"
#include "labeling_ui.h"

namespace hlk {

class UnifiedUI : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    enum class Mode { Remesh, Label };

    UnifiedUI(int rosy, std::string in_path, std::string out_path);

    // ViewerPlugin overrides
    void init(igl::opengl::glfw::Viewer* viewer) override;

    bool mouse_down(int button, int modifier) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_scroll(float delta_y) override;
    bool key_down(int key, int modifiers) override;
    bool key_up(int key, int modifiers) override;
    bool key_pressed(unsigned int key, int modifiers) override;

    // ImGuiMenu draw hooks. ImGuiMenu::draw_menu() calls these from
    // post_draw(); we override both to delegate to the active child
    // (after drawing our own mode toolbar in draw_viewer_menu).
    void draw_viewer_menu() override;
    void draw_custom_window() override;

    // Direct access for callers that need to preconfigure each child
    // before launch (e.g. main_hlk.cpp setting an input model).
    RemeshingMenu& remesh() { return remesh_; }
    LabelingUI& label() { return label_; }

    Mode mode() const { return mode_; }

private:
    Mode mode_ = Mode::Remesh;
    RemeshingMenu remesh_;
    LabelingUI label_;

    // Number of viewer data slots that belong to the remeshing UI. Captured
    // immediately after remesh_.init(), and used to toggle visibility when
    // we switch to/from labeling.
    int n_remesh_slots_ = 0;

    igl::opengl::glfw::imgui::ImGuiMenu& active() {
        return mode_ == Mode::Remesh
                 ? static_cast<igl::opengl::glfw::imgui::ImGuiMenu&>(remesh_)
                 : static_cast<igl::opengl::glfw::imgui::ImGuiMenu&>(label_);
    }

    void draw_mode_toolbar();

    // Remesh → Label handoff. Copies remesh_.quad_mesh into label_.M
    // (in-memory) and switches active mode. Idempotent — a second call
    // re-pushes the current quad mesh.
    void send_to_label();

    // Hide all data slots owned by the off-mode UI; show the on-mode ones.
    void apply_mode_visibility();
};

} // namespace hlk
