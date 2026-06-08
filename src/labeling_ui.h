#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/texture_from_file.h>
#include <igl/file_dialog_open.h>

#include <string>

#include "coarse_knit_mesh.h"

namespace hlk {
    class LabelingUI : public igl::opengl::glfw::imgui::ImGuiMenu {
    public:

        //void set_mesh(CoarseKnitMesh m);

        bool load_quad_mesh_file();

        bool mouse_down(int button, int modifier);

        bool mouse_up(int button, int modifier);

        bool mouse_move(int mouse_x, int mouse_y);

        void draw_viewer_menu();

        void update_mesh();

    private:

        CoarseKnitMesh M;
        bool mesh_loaded = false;
        int base_index = 0;
        int overlay_index = -1;

        enum Tool {
            ERASER, // Remove constraints
            TEXTURER, // Paint on textures
            SEAMER, // Add, remove, and split seams
            ORIENTER, // Paint directions and orientations
            MEASURER // Tape Measure - set lines of constrained length
        };

        enum EraserMode {
            ERASE_ORIENTATIONS,
            ERASE_SEAMS,
            ERASE_TEXTURES,
            ERASE_CONSTRAINTS
        };

        // Modes and settings

        Tool current_tool = ORIENTER;
        EraserMode eraser_mode = ERASE_ORIENTATIONS;
        KnitDirection orienter_mode = LOOP;

        std::string eraser_instructions = "CTRL-Click and drag to erase.";
        std::string texturer_instructions = "CTRL-Click and drag to add\ntexture.";
        std::string seamer_instructions = "CTRL-Click and drag to join\nseams.";
        std::string orienter_instructions = "CTRL-Click and drag to set\norientation. Left click for loop,\nright click for yarn.";
        std::string measurer_instructions = "CTRL-Click and drag to add a\nconstraint. Click an outgoing\nedge to add a constraint.\nShift-Click to add separate\nconstraints.";

        std::string instructions = orienter_instructions;

        bool is_dragging = false;
        int dragging_button = (int) igl::opengl::glfw::Viewer::MouseButton::Left;
        int drag_start_side = -1;
        int last_drag_side = -1;

        bool pick_face(int& fid, Eigen::Vector3f& bc);

        // Whether to re-run the solver on mouse-up or not
        bool auto_solve;

        // Texture Names - TODO - We probably want a second data structure to hold a database of these
        std::vector<const char*> textures{ "stockinette", "ribbing" };
        int current_texture = 0;

        // Debug Tooltips On
        bool show_debug_tooltip = false;

        // Texture Handles for UI
        bool textures_loaded = false;
        GLuint eraser_tex, brush_tex, seamer_tex, orienter_tex, measurer_tex;
        GLuint eraser_pressed, brush_pressed, seamer_pressed, orienter_pressed, measurer_pressed;

        void load_textures();

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;


        /*
        Icon Credits:
        <div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        <div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        <div>Icons made by <a href="https://www.flaticon.com/authors/vaadin" title="Vaadin">Vaadin</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        <div>Icons made by <a href="https://www.flaticon.com/authors/monkik" title="monkik">monkik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        <div>Icons made by <a href="https://www.flaticon.com/authors/good-ware" title="Good Ware">Good Ware</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        <div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
        */
    };
}
