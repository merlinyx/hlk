#include "labeling_ui.h"

#include <imgui.h>
#include <igl/png/readPNG.h>
#include <igl/unproject_onto_mesh.h>

#include "read_quad_mesh.h"

#include "glyph.h"
#include "glyphs.h"

namespace hlk {
    bool LabelingUI::load_quad_mesh_file()
    {
        std::string filename = igl::file_dialog_open();
        if (filename.size() > 0) {
            read_quad_mesh(filename, M, true);

            // Setup the base mesh
            viewer->data().set_mesh(M.V, M.F_t);
            viewer->data().set_colors(Eigen::RowVector4d(1.0, 1.0, 1.0, 1.0));
            base_index = viewer->selected_data_index;

            overlay_index = viewer->append_mesh();
            viewer->data().set_mesh(M.LV, M.LF);
            viewer->data().set_texture(R, G, B, A);
            viewer->data().set_uv(M.UV);
            viewer->data().show_texture = true;
            viewer->data().show_lines = false;
            viewer->data().set_colors(M.C);

            // Set the data index back to the underlying mesh
            viewer->selected_data_index = base_index;

            mesh_loaded = true;
            return true;
        }
        return false;
    }
    bool LabelingUI::mouse_down(int button, int modifier) {
        if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_down(button, modifier)) return true;
        
        if (modifier & IGL_MOD_CONTROL) {
            // Get the starting side
            
            int fid;
            Eigen::Vector3f bc;
            if (pick_face(fid, bc)) {

                drag_start_side = fid;
                last_drag_side = fid;

                is_dragging = true;
                dragging_button = button;

                if (current_tool == ORIENTER) {
                    std::cout << "mouse button = " << button << std::endl;
                    if (button == (int)igl::opengl::glfw::Viewer::MouseButton::Left) {
                        orienter_mode = LOOP;
                    }
                    else {
                        orienter_mode = YARN;
                    }
                }

                return true;
            }

            return false;
        }

        return false;
    }

    bool LabelingUI::mouse_up(int button, int modifier) {
        if (is_dragging) {
            // Finalize Dragging Action

            is_dragging = false;
            drag_start_side = -1;

            if (auto_solve) {
                bool sat = M.optimize_geometry();
                update_mesh();
                if (!sat) {
                    std::cout << "UNSAT!" << std::endl;
                }
                // TODO - Handle invalid constraints
            }

            return true;
        }

        return false;
    }

    bool LabelingUI::mouse_move(int mouse_x, int mouse_y) {
        if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_move(mouse_x, mouse_y)) return true;

        if (is_dragging) {
            int fid;
            Eigen::Vector3f bc;
            if (pick_face(fid, bc)) {

                if (current_tool == ORIENTER) {

                    if (M.flip_side(fid) == last_drag_side) {
                        M.paint_direction(last_drag_side, fid, orienter_mode);
                        update_mesh();
                    }
                    last_drag_side = fid;
                    return true;
                }

                if (current_tool == ERASER) {
                    if (eraser_mode == ERASE_ORIENTATIONS) {
                        M.erase_orientation(fid);
                        update_mesh();
                    }
                }
            }
        }

        return false;
    }


    void LabelingUI::draw_viewer_menu() {
        load_textures();
        auto tooltip = [](std::string text) {
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                ImGui::TextUnformatted(text.c_str());
                ImGui::PopTextWrapPos();
                ImGui::EndTooltip();
            }
        };
        auto mode_selector = [&](Tool t, GLuint pressed, GLuint unpressed, std::string tool_instructions, std::string name) {
            if (ImGui::ImageButton((void*)(intptr_t)(current_tool == t ? pressed : unpressed), ImVec2(32, 32))) {
                current_tool = t;
                instructions = tool_instructions;
            }
            tooltip(name);
        };

        mode_selector(ERASER, eraser_pressed, eraser_tex, eraser_instructions, "Eraser Tool");
        if (current_tool == ERASER) {
            ImGui::RadioButton("Erase Orientations", (int*)&eraser_mode, ERASE_ORIENTATIONS);
            ImGui::RadioButton("Erase Seams", (int*)&eraser_mode, ERASE_SEAMS);
            ImGui::RadioButton("Erase Textures", (int*)&eraser_mode, ERASE_TEXTURES);
            ImGui::RadioButton("Erase Constraints", (int*)&eraser_mode, ERASE_CONSTRAINTS);
        }
        mode_selector(TEXTURER, brush_pressed, brush_tex, texturer_instructions, "Texturing Tool");
        if (current_tool == TEXTURER) {
            ImGui::SameLine();
            ImGui::PushItemWidth(100);
            ImGui::Combo("", &current_texture, textures.data(), textures.size());
            ImGui::PopItemWidth();
        }
        mode_selector(SEAMER, seamer_pressed, seamer_tex, seamer_instructions, "Seaming Tool");
        if (current_tool == SEAMER) {

        }
        mode_selector(ORIENTER, orienter_pressed, orienter_tex, orienter_instructions, "Orienting Tool");
        if (current_tool == ORIENTER) {
            ImGui::RadioButton("Loop", (int*)& orienter_mode, LOOP);
            ImGui::RadioButton("Yarn", (int*)& orienter_mode, YARN);
        }
        mode_selector(MEASURER, measurer_pressed, measurer_tex, measurer_instructions, "Constraints Tool");
        if (current_tool == MEASURER) {

        }
        ImGui::Text("%s", instructions.c_str());

        if (ImGui::Button("Load Quad Mesh")) {
            load_quad_mesh_file();
        }
    
    }

    void LabelingUI::update_mesh()
    {
        if (mesh_loaded) {
            viewer->data_list[overlay_index].set_uv(M.UV);
            viewer->data_list[overlay_index].set_colors(M.C);
        }
    }

    bool LabelingUI::pick_face(int& fid, Eigen::Vector3f& bc)
    {
        if (mesh_loaded) {
            double x = viewer->current_mouse_x;
            double y = viewer->core().viewport(3) - viewer->current_mouse_y;
            return igl::unproject_onto_mesh(
                Eigen::Vector2f(x, y),
                viewer->core().view,
                viewer->core().proj,
                viewer->core().viewport,
                viewer->data_list[base_index].V,
                viewer->data_list[base_index].F,
                fid,
                bc);
        }
        return false;
    }

    // Cannot call this until _after_ a viewer window is open
    void LabelingUI::load_textures() {
        if (!textures_loaded) {
            // TODO - This might come back to bite us later if we
            // want to set textures before the viewer loads.
            // Consider moving the glyphs loading to init or
            // constructor
            igl::png::readPNG("glyphs.png", R, G, B, A);
            igl::png::texture_from_file("eraser.png", eraser_tex);
            igl::png::texture_from_file("brush.png", brush_tex);
            igl::png::texture_from_file("seamer.png", seamer_tex);
            igl::png::texture_from_file("orienter.png", orienter_tex);
            igl::png::texture_from_file("measurer.png", measurer_tex);

            igl::png::texture_from_file("eraser_pressed.png", eraser_pressed);
            igl::png::texture_from_file("brush_pressed.png", brush_pressed);
            igl::png::texture_from_file("seamer_pressed.png", seamer_pressed);
            igl::png::texture_from_file("orienter_pressed.png", orienter_pressed);
            igl::png::texture_from_file("measurer_pressed.png", measurer_pressed);
            textures_loaded = true;
        }
    }
}
