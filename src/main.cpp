#define IGL_VIEWER_VIEWER_QUIET 1
// This does nothing during static compilation - will need to add it to the
// CMAKE for it to stick
#include <iostream>

#include "labeling_ui.h"
#include "remeshing_plugin.h"

using namespace hlk;

inline uint32_t str_to_int32_t(const std::string& str) {
    char* end_ptr = nullptr;
    int32_t result = (int32_t)strtol(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw std::runtime_error("Could not parse signed integer \"" + str + "\"");
    return result;
}

int main(int argc, char* argv[]) {
    std::vector<std::string> args;
    bool help = false;
    int rosy = 4;
    std::string input_path = "./tmp_in.obj";
    std::string output_path = "./tmp_out.obj";
    std::string input_model = "";

    try {
        for (int i = 1; i < argc; ++i) {
            if (strcmp("--help", argv[i]) == 0 || strcmp("-h", argv[i]) == 0) {
                help = true;
            } else if (strcmp("--rosy", argv[i]) == 0 || strcmp("-r", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing rotation symmetry type!\n";
                    return -1;
                }
                rosy = str_to_int32_t(argv[i]);
                if ((rosy != 2 && rosy != 4 && rosy != 6)) {
                    std::cerr << "Error: Invalid symmetry type!\n";
                    help = true;
                }
            } else if (strcmp("--input_path", argv[i]) == 0 || strcmp("-i", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing input file argument!\n";
                    return -1;
                }
                input_path = argv[i];
            } else if (strcmp("--output_path", argv[i]) == 0 || strcmp("-o", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing output file argument!\n";
                    return -1;
                }
                output_path = argv[i];
            } else if (strcmp("--input_model", argv[i]) == 0 || strcmp("-m", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing input model argument!\n";
                    return -1;
                }
                input_model = argv[i];
            } else {
                if (strncmp(argv[i], "-", 1) == 0) {
                    std::cerr << "Invalid argument: \"" << argv[i] << "\"!\n";
                    help = true;
                }
                args.push_back(argv[i]);
            }
        }
    } catch (const std::exception & e) {
        std::cout << "Error: " << e.what() << "\n";
        help = true;
    }

    if (args.size() > 1 || help || ((output_path.empty() || input_path.empty()) && args.size() == 0)) {
        std::cout << "Syntax: " << argv[0] << " [options] <input mesh / point cloud / application state snapshot>\n";
        std::cout << "Options:\n"
                     "   -i, --input_path       Writes to the specified PLY/OBJ file path which is the input for libQEx\n"
                     "   -o, --output_path      Writes to the specified PLY/OBJ file path which is the output for libQEx\n"
                     "   -r, --rosy <number>    Specifies the orientation symmetry type (2, 4, or 6)\n"
                     "   -m, --input_model      Specifies the input model that we want to preload into the UI\n"
                     "   -h, --help             Display this message\n";
        return -1;
    }

    if (args.size() == 0) std::cout << "Running in GUI mode.\n";

    int mode = 1;

    igl::opengl::glfw::Viewer viewer;
    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    if (mode == 1) {
        RemeshingMenu remeshing_menu(rosy, input_path, output_path);
        std::cout <<
            "  The field will appear if indices are correct\n"
            "  0+left key  Select vertex cycle\n"
            "  B           Loop through boundary cycles\n"
            "  G           Loop through generator cycles\n"
            "  +           Increase index of current cycle\n"
            "  -           Decrease index  of current cycle\n"
            "  1           Rotate field globally\n"
            "  R           Recolor the mesh to show colors based on linfError\n";
        try {
            if (!input_model.empty()) {
                remeshing_menu.set_input_model(input_model);
            }
            plugin.widgets.push_back(&remeshing_menu);
            viewer.launch();
        } catch (const std::runtime_error & e) {
            std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
            if (remeshing_menu.save_workspace()) {
                std::cout << "all work saved.\n";
            } else {
                std::cout << "failed to save state...\n";
            }
            return -1;
        }
    } else {
        LabelingUI labelingui;
        plugin.widgets.push_back(&labelingui);
        viewer.launch();
    }

    return 0;
}
