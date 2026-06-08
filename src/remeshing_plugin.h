#pragma once

#include <string>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/texture_from_file.h>

#include "cgal_wrapper.h"
#include "quad_mesh.h"
#include "symmetrizer.h"

namespace hlk {

class RemeshingMenu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    RemeshingMenu(int nrosy, std::string input_path, std::string output_path) {
        name = "Remeshing";
        rosy = nrosy;
        in_path = input_path;
        out_path = output_path;

        click_threshold = 0.1f;
        loops_threshold = 0.01f;
        soft_constraint_strength = 0.5f;
        field_guidance_weight = 0.5f;
        gradient_size = 50.0f;
        stiffen_iter = 0;

        show_axis = false;
        show_stitches = false;
        symmetry_mode_yz = false;
        symmetry_mode_xz = false;
        symmetry_mode_xy = false;
        symmetrize_nrosy = false;
        symmetrize_loops = false;
        should_setup_boundary = true;
        has_direction_field = false;
        has_integer_grid = false;
        is_quad_meshed = false;
        should_redraw = false;
        in_composition_mode = false;

        geodesic_label = false;
        existing_edge_label = false;
        multi_points_drawing = true;
        use_optim_loop = true;
        do_matching = true;
        use_guiding_field = false;

        viewing_mode = ViewingMode::MESH_ONLY;
        drawing_mode = DrawingMode::WALE;
        seaming_mode = SeamingMode::SEAM;
        composition = Composition::PATCH_QUARTER_OUT;
        line_texture(texture_R, texture_G, texture_B);
        direction_field = { Eigen::MatrixXd(), Eigen::MatrixXd() };

        currVertex = 0;
        currCycle = 0;
        N = 4; // degree of field
        globalRotation = 0.;
        singularitySelect = false;
        constrainedRoot = false;

        window_width = 1280;
        window_height = 800;
    }
    ~RemeshingMenu() {
        clear();
        // remove ctrl+z saves.
        for (auto& temp : temps) {
            remove(temp.mesh.c_str());
            remove(temp.face.c_str());
            remove(temp.edge.c_str());
            remove(temp.sing.c_str());
        }
        remove(in_path.c_str());
        remove(out_path.c_str());
    };

    void init(igl::opengl::glfw::Viewer* _viewer, igl::opengl::glfw::imgui::ImGuiPlugin* _plugin) override;
    void draw_viewer_menu() override;
    void draw_custom_window() override;
    bool load(std::string filename);
    bool save(std::string filename);
    void load_temp_data(std::string filename);

    void load_textures();

    bool load_workspace();
    bool save_workspace();

    void set_input_model(std::string filename) { input_model = filename; }

    /////////// CORE UI CALLBACKS ///////////
    bool mouse_down(int button, int modifier) override;
    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_scroll(float delta_y);

    //////////// UTILITY METHODS ////////////
    void clear();

    void setup_mesh();
    void get_mesh_information();
    bool model_loaded() { return V.rows() > 0 && F.rows() > 0; }
    void update_polyhedron_tree(const std::vector<int>& face_refs = std::vector<int>(), bool is_seam_cutting = false);
    void apply_subdivision();

    void assign_vector(int face_id, Eigen::Vector3d n);
    void assign_vector();
    std::unordered_set<int> neighbor_faces(std::vector<std::unordered_set<int>>& igl_faces, int e_0, int e_1);
    void geodesic_assign_vector();
    bool same_line(const std::vector<Eigen::Vector3d>& feature_points_save, const std::vector<Eigen::Vector3d>& feature_points);
    void symmetry_assign_vector(std::vector<int> axes, int start, int end);
    void symmetry_assign_vector(std::vector<int> axes, const std::vector<Eigen::Vector3d>& feature_points_save);
    void symmetry_assign_vector(const std::vector<Eigen::Vector3d>& feature_points_save);

    void split_mesh(const float threshold);
    void geodesic_split_mesh();
    void split_existing_edges(int index_0, int index_1, int insert_index, std::vector<SplitEdge>& seam);
    void symmetry_split_mesh(std::vector<int> axes, int start, int end);
    void symmetry_split_mesh(std::vector<int> axes, const std::vector<Eigen::Vector3d>& feature_points_save);
    void symmetry_split_mesh(const std::vector<Eigen::Vector3d>& feature_points_save);
    void cut_along_seams();

    void set_mesh_overlays(const int mesh_id, const bool wireframe = true, const bool overlay = true, const bool fill = true);
    void stylize_tri_mesh(const Eigen::MatrixXd& colors);
    void stylize_quad_mesh(const Eigen::MatrixXd& colors);
    void update_visualization(unsigned char key = '\0');
    void update_drawing();
    void draw_direction_field();

    void draw_a_segment(const Eigen::Vector3d v0, const Eigen::Vector3d v1, 
        const int color_index, const double face_dis = -1., const int data_index = 0);
    void draw_segments(const std::vector<Eigen::Vector3d>& segments, const int color_index, const double face_dis = -1.);
    /* Color - Index mapping:
           red - 0
         green - 1
          blue - 2
        orange - 3
         black - 4
     */
    void draw_a_point(const Eigen::Vector3d v, const int color_index, const double face_dis = -1.);
    void draw_points(const std::vector<Eigen::Vector3d>& vecs, const int color_index, const double face_dis = -1.);

    Symmetrizer symmetrizer;
    bool highlight_symmetries();
    std::vector<int> v_symmetry_axes();
    std::vector<int> f_symmetry_axes();

    // field - impl in remeshing_field.cpp
    bool load_raw_field();
    bool save_raw_field();
    void reset_face_vectors();
    void reset_field();
    void setup_boundary();
    void interpolate_cross_field(Eigen::VectorXd& S, int direction = 1); // default wale interpolation
    void update_vectors_from_field(int direction = 1);
    void interpolate_field();
    void generate_integer_grid();
    void init_quad_mesh();
    void quad_helix_finding();
    std::vector<FaceVector> hard_faces();
    void setup_basis_cycles();
    void compute_target_curvature();
    void update_raw_field();
    void update_singularities();

    // loops - impl in remeshing_loops.cpp
    void clear_loops();
    void update_loop_graph();
    void compute_elastic_loop(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_v);
    void compute_elastic_loop_field_align(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
    void compute_elastic_loop_min_geodesic(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
    void symmetry_elastic_loop(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
    void symmetry_elastic_loop(std::vector<int> axes, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);

    void save_ctrlz();

    // bools...
    bool has_direction_field;
    bool has_integer_grid;
    bool is_quad_meshed;
    bool should_redraw;
    bool in_composition_mode;

    // numbers...
    float soft_constraint_strength;
    float field_guidance_weight;
    int stiffen_iter;
    float gradient_size;
    float loop_size;
    float click_threshold; // a certain percentage of mesh edge size
    float loops_threshold; // same as above, for using loops to split mesh
    double mesh_size;
    double mesh_edge_size;
    Eigen::Vector3d mesh_center;

    // feature points
    std::vector<Eigen::Vector3d> feature_points;
    std::vector<int> feature_face_ids;

    // geodesic feature
    Eigen::Vector3d geodesic_point;
    std::vector<int> geodesic_path;
    int geodesic_index;
    bool geodesic_label;
    
    // directional faces
    std::vector<FaceVector> face_vectors;
    std::vector<std::vector<SplitEdge>> seams;

    // loops data
    std::vector<TM_Node> loop_gi_nodes;
    std::vector<TM_Edge> loop_gi_edges;
    std::vector<std::unordered_set<int>> loop_g_iedges;
    std::vector<std::vector<Eigen::Vector3d>> loop_graph_adj;
    std::vector<bool> loop_graph_boundary;
    std::vector<Eigen::Vector3d> igl_v_ns;

    std::vector<Eigen::Vector3d> loop_points;
    std::vector<Eigen::Vector3d> loop_de_points;
    std::vector<std::vector<Eigen::Vector3d>> loop_polylines;
    std::vector<std::vector<Eigen::Vector3d>> loop_update_polylines;

    int loop_start_index = -1;
    int loop_end_index = -1;
    std::vector<Eigen::Vector3d> loop_path;
    std::vector<int> loop_feature_face_ids;

    // geometry data
    Polyhedron_3 igl_polyhedron;
    Tree igl_tree;
    std::vector<std::unordered_set<int>> igl_v_faces;
    std::vector<std::vector<double>> graph_adj;

    // line textures
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_R, texture_G, texture_B;

    // triangle mesh data
    Eigen::MatrixXd V, B;
    Eigen::MatrixXi F;
    double global_scale;  // Scale for visualizing the fields
    // Global parametrization
    Eigen::MatrixXd V_uv;
    Eigen::MatrixXi F_uv;
    // Local basis
    Eigen::MatrixXd B1, B2, B3;
    // Edge topology
    Eigen::MatrixXi EV, EF, FE;

    // quad mesh data
    QuadMesh quad_mesh;

    // cross field data
    std::vector<Eigen::MatrixXd> direction_field; // size 2, using stl for serialization
    Eigen::VectorXd field_sings;

    // trivial connections data
    Eigen::VectorXi singVertices, singIndices;
    Eigen::VectorXi cycleIndices;
    Eigen::VectorXd cycleCurvature, targetCurvature;
    Eigen::SparseMatrix<double> basisCycles;
    Eigen::VectorXi vertex2cycle, innerEdges;
    Eigen::MatrixXd rawField, combedField;
    Eigen::MatrixXi FField, FSings;
    Eigen::MatrixXd VField, VSings;
    Eigen::MatrixXd CMesh, CField, CSings;
    std::vector<std::vector<int>> cycleFaces;
    int eulerChar, numGenerators, numBoundaries, numDirectionConstraints;
    int currVertex, currCycle;
    int N; // degree of field
    Eigen::VectorXd linf;
    std::vector<int> misaligned_faces;
    float globalRotation, constrainedRootAngle;
    bool singularitySelect, constrainedRoot;
    std::vector<std::vector<int>> singGroups;
    std::vector<Composition> singGroupComp;
    std::map<int, int> vertex2singGroup;

    /////////////////// UI ///////////////////
    std::string in_path, out_path, input_model;
    ViewingMode viewing_mode;
    DrawingMode drawing_mode;
    SeamingMode seaming_mode;
    Composition composition;

    // Texture Handles for UI
    bool textures_loaded = false;
    GLuint patch_half, patch_quarter_in, patch_quarter_out, Y_one, Y_half, T_half, T_quarter, hole_half, hole_quarter;

    bool symmetry_mode_yz, symmetry_mode_xz, symmetry_mode_xy;
    bool symmetrize_nrosy, symmetrize_loops, should_setup_boundary;
    int rosy;

    bool show_axis, show_stitches, multi_points_drawing;
    bool do_matching, use_guiding_field, use_optim_loop;

    bool existing_edge_label;

    int mouse_key, window_width, window_height;
    double mouse_x, mouse_y;
    bool ctrl_on, alt_on, shift_on, mouse_down_on;

    CTRLZSL czsl;
    std::vector<TEMPDATA> temps;
};

}
