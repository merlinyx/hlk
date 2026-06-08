#include <directional/dual_cycles.h>
#include <directional/index_prescription.h>
#include <directional/read_raw_field.h>
#include <directional/representative_to_raw.h>
#include <directional/rotation_to_representative.h>
#include <directional/write_raw_field.h>
#include <igl/AABB.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/boundary_facets.h>
#include <igl/copyleft/comiso/nrosy.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/local_basis.h>
#include <igl/principal_curvature.h>
#include <igl/rotate_vectors.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>

#include "meshing_algorithms.h"
#include "remeshing_plugin.h"

namespace hlk {
bool RemeshingMenu::load_raw_field() {
    std::string fname = igl::file_dialog_open();
    if (fname.length() == 0) return false;

    directional::read_raw_field(fname, rosy, rawField);
    direction_field[1] = rawField.block(0, 0, F.rows(), 3);

    has_direction_field = true;
    return true;
}

bool RemeshingMenu::save_raw_field() {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0) return false;

    directional::representative_to_raw(V, F, direction_field[1], rosy, rawField);
    return directional::write_raw_field(fname, rawField);
}

void RemeshingMenu::reset_face_vectors() {
    face_vectors.clear();
    for (int i = 0; i < F.rows(); i++) {
        FaceVector fv;
        fv.face_id = i;
        Eigen::Vector3d v0 = V.row(F.row(i)[0]);
        Eigen::Vector3d v1 = V.row(F.row(i)[1]);
        Eigen::Vector3d v2 = V.row(F.row(i)[2]);
        fv.center = (v0 + v1 + v2) / 3.0;
        fv.normal = (v1 - v0).cross(v2 - v0).normalized();
        fv.assigned = { false, false };
        fv.frame = { Eigen::Vector3d(), Eigen::Vector3d() };
        face_vectors.push_back(fv);
    }
}

void RemeshingMenu::reset_field() {
    reset_face_vectors();
    seams.clear();
    setup_boundary();
    interpolate_field();
    setup_basis_cycles();
    update_visualization();
}

void RemeshingMenu::setup_boundary() {
    if (!should_setup_boundary) return;

    std::vector<std::vector<int>> indices;
    igl::boundary_loop(F, indices);

    int top_index = 0;
    double maximal_y = -100000.0;
    for (int i = 0; i < indices.size(); i++) {
        Eigen::Vector3d center(0.0, 0.0, 0.0);
        for (int j = 0; j < indices[i].size(); j++) {
            center += V.row(indices[i][j]);
        }
        center /= indices[i].size();
        if (maximal_y < center[1]) {
            maximal_y = center[1];
            top_index = i;
        }
    }

    for (int i = 0; i < F.rows(); i++) {
        int index_0 = F.row(i)[0];
        int index_1 = F.row(i)[1];
        int index_2 = F.row(i)[2];
        int b0 = row_index_of(indices, index_0);
        int b1 = row_index_of(indices, index_1);
        int b2 = row_index_of(indices, index_2);

        int index_00;
        int index_11;
        int index_22;
        bool goon = false;
        bool opposite = false;
        if (b0 >= 0 && b1 >= 0) { // NOTE: maybe these can be connected with else
            index_00 = index_0;
            index_11 = index_1;
            index_22 = index_2;
            goon = true;
            if (b0 == top_index) opposite = true;
        }
        if (b1 >= 0 && b2 >= 0) {
            index_00 = index_1;
            index_11 = index_2;
            index_22 = index_0;
            goon = true;
            if (b1 == top_index) opposite = true;
        }
        if (b2 >= 0 && b0 >= 0) {
            index_00 = index_2;
            index_11 = index_0;
            index_22 = index_1;
            goon = true;
            if (b2 == top_index) opposite = true;
        }

        if (goon) {
            Eigen::Vector3d edge = V.row(index_00) - V.row(index_11);
            Eigen::Vector3d v1 = face_vectors[i].normal.cross(edge);
            Eigen::Vector3d center = (V.row(index_00) + V.row(index_11)) / 2.0;
            Eigen::Vector3d v2 = (Eigen::Vector3d) V.row(index_22) - center;

            double angle = angle_between(v1, v2);
            if (angle > M_PI / 2.0) v1 = -v1;
            if (opposite) v1 = -v1;

            face_vectors[i].frame[1] = v1; // still set to be wale
            face_vectors[i].assigned[1] = true;
            // initialize the course direction to be perpendicular
            face_vectors[i].frame[0] = face_vectors[i].frame[1].cross(face_vectors[i].normal);
            face_vectors[i].assigned[0] = true;
        }
    }
}

void RemeshingMenu::interpolate_cross_field(Eigen::VectorXd& S, int direction) {
    // Set up cross field constraints.
    int hard_constraint_count = 0;
    int soft_constraint_count = 0;
    for (auto& face_vector : face_vectors) {
        if (face_vector.assigned[direction]) {
            if (face_vector.is_hard) {
                ++hard_constraint_count;
            } else {
                ++soft_constraint_count;
            }
        }
    }

    Eigen::VectorXi hard_constraint_indices(hard_constraint_count);
    Eigen::MatrixXd hard_constraints(hard_constraint_count, 3);
    Eigen::VectorXi soft_constraint_indices(soft_constraint_count);
    Eigen::VectorXd soft_constraint_weights(soft_constraint_count);
    Eigen::MatrixXd soft_constraints(soft_constraint_count, 3);

    int idx_hard = 0;
    int idx_soft = 0;
    for (auto& face_vector : face_vectors) {
        if (face_vector.assigned[direction]) {
            if (face_vector.is_hard) {
                hard_constraint_indices[idx_hard] = face_vector.face_id;
                hard_constraints.row(idx_hard) = face_vector.frame[direction].normalized();
                ++idx_hard;
            } else {
                soft_constraint_indices[idx_soft] = face_vector.face_id;
                soft_constraint_weights[idx_soft] = 1.0;
                soft_constraints.row(idx_soft) = face_vector.frame[direction].normalized();
                ++idx_soft;
            }
        }
    }

    igl::copyleft::comiso::nrosy(
        V, F,
        hard_constraint_indices, hard_constraints,
        soft_constraint_indices, soft_constraint_weights, soft_constraints,
        rosy, soft_constraint_strength, direction_field[direction], S);
}

void RemeshingMenu::update_vectors_from_field(int direction) {
    if (symmetrize_nrosy) {
        bool axes[3];
        axes[0] = symmetry_mode_yz;
        axes[1] = symmetry_mode_xz;
        axes[2] = symmetry_mode_xy;
        symmetrizer.symmetrize(direction_field[direction], axes);
    }

    // Populate face vectors.
    const Eigen::MatrixXd& PD1 = direction_field[direction];
    for (int i = 0; i < F.rows(); ++i) {
        if (face_vectors[i].assigned[direction]) continue;
        double x = PD1.row(i) * B1.row(i).transpose();
        double y = PD1.row(i) * B2.row(i).transpose();
        double angle = atan2(y, x);
        face_vectors[i].frame[direction] = cos(angle) * B1.row(i) + sin(angle) * B2.row(i);
        if (direction == 1) {
            face_vectors[i].base_vector = cos(angle + igl::PI / 2.0) * B1.row(i) + sin(angle + igl::PI / 2.0) * B2.row(i);
        }
    }
}

void RemeshingMenu::interpolate_field() {
    interpolate_cross_field(field_sings);
    update_vectors_from_field();
    directional::representative_to_raw(V, F, direction_field[1], rosy, rawField);

    int s_count = 0;
    for (int i = 0; i < field_sings.rows(); ++i) {
        s_count += abs(field_sings(i)) > 0.001 ? 1 : 0;
    }
    std::cout << "[interpolate] Singularity count = " << s_count << "\n";
    std::cout << "[interpolate] Field total index = " << field_sings.sum() << "\n";

    has_direction_field = true;
    has_integer_grid = false;
}

void RemeshingMenu::generate_integer_grid() {

    std::vector<std::vector<int>> hard_edges;

    Eigen::MatrixXi edges;
    Eigen::VectorXi face_inds, opp_verts;
    igl::boundary_facets(F, edges, face_inds, opp_verts);
    for (int i = 0; i < edges.rows(); ++i) {
        int v0 = edges(i, 0);
        int v1 = edges(i, 1);
        int f = face_inds(i);
        for (int j = 0; j < 3; ++j) {
            if ((F(f, j) == v0 && F(f, (j + 1) % 3) == v1) ||
                (F(f, j) == v1 && F(f, (j + 1) % 3) == v0)) {
                hard_edges.push_back({ f, j });
                break;
            }
        }
    }

    if (!seams.empty()) {
        std::vector<std::vector<int>> VF, VI;
        igl::vertex_triangle_adjacency(V.rows(), F, VF, VI);
        for (const std::vector<SplitEdge>& seam : seams) {
            for (const SplitEdge& split_edge : seam) {
                auto v0 = split_edge.index_0;
                auto v1 = split_edge.index_1;
                for (int i = 0; i < VF[v0].size(); ++i) {
                    int f = VF[v0][i];
                    int idx = VI[v0][i];
                    assert(F(f, idx) == v0);
                    if (F(f, (idx + 1) % 3) == v1) {
                        hard_edges.push_back({ f, idx });
                        continue;
                    }
                }
            }
        }
    }

    Meshing::cross_field_miq(
        direction_field[1],
        V,
        F,
        hard_edges,
        gradient_size,
        stiffen_iter,
        V_uv,
        F_uv
    );
    
    has_integer_grid = true;
    viewing_mode = ViewingMode::MESH_ONLY;
    update_visualization();
}

void RemeshingMenu::init_quad_mesh() {
    if (!is_quad_meshed) { return; }

    igl::AABB<Eigen::MatrixXd, 3> tri_aabb;
    tri_aabb.init(V, F);
    
    quad_mesh.tri_side_lengths.clear();
    for (int q = 0; q < quad_mesh.m; ++q) {
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d v0 = V.row(quad_mesh.F_q(q, i));
            Eigen::Vector3d v1 = V.row(quad_mesh.F_q(q, (i + 1) % 4));
            int fid0, fid1;
            Eigen::RowVector3d C0, C1;
            tri_aabb.squared_distance(V, F, v0, fid0, C0);
            tri_aabb.squared_distance(V, F, v1, fid1, C1);
            quad_mesh.tri_side_lengths.push_back((C1 - C0).norm());
        }
    }

    quad_mesh.is_seam_edge.clear();
    quad_mesh.is_seam_edge = std::vector<bool>(4 * quad_mesh.m, false);

    if (seams.empty()) return;

    igl::AABB<Eigen::MatrixXd, 3> aabb_tree;
    aabb_tree.init(quad_mesh.V, quad_mesh.F_t);

    for (const std::vector<SplitEdge>& seam : seams) {
        for (const SplitEdge& se : seam) {
            Eigen::Vector3d v0 = V.row(se.index_0);
            Eigen::Vector3d v1 = V.row(se.index_1);
            int fid;
            Eigen::RowVector3d C;
            aabb_tree.squared_distance(quad_mesh.V, quad_mesh.F_t, (v0 + v1) / 2., fid, C);
            quad_mesh.is_seam_edge[fid] = true;
            if (quad_mesh.flip_side(fid) > 0) {
                quad_mesh.is_seam_edge[quad_mesh.flip_side(fid)] = true;
            }
        }
    }
}

void RemeshingMenu::quad_helix_finding() {
    if (!is_quad_meshed) { return; }

    std::unordered_set<int> longest_helix;
    if (!quad_mesh.helix_free(longest_helix)) {
        std::cout << "[remeshing] there exists a helix somewhere... highlighted in green.\n";
        Eigen::MatrixXd interactive_colors(quad_mesh.m * 4, 3);
        interactive_colors.setOnes();
        for (auto iter = longest_helix.begin(); iter != longest_helix.end(); ++iter) {
            interactive_colors.row((*iter)) = Eigen::RowVector3d(0.8, 1., 0.6);
        }
        for (int face : quad_mesh.singular_quads) { // for debugging purposes
            interactive_colors.row(face) = Eigen::RowVector3d(1., 0., 0.6);
        }
        stylize_quad_mesh(interactive_colors);

    } else {
        std::cout << "[remeshing] helix free!\n";
        stylize_quad_mesh(Eigen::RowVector3d::Constant(1.0));
    }
}

std::vector<FaceVector> RemeshingMenu::hard_faces() {
    std::vector<FaceVector> faces;
    for (const FaceVector& fv : face_vectors) {
        if (fv.is_hard && fv.assigned[1]) {
            faces.push_back(fv);
        }
    }
    return faces;
}

void RemeshingMenu::setup_basis_cycles() {
    std::vector<FaceVector> directional_constraints = hard_faces();
    numDirectionConstraints = directional_constraints.empty() ? 0 : directional_constraints.size() - 1;

    basisCycles.setZero();
    cycleCurvature.setZero();
    vertex2cycle.setZero();
    innerEdges.setZero();
    cycleFaces.clear();

    constrainedRoot = false;
    if (directional_constraints.empty()) {
        directional::dual_cycles(V, F, EV, EF, basisCycles, cycleCurvature, vertex2cycle, innerEdges);
    } else {
        std::vector<int> face_inds;
        std::vector<double> constraint_angles;
        for (const FaceVector& fv : directional_constraints) {
            face_inds.push_back(fv.face_id);
            double x = fv.frame[1].dot(B1.row(fv.face_id));
            double y = fv.frame[1].dot(B2.row(fv.face_id));
            double angle = atan2(y, x);
            constraint_angles.push_back(angle);
        }
        directional::dual_cycles(V, F, EV, EF, basisCycles, cycleCurvature, vertex2cycle, innerEdges, face_inds, constraint_angles, N);
        constrainedRoot = true;
    }

    cycleIndices = Eigen::VectorXi::Constant(basisCycles.rows(), 0);

    for (int i = 0; i < singVertices.size(); i++)
        cycleIndices(vertex2cycle(singVertices(i))) = singIndices(i);

    std::vector<std::vector<int>> boundaryLoops;
    igl::boundary_loop(F, boundaryLoops);
    numBoundaries = boundaryLoops.size();
    eulerChar = V.rows() - EV.rows() + F.rows();
    numGenerators = 2 - eulerChar - boundaryLoops.size();

    std::cout << "Euler characteristic: " << eulerChar << std::endl;
    std::cout << "#generators: " << numGenerators << std::endl;
    std::cout << "#boundaries: " << numBoundaries << std::endl;
    std::cout << "#directional constraints: " << numDirectionConstraints << std::endl;

    // collecting cycle faces for visualization
    cycleFaces.resize(basisCycles.rows());
    for (int k = 0; k < basisCycles.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(basisCycles, k); it; ++it) {
            int f1 = EF(innerEdges(it.col()), 0);
            int f2 = EF(innerEdges(it.col()), 1);
            if (f1 != -1)
                cycleFaces[it.row()].push_back(f1);
            if (f2 != -1)
                cycleFaces[it.row()].push_back(f2);
        }
    }
}

void RemeshingMenu::compute_target_curvature() {
    interpolate_cross_field(field_sings);
    const Eigen::MatrixXd& PD1 = direction_field[1];
    // the difference in the angle representation of edge i from EF(i,0) to EF(i,1)
    Eigen::VectorXd edgeParallelAngleChange(basisCycles.cols());
    for (int i = 0; i < innerEdges.rows(); i++) {
        int currEdge = innerEdges(i);
        Eigen::RowVectorXd edgeVectors = (V.row(EV(currEdge, 1)) - V.row(EV(currEdge, 0))).normalized();
        // cross field bases
        Eigen::VectorXd b1_0 = PD1.row(EF(currEdge, 0)).normalized();
        double xx1 = b1_0.dot(B1.row(EF(currEdge, 0)));
        double yy1 = b1_0.dot(B2.row(EF(currEdge, 0)));
        double angle1 = atan2(yy1, xx1);
        angle1 += 2 * igl::PI / (double)N;
        Eigen::VectorXd b2_0 = cos(angle1) * B1.row(EF(currEdge, 0)) + sin(angle1) * B2.row(EF(currEdge, 0));

        Eigen::VectorXd b1_1 = PD1.row(EF(currEdge, 1)).normalized();
        double xx2 = b1_1.dot(B1.row(EF(currEdge, 1)));
        double yy2 = b1_1.dot(B2.row(EF(currEdge, 1)));
        double angle2 = atan2(yy2, xx2);
        angle2 += 2 * igl::PI / (double)N;
        Eigen::VectorXd b2_1 = cos(angle2) * B1.row(EF(currEdge, 1)) + sin(angle2) * B2.row(EF(currEdge, 1));

        // edge angle change
        double x1 = edgeVectors.dot(b1_0);
        double y1 = edgeVectors.dot(b2_0);
        double x2 = edgeVectors.dot(b1_1);
        double y2 = edgeVectors.dot(b2_1);
        edgeParallelAngleChange(i) = atan2(y2, x2) - atan2(y1, x1);
    }
    targetCurvature = basisCycles * edgeParallelAngleChange;
    for (int i = 0; i < targetCurvature.size(); i++) {
        while (targetCurvature(i) >= M_PI) targetCurvature(i) -= 2.0 * M_PI;
        while (targetCurvature(i) < -M_PI) targetCurvature(i) += 2.0 * M_PI;
    }
}

void RemeshingMenu::update_raw_field() {
    int sum = round(cycleIndices.head(cycleIndices.size() - numGenerators).sum());
    std::cout << "[Info] Total indices: " << sum << "/" << N << std::endl;
    if (eulerChar * N != sum) {
        std::cout << "[Info] Expected: " << eulerChar * N << "/" << N << std::endl;
        std::cout << "[Warn] All non-generator singularities should add up to N * the Euler characteristic." << std::endl;
        std::cout << "[Info] More " << (eulerChar * N < sum ? "contraction" : "expansion") << " composition guidelines are needed if a trivial connection cannot be found.\n" << std::endl;
    }

    Eigen::VectorXd rotationAngles;
    double linfError;
    if (use_guiding_field) {
        compute_target_curvature();
        directional::index_prescription(
            V, F, innerEdges, basisCycles, targetCurvature,
            cycleCurvature, cycleIndices, N, field_guidance_weight,
            rotationAngles, linfError);
    } else {
        directional::index_prescription(
            V, F, innerEdges, basisCycles,
            cycleCurvature, cycleIndices, N,
            rotationAngles, linfError);
    }
    //std::cout << "Index prescription linfError: " << linfError << std::endl;

    std::vector<FaceVector> cfaces = hard_faces();
    misaligned_faces.clear();
    if (!cfaces.empty()) {
        Eigen::MatrixXd representative;
        directional::rotation_to_representative(V, F, EV, EF, rotationAngles, N, 0, representative);
        if (do_matching) {
            directional::representative_to_raw(V, F, representative, N, rawField);
            Eigen::VectorXi combedMatching;
            Eigen::VectorXd combedEffort;
            Meshing::comb_field_from_connection(V, F, EV, EF, FE, rawField, combedField, combedMatching, combedEffort);
            representative = combedField.block(0, 0, F.rows(), 3);
        }
        int i = cfaces[0].face_id;
        double cx = cfaces[0].frame[1].dot(B1.row(i));
        double cy = cfaces[0].frame[1].dot(B2.row(i));
        double x = representative.row(i) * B1.row(i).transpose();
        double y = representative.row(i) * B2.row(i).transpose();
        constrainedRootAngle = N * (atan2(cy, cx) - atan2(y, x));
    }

    Eigen::MatrixXd representative;
    directional::rotation_to_representative(V, F, EV, EF, rotationAngles, N, constrainedRoot ? constrainedRootAngle : globalRotation, representative);
    directional::representative_to_raw(V, F, representative, N, rawField);
    if (do_matching) {
        Eigen::VectorXi combedMatching;
        Eigen::VectorXd combedEffort;
        Meshing::comb_field_from_connection(V, F, EV, EF, FE, rawField, combedField, combedMatching, combedEffort);
        direction_field[1] = combedField.block(0, 0, F.rows(), 3);
    } else {
        direction_field[1] = representative;
    }

    if (linfError > 1e-13) {
        std::cout << "[Warn] cannot find a trivial connection.\n";
    } else {
        double angle_error = 0;
        for (const FaceVector& fv : cfaces) {
            int i = fv.face_id;
            double cx = fv.frame[1].dot(B1.row(i));
            double cy = fv.frame[1].dot(B2.row(i));
            double x = direction_field[1].row(i) * B1.row(i).transpose();
            double y = direction_field[1].row(i) * B2.row(i).transpose();
            double angle_error_i = atan2(cy, cx) - atan2(y, x);
            //std::cout << i << " " << angle_error_i << "->";
            while (angle_error_i >= 0.001) angle_error_i -= M_PI / 4;
            while (angle_error_i < -0.001) angle_error_i += M_PI / 4;
            //std::cout << angle_error_i << std::endl;
            if (fabs(angle_error_i) > 0.01) {
                misaligned_faces.push_back(i);
            }
        }
        if (misaligned_faces.empty()) {
            std::cout << "[Info] found trivial connection.\n";
        } else {
            std::cout << "[Warn] trivial connection cannot satisfy all directional constraints. \n";
        }
    }

    has_direction_field = true;
}

void RemeshingMenu::update_singularities() {
    std::vector<int> singVerticesList, singIndicesList;
    for (int i = 0; i < V.rows(); i++) {
        if (cycleIndices(vertex2cycle(i))) {
            singVerticesList.push_back(i);
            singIndicesList.push_back(cycleIndices(vertex2cycle(i)));
        }
    }
    singVertices.resize(singVerticesList.size());
    singIndices.resize(singIndicesList.size());
    for (int i = 0; i < singVerticesList.size(); i++) {
        singVertices(i) = singVerticesList[i];
        singIndices(i) = singIndicesList[i];
    }
}

}
