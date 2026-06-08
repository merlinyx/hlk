#include <directional/combing.h>
#include <directional/principal_matching.h>
#include <igl/copyleft/comiso/miq.h>
#include <igl/rotate_vectors.h>
#include <igl/PI.h>

#include "meshing_algorithms.h"

namespace hlk {
void Meshing::comb_field_from_connection(
    const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh,
    const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
    const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
    Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort) {

    // combing
    Eigen::VectorXi matching;
    Eigen::VectorXd effort;
    Eigen::VectorXi singVertices;
    Eigen::VectorXi singIndices;
    directional::principal_matching(VMesh, FMesh, EV, EF, FE, rawField, matching, effort, singVertices, singIndices);
    directional::combing(VMesh, FMesh, EV, EF, FE, rawField, matching, combedField);
    directional::principal_matching(VMesh, FMesh, EV, EF, FE, combedField, combedMatching, combedEffort, singVertices, singIndices);
}

void Meshing::cross_field_miq(const Eigen::MatrixXd& X1,
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
    const std::vector<std::vector<int>>& hard_edges,
    double gradient_size, int stiffen_iter,
    Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV) {

    // Find the orthogonal field
    Eigen::MatrixXd B1, B2, B3;
    igl::local_basis(V, F, B1, B2, B3);
    Eigen::MatrixXd X2 = igl::rotate_vectors(X1, Eigen::VectorXd::Constant(1, igl::PI / 2), B1, B2);

    // Global parametrization
    igl::copyleft::comiso::miq(
        V,
        F,
        X1,
        X2,
        UV,
        FUV,
        gradient_size,
        5.0,   // stiffness, reserved but unused
        false, // direct round, default to the greedy rounding proposed in the MIQ paper
        stiffen_iter,
        5,     // local # iter of integer rounding
        true,  // do round = isInteger in Directional's parameterize()
        true,  // singularity round
        std::vector<int>(), // vertices to round, none other than singularities
        hard_edges
    );
}

}
