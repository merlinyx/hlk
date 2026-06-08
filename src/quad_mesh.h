#pragma once

#include <map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace hlk {

    enum Cardinal {
        N,
        E,
        S,
        W
    };

    struct QuadMesh {
        // Source-of-Truth Matrices
        Eigen::MatrixXd V;
        Eigen::MatrixXi F_q;

        // Derived Structures
        int n; // Number of vertices
        int m; // Number of quads
        int e; // Number of edges

        // Triangulated Quads (center-fanned)
        // Each triangle represents a side (half-edge) of the quad
        // F_t[:,2] are the center vertices. These correspond to faces
        // V is augmented to have these coordinates
        Eigen::MatrixXi F_t;

        // Edges
        std::vector<int> sides_to_edges; // -1 if it is a border
        Eigen::MatrixXi edges_to_sides;

        // Unique Sides - A single rep chosen for each side
        Eigen::MatrixXi unique_sides;

        // Triangle-Triangle Adjacency in F_t
        // TT[:,0] is side-side adjacency (opposite half-edges)
        // TT[:,1] iterates clockwise through quad sides
        // TT[:,2] iterates counter-clockwise through quad sides
        // -1 if no neighbor
        // TTi is the inverse: it says which side of the neigboring quad
        // was opposite
        // TT[4*q+i,0] / 4 is the i-th quad neighbor of quad q
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TTi;

        // Vertex-Triangle Adjacency in F_t
        // VF matches sides to vertices
        // For a quad-mesh-vertex it gives the side adjacencies
        // For a quad-center vertex it gives the sides of the quad
        // There is not a guaranteed order
        // VI gives the index of the vertex in the adjacent triangle
        // for side-to-mesh-vertex matching, this is 0 if outgoing
        // and 1 if incoming - this can be used to filter outgoing and
        // incoming sides. If VI is 2, then the vertex must be a face
        // center
        std::vector<std::vector<int>> VF;
        std::vector<std::vector<int>> VI;

        std::vector<int> valence;

        std::vector<bool> is_singularity;
        std::vector<int> singular_vertices;
        std::vector<bool> is_seam_edge;
        std::vector<int> singular_quads;

        // boundary information
        std::vector<bool> is_boundary_side;
        std::vector<bool> is_border_vertex;
        Eigen::MatrixXi boundary_edges; // vertex pairs
        Eigen::VectorXi boundary_sides; // indices
        Eigen::VectorXi boundary_quads; // indices

        std::vector<double> tri_side_lengths;

        virtual void init();

        // Mesh Queries
        int quad(int side); // Get the quad a side belongs to
        int nth_side(int quad, int index);
        int side_u(int side); // src vertex of a side
        int side_v(int side); // dst vertex of a side
        int flip_side(int side); // Get the side opposite between quads, or -1 if a border
        int next_side(int side); // Get the next side (CCW) in a quad
        int prev_side(int side); // Get the prev side (CW) in a quad
        int opposite_side(int side); // Get the side opposite across a quad
        int next_cross_vertex(int side); // Get the next side across vertices if possible
        int prev_cross_vertex(int side); // Get the previous side across vertices if possible
        std::vector<int> out_sides(int vertex); // Get edges adjacent to a vertex with outward orientation
        std::vector<int> in_sides(int vertex); // Get edges adjacent to a vertex with inward orientation
        std::vector<int> sides(int quad); // Get all sides of a quad
        std::vector<int> dual_loop(int side); // Get a quad-dual loop starting from a side
        std::vector<int> dual_loop(int quad, int index); // get a quad-dual look starting from side index of a quad
        std::vector<int> side_loop(int start_side); // trace sides until a singularity or border
        std::vector<int> reverse_side_loop(int start_side); // trace backwards along sides until a singularity or border

        // Helix Finding
        bool is_course_loop(int curr_he);
        bool perp_direction_check(
            int curr_he, const std::map<int, Cardinal>& face_directions);
        bool helix_free(std::unordered_set<int>& helix, Cardinal c = Cardinal::N);
    };

}

#include <igl/serialize.h>

namespace igl { namespace serialization {
inline void _serialization(bool s, hlk::QuadMesh& obj, std::vector<char>& buffer) {
    SERIALIZE_MEMBER(V)
    SERIALIZE_MEMBER(F_q)
    SERIALIZE_MEMBER(n)
    SERIALIZE_MEMBER(m)
    SERIALIZE_MEMBER(e)
    SERIALIZE_MEMBER(F_t)
    SERIALIZE_MEMBER(sides_to_edges)
    SERIALIZE_MEMBER(edges_to_sides)
    SERIALIZE_MEMBER(unique_sides)
    SERIALIZE_MEMBER(TT)
    SERIALIZE_MEMBER(TTi)
    SERIALIZE_MEMBER(VF)
    SERIALIZE_MEMBER(VI)
    SERIALIZE_MEMBER(valence)
    SERIALIZE_MEMBER(is_singularity)
    SERIALIZE_MEMBER(singular_vertices)
    SERIALIZE_MEMBER(is_seam_edge)
    SERIALIZE_MEMBER(singular_quads)
    SERIALIZE_MEMBER(is_boundary_side)
    SERIALIZE_MEMBER(is_border_vertex)
    SERIALIZE_MEMBER(boundary_edges)
    SERIALIZE_MEMBER(boundary_sides)
    SERIALIZE_MEMBER(boundary_quads)
}

template<> inline void serialize(const hlk::QuadMesh& obj, std::vector<char>& buffer) {
    _serialization(true, const_cast<hlk::QuadMesh&>(obj), buffer);
}

template<> inline void deserialize(hlk::QuadMesh& obj, const std::vector<char>& buffer) {
    _serialization(false, obj, const_cast<std::vector<char>&>(buffer));
}
}}
