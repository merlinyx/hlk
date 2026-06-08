#pragma once

#include <cstdlib>
#include <fstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>                   
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/Simple_cartesian.h>

#include "remeshing_utils.h"

namespace hlk {

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Point_3 Point_3;
typedef K::Line_3 Line_3;
typedef K::Vector_3 Vector_3;
typedef K::Segment_3 Segment_3;
typedef K::Plane_3 Plane_3;

typedef CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> > Polyhedron_3;
typedef Polyhedron_3::Facet_iterator Poly_facet_iterator;
typedef Polyhedron_3::Point_3 Poly_point_3;
typedef Polyhedron_3::HalfedgeDS Poly3_HalfedgeDS;
typedef Polyhedron_3::Halfedge_handle Halfedge_handle;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron_3> Primitive;
typedef CGAL::AABB_traits_3<K, Primitive> Traits_poly;
typedef CGAL::AABB_tree<Traits_poly> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;

typedef std::vector<K::Point_3> Polyline_type;
typedef std::list<Polyline_type> Polylines;

bool first_intersection(Halfedge_handle& hh, int nb,
    Eigen::Vector3d inside, Eigen::Vector3d outside,
    Halfedge_handle& handle, Eigen::Vector3d& intersection);

bool point_inside_triangle(Poly_facet_iterator& face, Eigen::Vector3d& p);

bool detect_edge_point(const Point_and_primitive_id& pp, Halfedge_handle& handle, Eigen::Vector3d& n);

void CGAL_Mesh_Cutting(
    const std::vector<Eigen::Vector3d>& features, const double insert_threshold, 
    const Tree& tree, std::vector<int>& face_ids,
    std::vector<int>& igl_cutting_0_edges, std::vector<int>& igl_cutting_1_edges,
    std::vector<Eigen::Vector3d>& igl_cutting_points,
    std::vector<std::vector<int>>& cutting_faces);

std::vector<Eigen::Vector3d> CGAL_Mesh_Projection(
    const std::vector<Eigen::Vector3d>& features, const double insert_threshold, const Tree& tree);

Point_3 VectorPoint3d(Eigen::Vector3d p);
Eigen::Vector3d Point3dVector(Point_3 p);
int CGAL_Closest_Point(const Tree& tree, const Eigen::Vector3d& point);
int CGAL_Closest_Face(const Tree& tree, const Eigen::Vector3d& point);
Eigen::Vector3d CGAL_Project(const Tree& tree, const Eigen::Vector3d& point);

void CGAL_Plane_Cutting(const Polyhedron_3& mesh, const Tree& tree, 
    const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n, std::vector<Eigen::Vector3d> &loop_polyline);

std::vector<Eigen::Vector3d> CGAL_Plane_Projection(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
Eigen::Vector3d CGAL_Plane_Projection(const Eigen::Vector3d &point, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);

// Export utilities for debugging.

void CGAL_Export_Point(std::ofstream& export_file_output, int& export_index,
    std::string s_name, double r, double g, double b, const Eigen::Vector3d& point, double radius);
void CGAL_Export_Points(std::string path, double r, double g, double b, double radius, const std::vector<Eigen::Vector3d> &points);
void CGAL_Export_Points(std::string path, double r, double g, double b, double radius, const std::vector<std::vector<Eigen::Vector3d>>& pointses);

void CGAL_Export_Segment(std::ofstream& export_file_output, int& export_index,
    std::string s_name, double r, double g, double b, Eigen::Vector3d start, Eigen::Vector3d end, double radius);
void CGAL_Export_Segments(std::string path, double r, double g, double b, double radius, const std::vector<Eigen::Vector3d>& points);
void CGAL_Export_Segments(std::string path, double r, double g, double b, double radius, const std::vector<std::vector<Eigen::Vector3d>>& segments);

}
