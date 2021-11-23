#include "Repair.h"
#include <igl/remove_duplicate_vertices.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/Properties.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>
#include <stdio.h>

namespace MeshPolyRefinement {
	namespace IO {
		void repair_non_manifold(Base::TriMesh& mesh){
			typedef CGAL::Exact_predicates_inexact_constructions_kernel          K;
			typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>      Polyhedron;
			typedef CGAL::Surface_mesh<K::Point_3> Surface_mesh;
			typedef Surface_mesh::Vertex_index Vertex_index;
			namespace PMP = CGAL::Polygon_mesh_processing;
			std::vector<K::Point_3> points;
  			std::vector<std::vector<std::size_t> > polygons;
			std::vector<Vertex_index> v_index_map;
			Surface_mesh surface_mesh;
			for(const auto& v : mesh.m_vertices.rowwise()){
				auto pt = K::Point_3(v[0],v[1],v[2]);
				points.push_back(pt);
				v_index_map.push_back(surface_mesh.add_vertex(pt));
			}

			for(const auto& f : mesh.m_faces.rowwise()) {
				polygons.push_back({(std::size_t)f[0], (std::size_t)f[1], (std::size_t)f[2]});
				surface_mesh.add_face(v_index_map[f[0]], v_index_map[f[1]], v_index_map[f[2]]);
			}

			printf("input %ld points, %ld polygons\n", points.size(), polygons.size());
			PMP::orient_polygon_soup(points, polygons);
			
			printf("output %ld points, %ld polygons\n", points.size(), polygons.size());
			PMP::orient_triangle_soup_with_reference_triangle_mesh(surface_mesh,points,polygons);
			// Polyhedron poly_mesh;
  			// PMP::polygon_soup_to_polygon_mesh(points, polygons, poly_mesh);
			// std::ofstream out("polygon-mesh-clean.off");
  			// out.precision(17);
  			// out << poly_mesh;
  			// out.close();


			mesh.m_vertices.resize(points.size(), 3);
			mesh.m_faces.resize(polygons.size(), 3);

			int n = 0;
			for(const auto& pt : points){
				mesh.m_vertices.row(n++) = Base::Vec3(pt[0], pt[1], pt[2]);
			}
			n = 0;
			for(const auto& poly : polygons){
				mesh.m_faces.row(n++) = Base::Vec3i(poly[0], poly[1], poly[2]);
			}
			

		}

		void merge_close_vertex(Base::AttributeMatrix& vertices, Base::AttributeMatrix& vertex_colors, Base::IndexMatrix& faces) {
			Eigen::VectorXi SVI, SVJ;
			Base::AttributeMatrix temp_v_out;
			Base::IndexMatrix temp_f_out;

			
			igl::remove_duplicate_vertices(vertices, faces, 1e-7, temp_v_out, SVI, SVJ, temp_f_out);
			/*std::cout << "V: " << vertices << std::endl;
			std::cout << "temp_v: " << temp_v_out << std::endl;
			std::cout << "SVI: " << SVI << std::endl;
			std::cout << "SVJ: " << SVJ << std::endl;
			std::cout << "temp_f: " << temp_f_out << std::endl;*/
			faces.resize(temp_f_out.rows(), 3);
			faces = temp_f_out;

			vertices.resize(temp_v_out.rows(), 3);
			vertices = temp_v_out;

			if (vertex_colors.rows() > 0) {
				Base::AttributeMatrix temp_color = vertex_colors(SVI, Eigen::all);
				vertex_colors.resize(temp_color.rows(), 3);
				vertex_colors = temp_color;
			}
			
			
			
		}

		void remove_identical_index_faces(Base::IndexMatrix& faces) {
			std::vector<int> valid_faces;
			for (int i = 0; i < faces.rows(); ++i) {
				if (faces(i, 0) != faces(i, 1)
					&& faces(i, 1) != faces(i, 2)
					&& faces(i, 2) != faces(i, 0)) {
					valid_faces.push_back(i);
				}
			}
			Base::IndexMatrix tmp = faces(valid_faces, Eigen::all);
			faces.resize(tmp.rows(), 3);
			faces = tmp;
		}

		void repair_non_manifold_edge(Base::TriMesh& mesh) {
			//transfer to cgal mesh
			namespace PMP = CGAL::Polygon_mesh_processing;
			namespace NP = CGAL::parameters;
			typedef CGAL::Simple_cartesian<Base::Scalar>          K;
			typedef K::Point_3									Point_3;
			typedef CGAL::Surface_mesh<K::Point_3>              Mesh;
			using Vertex_index = Mesh::Vertex_index;
			using Face_index = Mesh::Face_index;


			Mesh cgal_mesh;
			bool created;
			Mesh::Property_map<Face_index, Base::FaceSemanticLabel>face_label_property;
			boost::tie(face_label_property, created) = cgal_mesh.add_property_map<Face_index, Base::FaceSemanticLabel>("f:face_label", Base::FaceSemanticLabel::UNSET);

			Mesh::Property_map<Face_index, double> face_plane_score_property;
			boost::tie(face_plane_score_property, created) = cgal_mesh.add_property_map<Face_index, double>("f:face_plane_score", 0.0);

			Mesh::Property_map<Face_index, int> face_plane_index_property;
			boost::tie(face_plane_index_property, created) = cgal_mesh.add_property_map<Face_index, int>("f:face_plane_index", -1);

			Base::AttributeMatrix& vertices = mesh.m_vertices;
			Base::IndexMatrix& faces = mesh.m_faces;
			std::vector<Vertex_index> vertex_map(vertices.rows());
			//add vertex
			for (int i = 0; i < vertices.rows(); ++i) {
				const Base::Vec3 &p = vertices.row(i);
				Vertex_index vi = cgal_mesh.add_vertex(Point_3(p[0], p[1], p[2]));
				vertex_map[i] = vi;
			}

			//add face
			for (int i = 0; i < faces.rows(); ++i) {
				const Base::Vec3i &f = faces.row(i);
				Face_index fi = cgal_mesh.add_face(vertex_map[f[0]], vertex_map[f[1]], vertex_map[f[2]]);
				if (fi == Mesh::null_face()) {
					std::cout << "warning --- add null face: " << f << std::endl;
				}
				else {
					face_label_property[fi] = mesh.m_face_labels[i];
					face_plane_index_property[fi] = mesh.m_face_plane_index[i];
					face_plane_score_property[fi] = mesh.m_face_planar_score[i];
				}

			}



		}

		void split_non_manifold_vertex_with_property(Base::AttributeMatrix& vertices, Base::AttributeMatrix& vertex_colors, Base::IndexMatrix& faces) {
			//transfer to cgal mesh
			namespace PMP = CGAL::Polygon_mesh_processing;
			namespace NP = CGAL::parameters;
			typedef CGAL::Simple_cartesian<Base::Scalar>          K;
			typedef K::Point_3									Point_3;
			typedef CGAL::Surface_mesh<K::Point_3>              Mesh;
			using Vertex_index = Mesh::Vertex_index;
			using Face_index = Mesh::Face_index;
			

			Mesh cgal_mesh;
			bool has_color = (vertex_colors.rows() > 0);
			Mesh::Property_map<Vertex_index, Base::Vec3> vertex_color_property;
			bool created;
			boost::tie(vertex_color_property, created) = cgal_mesh.add_property_map<Vertex_index, Base::Vec3>("v:vertex_color", Base::Vec3::Zero());
			//add vertex
			std::vector<Vertex_index> vertex_map(vertices.rows());
			for (int i = 0; i < vertices.rows(); ++i) {
				const Base::Vec3 &p = vertices.row(i);
				Vertex_index vi = cgal_mesh.add_vertex(Point_3(p[0], p[1], p[2]));
				vertex_map[i] = vi;
			}

			if (has_color) {
				for (int i = 0; i < vertices.rows(); ++i) {
					vertex_color_property[vertex_map[i]] = vertex_colors.row(i);
				}
			}
			//add face
			for (int i = 0; i < faces.rows(); ++i) {
				const Base::Vec3i &f = faces.row(i);
				Face_index fi = cgal_mesh.add_face(vertex_map[f[0]], vertex_map[f[1]], vertex_map[f[2]]);
				if (fi == Mesh::null_face()) {
					std::cout << "warning --- add null face: " << f << std::endl;
				}
				
			}

			//split non-manifold vertices
			std::vector<std::vector<Vertex_index> > duplicated_vertices;
			std::size_t new_vertices_nb = PMP::duplicate_non_manifold_vertices(cgal_mesh,
				NP::output_iterator(
					std::back_inserter(duplicated_vertices)));
//			printf("Create %d new vertices\nProcessed face: %d\n", new_vertices_nb, cgal_mesh.num_faces());
			
			
			//save to Eigen matrix
			vertices.resize(cgal_mesh.num_vertices(), 3);
			if(has_color)
				vertex_colors.resize(cgal_mesh.num_vertices(), 3);
			faces.resize(cgal_mesh.num_faces(), 3);
			
			for (Vertex_index vi : cgal_mesh.vertices()) {
				const Point_3&p = cgal_mesh.point(vi);
				vertices.row(vi) = Base::Vec3(p[0], p[1], p[2]);
			}
			if (has_color) {
				for (Vertex_index vi : cgal_mesh.vertices()) {
					vertex_colors.row(vi) = vertex_color_property[vi];
				}
				for (int i = 0; i < duplicated_vertices.size(); ++i) {
					for (int j = 0; j < duplicated_vertices[i].size(); ++j) {
						vertex_colors.row(duplicated_vertices[i][j]) = vertex_colors.row(duplicated_vertices[i][0]);
					}
				}
				/*printf("First created vertex color: %f,%f,%f\n", vertex_colors.row(duplicated_vertices[0][1])[0],
					vertex_colors.row(duplicated_vertices[0][1])[1],
					vertex_colors.row(duplicated_vertices[0][1])[2]);*/
			}
			

			for (Face_index fi : cgal_mesh.faces()) {
				if (PMP::is_degenerate_triangle_face(fi, cgal_mesh)) {
					std::cout << "Found degenerate face" << std::endl;
				}
				else {
					int n = 0;
					for (Vertex_index vi : cgal_mesh.vertices_around_face(cgal_mesh.halfedge(fi))) {
						faces((uint32_t)fi, n++) = vi;
					}
				}
			}



		}

		void remove_degenerate_faces(Base::TriMesh& mesh) {
			int num_face = mesh.m_faces.rows();
			std::vector<int> valid_faces;
			for (int fi = 0; fi < num_face; ++fi) {
				Eigen::Matrix3d F_V = mesh.m_vertices(mesh.m_faces.row(fi), Eigen::all);
				if (((F_V.row(1) - F_V.row(0)).cross(F_V.row(2) - F_V.row(0))).norm() >= 1e-15) {
					valid_faces.push_back(fi);
				}
			}
			mesh.m_faces = mesh.m_faces(valid_faces, Eigen::all).eval();
		}
	}
}