#include "Visualize.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/jet.h>

namespace MeshPolyRefinement {
	namespace Visualize {

		void show_mesh(const Base::TriMesh& mesh) {
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.launch();
		}

		void show_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi &F){
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(V, F);
			viewer.launch();
		}

		void show_mesh_cluster(const Base::TriMesh& mesh, const std::vector<int>& cluster_ids, int num_clusters) {
			std::cout << "Cluster number: " << num_clusters << std::endl;
			Eigen::MatrixXd cluster_color(num_clusters, 3);
			for (int i = 0; i < num_clusters; ++i) {
				cluster_color.row(i) = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
			}
			Eigen::MatrixXd C(mesh.m_faces.rows(), 3);
			for (int i = 0; i < C.rows(); ++i) {
				C.row(i) = cluster_color.row(cluster_ids[i]);
			}

			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();
		}

		void show_mesh_labels(const Base::TriMesh& mesh) {
			Eigen::MatrixXd C(mesh.m_faces.rows(),3);

			auto label2color = [&](int face_index) {
				switch (mesh.m_face_labels(face_index)) {
				case Base::FaceSemanticLabel::BUILDING:
					C.row(face_index) = Eigen::RowVector<double, 3>(156.0, 102.0, 102.0);
					break;
				case Base::FaceSemanticLabel::ROAD:
					C.row(face_index) = Eigen::RowVector<double, 3>(128.0, 64.0, 128.0);
					break;
				case Base::FaceSemanticLabel::VEGETATION:
					C.row(face_index) = Eigen::RowVector<double, 3>(107.0, 142.0, 35.0);
					break;
				case Base::FaceSemanticLabel::VEHICLE:
					C.row(face_index) = Eigen::RowVector<double, 3>(0.0, 0.0, 142.0);
					break;
				case Base::FaceSemanticLabel::ROOF:
					C.row(face_index) = Eigen::RowVector<double, 3>(70.0, 70.0, 70.0);
					break;
				case Base::FaceSemanticLabel::OTHER:
					C.row(face_index) = Eigen::RowVector<double, 3>(0.0, 0.0, 0.0);
					break;
				case Base::FaceSemanticLabel::MULTILABELED:
					C.row(face_index) = Eigen::RowVector<double, 3>(255.0, 0.0, 0.0);
					break;
				default://UNSET
					C.row(face_index) = Eigen::RowVector<double, 3>(255.0, 255.0, 0.0);
					break;
				}
				C.row(face_index) /= 255.0;
			};
			igl::parallel_for(mesh.m_faces.rows(), label2color);
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();

		}

		/*void show_mesh_seed_map(const Base::TriMesh& mesh, float thres) {
			
			PlaneEstimation::SeedMap seed_map(mesh, thres);

			Eigen::MatrixXd C(mesh.m_faces.rows(), 3);
			
			C.setZero();
			auto seed2color = [&](int seed_index) {
				if (seed_map[seed_index] != std::size_t(-1)) {
					C.row(seed_map[seed_index]) = Eigen::Vector3d(1.0, 1.0, 0.0);
				}
			};
			igl::parallel_for(mesh.m_faces.rows(), seed2color);

			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();
		}*/

		void show_mesh_planar_score(const Base::TriMesh& mesh, float thres) {
			//PlaneEstimation::SeedMap seed_map(mesh, thres);

			Eigen::MatrixXd C(mesh.m_faces.rows(), 3);

			auto score2color = [&](int face_index) {
				if (mesh.m_face_planar_score[face_index] > thres) {
					C.row(face_index) = Eigen::RowVector3d(1.0, 1.0, 0.0);
				}
				else {
					C.row(face_index) = Eigen::RowVector3d(1.0, 0.0, 0.0);
				}
			};
			igl::parallel_for(mesh.m_faces.rows(), score2color);
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();
		}

		void show_mesh_plane_segments(const Base::TriMesh& mesh) {
			Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.m_faces.rows(), 3);
			const auto&groups = mesh.m_plane_groups;
			for (const auto& g : groups) {
				Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				for (const std::size_t face_index : g.m_indices) {
					C.row(face_index) = color;
				}
			}

			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();
		}

		void show_mesh_plane_segments(const Base::TriMesh& mesh, int gi){
			
			Eigen::MatrixXd C = Eigen::RowVector3d(1.0,1.0,0.0).replicate(mesh.m_faces.rows(),1).eval();
			for(const auto fi : mesh.m_plane_groups[gi].m_indices){
				C.row(fi) = Eigen::RowVector3d(1.0,0.0,0.0);
			}
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();
		}

		void show_mesh_decimation_edge_cost(const Base::TriMesh & mesh, const Eigen::MatrixXi& E, const Eigen::MatrixXd& C) {
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_edges(mesh.m_vertices, E, C);
			viewer.data().line_width = 2.0;
			Eigen::MatrixXd FC = Eigen::MatrixXd::Zero(mesh.m_faces.rows(), 3);
			const auto&groups = mesh.m_plane_groups;
			for (const auto& g : groups) {
				Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				for (const std::size_t face_index : g.m_indices) {
					FC.row(face_index) = color;
				}
			}
			viewer.data().set_colors(FC);
			viewer.launch();
		}

		void show_mesh_face_feature(const Base::TriMesh& mesh, const std::vector<double>& feature) {
			if (feature.size() != mesh.m_faces.rows()) {
				std::cout << "Feature should be consistent with face number" << std::endl;
			}
			Eigen::VectorXd fv(feature.size());
			memcpy(fv.data(), feature.data(), sizeof(double)*feature.size());
			Eigen::MatrixXd C;
			igl::jet(fv, true, C);
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
			viewer.data().set_colors(C);
			viewer.launch();

		}

		void show_mesh_plane_segments(const Base::TriMesh& mesh1, const Base::TriMesh& mesh2) {

			Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero(mesh1.m_faces.rows(), 3);
			const auto&groups1 = mesh1.m_plane_groups;
			for (const auto& g : groups1) {
				Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				for (const std::size_t face_index : g.m_indices) {
					C1.row(face_index) = color;
				}
			}

			Eigen::MatrixXd C2 = Eigen::MatrixXd::Zero(mesh2.m_faces.rows(), 3);
			/*const auto& groups2 = mesh2.m_plane_groups;
			for (const auto& g : groups2) {
				Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				for (const std::size_t face_index : g.m_indices) {
					C2.row(face_index) = color;
				}
			}*/
			for (int i = 0; i < mesh2.m_faces.rows(); ++i) {
				if (mesh1.m_face_plane_index[i] == -1
					&& mesh2.m_face_plane_index[i] != -1) {
					C2.row(i) = Eigen::RowVector3d(1.0, 1.0, 0.0);
				}
			}

			igl::opengl::glfw::Viewer viewer;
			viewer.append_mesh();
			//std::cout << viewer.data_list.size() << std::endl;
			unsigned int left_view, right_view;
			int mesh1_id = viewer.data_list[0].id;
			int mesh2_id = viewer.data_list[1].id;
			viewer.callback_init = [&](igl::opengl::glfw::Viewer &)
			{
				viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
				left_view = viewer.core_list[0].id;
				right_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
				return false;
			};

			viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
				v.core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
				v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
				return true;
			};
			viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
			{
				if (key == GLFW_KEY_SPACE)
				{
					// By default, when a core is appended, all loaded meshes will be displayed in that core.
					// Displaying can be controlled by calling viewer.data().set_visible().
					viewer.data(mesh2_id).set_visible(false, left_view);
					viewer.data(mesh1_id).set_visible(false, right_view);
				}
				return false;
			};

			viewer.data(mesh1_id).set_mesh(mesh1.m_vertices, mesh1.m_faces);
			viewer.data(mesh1_id).set_colors(C1);
			//viewer.data(mesh1_id).set_visible(false, right_view);
			viewer.data(mesh1_id).show_lines = false;

			viewer.data(mesh2_id).set_mesh(mesh2.m_vertices, mesh2.m_faces);
			viewer.data(mesh2_id).set_colors(C2);
			//viewer.data(mesh2_id).set_visible(false, left_view);
			viewer.data(mesh2_id).show_lines = false;
			viewer.launch();
		}

		void show_polygon(const Base::AttributeMatrix& V) {
			igl::opengl::glfw::Viewer viewer;
			int np = V.rows();
			Eigen::MatrixXi E(np, 2);
			for (int i = 0; i < np; ++i) {
				int ni = (i + 1) % np;
				E.row(i) = Eigen::RowVector2i(i, ni);
			}
			viewer.data().set_edges(V, E, Eigen::RowVector3d(1.0, 1.0, 0.0));
			viewer.launch();
		}

		void show_plane_polygon(const Base::TriMesh& mesh, const Base::PlaneGroup& g, const std::vector<int>& poly) {
			igl::opengl::glfw::Viewer viewer;
			Base::IndexMatrix plane_face = mesh.m_faces(g.m_indices, Eigen::all);
			viewer.data().set_mesh(mesh.m_vertices, plane_face);
			int ep = poly.size();
			Eigen::MatrixXi E(ep, 2);
			for (int i = 0; i < ep; ++i) {
				int ni = (i + 1) % ep;
				E.row(i) = Eigen::RowVector2i(poly[i], poly[ni]);
			}

			viewer.data().set_edges(mesh.m_vertices, E, Eigen::RowVector3d(1.0, 0.0, 0.0));
			viewer.launch();

		}

		void show_boundary_and_holes(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const std::vector<std::vector<int>>&loops, int bound_loop_id){
			
			//std::cout << V << std::endl;
			printf("%d loops, outter boundary %d\n", loops.size(),bound_loop_id);
			int edge_number = 0;
			for(const auto& loop : loops){
				edge_number += loop.size();
			}
			Eigen::MatrixXi E(edge_number,2);
			Eigen::MatrixXd C(edge_number,3);
			int n = 0;
			for(int l = 0; l < loops.size();++l){
				printf("loop %d, size %d\n",l, loops[l].size());
				
				Eigen::RowVector3d color(1.0,1.0,1.0);
				if(l == bound_loop_id){
					color = Eigen::RowVector3d(1.0,0.0,0.0);
				}
				for(int i = 0; i < loops[l].size() - 1; ++i){
					//printf("loop %d, vi %d\n",l,loops[l][i]);
					//std::cout << V.row(loops[l][i]) <<std::endl;
					//viewer.data().add_edges(V.row(loops[l][i]),V.row(loops[l][i+1]),color);
					E.row(n) = Eigen::RowVector2i(loops[l][i], loops[l][i+1]);
					C.row(n++) = color;
				}
				//viewer.data().add_edges(V.row(loops[l][loops[l].size() - 1]),V.row(loops[l][0]),color);
				E.row(n) = Eigen::RowVector2i(loops[l][loops[l].size() - 1],loops[l][0]);
				C.row(n++) = color;

			}
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(V,F);
			viewer.data().set_edges(V,E,C);
			viewer.data().line_width =1.5;
			viewer.launch();
		}

		void show_mesh_constrained_edge(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const Eigen::MatrixXi& E){
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(V,F);
			viewer.data().set_edges(V,E,Eigen::RowVector3d(1.0,0.0,1.0));
			viewer.data().line_width = 1.5;
			viewer.launch();
		}

		void show_path(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const std::vector<int>& path){
			Eigen::MatrixXi E(path.size() - 1, 2);
			Eigen::MatrixXd C(path.size() - 1, 3);
			for(int i = 0; i < E.rows() - 1; ++i){
				E.row(i) = Eigen::RowVector2i(path[i], path[i+1]);
				C.row(i) = Eigen::RowVector3d(1.0,1.0,1.0);
			}
			E.row(E.rows() - 1) = Eigen::RowVector2i(path[E.rows() - 1], path[E.rows()]);
			C.row(E.rows() - 1) = Eigen::RowVector3d(1.0,0.0,0.0);
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(V,F);
			viewer.data().set_edges(V,E,C);
			viewer.data().line_width = 1.5;
			viewer.launch();
		}

		void show_mesh_edge(const Eigen::MatrixXd& V,const Eigen::MatrixXi& F, const Eigen::MatrixXi& E, const Eigen::MatrixXd& C){
			igl::opengl::glfw::Viewer viewer;
			viewer.data().set_mesh(V,F);
			viewer.data().set_edges(V,E,C);
			viewer.data().line_width = 1.5;
			viewer.launch();
		}
	}
}