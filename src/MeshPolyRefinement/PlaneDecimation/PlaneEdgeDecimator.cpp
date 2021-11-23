#include "PlaneEdgeDecimator.h"
#include <igl/circulation.h>
#include <igl/triangle_triangle_adjacency.h>

namespace MeshPolyRefinement {
	namespace PlaneDecimation {
		void PlaneEdgeDecimator::plane_cost_and_placement(const int e,
			const Eigen::MatrixXd & V,
			const Eigen::MatrixXi & F,
			const Eigen::MatrixXi & E,
			const Eigen::VectorXi & EMAP,
			const Eigen::MatrixXi & EF,
			const Eigen::MatrixXi & EI,
			double &               cost,
			Eigen::RowVectorXd &   p) {
			int f0 = EF(e, 0), f1 = EF(e, 1);
			if (!is_valid_face(f0, V, F) || !is_valid_face(f1, V, F)) {
				//geometry boundary
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
				return;
			}

			int p0 = m_mesh.m_face_plane_index[f0], p1 = m_mesh.m_face_plane_index[f1];
			if (p0 == -1 || p1 == -1) {
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
				return;
			}

			int v_s = E(e, 0), v_d = E(e, 1);

			std::vector<int> f_s = igl::circulation(e, false, EMAP, EF, EI);

			std::vector<int> f_d = igl::circulation(e, true, EMAP, EF, EI);
			if (p0 == p1) {
				

				bool is_v_s_on_border = false, is_v_d_on_border = false;// on geometry boundaries or plane border

				std::unordered_set<int> plane_set;
				bool adjacent_to_non_planar_face = false;
				for (const int fi : f_s) {
					if (is_valid_face(fi, V, F)) {
						if (m_mesh.m_face_plane_index[fi] != -1)
							plane_set.insert(m_mesh.m_face_plane_index[fi]);
						else
							adjacent_to_non_planar_face = true;
					}
					else if (fi >= orig_m) {
						is_v_s_on_border = true;
					}
				}
				if (plane_set.size() > 1
					|| (plane_set.size() == 1 && adjacent_to_non_planar_face)) {
					is_v_s_on_border = true;
				}

				plane_set.clear();
				adjacent_to_non_planar_face = false;

				for (const int fi : f_d) {
					if (is_valid_face(fi, V, F)) {
						if (m_mesh.m_face_plane_index[fi] != -1)
							plane_set.insert(m_mesh.m_face_plane_index[fi]);
						else
							adjacent_to_non_planar_face = true;
					}
					else if (fi >= orig_m) {
						is_v_d_on_border = true;
					}
				}
				if (plane_set.size() > 1
					|| (plane_set.size() == 1 && adjacent_to_non_planar_face)) {
					is_v_d_on_border = true;
				}

				if (is_v_s_on_border && is_v_d_on_border) {
					/*Both vertices are on plane border*/
					cost = std::numeric_limits<double>::infinity();
					p = Eigen::RowVector3d::Constant(cost);
					return;
				}

				cost = (V.row(v_s) - V.row(v_d)).norm();
				if (is_v_s_on_border) {
					p = V.row(v_s);
				}
				else if (is_v_d_on_border) {
					p = V.row(v_d);
				}
				else {
					p = (V.row(v_s) + V.row(v_d))*0.5;
				}
			}
			else {
				bool v_s_should_fix = false, v_d_should_fix = false;
				std::unordered_set<int> plane_set;
				bool adjacent_to_non_planar_face = false;
				for (const int fi : f_s) {
					if (is_valid_face(fi, V, F)) {
						if (m_mesh.m_face_plane_index[fi] != -1)
							plane_set.insert(m_mesh.m_face_plane_index[fi]);
						else
							adjacent_to_non_planar_face = true;
					}
					else if (fi >= orig_m) {
						v_s_should_fix = true;
					}
				}
				if (plane_set.size() > 2 || adjacent_to_non_planar_face) {
					v_s_should_fix = true;
				}

				plane_set.clear();

				adjacent_to_non_planar_face = false;

				for (const int fi : f_d) {
					if (is_valid_face(fi, V, F)) {
						if (m_mesh.m_face_plane_index[fi] != -1)
							plane_set.insert(m_mesh.m_face_plane_index[fi]);
						else
							adjacent_to_non_planar_face = true;
					}
					else if (fi >= orig_m) {
						v_d_should_fix = true;
					}
				}
				if (plane_set.size() > 2 || adjacent_to_non_planar_face) {
					v_d_should_fix = true;
				}

				if (v_s_should_fix && v_d_should_fix) {
					/*Both vertices are unmovable*/
					cost = std::numeric_limits<double>::infinity();
					p = Eigen::RowVector3d::Constant(cost);
					return;
				}

				cost = (V.row(v_s) - V.row(v_d)).norm();
				if (v_s_should_fix) {
					p = V.row(v_s);
				}
				else if (v_d_should_fix) {
					p = V.row(v_d);
				}
				else {
					p = (V.row(v_s) + V.row(v_d))*0.5;
				}
			}
			if(!check_flip(V,F,E,EMAP,EF,EI,p,e,f_s,f_d)){
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
			}
		}

		void PlaneEdgeDecimator::remove_degenerate_face(Eigen::MatrixXd& U,
				Eigen::MatrixXi& G,
				Eigen::VectorXi& J,
				Eigen::VectorXi& I){
			Decimator::remove_degenerate_face(U,G,J,I);
			// std::vector<int> valid_faces;
			// Base::IndexMatrix FF;
			// igl::triangle_triangle_adjacency(G, FF);
			// double thres = std::cos(175.0 * M_PI / 180.0);
			// auto test_face = [&](int fi)->bool{
			// 	for(int nfi:FF.row(fi)){
			// 		if(nfi != -1){
			// 			int npi = m_mesh.m_face_plane_index[J[nfi]];
			// 			if(npi != -1){
			// 				Eigen::Matrix3d F_V = U(G.row(fi), Eigen::all);
			// 				Base::Vec3 f_n = (F_V.row(1) - F_V.row(0)).cross(F_V.row(2) - F_V.row(0)).normalized();
			// 				if(f_n.dot(m_mesh.m_plane_groups[npi].m_plane_normal) <= thres)
			// 					return false;
			// 			}
			// 		}
			// 	}
			// 	return true;
			// };
			// for(int fi = 0; fi < G.rows(); ++fi){
			// 	if(test_face(fi))
			// 		valid_faces.push_back(fi);
			// }
			// printf("Removed %d folded faces after decimation\n", G.rows() - valid_faces.size());
			// G = G(valid_faces, Eigen::all).eval();
			// J = J(valid_faces).eval();
			// double thres = std::cos(175.0 * M_PI / 180.0);
			// std::vector<int> valid_faces;
			// for (int i = 0; i < G.rows(); ++i) {
			// 	Eigen::Matrix3d F_V = U(G.row(i), Eigen::all);
			// 	int pi = m_mesh.m_face_plane_index[J[i]];
			// 	Base::Vec3 f_n = (F_V.row(1) - F_V.row(0)).cross(F_V.row(2) - F_V.row(0));
			// 	if (f_n.norm() >= 1e-15) {
			// 		f_n.normalize();
			// 		if(pi == -1 || f_n.dot(m_mesh.m_plane_groups[pi].m_plane_normal) >thres)
			// 			valid_faces.push_back(i);
			// 	}
				
			// }
			// printf("Removed %d degenerated faces after decimation\n", G.rows() - valid_faces.size());
			// G = G(valid_faces, Eigen::all).eval();
			// J = J(valid_faces).eval();
		}
	}
}