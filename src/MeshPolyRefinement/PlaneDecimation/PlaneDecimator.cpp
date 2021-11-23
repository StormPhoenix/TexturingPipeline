#include "PlaneDecimator.h"
#include <igl/decimate.h>
#include <igl/collapse_edge.h>
#include <igl/write_triangle_mesh.h>
#include <igl/circulation.h>
#include <igl/is_border_vertex.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/decimate_callback_types.h>
namespace MeshPolyRefinement {
	namespace PlaneDecimation {
		Decimator::Decimator(Base::TriMesh& mesh) :m_mesh(mesh) {
			orig_m = m_mesh.m_faces.rows();
			
			
		}

		void Decimator::edge_cost_color(Eigen::MatrixXi & E, Eigen::MatrixXd& C) {
			Eigen::VectorXi EMAP;
			Eigen::MatrixXi  OE, EF, EI, OF;
			Eigen::MatrixXd OV;
			E.resize(0, 0);
			C.resize(0, 0);
			igl::connect_boundary_to_infinity(m_mesh.m_vertices, m_mesh.m_faces, OV, OF);
			int infinity_vertex_index = OV.rows() - 1;
			igl::edge_flaps(OF, OE, EMAP, EF, EI);
			//printf("Closed mesh edge number: %d\n", OE.rows());
			std::vector<int> valid_edges;
			for (int e = 0; e < OE.rows(); ++e) {
				if (OE(e, 0) != infinity_vertex_index && OE(e, 1) != infinity_vertex_index) {
					valid_edges.push_back(e);
					/*double cost;
					Eigen::RowVectorXd p;
					plane_cost_and_placement(
						e, OV, OF, OE, EMAP, EF, EI, cost, p
					);
					E.conservativeResize(E.rows() + 1, 2);
					E.bottomRows(1) = OE.row(e);
					C.conservativeResize(C.rows() + 1, 3);
					if (std::isfinite(cost)) {
						Eigen::RowVectorXd mid_point = (OV.row(OE(e, 0)) + OV.row(OE(e, 1)))*0.5;
						if(p.isApprox(mid_point,1e-8))
							C.bottomRows(1) = Eigen::RowVector3d(0.0, 0.0, 1.0);
						else
							C.bottomRows(1) = Eigen::RowVector3d(0.0, 1.0, 0.0);
					}
					else {
						C.bottomRows(1) = Eigen::RowVector3d(1.0, 0.0, 0.0);
					}*/
					
				}


			}

			E = OE(valid_edges, Eigen::all);
			C.resize(valid_edges.size(),3);
			for (int i = 0; i < valid_edges.size(); ++i) {
				int e = valid_edges[i];
				double cost;
				Eigen::RowVectorXd p;
				this->plane_cost_and_placement(
					e, OV, OF, OE, EMAP, EF, EI, cost, p
				);
				if (std::isfinite(cost)) {
					Eigen::RowVectorXd mid_point = (OV.row(OE(e, 0)) + OV.row(OE(e, 1)))*0.5;
					if (p.isApprox(mid_point, 1e-8))
						C.row(i) = Eigen::RowVector3d(0.0, 0.0, 1.0);
					else
						C.row(i) = Eigen::RowVector3d(1.0, 1.0, 1.0);
				}
				else {
					C.row(i) = Eigen::RowVector3d(1.0, 0.0, 0.0);
				}
			}

			//std::cout << E.maxCoeff() << std::endl;
		}

		bool Decimator::check_flip(
				const Eigen::MatrixXd &                             V,
      			const Eigen::MatrixXi &                             F,
      			const Eigen::MatrixXi &                             E,
      			const Eigen::VectorXi &                             EMAP,
      			const Eigen::MatrixXi &                             EF,
      			const Eigen::MatrixXi &                             EI,
      			const Eigen::RowVectorXd &							p,
      			const int 											e,
				const std::vector<int>&                             f_s,
				const std::vector<int>& 							f_d
			) {
				return true;
				int v_s = E(e, 0), v_d = E(e, 1);
				int f0 = EF(e, 0), f1 = EF(e, 1);
			
				

				Base::AttributeMatrix updated_V(V);
				updated_V.row(v_s) = p;
				updated_V.row(v_d) = p;
				for(int fi: f_s){
					if(is_valid_face(fi, V, F) && 
					m_mesh.m_face_plane_index[fi] != -1 &&
					fi != f0 &&
					fi != f1){
						
						Eigen::Matrix<double,3,3,Eigen::RowMajor> f_v = updated_V(F.row(fi),Eigen::all);
						Base::Vec3 f_n = (f_v.row(1)-f_v.row(0)).eval().cross(f_v.row(2) - f_v.row(0)).eval().normalized();
						
						Base::Vec3 &p_n = m_mesh.m_plane_groups[m_mesh.m_face_plane_index[fi]].m_plane_normal;
						if(f_n.dot(p_n) <= 0.0){
							//this means degenerated faces or flipped faces
							return false;
						}
					}
				}
				for(int fi: f_d){
					if(is_valid_face(fi, V, F) && 
					m_mesh.m_face_plane_index[fi] != -1 &&
					fi != f0 &&
					fi != f1){
						
						Eigen::Matrix<double,3,3,Eigen::RowMajor> f_v = updated_V(F.row(fi),Eigen::all);
						Base::Vec3 f_n = (f_v.row(1)-f_v.row(0)).eval().cross(f_v.row(2) - f_v.row(0)).eval();
						
						Base::Vec3 &p_n = m_mesh.m_plane_groups[m_mesh.m_face_plane_index[fi]].m_plane_normal;
						if(f_n.dot(p_n) <= 0.0){
							//this means degenerated faces or flipped faces
							return false;
						}
					}
				}
				return true;//safe to collapse

		}
		void Decimator::plane_cost_and_placement(const int e,
			const Eigen::MatrixXd & V,
			const Eigen::MatrixXi & F,
			const Eigen::MatrixXi & E,
			const Eigen::VectorXi & EMAP,
			const Eigen::MatrixXi & EF,
			const Eigen::MatrixXi & EI,
			double &               cost,
			Eigen::RowVectorXd &   p) {
			//printf("calculating cost of edge %d, face number %d\n",e, F.rows());
			int f0 = EF(e, 0), f1 = EF(e, 1);
			if (!is_valid_face(f0, V, F) || !is_valid_face(f1, V, F)) {
				//geometry boundary
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
				return;
			}
			int p0 = m_mesh.m_face_plane_index[f0], p1 = m_mesh.m_face_plane_index[f1];
			if (p0 == -1 || p1 == -1 || p0 != p1) {
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
				return;
			}

			int v_s = E(e, 0), v_d = E(e, 1);
			
			std::vector<int> f_s = igl::circulation(e, false, EMAP, EF, EI);
			
			std::vector<int> f_d = igl::circulation(e, true, EMAP, EF, EI);
			
			bool is_v_s_on_border = false, is_v_d_on_border = false;// on geometry boundaries or plane border
			
			std::unordered_set<int> plane_set;
			bool adjacent_to_non_planar_face = false;
			for (const int fi : f_s) {
				if (is_valid_face(fi,V,F)) {
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
				||(plane_set.size() == 1 && adjacent_to_non_planar_face)) {
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
			
			


			
			//cost = (double)std::max(f_s.size(), f_d.size());
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

			if(!check_flip(V,F,E,EMAP,EF,EI,p,e,f_s,f_d)){
				cost = std::numeric_limits<double>::infinity();
				p = Eigen::RowVector3d::Constant(cost);
			}
			// else{
			// 	printf("Check flip safe\n");
			// }
			
		}

		bool Decimator::is_valid_face(const int fi,
			const Eigen::MatrixXd& V,
			const Eigen::MatrixXi& F) const {
			return (fi >= 0 && fi < orig_m)
				&& (F(fi, 0) != IGL_COLLAPSE_EDGE_NULL ||
					F(fi, 1) != IGL_COLLAPSE_EDGE_NULL ||
					F(fi, 2) != IGL_COLLAPSE_EDGE_NULL);
		}

		void Decimator::update_mesh_info(const Eigen::MatrixXd& U,
			const Eigen::MatrixXi& G,
			const Eigen::VectorXi& J,
			const Eigen::VectorXi& I){
			//update geometry info
			m_mesh.m_vertex_colors = m_mesh.m_vertex_colors(I, Eigen::all).eval();
			m_mesh.m_vertices = U;
			m_mesh.m_faces = G;
			igl::triangle_triangle_adjacency(m_mesh.m_faces, m_mesh.m_ff_adjacency);
			std::vector<std::vector<int>> VFi;
			igl::vertex_triangle_adjacency(m_mesh.m_vertices.rows(), m_mesh.m_faces, m_mesh.m_vf_adjacency, VFi);
			igl::per_face_normals(m_mesh.m_vertices, m_mesh.m_faces, m_mesh.m_face_normals);

			/*int fi = 5012;
			printf("F(%d): %d, %d, %d\n", fi, m_mesh.m_faces(fi, 0), m_mesh.m_faces(fi, 1), m_mesh.m_faces(fi, 2));
			int vi = m_mesh.m_faces(fi, 0);
			printf("V(%d): %f, %f, %f\n", vi, m_mesh.m_vertices(vi, 0), m_mesh.m_vertices(vi, 1), m_mesh.m_vertices(vi, 2));
			vi = m_mesh.m_faces(fi, 1);
			printf("V(%d): %f, %f, %f\n", vi, m_mesh.m_vertices(vi, 0), m_mesh.m_vertices(vi, 1), m_mesh.m_vertices(vi, 2));
			vi = m_mesh.m_faces(fi, 2);
			printf("V(%d): %f, %f, %f\n", vi, m_mesh.m_vertices(vi, 0), m_mesh.m_vertices(vi, 1), m_mesh.m_vertices(vi, 2));*/

			//update plane info
			/*m_mesh.m_face_labels = m_mesh.m_face_labels(J).eval();
			m_mesh.m_face_planar_score = m_mesh.m_face_planar_score(J).eval();*/
			/*After decimation, the face label and face planar score are meaningless, 
			so it's cleared.*/
			m_mesh.m_face_labels.resize(0);
			m_mesh.m_face_planar_score.resize(0);
			m_mesh.m_face_plane_index = m_mesh.m_face_plane_index(J).eval();

			for (Base::PlaneGroup& g : m_mesh.m_plane_groups) {
				g.m_indices.clear();
			}

			for (int fi = 0; fi < m_mesh.m_faces.rows(); ++fi) {
				int pi = m_mesh.m_face_plane_index[fi];
				if (pi != -1) {
					m_mesh.m_plane_groups[pi].m_indices.push_back(fi);
				}
			}
			
			std::vector<Base::PlaneGroup> tmp(m_mesh.m_plane_groups);
			m_mesh.m_plane_groups.clear();
			for(const auto &g : tmp){
				if(g.m_indices.size() > 0){
					m_mesh.m_plane_groups.push_back(g);
				}
			}
			for(int i = 0 ; i < m_mesh.m_plane_groups.size();++i){
				for(auto fi: m_mesh.m_plane_groups[i].m_indices){
					m_mesh.m_face_plane_index[fi] = i;
				}
			}
		}

		bool Decimator::decimate() {
			std::function<void(
				const int              /*e*/,
				const Eigen::MatrixXd &/*V*/,
				const Eigen::MatrixXi &/*F*/,
				const Eigen::MatrixXi &/*E*/,
				const Eigen::VectorXi &/*EMAP*/,
				const Eigen::MatrixXi &/*EF*/,
				const Eigen::MatrixXi &/*EI*/,
				double &               /*cost*/,
				Eigen::RowVectorXd &   /*p*/
				)> cost_and_placement =
				std::bind(&Decimator::plane_cost_and_placement, this,
					std::placeholders::_1,
					std::placeholders::_2,
					std::placeholders::_3,
					std::placeholders::_4,
					std::placeholders::_5,
					std::placeholders::_6,
					std::placeholders::_7,
					std::placeholders::_8,
					std::placeholders::_9);
			std::function<bool(
      const Eigen::MatrixXd &                             ,/*V*/
      const Eigen::MatrixXi &                             ,/*F*/
      const Eigen::MatrixXi &                             ,/*E*/
      const Eigen::VectorXi &                             ,/*EMAP*/
      const Eigen::MatrixXi &                             ,/*EF*/
      const Eigen::MatrixXi &                             ,/*EI*/
      const igl::min_heap< std::tuple<double,int,int> > & ,/*Q*/
      const Eigen::VectorXi &                             ,/*EQ*/
      const Eigen::MatrixXd &                             ,/*C*/
      const int                                           ,/*e*/
      const int                                           ,/*e1*/
      const int                                           ,/*e2*/
      const int                                           ,/*f1*/
      const int                                            /*f2*/
      )> stop_func = [](
					const Eigen::MatrixXd &                             ,/*V*/
      const Eigen::MatrixXi &                             ,/*F*/
      const Eigen::MatrixXi &                             ,/*E*/
      const Eigen::VectorXi &                             ,/*EMAP*/
      const Eigen::MatrixXi &                             ,/*EF*/
      const Eigen::MatrixXi &                             ,/*EI*/
      const igl::min_heap< std::tuple<double,int,int> > & ,/*Q*/
      const Eigen::VectorXi &                             ,/*EQ*/
      const Eigen::MatrixXd &                             ,/*C*/
      const int                                           ,/*e*/
      const int                                           ,/*e1*/
      const int                                           ,/*e2*/
      const int                                           ,/*f1*/
      const int                                            /*f2*/)->bool { return false; };

		igl::decimate_pre_collapse_callback pre_collapse = [](
			const Eigen::MatrixXd &                             ,/*V*/
      const Eigen::MatrixXi &                             ,/*F*/
      const Eigen::MatrixXi &                             ,/*E*/
      const Eigen::VectorXi &                             ,/*EMAP*/
      const Eigen::MatrixXi &                             ,/*EF*/
      const Eigen::MatrixXi &                             ,/*EI*/
      const igl::min_heap< std::tuple<double,int,int> > & ,/*Q*/
      const Eigen::VectorXi &                             ,/*EQ*/
      const Eigen::MatrixXd &                             ,/*C*/
      const int                                            /*e*/
			)->bool{return true;};
	
			igl::decimate_post_collapse_callback never_care =[](
    const Eigen::MatrixXd &                             ,/*V*/
    const Eigen::MatrixXi &                             F,/*F*/
    const Eigen::MatrixXi &                             ,/*E*/
    const Eigen::VectorXi &                             ,/*EMAP*/
    const Eigen::MatrixXi &                             ,/*EF*/
    const Eigen::MatrixXi &                             ,/*EI*/
    const igl::min_heap< std::tuple<double,int,int> > & ,/*Q*/
    const Eigen::VectorXi &                             ,/*EQ*/
    const Eigen::MatrixXd &                             ,/*C*/
    const int                                           e,/*e*/
    const int                                           ,/*e1*/
    const int                                           ,/*e2*/
    const int                                           ,/*f1*/
    const int                                           ,/*f2*/
    const bool                                          collapsed /*collapsed*/
    )-> void { 
		if(true){
			printf("Edge %d collapsed, face number %d\n",e,F.rows());
		}
	};

			Eigen::MatrixXd U, OV;
			Eigen::MatrixXi G, OF;
			Eigen::VectorXi J, I;
			
			igl::connect_boundary_to_infinity(m_mesh.m_vertices, m_mesh.m_faces, OV, OF);
			if (!igl::is_edge_manifold(OF)) {
				std::cout << "Mesh is not manifold" << std::endl;
				return false;
			}
			//igl::decimate(OV, OF, cost_and_placement, stop_func, pre_collapse, never_care, U, G, J, I);
			igl::decimate(OV, OF, cost_and_placement, stop_func, U, G, J, I);
			//bool success = igl::decimate(m_mesh.m_vertices, m_mesh.m_faces, m_mesh.m_faces.rows() / 3, U, G, J);
			if (U.rows() > 0 && G.rows() > 0) {
				const Eigen::Array<bool, Eigen::Dynamic, 1> keep = (J.array() < orig_m);
				igl::slice_mask(Eigen::MatrixXi(G), keep, 1, G);
				igl::slice_mask(Eigen::VectorXi(J), keep, 1, J);
				Eigen::VectorXi _1, I2;
				igl::remove_unreferenced(Eigen::MatrixXd(U), Eigen::MatrixXi(G), U, G, _1, I2);
				igl::slice(Eigen::VectorXi(I), I2, 1, I);
				//igl::write_triangle_mesh("decimated.obj", U, G);
				this->remove_degenerate_face(U, G, J, I);
				update_mesh_info(U, G, J, I);
				std::cout << "Decimation success" << std::endl;
				return true;
			}
			return false;

		}

		void Decimator::remove_degenerate_face(Eigen::MatrixXd& U,
			Eigen::MatrixXi& G,
			Eigen::VectorXi& J,
			Eigen::VectorXi& I) {
			/*Eigen::VectorXi SVI, SVJ;
			Eigen::MatrixXd OV;
			Eigen::MatrixXi OF;*/
			//igl::remove_duplicate_vertices(U, G, 1e-3, OV, SVI, SVJ, OF);
			//printf("Removed %d duplicate vertices after decimation\n", U.rows() - OV.rows());
			//std::cout << (G - OF).maxCoeff() << std::endl;
			//I = I(SVI).eval();
			//U = OV;
			//assert(OF.rows() == G.rows());//no face should be removed by now
			std::vector<int> valid_faces;
			for (int i = 0; i < G.rows(); ++i) {
				Eigen::Matrix3d F_V = U(G.row(i), Eigen::all);
				if (((F_V.row(1) - F_V.row(0)).cross(F_V.row(2) - F_V.row(0))).norm() >= 1e-15) {
					valid_faces.push_back(i);
				}
			}
			printf("Removed %d degenerated faces after decimation\n", G.rows() - valid_faces.size());
			G = G(valid_faces, Eigen::all).eval();
			J = J(valid_faces).eval();
			
			
		}


		

		
	}
}