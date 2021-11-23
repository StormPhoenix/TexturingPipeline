#include "Optimize.h"

#include <ceres/ceres.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/writePLY.h>
#include <igl/line_search.h>

#include "EnergyFunction.h"

#include <fstream>

namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		void update_mesh(Base::TriMesh& mesh) {//update mesh topology and plane group info
			igl::triangle_triangle_adjacency(mesh.m_faces, mesh.m_ff_adjacency);
			std::vector<std::vector<int>> VFi;
			igl::vertex_triangle_adjacency(mesh.m_vertices.rows(), mesh.m_faces, mesh.m_vf_adjacency, VFi);

			for (auto & g : mesh.m_plane_groups) {
				g.m_indices.clear();
			}

			for (int fi = 0; fi < mesh.m_faces.rows(); ++fi) {
				if (mesh.m_face_plane_index[fi] != -1) {
					mesh.m_plane_groups[mesh.m_face_plane_index[fi]].m_indices.push_back(fi);
				}
			}

		}

		void copy_face_info(Base::TriMesh& mesh, int from, int to) {
			mesh.m_face_labels[to] = mesh.m_face_labels[from];
			mesh.m_face_normals.row(to) = mesh.m_face_normals.row(from);
			mesh.m_face_planar_score[to] = mesh.m_face_planar_score[from];
			mesh.m_face_plane_index[to] = mesh.m_face_plane_index[from];
			
		}

		void slice_mesh_faces(Base::TriMesh& mesh, std::vector<int>& valid_faces) {
			
			auto tmp_f = mesh.m_faces(valid_faces, Eigen::all).eval();
			//std::cout << tmp_f << std::endl;
			//mesh.m_faces.resizeLike(tmp_f);
			mesh.m_faces = tmp_f;
			//std::cout << mesh.m_faces << std::endl;
			//igl::writePLY("split.ply", mesh.m_vertices, mesh.m_faces, true);

			auto tmp_f_label = mesh.m_face_labels(valid_faces, Eigen::all).eval();
			//mesh.m_face_labels.resizeLike(tmp_f_label);
			mesh.m_face_labels = tmp_f_label;

			auto tmp_f_normals = mesh.m_face_normals(valid_faces, Eigen::all).eval();
			//mesh.m_face_normals.resizeLike(tmp_f_normals);
			mesh.m_face_normals = tmp_f_normals;

			auto tmp_f_planar_score = mesh.m_face_planar_score(valid_faces, Eigen::all).eval();
			//mesh.m_face_planar_score.resizeLike(tmp_f_planar_score);
			mesh.m_face_planar_score = tmp_f_planar_score;

			auto tmp_f_plane_index = mesh.m_face_plane_index(valid_faces, Eigen::all).eval();
			//mesh.m_face_plane_index.resizeLike(tmp_f_plane_index);
			mesh.m_face_plane_index = tmp_f_plane_index;
		}

		/*i means ith vertex in fi.
		nfi and fi share the edge [i, i+1] in fi
		*/
		void split_face(Base::TriMesh& mesh, int fi, int nfi, int i) {
			const auto& vertex_indices = mesh.m_faces.row(fi);
			int vi0 = vertex_indices[i];
			int vi1 = vertex_indices[(i + 1) % 3];
			int vi2 = vertex_indices[(i + 2) % 3];
			//create new vertex
			Base::Vec3 new_vertex = (mesh.m_vertices.row(vi0) + mesh.m_vertices.row(vi1))*0.5;
			int new_vertex_index = mesh.m_vertices.rows();
			mesh.m_vertices.conservativeResize(mesh.m_vertices.rows() + 1, Eigen::NoChange_t::NoChange);
			mesh.m_vertices.row(new_vertex_index) = new_vertex;

			int face_start = mesh.m_faces.rows();
			mesh.m_faces.conservativeResize(mesh.m_faces.rows() + 4, Eigen::NoChange_t::NoChange);
			mesh.m_face_normals.conservativeResize(mesh.m_faces.rows(), Eigen::NoChange_t::NoChange);
			mesh.m_face_planar_score.conservativeResize(mesh.m_faces.rows(), Eigen::NoChange_t::NoChange);
			mesh.m_face_labels.conservativeResize(mesh.m_faces.rows(),Eigen::NoChange_t::NoChange);
			mesh.m_face_plane_index.conservativeResize(mesh.m_faces.rows(), Eigen::NoChange_t::NoChange);

			//split current face
			mesh.m_faces.row(face_start) = Base::Vec3i(new_vertex_index, vi1, vi2);
			mesh.m_faces.row(face_start + 1) = Base::Vec3i(vi2, vi0, new_vertex_index);
			copy_face_info(mesh, fi, face_start);
			copy_face_info(mesh, fi, face_start + 1);
			//split opposite face
			const auto& oppo_vertices = mesh.m_faces.row(nfi);
			int oppo_v_index = -1;
			for (int ov : oppo_vertices) {
				if (ov != vi0 && ov != vi1) {
					oppo_v_index = ov;
					break;
				}
			}
			//std::cout << "new vertex: " << new_vertex_index << ", oppo vertex: " << oppo_v_index << std::endl;
			mesh.m_faces.row(face_start + 2) = Base::Vec3i(new_vertex_index, oppo_v_index, vi1);
			mesh.m_faces.row(face_start + 3) = Base::Vec3i(vi0, oppo_v_index, new_vertex_index);
			copy_face_info(mesh, nfi, face_start + 2);
			copy_face_info(mesh, nfi, face_start + 3);

		}

		void split_mesh_triangle(Base::TriMesh& mesh) {
			int original_face_number = mesh.m_faces.rows();
			std::vector<int> valid_faces;
			std::vector<bool> is_valid(original_face_number, true);
			for (int fi = 0; fi < original_face_number; ++fi) {
				if (!is_valid[fi])
					continue;
				int plane0 = mesh.m_face_plane_index[fi];
				if (plane0 == -1)
					continue;
				const auto& neighbors = mesh.m_ff_adjacency.row(fi);
				const auto& vertices = mesh.m_faces.row(fi);
				
				if (neighbors.minCoeff() >= 0) {//This condition must be satisfied when the plane region size is larger than 1.
					for (int i = 0; i < 3; ++i) {
						int nf0 = neighbors[i];
						int nf1 = neighbors[(i + 1) % 3];
						int nf2 = neighbors[(i + 2) % 3];
						if (mesh.m_face_plane_index[nf0] == plane0
							&& mesh.m_face_plane_index[nf1] == mesh.m_face_plane_index[nf2]
							&& mesh.m_face_plane_index[nf1] != -1
							&& mesh.m_face_plane_index[nf1] != plane0) {
							int face_start = mesh.m_faces.rows();
							
							split_face(mesh, fi, nf0, i);
							
							valid_faces.push_back(face_start);
							valid_faces.push_back(face_start + 1);
							valid_faces.push_back(face_start + 2);
							valid_faces.push_back(face_start + 3);
							is_valid[fi] = false;
							is_valid[nf0] = false;
							break;
						}
					}
				}

				
			}

			for (int fi = 0; fi < original_face_number; ++fi) {
				if (is_valid[fi]) {
					valid_faces.push_back(fi);
				}
			}

			slice_mesh_faces(mesh, valid_faces);
			
			update_mesh(mesh);
		}

		void optimize_mesh(Base::TriMesh& mesh, int max_iter) {
			
			//std::vector<double> variables(mesh.m_vertices.rows() * 3);
			double *variables = new double[mesh.m_vertices.rows() * 3];
			
			memcpy(variables, mesh.m_vertices.data(), sizeof(double)*mesh.m_vertices.size());

			ceres::GradientProblem problem(new NonFlipPlaneEnergy(mesh));
			ceres::GradientProblemSolver::Options options;
			options.minimizer_progress_to_stdout = true;
			options.max_num_iterations = max_iter;
			
			options.max_num_line_search_step_size_iterations = 50;
			options.max_line_search_step_expansion = 100.0;
			/*options.line_search_direction_type = ceres::LineSearchDirectionType::STEEPEST_DESCENT;
			options.line_search_type = ceres::LineSearchType::ARMIJO;
			options.max_num_line_search_step_size_iterations = 50;
			options.max_num_line_search_direction_restarts = 20;*/



			ceres::GradientProblemSolver::Summary summary;
			std::cout << "Optimizing..." << std::endl;
			ceres::Solve(options, problem, variables, &summary);

			mesh.m_vertices = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(variables,mesh.m_vertices.rows(),3);
			//update face normals
			//
			std::cout << summary.FullReport() << std::endl;
			std::ofstream log_file;
			log_file.open("log.txt", std::ios::out);
			log_file << summary.FullReport() << std::endl;
			log_file.close();
			delete[]variables;
			variables = nullptr;
		}

		Eigen::MatrixXd grad2direction(const Eigen::MatrixXd& grad) {
			return -grad.eval();
		}
		
		void optimize_mesh_line_search(Base::TriMesh& mesh) {
			std::size_t variable_number = mesh.m_vertices.rows() * 3;
			Eigen::MatrixXd variables(variable_number, 1), grad(variable_number, 1);
			memcpy(variables.data(), mesh.m_vertices.data(), sizeof(double)*variable_number);
			NonFlipPlaneEnergy energy(mesh);
			auto func = [&](Eigen::MatrixXd& x)->double {
				return energy.compute_energy(x);
			};
			std::function<double(Eigen::MatrixXd&)> compute = func;
			double cost = energy.compute_energy(variables);
			double last_cost = cost;
			int max_iteration = 500;
			int iteration = 0;
			double step_size = 0.1, energy_delta_threshold = 1e-10;
			std::cout << "Initial cost: " << cost << std::endl;
			while (iteration < max_iteration) {
				energy.compute_gradient(variables, grad);
				Eigen::MatrixXd direction = grad2direction(grad);
				cost = igl::line_search(variables, direction, step_size, compute, last_cost);
				if (cost > last_cost) {
					std::cout << "Cannot reduce energy, optimization failed." << std::endl;
					break;
				}
				else if (last_cost - cost <= energy_delta_threshold){
					std::cout << "Energy converged." << std::endl;
					break;
				}
				else {
					std::cout << "Iteration: " <<iteration << ", cost: " << cost << std::endl;
					last_cost = cost;
				}
				++iteration;
			}
			if (iteration == max_iteration) {
				std::cout << "Iteration number limited" << std::endl;
			}
			memcpy(mesh.m_vertices.data(), variables.data(), sizeof(double)*variable_number);
			
		}
	}
}