#include "EnergyFunction.h"

#include <igl/parallel_for.h>

#include <unordered_set>

#include <math.h>
#include <float.h>


namespace MeshPolyRefinement {
	namespace PlaneOptimization {
		NonFlipPlaneEnergy::NonFlipPlaneEnergy(const Base::TriMesh& mesh) 
			:m_initial_face_normals(mesh.m_face_normals),
			m_faces(mesh.m_faces),
			m_planes(mesh.m_plane_groups),
			m_face_plane_index(mesh.m_face_plane_index)
		{
			m_num_vertices = mesh.m_vertices.rows();
			m_num_faces = mesh.m_faces.rows();
			gamma = 0.1;
			exp_n = 1.0;
			
			barrier_weight = 1.0;
			plane_weight = 1.0;
			normal_weight = 1.0;
			distance_weight = 0.3;
			
			
			

			angle_cos_thres = cos(90.0 * M_PI / 180.0);

			m_initial_vertices.resize(m_num_vertices*3);
			memcpy(m_initial_vertices.data(),mesh.m_vertices.data(),sizeof(double)*3*m_num_vertices);

			m_zero_quadric_mat = new Eigen::Matrix4d();
			m_zero_quadric_mat->setZero();
			
			m_vertex_quad_mats.resize(m_num_vertices);
			m_vertex_plane_index.resize(m_num_vertices);
			auto compute_quadric_matrix = [&](int vertex_index) {
				std::unordered_set<int> plane_set = mesh.get_vertex_plane_set(vertex_index);
				
				if (plane_set.empty()) {
					m_vertex_quad_mats[vertex_index] = m_zero_quadric_mat;
				}
				else {
					Eigen::Matrix4d* quadric_mat = new Eigen::Matrix4d(*m_zero_quadric_mat);
					
					for (int pi : plane_set) {
						
						const auto& g = mesh.m_plane_groups[pi];
						Eigen::Vector4d p = g.params.cast<double>().transpose();
						*quadric_mat += p*p.transpose();
					}
					m_vertex_quad_mats[vertex_index] = quadric_mat;
				}
				m_vertex_plane_index[vertex_index] = plane_set;
			};
			std::cout << "Computing vertex quadric matrix" << std::endl;
#ifdef DEBUG
			//std::cout << mesh.m_vertices << std::endl;
			//std::cout << mesh.m_faces << std::endl;
			for (int vertex_index = 0; vertex_index < m_num_vertices; ++vertex_index) {
				compute_quadric_matrix(vertex_index);
			}
#else
			igl::parallel_for(m_num_vertices, compute_quadric_matrix);
#endif

			// m_initial_face_area.resize(mesh.m_faces.rows());
			// auto compute_face_area = [&](int face_index) {
			// 	Base::Vec3d A = mesh.m_vertices.row(mesh.m_faces(face_index, 0));
			// 	Base::Vec3d B = mesh.m_vertices.row(mesh.m_faces(face_index, 1));
			// 	Base::Vec3d C = mesh.m_vertices.row(mesh.m_faces(face_index, 2));
			// 	m_initial_face_area[face_index] = (B - A).cross(C - A).norm()*0.5;
			// };
			/*std::cout << "Computing face area" << std::endl;
			igl::parallel_for(m_num_faces, compute_face_area);*/

			build_quadirc_mat();

			//build_face_matrix();
		
		}

		void NonFlipPlaneEnergy::build_face_matrix() {

			m_mask.resize(m_num_faces,3);
			m_mask.setZero();
			m_target_normal.resize(m_num_faces,3);
			m_target_normal.setZero();
			m_AB_mask.resize(m_num_faces, m_num_vertices);
			m_AC_mask.resize(m_num_faces, m_num_vertices);
			std::vector<int> num_elem_per_row(m_num_faces, 2);
			m_AB_mask.reserve(num_elem_per_row);
			m_AC_mask.reserve(num_elem_per_row);

			auto func = [&](int face_index) {
				m_AB_mask.insert(face_index, m_faces(face_index, 0)) = -1.0;
				m_AB_mask.insert(face_index, m_faces(face_index, 1)) = 1.0;
				m_AC_mask.insert(face_index, m_faces(face_index, 0)) = -1.0;
				m_AC_mask.insert(face_index, m_faces(face_index, 2)) = 1.0;
				if (m_face_plane_index[face_index] == -1) {
					m_mask.row(face_index) = Eigen::RowVector3d(1.0, 1.0, 1.0);
				}
				else {
					m_target_normal.row(face_index) = m_planes[m_face_plane_index[face_index]].m_plane_normal.cast<double>();
				}
			};

			igl::parallel_for(m_num_faces, func);
		}

		void NonFlipPlaneEnergy::build_quadirc_mat() {
			m_quadric_mat = Eigen::SparseMatrix<double>(4 * m_num_vertices, 4 * m_num_vertices);
			Eigen::VectorXi num_elem_per_col = Eigen::VectorXi::Constant(4 * m_num_vertices, 4);
			m_quadric_mat.reserve(num_elem_per_col);
			
			auto insert_quadric_mat_elements = [&](int vertex_index) {
				int row_start = 4 * vertex_index;
				int col_start = 4 * vertex_index;

				//col-major traverse
				for (int col = 0; col < 4; ++col) {
					for (int row = 0; row < 4; ++row) {
						m_quadric_mat.insert(row_start + row, col_start + col) = (*m_vertex_quad_mats[vertex_index])(row, col);
					}
				}
			};
			igl::parallel_for(m_num_vertices, insert_quadric_mat_elements);

		}

		NonFlipPlaneEnergy::~NonFlipPlaneEnergy() {
			for (int i = 0; i < m_num_vertices; ++i) {
				if (m_vertex_quad_mats[i] != m_zero_quadric_mat) {
					delete m_vertex_quad_mats[i];
				}
				m_vertex_quad_mats[i] = nullptr;
			}
			delete m_zero_quadric_mat;
			m_zero_quadric_mat = nullptr;
		}

		Base::Vec3 NonFlipPlaneEnergy::project_gradient_onto_plane(int vertex_index, Base::Vec3 grad) const{
			return grad;
			Base::Vec3 result;
			switch (m_vertex_plane_index[vertex_index].size()) {
			case 0:
				result = grad;
				break;
			case 1:
			{
				Base::Vec3 N = m_planes[*(m_vertex_plane_index[vertex_index].begin())].m_plane_normal;
				result = grad - (grad.dot(N)*N);
				//std::cout << "grad: " << grad << std::endl;
				//std::cout << "projected grad: " << result << std::endl;
				break;
			}
			case 2:
			{
				Base::Vec3 N0 = m_planes[*(m_vertex_plane_index[vertex_index].begin())].m_plane_normal;
				Base::Vec3 N1 = m_planes[*(++(m_vertex_plane_index[vertex_index].begin()))].m_plane_normal;
				Base::Vec3 N_line = N0.cross(N1).normalized();
				result = grad.dot(N_line)*N_line;
				//std::cout << "grad: " << grad << std::endl;
				//std::cout << "projected grad: " << result << std::endl;
				break;
			}
			default:
				result.setZero();
				break;
			}
			return result;
		}

		double NonFlipPlaneEnergy::compute_distance_energy(const double* parameters) const{
			Eigen::VectorXd variable(NumParameters());
			memcpy(variable.data(),parameters,sizeof(double)*NumParameters());
			return distance_weight*(variable- m_initial_vertices).squaredNorm();
		}

		Eigen::VectorXd NonFlipPlaneEnergy::compute_distance_gradient(const double* parameters) const{
			Eigen::VectorXd variable(NumParameters());
			memcpy(variable.data(),parameters,sizeof(double)*NumParameters());
			return distance_weight * 2 * (variable- m_initial_vertices);
		}

		double NonFlipPlaneEnergy::compute_quadric_energy(const double* parameters) const {
			

			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords(m_num_vertices, 3);
			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords_homo(m_num_vertices, 4);
			memcpy(coords.data(), parameters, sizeof(double)*NumParameters());
			coords_homo = coords.rowwise().homogeneous();
			Eigen::VectorXd variable_vector = Eigen::Map<Eigen::VectorXd>(coords_homo.data(), 4 * m_num_vertices, 1);
			
			return plane_weight*variable_vector.transpose()*m_quadric_mat*variable_vector;
			//std::cout << "Plane quadric energy: " << *cost - init_cost << ", ";
		}

		Eigen::VectorXd NonFlipPlaneEnergy::compute_quadric_gradient(const double* parameters) const {
			

			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords(m_num_vertices, 3);
			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords_homo(m_num_vertices, 4);
			memcpy(coords.data(), parameters, sizeof(double)*NumParameters());
			coords_homo = coords.rowwise().homogeneous();
			Eigen::VectorXd variable_vector = Eigen::Map<Eigen::VectorXd>(coords_homo.data(), 4 * m_num_vertices, 1);
			Eigen::VectorXd grad_homo = plane_weight * 2 * m_quadric_mat*variable_vector;
			
			coords_homo = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(grad_homo.data(), m_num_vertices, 4);
			coords = coords_homo(Eigen::all, { 0,1,2 });
			Eigen::VectorXd gradient = Eigen::Map<Eigen::VectorXd>(coords.data(), 3 * m_num_vertices, 1);
			return gradient;
			//std::cout << "Plane grad: " << func_grad.squaredNorm() << ", ";
		}

		double NonFlipPlaneEnergy::compute_quadric_energy_gradient(const double* parameters, Eigen::VectorXd& gradient) {
			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords(m_num_vertices, 3);
			Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords_homo(m_num_vertices, 4);
			memcpy(coords.data(), parameters, sizeof(double)*NumParameters());
			coords_homo = coords.rowwise().homogeneous();
			Eigen::VectorXd variable_vector = Eigen::Map<Eigen::VectorXd>(coords_homo.data(), 4 * m_num_vertices, 1);
			Eigen::MatrixXd tmp_mat = m_quadric_mat*variable_vector;
			double cost = plane_weight*(variable_vector.transpose()*tmp_mat)(0,0);

			Eigen::VectorXd grad_homo = plane_weight * 2 * tmp_mat;
			coords_homo = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(grad_homo.data(), m_num_vertices, 4);
			coords = coords_homo(Eigen::all, { 0,1,2 });
			gradient = Eigen::Map<Eigen::VectorXd>(coords.data(), 3 * m_num_vertices, 1);

			return cost;
		}

		double NonFlipPlaneEnergy::compute_normal_energy(const double* parameters) const {
			double cost = 0;
			for (int i = 0; i < m_num_faces; ++i) {
				int offset = m_faces(i, 0) * 3;
				Base::Vec3d A(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
				offset = m_faces(i, 1) * 3;
				Base::Vec3d B(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
				offset = m_faces(i, 2) * 3;
				Base::Vec3d C(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
				Base::Vec3d orientation = (B - A).cross(C - A);
				double orientation_norm = orientation.norm();
				if (std::fabs(orientation_norm) < 1e-15) {
					cost = std::numeric_limits<double>::infinity();//found degenerate face
					std::cout << "Found degenerate face "<< i << ": " << m_faces.row(i) << std::endl;
					std::cout << "Vertices: " << std::endl << A << std::endl << B << std::endl << C << std::endl;
					break;
				}
				else if (m_face_plane_index[i] != -1){
					
					Base::Vec3d plane_normal = m_planes[m_face_plane_index[i]].m_plane_normal.cast<double>();
					double inner_product = orientation.dot(plane_normal);
					cost += normal_weight*std::pow(orientation_norm - inner_product, exp_n);
				}
			}
			return cost;
			
			
			
		}

		Eigen::VectorXd NonFlipPlaneEnergy::compute_normal_gradient(const double* parameters) const {
			Eigen::VectorXd gradient = Eigen::VectorXd::Zero(NumParameters());
			
			//Eigen::Matrix<double, -1, -1, Eigen::RowMajor> coords(m_num_vertices, 3);
			//memcpy(coords.data(), parameters, sizeof(double) * 3 * m_num_vertices);
			for (const auto & plane : m_planes) {
				for (int i : plane.m_indices) {
					int offset = m_faces(i, 0) * 3;
					Base::Vec3d A(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
					offset = m_faces(i, 1) * 3;
					Base::Vec3d B(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
					offset = m_faces(i, 2) * 3;
					Base::Vec3d C(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
					//const Base::Vec3d &A = coords.row(m_faces(i, 0)), &B = coords.row(m_faces(i, 1)), &C = coords.row(m_faces(i, 2));

					Base::Vec3d orientation = (B - A).cross(C - A);

					double orientation_norm = orientation.norm();


					Base::Vec3d plane_normal = m_planes[m_face_plane_index[i]].m_plane_normal.cast<double>();

					double diff = orientation_norm - orientation.dot(plane_normal);
					if (fabs(diff) < 1e-8) {
						continue;
					}
					double diff_gradient = normal_weight*exp_n*std::pow(diff, exp_n - 1.0);

					double diff_partial_oritation_x = orientation[0] / orientation_norm - plane_normal[0];
					double diff_partial_oritation_y = orientation[1] / orientation_norm - plane_normal[1];
					double diff_partial_oritation_z = orientation[2] / orientation_norm - plane_normal[2];

					double dx, dy, dz;

					offset = m_faces(i, 0) * 3;
					dx = diff_gradient*(diff_partial_oritation_y*(C[2] - B[2]) + diff_partial_oritation_z*(B[1] - C[1]));
					dy = diff_gradient*(diff_partial_oritation_x*(B[2] - C[2]) + diff_partial_oritation_z*(C[0] - B[0]));
					dz = diff_gradient*(diff_partial_oritation_x*(C[1] - B[1]) + diff_partial_oritation_y*(B[0] - C[0]));
					gradient[offset] += dx;
					gradient[offset + 1] += dy;
					gradient[offset + 2] += dz;

					offset = m_faces(i, 1) * 3;
					dx = diff_gradient*(diff_partial_oritation_y*(A[2] - C[2]) + diff_partial_oritation_z*(C[1] - A[1]));
					dy = diff_gradient*(diff_partial_oritation_x*(C[2] - A[2]) + diff_partial_oritation_z*(A[0] - C[0]));
					dz = diff_gradient*(diff_partial_oritation_x*(A[1] - C[1]) + diff_partial_oritation_y*(C[0] - A[0]));
					gradient[offset] += dx;
					gradient[offset + 1] += dy;
					gradient[offset + 2] += dz;

					offset = m_faces(i, 2) * 3;
					dx = diff_gradient*(diff_partial_oritation_y*(B[2] - A[2]) + diff_partial_oritation_z*(A[1] - B[1]));
					dy = diff_gradient*(diff_partial_oritation_x*(A[2] - B[2]) + diff_partial_oritation_z*(B[0] - A[0]));
					dz = diff_gradient*(diff_partial_oritation_x*(B[1] - A[1]) + diff_partial_oritation_y*(A[0] - B[0]));
					gradient[offset] += dx;
					gradient[offset + 1] += dy;
					gradient[offset + 2] += dz;
				}
			}
			
			return gradient;
			

		}

		// void NonFlipPlaneEnergy::compute_barrier_energy(const double* parameters,
		// 	double* cost) const {
		// 	double init_cost = *cost;
		// 	for (int i = 0; i < m_num_faces; ++i) {
		// 		//std::cout << m_faces.row(i) << std::endl;
		// 		int offset = m_faces(i, 0) * 3;
		// 		Base::Vec3d A(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		offset = m_faces(i, 1) * 3;
		// 		Base::Vec3d B(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		offset = m_faces(i, 2) * 3;
		// 		Base::Vec3d C(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		Base::Vec3d orientation = (B - A).cross(C - A);
				
		// 		double area = orientation.norm()*0.5;
		// 		Base::Vec3d N = orientation.normalized();
		// 		//if (m_face_plane_index[i] != -1 && N.dot(m_planes[m_face_plane_index[i]].m_plane_normal) <= cos(60.0*M_PI / 180.0))
		// 		//{
		// 		//	//std::cout << N << std::endl;
		// 		//	//std::cout << m_planes[m_face_plane_index[i]].m_plane_normal << std::endl;
		// 		//	*cost = std::numeric_limits<double>::infinity();
		// 		//	break;
		// 		//}
		// 		//else if (N.dot(m_initial_face_normals.row(i)) <= angle_cos_thres) {
		// 		if (N.dot(m_initial_face_normals.row(i)) <= angle_cos_thres) {
		// 			*cost = std::numeric_limits<double>::infinity();//Found face orientation change
		// 			std::cout << "Found face orientation change" << std::endl;
		// 			/*std::cout << N << std::endl;
		// 			std::cout << m_initial_face_normals.row(i) << std::endl; 
		// 			std::cout << "F: " << m_faces.row(i) << std::endl;
		// 			std::cout << "A: " << A << std::endl;
		// 			std::cout << "B: " << B << std::endl;
		// 			std::cout << "C: " << C << std::endl;*/
		// 			break;
		// 		}

				
		// 		double critical_area = m_initial_face_area[i] * gamma;
		// 		if (area < critical_area) {
		// 			if (area > 0.0) {
		// 				double coeffA = 1 / pow(critical_area, 3);
		// 				double coeffB = -3 / pow(critical_area, 2);
		// 				double coeffC = 3 / critical_area;

		// 				double area2 = area*area;
		// 				double area3 = area2 * area;
		// 				*cost += barrier_weight * (1.0 / (coeffA*area3 + coeffB*area2 + coeffC*area) - 1.0);
		// 			}
		// 			else {
		// 				*cost = std::numeric_limits<double>::infinity();//Found degenerated face
		// 				std::cout << "Found degenerated face" << std::endl;
		// 				break;
		// 			}
					
		// 		}

		// 		/*if (std::isnan(*cost)) {
		// 			std::cout << "NAN !!!" << std::endl;
		// 			std::cout << "A: " << A << std::endl;
		// 			std::cout << "B: " << B << std::endl;
		// 			std::cout << "C: " << C << std::endl;
		// 		}*/
		// 	}
		// 	std::cout << "Barrier energy: " << *cost - init_cost << std::endl;
		// }

		// void NonFlipPlaneEnergy::compute_barrier_gradient(const double* parameters,
		// 	double* gradient) const {
		// 	Eigen::VectorXd init_grad(NumParameters());
		// 	memcpy(init_grad.data(), gradient, sizeof(double)*NumParameters());
		// 	for (int i = 0; i < m_num_faces; ++i) {
		// 		int offset = m_faces(i, 0) * 3;
		// 		Base::Vec3d A(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		offset = m_faces(i, 1) * 3;
		// 		Base::Vec3d B(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		offset = m_faces(i, 2) * 3;
		// 		Base::Vec3d C(parameters[offset], parameters[offset + 1], parameters[offset + 2]);
		// 		Base::Vec3d orientation = (B - A).cross(C - A);
				
		// 		double area = orientation.norm()*0.5;
		// 		double critical_area = m_initial_face_area[i] * gamma;
		// 		if (area < critical_area) {//line search gurantees we're always in feasible region when computing gradient
		// 			double coeffA = 1 / pow(critical_area, 3);
		// 			double coeffB = -3 / pow(critical_area, 2);
		// 			double coeffC = 3 / critical_area;

		// 			double area2 = area*area;
		// 			double area3 = area2 * area;

		// 			double area_gradient = -barrier_weight*(3 * coeffA*area2 + 2 * coeffB*area + coeffC) / pow(coeffA*area3 + coeffB*area2 + coeffC*area, 2.0);

		// 			double area_partial_oritation_x = orientation[0] / (4 * area);
		// 			double area_partial_oritation_y = orientation[1] / (4 * area);
		// 			double area_partial_oritation_z = orientation[2] / (4 * area);

		// 			double dx, dy, dz;
		// 			offset = m_faces(i, 0) * 3;
		// 			dx = area_gradient * (area_partial_oritation_y*(C[2] - B[2]) + area_partial_oritation_z*(B[1] - C[1]));
		// 			dy = area_gradient *(area_partial_oritation_x*(B[2] - C[2]) + area_partial_oritation_z*(C[0] - B[0]));
		// 			dz = area_gradient *(area_partial_oritation_x*(C[1] - B[1]) + area_partial_oritation_y*(B[0] - C[0]));
		// 			Base::Vec3 grad = project_gradient_onto_plane(m_faces(i, 0), Base::Vec3(dx, dy, dz));
		// 			gradient[offset] += grad[0];
		// 			gradient[offset + 1] += grad[1];
		// 			gradient[offset + 2] += grad[2];
		// 			/*gradient[offset] += area_gradient * (area_partial_oritation_y*(C[2] - B[2]) + area_partial_oritation_z*(B[1] - C[1]));
		// 			gradient[offset + 1] += area_gradient *(area_partial_oritation_x*(B[2] - C[2]) + area_partial_oritation_z*(C[0] - B[0]));
		// 			gradient[offset + 2] += area_gradient *(area_partial_oritation_x*(C[1] - B[1]) + area_partial_oritation_y*(B[0] - C[0]));*/

		// 			offset = m_faces(i, 1) * 3;
		// 			dx = area_gradient * (area_partial_oritation_y*(A[2] - C[2]) + area_partial_oritation_z*(C[1] - A[1]));
		// 			dy = area_gradient *(area_partial_oritation_x*(C[2] - A[2]) + area_partial_oritation_z*(A[0] - C[0]));
		// 			dz = area_gradient *(area_partial_oritation_x*(A[1] - C[1]) + area_partial_oritation_y*(C[0] - A[0]));
		// 			grad = project_gradient_onto_plane(m_faces(i, 1), Base::Vec3(dx, dy, dz));
		// 			gradient[offset] += grad[0];
		// 			gradient[offset + 1] += grad[1];
		// 			gradient[offset + 2] += grad[2];
		// 			/*gradient[offset] += area_gradient * (area_partial_oritation_y*(A[2] - C[2]) + area_partial_oritation_z*(C[1] - A[1]));
		// 			gradient[offset + 1] += area_gradient *(area_partial_oritation_x*(C[2] - A[2]) + area_partial_oritation_z*(A[0] - C[0]));
		// 			gradient[offset + 2] += area_gradient *(area_partial_oritation_x*(A[1] - C[1]) + area_partial_oritation_y*(C[0] - A[0]));*/

		// 			offset = m_faces(i, 2) * 3;
		// 			dx = area_gradient * (area_partial_oritation_y*(B[2] - A[2]) + area_partial_oritation_z*(A[1] - B[1]));
		// 			dy = area_gradient *(area_partial_oritation_x*(A[2] - B[2]) + area_partial_oritation_z*(B[0] - A[0]));
		// 			dz = area_gradient *(area_partial_oritation_x*(B[1] - A[1]) + area_partial_oritation_y*(A[0] - B[0]));
		// 			grad = project_gradient_onto_plane(m_faces(i, 2), Base::Vec3(dx, dy, dz));
		// 			gradient[offset] += grad[0];
		// 			gradient[offset + 1] += grad[1];
		// 			gradient[offset + 2] += grad[2];
		// 			/*gradient[offset] += area_gradient * (area_partial_oritation_y*(B[2] - A[2]) + area_partial_oritation_z*(A[1] - B[1]));
		// 			gradient[offset + 1] += area_gradient *(area_partial_oritation_x*(A[2] - B[2]) + area_partial_oritation_z*(B[0] - A[0]));
		// 			gradient[offset + 2] += area_gradient *(area_partial_oritation_x*(B[1] - A[1]) + area_partial_oritation_y*(A[0] - B[0]));*/  
		// 		}
				
		// 	}
			
		// 	//Eigen::VectorXd func_grad(NumParameters());
		// 	//memcpy(func_grad.data(), gradient, sizeof(double)*NumParameters());
		// 	//func_grad -= init_grad;

		// 	////Eigen::VectorXd numeric_grad = compute_numeric_gradient(parameters);

		// 	////std::cout <<"Func: " << func_grad.squaredNorm()<<", Numeric: " << numeric_grad.squaredNorm() <<", Diff: " << (numeric_grad - func_grad).squaredNorm() << std::endl;
		// 	//std::cout << "Barrier grad: " << func_grad.squaredNorm() << std::endl;
		// 	//std::cout << "Func: " << func_grad.transpose() << "\nNumeric: " << numeric_grad.transpose() << std::endl;
		// }

		Eigen::VectorXd NonFlipPlaneEnergy::compute_numeric_gradient(const double* parameters, std::function<void(const double*, double*)> func) const{
			
			Eigen::VectorXd grad(NumParameters());
			double* variables = new double[NumParameters()];
			memcpy(variables, parameters, sizeof(double)*NumParameters());
			double delta = 1e-10;
			double cur_cost = 0;
			
			//compute_barrier_energy(variables, &cur_cost);
			func(variables, &cur_cost);
			if (std::isinf(cur_cost)) {
				std::cout << "Cur cost not in feasible region" << std::endl;
			}
			for (int i = 0; i < NumParameters(); ++i) {
				double delta_cost = 0;
				variables[i] += delta;
				//compute_barrier_energy(variables, &delta_cost);
				func(variables, &delta_cost);
				variables[i] -= delta;
				/*if (std::isinf(delta_cost)) {
					std::cout << " Delta cost not in feasible region" << std::endl;
				}*/
				grad[i] = (delta_cost - cur_cost) / delta;
				
			}
			return grad;
		}

		bool NonFlipPlaneEnergy::Evaluate(const double* parameters,
			double* cost,
			double* gradient) const {
			*cost = 0;//reset cost energy
			*cost += compute_quadric_energy(parameters);
			
			*cost += compute_normal_energy(parameters);

			*cost += compute_distance_energy(parameters);

			if (gradient != nullptr) {
				Eigen::VectorXd grad_vector = compute_quadric_gradient(parameters);
				
				grad_vector += compute_normal_gradient(parameters);

				grad_vector += compute_distance_gradient(parameters);
			      
				memcpy(gradient, grad_vector.data(), sizeof(double)*NumParameters());
			}

			return true;
		}

		double NonFlipPlaneEnergy::compute_energy(Eigen::MatrixXd& x) {
			double cost = 0;
			cost += compute_quadric_energy(x.data());
			cost += compute_normal_energy(x.data());
			
			return cost;
		}

		void NonFlipPlaneEnergy::compute_gradient(Eigen::MatrixXd& x, Eigen::MatrixXd& grad) const {
			grad.setZero();
			grad += compute_quadric_gradient(x.data());
			grad += compute_normal_gradient(x.data());
			//compute_barrier_gradient(x.data(), grad.data());
		}

		int NonFlipPlaneEnergy::NumParameters() const {
			return 3 * m_num_vertices;
		}
	}
}