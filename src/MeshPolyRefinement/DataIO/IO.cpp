#include "IO.h"
#include "Repair.h"

#include <igl/readOBJ.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/parallel_for.h>
#include <igl/write_triangle_mesh.h>
#include <tinyply.h>


#include <unordered_set>
#include <queue>
#include <fstream>

namespace MeshPolyRefinement {
	namespace IO {
		template <int N>
		class FaceNeighborVertex {
		private:
			const Base::IndexMatrix& m_faces;
			const Base::IndexMatrix& m_TT;//see libigl <triangle_triangle_adjacency.h>
		public:
			FaceNeighborVertex(const Base::IndexMatrix& F, const Base::IndexMatrix& FF) :
				m_faces(F), m_TT(FF) {

			}

			std::vector<int> operator()(int face_index) {
				std::unordered_set<int> visited_faces, visited_vertices;
				std::queue<std::pair<int, int>> nodes;
				nodes.push(std::make_pair(face_index, 0));
				while (!nodes.empty()) {
					int cur_index = nodes.front().first;
					int cur_depth = nodes.front().second;
					nodes.pop();
					//insert vertices
					for (int vi : m_faces.row(cur_index)) {
						visited_vertices.insert(vi);
					}
					//insert neighbors
					if (cur_depth < N) {
						for (int ni : m_TT.row(cur_index)) {
							if (ni >= 0 && visited_faces.find(ni) == visited_faces.end()) {
								visited_faces.insert(ni);
								nodes.push(std::make_pair(ni, cur_depth + 1));
							}
						}
					}
				}
				std::vector<int> result(visited_vertices.size());
				std::copy(visited_vertices.begin(), visited_vertices.end(), result.begin());
				return result;
			}
		};

		

		struct memory_buffer : public std::streambuf
		{
		    char * p_start {nullptr};
		    char * p_end {nullptr};
		    size_t size;
		
		    memory_buffer(char const * first_elem, size_t size)
		        : p_start(const_cast<char*>(first_elem)), p_end(p_start + size), size(size)
		    {
		        setg(p_start, p_start, p_end);
		    }
		
		    pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
		    {
		        if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
		        else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
		        return gptr() - p_start;
		    }
		
		    pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
		    {
		        return seekoff(pos, std::ios_base::beg, which);
		    }
		};
		
		struct memory_stream : virtual memory_buffer, public std::istream
		{
		    memory_stream(char const * first_elem, size_t size)
		        : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf*>(this)) {}
		};

		inline std::vector<uint8_t> read_file_binary(const std::string & pathToFile)
		{
		    std::ifstream file(pathToFile, std::ios::binary);
		    std::vector<uint8_t> fileBufferBytes;

		    if (file.is_open())
		    {
		        file.seekg(0, std::ios::end);
		        size_t sizeBytes = file.tellg();
		        file.seekg(0, std::ios::beg);
		        fileBufferBytes.resize(sizeBytes);
		        if (file.read((char*)fileBufferBytes.data(), sizeBytes)) return fileBufferBytes;
		    }
		    else throw std::runtime_error("could not open binary ifstream to path " + pathToFile);
		    return fileBufferBytes;
		}

		Base::Scalar  compute_planar_score(Base::AttributeMatrix V) {
			Base::AttributeMatrix centered = V.rowwise() - V.colwise().mean();
			Base::AttributeMatrix cov = centered.transpose()*centered;
			Eigen::Vector<Base::Scalar, -1> eigen_values = cov.eigenvalues().real();
			return 1 - 3 * eigen_values.minCoeff() / eigen_values.sum();
		}

		void compute_face_planar_score(Base::TriMesh& mesh) {
			using key_type = std::size_t;
			using value_type = std::size_t;
			
			mesh.m_face_planar_score.resize(mesh.m_faces.rows());
			FaceNeighborVertex<5> neighbor_query(mesh.m_faces, mesh.m_ff_adjacency);//find n-ring neighbors
			int valid_face_number = 0;
			auto compute = [&](int face_index) {
				//m_seed_order[face_index] = face_index;
				Base::AttributeMatrix neighbors = mesh.m_vertices(neighbor_query(face_index), Eigen::all);
				mesh.m_face_planar_score(face_index) = std::pow(compute_planar_score(neighbors), 10);//use std::pow to equalize the score evalue range
			};
			igl::parallel_for(mesh.m_faces.rows(), compute);

			
		}
		
		void build_mesh(Base::TriMesh& mesh) {
			merge_close_vertex(mesh.m_vertices, mesh.m_vertex_colors, mesh.m_faces);

			remove_identical_index_faces(mesh.m_faces);

			split_non_manifold_vertex_with_property(mesh.m_vertices, mesh.m_vertex_colors, mesh.m_faces);

			//remove_identical_index_faces(mesh.m_faces);

			mesh.m_face_plane_index = Eigen::Vector<int, -1>::Constant(mesh.m_faces.rows(), -1);
//			std::cout << "File loaded. Constructing topology..." << std::endl;
			igl::triangle_triangle_adjacency(mesh.m_faces, mesh.m_ff_adjacency);

			std::vector<std::vector<int>> VFi;
			igl::vertex_triangle_adjacency(mesh.m_vertices.rows(), mesh.m_faces, mesh.m_vf_adjacency, VFi);

			Eigen::MatrixXd N;

			igl::per_face_normals(mesh.m_vertices, mesh.m_faces, N);

			mesh.m_face_normals = std::move(std::move(N));
//			std::cout << "Computing planar score..." << std::endl;
			compute_face_planar_score(mesh);
			//std::cout << "Test face semantic labels" << std::endl;
			if (mesh.m_vertex_colors.rows() == 0) {
				mesh.m_face_labels = Base::FaceSemanticVector::Constant(mesh.m_faces.rows(), Base::FaceSemanticLabel::UNSET);
			}
			else {
				//set color vector(Due to precision, we cannot use hash or equal function), see https://github.com/MarcWong/UDD
				Eigen::Matrix<Base::Scalar, 6, 3, Eigen::RowMajor> color_vector;
				std::vector<Base::FaceSemanticLabel> labels(6);
				mesh.m_face_labels.resize(mesh.m_faces.rows());

				color_vector.row(0) = Eigen::RowVector<Base::Scalar, 3>(156.0, 102.0, 102.0);
				labels[0] = Base::FaceSemanticLabel::BUILDING;
				color_vector.row(1) = Eigen::RowVector<Base::Scalar, 3>(128.0, 64.0, 128.0);
				labels[1] = Base::FaceSemanticLabel::ROAD;
				color_vector.row(2) = Eigen::RowVector<Base::Scalar, 3>(107.0, 142.0, 35.0);
				labels[2] = Base::FaceSemanticLabel::VEGETATION;
				color_vector.row(3) = Eigen::RowVector<Base::Scalar, 3>(0.0, 0.0, 142.0);
				labels[3] = Base::FaceSemanticLabel::VEHICLE;
				color_vector.row(4) = Eigen::RowVector<Base::Scalar, 3>(70.0, 70.0, 70.0);
				labels[4] = Base::FaceSemanticLabel::ROOF;
				color_vector.row(5) = Eigen::RowVector<Base::Scalar, 3>(0.0, 0.0, 0.0);
				labels[5] = Base::FaceSemanticLabel::OTHER;
				color_vector /= 255.0;
				//std::cout << color_vector << std::endl;

				Eigen::Vector<Base::FaceSemanticLabel, -1> vertex_labels(mesh.m_vertices.rows());
				auto color2label = [&](int vertex_index) {
					auto diff = color_vector.rowwise() - mesh.m_vertex_colors.row(vertex_index);
					int idx;
					diff.rowwise().squaredNorm().minCoeff(&idx);
					vertex_labels(vertex_index) = labels[idx];

				};
				igl::parallel_for(mesh.m_vertices.rows(), color2label);
				//std::cout << "label: " << vertex_labels.minCoeff() << std::endl;
				auto vertex2face = [&](int face_index) {
					Base::FaceSemanticLabel l0 = vertex_labels[mesh.m_faces(face_index, 0)];
					Base::FaceSemanticLabel l1 = vertex_labels[mesh.m_faces(face_index, 1)];
					Base::FaceSemanticLabel l2 = vertex_labels[mesh.m_faces(face_index, 2)];
					if (l0 == l1 && l1 == l2) {
						mesh.m_face_labels(face_index) = l0;
					} else {
						mesh.m_face_labels(face_index) = Base::FaceSemanticLabel::MULTILABELED;
					}
				};

				igl::parallel_for(mesh.m_faces.rows(), vertex2face);
				//mesh.m_vertex_colors.resize(0, 0);
			}
		}

		bool read_mesh_from_memory(const std::vector<double> &vertices,
								   const std::vector<std::size_t> &faces,
								   Base::TriMesh &mesh) {
			if (vertices.size() % 3 != 0 || faces.size() % 3 != 0) {
				return false;
			}

			Base::AttributeMatrix eigen_vertices(vertices.size() / 3, 3);
			for (int i = 0; i < vertices.size(); i += 3) {
				for (int j = 0; j < 3; j++) {
					eigen_vertices(i / 3, j) = vertices[i + j];
				}
			}

			Base::IndexMatrix eigen_faces(faces.size() / 3, 3);
			for (int f_i = 0; f_i < faces.size(); f_i += 3) {
				for (int j = 0; j < 3; j++) {
					eigen_faces(f_i / 3, j) = faces[f_i + j];
				}
			}

			mesh.m_vertices = eigen_vertices.cast<Base::Scalar>();
			mesh.m_faces = eigen_faces;
			IO::build_mesh(mesh);
			return true;
		}

		bool read_mesh_from_ply(const std::string &file_name, Base::TriMesh &mesh) {
			using namespace tinyply;
			struct float2 {
				float x, y;
			};
			struct float3 {
				float x, y, z;
			};
			struct double3 {
				double x, y, z;
			};
			struct int3 {
				int32_t x, y, z;
			};
			struct uint3 {
				uint32_t x, y, z;
			};
			struct uint4 {
				uint32_t x, y, z, w;
			};
			struct uchar3 {
				uint8_t r, g, b;

				uchar3() {
					r = g = b = 0;
				}

				uchar3(uint8_t r_, uint8_t g_, uint8_t b_)
						: r(r_), g(g_), b(b_) {}
			};

			

			std::unique_ptr<std::istream> file_stream;
    		std::vector<uint8_t> byte_buffer;

			try{
				byte_buffer = read_file_binary(file_name);
            	file_stream.reset(new memory_stream((char*)byte_buffer.data(), byte_buffer.size()));
				if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + file_name);
				file_stream->seekg(0, std::ios::end);
        		const float size_mb = file_stream->tellg() * float(1e-6);
        		file_stream->seekg(0, std::ios::beg);

        		PlyFile file;
        		file.parse_header(*file_stream);

				std::shared_ptr<PlyData> vertices, colors,  faces;
				try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

				
				try{colors = file.request_properties_from_element("vertex",{"red","green","blue"});}
				catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

				try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
				catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

				file.read(*file_stream);
				//read vertices
				if (vertices->t == tinyply::Type::FLOAT32){
					Eigen::Matrix<float,-1,-1,Eigen::RowMajor> eigen_vertices(vertices->count,3);
					memcpy(eigen_vertices.data(),vertices->buffer.get(),vertices->buffer.size_bytes());
					mesh.m_vertices=eigen_vertices.cast<Base::Scalar>();
				}
				else if(vertices->t == tinyply::Type::FLOAT64){
					Eigen::Matrix<double,-1,-1,Eigen::RowMajor> eigen_vertices(vertices->count,3);
					memcpy(eigen_vertices.data(),vertices->buffer.get(),vertices->buffer.size_bytes());
					mesh.m_vertices=eigen_vertices.cast<Base::Scalar>();
				}
				//read vertex colors if exists
				if(colors.get() != nullptr && colors->count > 0 && colors->t == tinyply::Type::UINT8){
					std::vector<uchar3> vertex_colors(colors->count);
					memcpy(vertex_colors.data(),colors->buffer.get(),colors->buffer.size_bytes());
					mesh.m_vertex_colors.resize(colors->count,3);
					int r = 0;
					for(const uchar3& c : vertex_colors){
						mesh.m_vertex_colors.row(r++) = Eigen::RowVector3d((double)c.r/255.0,(double)c.g/255.0,(double)c.b/255.0);
					}
				}
				//read triangles
				mesh.m_faces.resize(faces->count,3);
				memcpy(mesh.m_faces.data(),faces->buffer.get(),faces->buffer.size_bytes());
			}
			catch (const std::exception & e)
    		{
    		    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
				return false;
    		}
			if(mesh.m_vertices.rows() == 0 || mesh.m_faces.rows() == 0)
				return false;
			build_mesh(mesh);
			return true;
		}
		
		bool read_mesh_from_obj(const std::string& file_name, Base::TriMesh& mesh) {
			Base::AttributeMatrix V;
			Base::IndexMatrix F;
			igl::readOBJ(file_name, V, F); //I don't know why this function returns false even if V and F are loaded correctly.
			
			if (V.cols() == 6) {
				mesh.m_vertices = std::move(V.leftCols(3));
				mesh.m_vertex_colors = std::move(V.rightCols(3));
			}
			else {
				 mesh.m_vertices = std::move(V);
			}
			
			mesh.m_faces = std::move(F);
			
			build_mesh(mesh);
			return true;
		}

		bool save_mesh_clusters(const std::string& file_name, const Base::TriMesh& mesh, const std::vector<int>& cluster_ids, int num_clusters) {
			Eigen::Matrix<uint8_t,-1,-1,Eigen::RowMajor> cluster_color(num_clusters, 3);
			for (int i = 0; i < num_clusters; ++i) {
				auto color_double = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				cluster_color.row(i) = (255.0*color_double).cast<uint8_t>();
			}
			
			struct float2 { float x, y; };
			struct float3 { float x, y, z; };
			struct double3 { double x, y, z; };
			struct int3 {int32_t x, y, z;};
			struct uint3 { uint32_t x, y, z; };
			struct uint4 { uint32_t x, y, z, w; };
			struct uchar3 {
				uint8_t r, g, b;
				uchar3() {
					r = g = b = 0;
				}
				uchar3(uint8_t r_, uint8_t g_, uint8_t b_)
					:r(r_), g(g_), b(b_) {}
			};

			struct geometry
			{
				std::vector<float3> vertices;
				std::vector<int3> triangles;
				std::vector<uchar3> triangle_colors;
			};

			geometry geom;
			for (const auto&v : mesh.m_vertices.rowwise()) {
				geom.vertices.push_back({ (float)v[0],(float)v[1],(float)v[2] });
			}

			for (const auto& f : mesh.m_faces.rowwise()) {
				geom.triangles.push_back({ f[0],f[1],f[2] });
			}
			geom.triangle_colors.resize(mesh.m_faces.rows());

			for (int fi = 0; fi < mesh.m_faces.rows(); ++fi) {
				const auto& cluster_c = cluster_color.row(cluster_ids[fi]);
				geom.triangle_colors[fi] = { cluster_c[0],cluster_c[1],cluster_c[2] };
			}
			std::filebuf fb_binary;
			fb_binary.open(file_name, std::ios::out | std::ios::binary);
			std::ostream outstream_binary(&fb_binary);
			if (outstream_binary.fail()) throw std::runtime_error("failed to open " + file_name);

			tinyply::PlyFile geom_file;
			geom_file.add_properties_to_element("vertex", { "x", "y", "z" },
				tinyply::Type::FLOAT32, geom.vertices.size(), reinterpret_cast<uint8_t*>(geom.vertices.data()), tinyply::Type::INVALID, 0);

			geom_file.add_properties_to_element("face", { "vertex_indices" },
				tinyply::Type::INT32, geom.triangles.size(), reinterpret_cast<uint8_t*>(geom.triangles.data()), tinyply::Type::UINT8, 3);

			geom_file.add_properties_to_element("face", { "red", "green", "blue" },
				tinyply::Type::UINT8, geom.triangle_colors.size(), reinterpret_cast<uint8_t*>(geom.triangle_colors.data()), tinyply::Type::INVALID, 0);

			geom_file.write(outstream_binary, true);
			return true;
		}

		bool save_mesh_plane_segments(const std::string& file_name, const Base::TriMesh& mesh) {
			struct float2 { float x, y; };
			struct float3 { float x, y, z; };
			struct double3 { double x, y, z; };
			struct int3 {int32_t x,y,z;};
			struct uint3 { uint32_t x, y, z; };
			struct uint4 { uint32_t x, y, z, w; };
			struct uchar3 { 
				uint8_t r, g, b; 
				uchar3() {
					r = g = b = 0;
				}
				uchar3(uint8_t r_, uint8_t g_, uint8_t b_)
					:r(r_),g(g_),b(b_){}
			};

			struct geometry
			{
				std::vector<float3> vertices;
				std::vector<int3> triangles;
				std::vector<uchar3> triangle_colors;
			};

			geometry geom;
			for (const auto&v : mesh.m_vertices.rowwise()) {
				geom.vertices.push_back({ (float)v[0],(float)v[1],(float)v[2] });
			}

			for (const auto& f : mesh.m_faces.rowwise()) {
				geom.triangles.push_back({ f[0],f[1],f[2] });
			}

			geom.triangle_colors.resize(mesh.m_faces.rows());

			for (const auto&g : mesh.m_plane_groups) {
				Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
				Eigen::RowVector<uint8_t, 3> color_uchar = (255.0*color).cast<uint8_t>();
				for (std::size_t fi : g.m_indices) {
					geom.triangle_colors[fi] = { color_uchar[0],color_uchar[1],color_uchar[2] };
				}
			}
			std::filebuf fb_binary;
			fb_binary.open(file_name, std::ios::out | std::ios::binary);
			std::ostream outstream_binary(&fb_binary);

            if (outstream_binary.fail()) throw std::runtime_error("failed to open " + file_name);

            tinyply::PlyFile geom_file;
			geom_file.add_properties_to_element("vertex", { "x", "y", "z" },
				tinyply::Type::FLOAT32, geom.vertices.size(), reinterpret_cast<uint8_t*>(geom.vertices.data()), tinyply::Type::INVALID, 0);

			geom_file.add_properties_to_element("face", { "vertex_indices" },
				tinyply::Type::INT32, geom.triangles.size(), reinterpret_cast<uint8_t*>(geom.triangles.data()), tinyply::Type::UINT8, 3);

			geom_file.add_properties_to_element("face", { "red", "green", "blue" },
				tinyply::Type::UINT8, geom.triangle_colors.size(), reinterpret_cast<uint8_t*>(geom.triangle_colors.data()), tinyply::Type::INVALID, 0);

            geom_file.write(outstream_binary, true);
            return true;
        }

		bool save_mesh_plane_parameters(const std::string& file_name, const Base::TriMesh& mesh) {
			std::ofstream fout_binary;
			fout_binary.open(file_name, std::ios::out | std::ios::binary);
			if (!fout_binary.is_open()) {
				std::cout << "Error, cannot open file " << file_name << std::endl;
				return false;
			}
			/*first line: vertex number, face number, plane number*/
			Eigen::Index v_n = mesh.m_vertices.rows(), f_n = mesh.m_faces.rows(), p_n = mesh.m_plane_groups.size();
			fout_binary.write((char*)&v_n, sizeof(Eigen::Index));
			fout_binary.write((char*)&f_n, sizeof(Eigen::Index));
			fout_binary.write((char*)&p_n, sizeof(Eigen::Index));

			//write vertices 
			fout_binary.write((char*)mesh.m_vertices.data(), v_n * 3 * sizeof(decltype(mesh.m_vertices)::Scalar));

			//write faces
			fout_binary.write((char*)mesh.m_faces.data(), f_n * 3 * sizeof(decltype(mesh.m_faces)::Scalar));

			//write face plane index
			fout_binary.write((char*)mesh.m_face_plane_index.data(), f_n * 1 * sizeof(decltype(mesh.m_face_plane_index)::Scalar));

			/*write plane parameters
			nx,ny,nz,cx,cy,cz
			*/
			Base::AttributeMatrix plane_params(p_n, 6);
			for (int i = 0; i < mesh.m_plane_groups.size();++i) {
				plane_params.row(i).head(3) = mesh.m_plane_groups[i].m_plane_normal;
				plane_params.row(i).tail(3) = mesh.m_plane_groups[i].m_plane_center;
			}
			fout_binary.write((char*)plane_params.data(), p_n * 6 * sizeof(Base::AttributeMatrix::Scalar));
			fout_binary.close();
			return true;
		}

		bool read_mesh_plane_parameters(const std::string& file_name, Base::TriMesh& mesh) {
			std::ifstream fin_binary;
			fin_binary.open(file_name, std::ios::in | std::ios::binary);
			if (!fin_binary.is_open()) {
				std::cout << "Error, cannot open file " << file_name << std::endl;
				return false;
			}
			Eigen::Index v_n, f_n, p_n;
			fin_binary.read((char*)&v_n, sizeof(Eigen::Index));
			fin_binary.read((char*)&f_n, sizeof(Eigen::Index));
			fin_binary.read((char*)&p_n, sizeof(Eigen::Index));

			//read vertices
			mesh.m_vertices.resize(v_n, 3);
			fin_binary.read((char*)mesh.m_vertices.data(), v_n * 3 * sizeof(decltype(mesh.m_vertices)::Scalar));

			//read faces
			mesh.m_faces.resize(f_n, 3);
			fin_binary.read((char*)mesh.m_faces.data(), f_n * 3 * sizeof(decltype(mesh.m_faces)::Scalar));

			//read face plane index
			mesh.m_face_plane_index.resize(f_n);
			fin_binary.read((char*)mesh.m_face_plane_index.data(), f_n * 1 * sizeof(decltype(mesh.m_face_plane_index)::Scalar));

			//read plane parameter
			Base::AttributeMatrix plane_params(p_n, 6);
			fin_binary.read((char*)plane_params.data(), p_n * 6 * sizeof(Base::AttributeMatrix::Scalar));
			fin_binary.close();
			mesh.m_plane_groups.resize(p_n);
			for (Eigen::Index i = 0; i < p_n; ++i) {
				mesh.m_plane_groups[i].m_plane_normal = plane_params.row(i).head(3);
				mesh.m_plane_groups[i].m_plane_center = plane_params.row(i).tail(3);
			}
			return true;
		}

		bool save_mesh(const std::string& file_name, const Base::TriMesh& mesh) {
			return igl::write_triangle_mesh(file_name, mesh.m_vertices, mesh.m_faces);
		}

		bool save_mesh(const std::string& file_name, const Eigen::MatrixXd & V, const Eigen::MatrixXi& F){
			return igl::write_triangle_mesh(file_name,V,F);
		}
	}
}