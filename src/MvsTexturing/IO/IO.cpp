//
// Created by Storm Phoenix on 2021/10/16.
//

#include <fstream>
#include <iostream>

#include <mve/mesh.h>
#include <mve/mesh_io_ply.h>
#include <mve/mesh_io_obj.h>
#include <mve/depthmap.h>
#include <util/file_system.h>
#include <util/exception.h>
#include <util/tokenizer.h>

#include "IO/ObjModel.h"
#include "Base/TextureAtlas.h"
#include "Utils/MeshAdapter.h"

#define TINYPLY_IMPLEMENTATION

#include <tinyply.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <igl/write_triangle_mesh.h>

namespace MvsTexturing {
    namespace IO {
        namespace {
            struct ObjVertex {
                unsigned int vertex_id;
                unsigned int texcoord_id;
                unsigned int normal_id;

                ObjVertex(void);

                bool operator<(ObjVertex const &other) const;
            };

            inline
            ObjVertex::ObjVertex(void)
                    : vertex_id(0), texcoord_id(0), normal_id(0) {
            }

            inline bool
            ObjVertex::operator<(ObjVertex const &other) const {
                return std::lexicographical_compare(&vertex_id, &normal_id + 1,
                                                    &other.vertex_id, &other.normal_id + 1);
            }
        }

        void load_mtl_file(std::string const &filename,
                           std::map<std::string, std::string> *result) {
            if (filename.empty())
                throw std::invalid_argument("No filename given");

            /* Open file. */
            std::ifstream input(filename.c_str(), std::ios::binary);
            if (!input.good())
                throw util::FileException(filename, std::strerror(errno));

            std::string material_name;
            std::string buffer;
            while (input.good()) {
                std::getline(input, buffer);
                util::string::clip_newlines(&buffer);
                util::string::clip_whitespaces(&buffer);

                if (buffer.empty())
                    continue;
                if (input.eof())
                    break;

                if (buffer[0] == '#') {
                    std::cout << "MTL Loader: " << buffer << std::endl;
                    continue;
                }

                util::Tokenizer line;
                line.split(buffer);

                if (line[0] == "newmtl") {
                    if (line.size() != 2)
                        throw util::Exception("Invalid material specification");

                    material_name = line[1];
                } else if (line[0] == "map_Kd") {
                    if (line.size() != 2)
                        throw util::Exception("Invalid diffuse map specification");
                    if (material_name.empty())
                        throw util::Exception("Unbound material property");

                    std::string path = util::fs::join_path
                            (util::fs::dirname(filename), line[1]);
                    result->insert(std::make_pair(material_name, path));
                    material_name.clear();
                } else {
                    std::cout << "MTL Loader: Skipping unimplemented material property "
                              << line[0] << std::endl;
                }
            }

            /* Close the file stream. */
            input.close();
        }

        void load_mesh_from_obj(std::string const &filename,
                                std::vector<math::Vec3f> &V, std::vector<math::Vec3f> &N,
                                std::vector<math::Vec2f> &T, std::vector<std::size_t> &F,
                                std::vector<std::size_t> &FN, std::vector<std::size_t> &FT,
                                std::vector<std::string> &face_materials,
                                std::map<std::string, std::string> &material_map) {
            // V, N, T, Fv, Ft, Fn, Fm
            std::vector<mve::geom::ObjModelPart> obj_model_parts;
            /* Precondition checks. */
            if (filename.empty())
                throw std::invalid_argument("No filename given");

            /* Open file. */
            std::ifstream input(filename.c_str(), std::ios::binary);
            if (!input.good())
                throw util::FileException(filename, std::strerror(errno));

            typedef std::map<ObjVertex, unsigned int> VertexIndexMap;
            std::string material_name;
            std::string new_material_name;
            std::string buffer;
            while (input.good()) {
                std::getline(input, buffer);
                util::string::clip_newlines(&buffer);
                util::string::clip_whitespaces(&buffer);

                if (input.eof() || !new_material_name.empty()) {
                    if (!FT.empty() && FT.size() != F.size()) {
                        throw util::Exception("Invalid number of texture coords");
                    }
                    if (!FN.empty() && FN.size() != F.size()) {
                        throw util::Exception("Invalid number of vertex normals");
                    }
                    if (!face_materials.empty() && (face_materials.size() * 3) != F.size()) {
                        throw util::Exception("Invalid number of face materials");
                    }

                    material_name.swap(new_material_name);
                    new_material_name.clear();
                }

                if (buffer.empty())
                    continue;
                if (input.eof())
                    break;

                if (buffer[0] == '#') {
                    /* Print all comments to STDOUT and forget data. */
                    std::cout << "OBJ Loader: " << buffer << std::endl;
                    continue;
                }

                util::Tokenizer line;
                line.split(buffer);

                if (line[0] == "v") {
                    if (line.size() != 4 && line.size() != 5)
                        throw util::Exception("Invalid vertex coordinate specification");

                    math::Vec3f vertex(0.0f);
                    for (int i = 0; i < 3; ++i)
                        vertex[i] = util::string::convert<float>(line[1 + i]);

                    /* Convert homogeneous coordinates. */
                    if (line.size() == 5)
                        vertex /= util::string::convert<float>(line[4]);

                    V.push_back(vertex);
                } else if (line[0] == "vt") {
                    if (line.size() != 3 && line.size() != 4)
                        throw util::Exception("Invalid texture coords specification");

                    math::Vec2f texcoord(0.0f);
                    for (int i = 0; i < 2; ++i)
                        texcoord[i] = util::string::convert<float>(line[1 + i]);

                    /* Convert homogeneous coordinates. */
                    if (line.size() == 4)
                        texcoord /= util::string::convert<float>(line[3]);

                    /* Invert y coordinate */
                    texcoord[1] = 1.0f - texcoord[1];

                    T.push_back(texcoord);
                } else if (line[0] == "vn") {
                    if (line.size() != 4 && line.size() != 5)
                        util::Exception("Invalid vertex normal specification");

                    math::Vec3f normal(0.0f);
                    for (int i = 0; i < 3; ++i)
                        normal[i] = util::string::convert<float>(line[1 + i]);

                    /* Convert homogeneous coordinates. */
                    if (line.size() == 5)
                        normal /= util::string::convert<float>(line[4]);

                    N.push_back(normal);
                } else if (line[0] == "f") {
                    if (line.size() != 4)
                        throw util::Exception("Only triangles supported");

                    face_materials.push_back(material_name);
                    for (int i = 0; i < 3; ++i) {
                        util::Tokenizer tok;
                        tok.split(line[1 + i], '/');

                        if (tok.size() > 3)
                            throw util::Exception("Invalid face specification");

                        ObjVertex v;
                        v.vertex_id = util::string::convert<unsigned int>(tok[0]);
                        if (tok.size() >= 2 && !tok[1].empty())
                            v.texcoord_id = util::string::convert<unsigned int>(tok[1]);
                        if (tok.size() == 3 && !tok[2].empty())
                            v.normal_id = util::string::convert<unsigned int>(tok[2]);

                        if (v.vertex_id > V.size()
                            || v.texcoord_id > T.size()
                            || v.normal_id > N.size())
                            throw util::Exception("Invalid index in: " + buffer);

                        F.push_back(v.vertex_id - 1);
                        FN.push_back(v.normal_id - 1);
                        FT.push_back(v.texcoord_id - 1);
                    }
                } else if (line[0] == "usemtl") {
                    if (line.size() != 2)
                        throw util::Exception("Invalid usemtl specification");
                    new_material_name = line[1];
                } else if (line[0] == "mtllib") {
                    if (line.size() != 2)
                        throw util::Exception("Invalid material library specification");

                    std::string dir = util::fs::dirname(filename);
                    load_mtl_file(util::fs::join_path(dir, line[1]), &material_map);
                } else {
                    std::cout << "OBJ Loader: Skipping unsupported element: "
                              << line[0] << std::endl;
                }
            }

            /* Close the file stream. */
            input.close();
        }

        typedef double Scalar;
        typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
        typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

        bool load_mesh_from_obj(const std::string &filename, AttributeMatrix &V, AttributeMatrix &N,
                                AttributeMatrix &T, IndexMatrix &F, IndexMatrix &FN, IndexMatrix &FT,
                                std::vector<std::string> &face_materials,
                                std::map<std::string, std::string> &material_map) {
            std::vector<math::Vec3f> mesh_vertices;
            std::vector<math::Vec3f> mesh_normals;
            std::vector<math::Vec2f> mesh_texcoords;
            std::vector<std::size_t> mesh_faces;
            std::vector<std::size_t> mesh_normal_ids, mesh_texcoord_ids;

            MvsTexturing::IO::load_mesh_from_obj(filename, mesh_vertices, mesh_normals, mesh_texcoords,
                                                 mesh_faces, mesh_normal_ids, mesh_texcoord_ids,
                                                 face_materials, material_map);

            if (mesh_faces.size() % 3 != 0) {
                return false;
            }

            // Vertices
            AttributeMatrix eigen_vertices(mesh_vertices.size(), 3);
            for (int i = 0; i < mesh_vertices.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    eigen_vertices(i, j) = mesh_vertices[i][j];
                }
            }
            V = eigen_vertices.cast<Scalar>();

            // Normals
            AttributeMatrix eigen_normals(mesh_normals.size(), 3);
            for (int i = 0; i < mesh_normals.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    eigen_normals(i, j) = mesh_normals[i][j];
                }
            }
            N = eigen_normals.cast<Scalar>();

            // Texcoords
            AttributeMatrix eigen_texcoords(mesh_texcoords.size(), 2);
            for (int i = 0; i < mesh_texcoords.size(); i++) {
                for (int j = 0; j < 2; j++) {
                    eigen_texcoords(i, j) = mesh_texcoords[i][j];
                }
            }
            T = eigen_texcoords.cast<Scalar>();

            // Faces
            IndexMatrix eigen_faces(mesh_faces.size() / 3, 3);
            for (int f_i = 0; f_i < mesh_faces.size(); f_i += 3) {
                for (int j = 0; j < 3; j++) {
                    eigen_faces(f_i / 3, j) = mesh_faces[f_i + j];
                }
            }
            F = eigen_faces;

            // Face normals
            IndexMatrix eigen_face_normals(mesh_normal_ids.size() / 3, 3);
            for (int fn_i = 0; fn_i < mesh_normal_ids.size(); fn_i += 3) {
                for (int j = 0; j < 3; j++) {
                    eigen_face_normals(fn_i / 3, j) = mesh_normal_ids[fn_i + j];
                }
            }
            FN = eigen_face_normals;

            // Face texcoords
            IndexMatrix eigen_face_texcoords(mesh_texcoord_ids.size() / 3, 3);
            for (int ft_i = 0; ft_i < mesh_texcoord_ids.size(); ft_i += 3) {
                for (int j = 0; j < 3; j++) {
                    eigen_face_texcoords(ft_i / 3, j) = mesh_texcoord_ids[ft_i + j];
                }
            }
            FT = eigen_face_texcoords;

            return true;
        }

        struct memory_buffer : public std::streambuf {
            char *p_start{nullptr};
            char *p_end{nullptr};
            size_t size;

            memory_buffer(char const *first_elem, size_t size)
                    : p_start(const_cast<char *>(first_elem)), p_end(p_start + size), size(size) {
                setg(p_start, p_start, p_end);
            }

            pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override {
                if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
                else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
                return gptr() - p_start;
            }

            pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
                return seekoff(pos, std::ios_base::beg, which);
            }
        };

        struct memory_stream : virtual memory_buffer, public std::istream {
            memory_stream(char const *first_elem, size_t size)
                    : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf *>(this)) {}
        };

        inline std::vector<uint8_t> read_file_binary(const std::string &pathToFile) {
            std::ifstream file(pathToFile, std::ios::binary);
            std::vector<uint8_t> fileBufferBytes;

            if (file.is_open()) {
                file.seekg(0, std::ios::end);
                size_t sizeBytes = file.tellg();
                file.seekg(0, std::ios::beg);
                fileBufferBytes.resize(sizeBytes);
                if (file.read((char *) fileBufferBytes.data(), sizeBytes)) return fileBufferBytes;
            } else throw std::runtime_error("could not open binary ifstream to path " + pathToFile);
            return fileBufferBytes;
        }

        bool load_mesh_from_ply(const std::string &filename, AttributeMatrix &V, IndexMatrix &F) {
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

            try {
                byte_buffer = read_file_binary(filename);
                file_stream.reset(new memory_stream((char *) byte_buffer.data(), byte_buffer.size()));
                if (!file_stream || file_stream->fail()) {
                    throw std::runtime_error("file_stream failed to open " + filename);
                }
                file_stream->seekg(0, std::ios::end);
                const float size_mb = file_stream->tellg() * float(1e-6);
                file_stream->seekg(0, std::ios::beg);

                PlyFile file;
                file.parse_header(*file_stream);

                std::shared_ptr<PlyData> vertices, colors, faces;
                try { vertices = file.request_properties_from_element("vertex", {"x", "y", "z"}); }
                catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }


                try { colors = file.request_properties_from_element("vertex", {"red", "green", "blue"}); }
                catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

                try { faces = file.request_properties_from_element("face", {"vertex_indices"}, 3); }
                catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

                file.read(*file_stream);
                //read vertices
                if (vertices->t == tinyply::Type::FLOAT32) {
                    Eigen::Matrix<float, -1, -1, Eigen::RowMajor> eigen_vertices(vertices->count, 3);
                    memcpy(eigen_vertices.data(), vertices->buffer.get(), vertices->buffer.size_bytes());
                    V = eigen_vertices.cast<Scalar>();
                } else if (vertices->t == tinyply::Type::FLOAT64) {
                    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> eigen_vertices(vertices->count, 3);
                    memcpy(eigen_vertices.data(), vertices->buffer.get(), vertices->buffer.size_bytes());
                    V = eigen_vertices.cast<Scalar>();
                }
                //read vertex colors if exists
                if (colors.get() != nullptr && colors->count > 0 && colors->t == tinyply::Type::UINT8) {
                    /* TODO ignored
                    std::vector<uchar3> vertex_colors(colors->count);
                    memcpy(vertex_colors.data(), colors->buffer.get(), colors->buffer.size_bytes());
                    mesh.m_vertex_colors.resize(colors->count, 3);
                    int r = 0;
                    for (const uchar3 &c : vertex_colors) {
                        mesh.m_vertex_colors.row(r++) = Eigen::RowVector3d((double) c.r / 255.0, (double) c.g / 255.0,
                                                                           (double) c.b / 255.0);
                    }
                     */
                }
                //read triangles
                F.resize(faces->count, 3);
                memcpy(F.data(), faces->buffer.get(), faces->buffer.size_bytes());
            } catch (const std::exception &e) {
                std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
                return false;
            }
            if (V.rows() == 0 || F.rows() == 0) {
                return false;
            }
            return true;
        }

        bool save_ply_mesh(const std::string &file_name, const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
            return igl::write_triangle_mesh(file_name, V, F);
        }

        bool save_ply_mesh(const std::string &file_name, const Eigen::MatrixXd &V,
                           const Eigen::MatrixXi &F, const Eigen::MatrixXd &FC) {
            MeshPtr mesh_ptr = Utils::eigenMesh_to_mveMesh(V, F, FC);
            if (mesh_ptr != nullptr) {
                MVE::save_ply_mesh(file_name, mesh_ptr, false);
                return true;
            } else {
                return false;
            }
        }

        namespace MVE {
            mve::TriangleMesh::Ptr load_ply_mesh(const std::string &in_mesh) {
                return mve::geom::load_ply_mesh(in_mesh);
            }

            void save_obj_mesh(const std::string &filename, mve::TriangleMesh::ConstPtr mesh,
                               const std::vector<Base::TextureAtlas::Ptr> &texture_atlases) {
                mve::TriangleMesh::VertexList const &mesh_vertices = mesh->get_vertices();
                mve::TriangleMesh::NormalList const &mesh_normals = mesh->get_vertex_normals();
                mve::TriangleMesh::FaceList const &mesh_faces = mesh->get_faces();

                Obj::ObjModel obj_model;
                Obj::ObjModel::Vertices &vertices = obj_model.get_vertices();
                vertices.insert(vertices.begin(), mesh_vertices.begin(), mesh_vertices.end());
                Obj::ObjModel::Normals &normals = obj_model.get_normals();
                normals.insert(normals.begin(), mesh_normals.begin(), mesh_normals.end());
                Obj::ObjModel::TexCoords &texcoords = obj_model.get_texcoords();

                Obj::ObjModel::Groups &groups = obj_model.get_groups();
                Obj::MaterialLib &material_lib = obj_model.get_material_lib();

                for (Base::TextureAtlas::Ptr texture_atlas : texture_atlases) {
                    Obj::Material material;
                    const std::size_t n = material_lib.size();
                    material.name = std::string("material") + util::string::get_filled(n, 4);
                    material.diffuse_map = texture_atlas->get_image();
                    material_lib.push_back(material);

                    groups.push_back(Obj::ObjModel::Group());
                    Obj::ObjModel::Group &group = groups.back();
                    group.material_name = material.name;

                    Base::TextureAtlas::Faces const &atlas_faces = texture_atlas->get_faces();
                    Base::TextureAtlas::Texcoords const &atlas_texcoords = texture_atlas->get_texcoords();
                    Base::TextureAtlas::TexcoordIds const &atlas_texcoord_ids = texture_atlas->get_texcoord_ids();

                    std::size_t texcoord_id_offset = texcoords.size();

                    texcoords.insert(texcoords.end(), atlas_texcoords.begin(),
                                     atlas_texcoords.end());

                    for (std::size_t i = 0; i < atlas_faces.size(); ++i) {
                        std::size_t mesh_face_pos = atlas_faces[i] * 3;

                        std::size_t vertex_ids[] = {
                                mesh_faces[mesh_face_pos],
                                mesh_faces[mesh_face_pos + 1],
                                mesh_faces[mesh_face_pos + 2]
                        };
                        std::size_t *normal_ids = vertex_ids;

                        std::size_t texcoord_ids[] = {
                                texcoord_id_offset + atlas_texcoord_ids[i * 3],
                                texcoord_id_offset + atlas_texcoord_ids[i * 3 + 1],
                                texcoord_id_offset + atlas_texcoord_ids[i * 3 + 2]
                        };

                        group.faces.push_back(Obj::ObjModel::Face());
                        Obj::ObjModel::Face &face = group.faces.back();
                        std::copy(vertex_ids, vertex_ids + 3, face.vertex_ids);
                        std::copy(texcoord_ids, texcoord_ids + 3, face.texcoord_ids);
                        std::copy(normal_ids, normal_ids + 3, face.normal_ids);
                    }
                }
                //TODO remove unreferenced vertices/normals.

                std::string prefix = "";
                {
                    std::size_t dotpos = filename.find_last_of('.');
                    if (dotpos == std::string::npos || dotpos == 0) {
                        prefix = filename;
                    } else {
                        prefix = filename.substr(0, dotpos);
                    }
                }

                Obj::ObjModel::save(obj_model, prefix);
            }

            void save_ply_mesh(const std::string &filename, mve::TriangleMesh::ConstPtr mesh, bool binary_format) {
                mve::geom::SavePLYOptions options;
                options.format_binary = binary_format;
                options.write_face_colors = true;
                mve::geom::save_ply_mesh(mesh, filename, options);
            }
        }
    }
}