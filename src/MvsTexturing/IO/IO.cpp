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

        namespace MVE {
            using namespace mve;
            using namespace mve::geom;

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
                    if (dotpos == std::string::npos) {
                        prefix = filename;
                    } else {
                        prefix = filename.substr(0, dotpos);
                    }
                }

                Obj::ObjModel::save(obj_model, prefix);
            }
        }
    }
}