//
// Created by Storm Phoenix on 2021/10/16.
//

#include <fstream>
#include <iostream>

#include <mve/mesh.h>
#include <mve/mesh_io_ply.h>
#include <mve/depthmap.h>
#include <util/exception.h>
#include <util/tokenizer.h>

#include "IO/ObjModel.h"
#include "Base/TextureAtlas.h"

namespace MvsTexturing {
    namespace IO {
        namespace MVE {
            using namespace mve;
            using namespace mve::geom;

            mve::TriangleMesh::Ptr load_ply_mesh(const std::string &in_mesh) {
                return mve::geom::load_ply_mesh(in_mesh);
            }

            void save_obj_mesh(const std::string &filename, mve::TriangleMesh::Ptr mesh,
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

                Obj::ObjModel::save(obj_model, filename);
            }
        }
    }
}