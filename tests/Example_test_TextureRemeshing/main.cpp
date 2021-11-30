//
// Created by Storm Phoenix on 2021/10/22.
//
#include <iostream>
#include <queue>

#include <boost/program_options.hpp>

#include <mve/image.h>
#include <mve/image_io.h>
#include <mve/mesh_io_ply.h>

#define TINYPLY_IMPLEMENTATION

#include <Base/TriMesh.h>
#include <PlaneEstimation/RegionGrowing.h>
#include <PlaneEstimation/RegionExpand.h>
#include <DataIO/Repair.h>
#include <DataIO/IO.h>

#include <IO/IO.h>
#include <Base/TexturePatch.h>
#include <Base/TextureAtlas.h>
#include <Mapper/AtlasMapper.h>

#include <util/timer.h>
#include "RemeshingUtils.h"

#define PLANE_DENSITY 650
#define TEXTURE_PADDING 5

#include <common.h>

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> AttributeMatrix;
typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
typedef std::set<std::size_t> FaceGroup;
using Vec3 = Eigen::Matrix<double, 1, 3>;

void find_connected_face_group(const IndexMatrix &ff_adjacency, const std::set<std::size_t> &outlier_faces,
                               const int face_index, FaceGroup &face_group);

int main(int argc, char **argv) {
    namespace bpo = boost::program_options;
    bpo::options_description opts("Example test MeshRemeshing options");
    bpo::variables_map vm;

    opts.add_options()
            ("help", "produce help message")
            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
            ("output_mesh", bpo::value<std::string>()->default_value("res.ply"), "path of output simplified mesh")
            ("planar_score", bpo::value<double>()->default_value(0.3),
             "minimal planar score of face during plane estimation, default 0.8")
            ("angle", bpo::value<double>()->default_value(30.0),
             "maximal angle in degree that plane segment should accept face, default 15.0")
            ("ratio", bpo::value<double>()->default_value(20.0),
             "maximal distance ratio that plane segment should accept face, default 10.0")
            ("plane_size", bpo::value<int>()->default_value(10),
             "minimal number of facets in plane segment, default 10")
            ("write_intermedia", bpo::value<bool>()->default_value(false),
             "options for write intermedia files, default false")
            ("debug_mode", bpo::value<bool>()->default_value(false),
             "options for enter debug mode, default false");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        std::cout << "undefine options in command lines.\n";
        return 0;
    }

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();
    const bool write_intermedia = vm["write_intermedia"].as<bool>();
    const bool debug_mode = vm["debug_mode"].as<bool>();

    if (debug_mode) {
        spdlog::set_level(spdlog::level::debug);
    }

    AttributeMatrix mesh_vertices, mesh_normals, mesh_texcoords;
    IndexMatrix mesh_faces, mesh_normal_ids, mesh_texcoord_ids;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> _material_map;

    std::string input_extension = "";
    {
        std::size_t dotpos = in_mesh_path.find_last_of('.');
        if (dotpos != std::string::npos && dotpos != 0) {
            input_extension = in_mesh_path.substr(dotpos, in_mesh_path.size());
        }
    }

    std::string output_prefix = "";
    {
        std::size_t dotpos = out_mesh_path.find_last_of('.');
        if (dotpos == std::string::npos || dotpos == 0) {
            output_prefix = out_mesh_path;
        } else {
            output_prefix = out_mesh_path.substr(0, dotpos);
        }
    }

    LOG_INFO("###### TextureRemeshing ------ Load models");
    if (input_extension == ".ply") {
        MvsTexturing::IO::load_mesh_from_ply(in_mesh_path, mesh_vertices, mesh_faces);
    } else if (input_extension == ".obj") {
        MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, mesh_vertices, mesh_normals, mesh_texcoords, mesh_faces,
                                             mesh_normal_ids, mesh_texcoord_ids, face_materials, _material_map);
    }
    LOG_INFO(" - load mesh from {}", in_mesh_path);
    LOG_INFO(" - build triangle adjacency ... ", in_mesh_path);

    MeshPolyRefinement::Base::TriMesh mesh;
    {
        std::vector<double> tmp_vertices(mesh_vertices.rows() * 3);
        for (int i = 0; i < mesh_vertices.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                tmp_vertices[i * 3 + j] = mesh_vertices(i, j);
            }
        }

        std::vector<std::size_t> tmp_faces(mesh_faces.rows() * 3);
        for (int i = 0; i < mesh_faces.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                tmp_faces[i * 3 + j] = mesh_faces(i, j);
            }
        }
        MeshPolyRefinement::IO::read_mesh_from_memory(tmp_vertices, tmp_faces, mesh);
    }

    LOG_INFO(" - build triangle adjacency done, vertices: {0}, faces: {1}", mesh.m_vertices.rows(),
             mesh.m_faces.rows());

    LOG_INFO("###### MeshPolyRefinement ------ Detecting Planes");
    using namespace MeshPolyRefinement;
    // Texture remeshing pipeline
    // detect planes on mesh using region-growing
    LOG_INFO(" - plane estimating ... ");
    PlaneEstimation::region_growing_plane_estimate(mesh, vm["planar_score"].as<double>(),
                                                   vm["angle"].as<double>(), vm["ratio"].as<double>(),
                                                   vm["plane_size"].as<int>());

    //expand plane segment regions
    LOG_INFO(" - plane expanding ... ");
    PlaneEstimation::plane_region_expand(mesh, 50.0, 45.0);

    //filter small non-plane regions
    LOG_INFO(" - plane region refine ... ");
    PlaneEstimation::plane_region_refine(mesh);

    //merge parallel adjacent plane segments
    LOG_INFO(" - plane region merge ... ");
    PlaneEstimation::plane_region_merge(mesh);

    if (debug_mode) {
        const std::string DebugMode_Postfix = "_DebugMode_plane-detection.ply";
        IO::save_mesh_plane_segments(output_prefix + DebugMode_Postfix, mesh);
        LOG_DEBUG(" - save plane detection model: {0}{1}", output_prefix, DebugMode_Postfix);
    }

    LOG_INFO("###### TextureRemeshing ------ Search None-plane Group Faces");
    std::vector<FaceGroup> none_plane_face_groups;
    {
        std::set<std::size_t> off_plane_faces;
        LOG_INFO(" - search non-planar faces ... ");
        std::size_t rows = mesh.m_face_plane_index.rows();

        for (int i = 0; i < rows; i++) {
            if (mesh.m_face_plane_index(i, 0) == -1) {
                off_plane_faces.insert(i);
            }
        }
        LOG_INFO(" - {} non-planar faces found", off_plane_faces.size());

        LOG_INFO(" - connect non-planar faces into groups ... ", off_plane_faces.size());
        // find connected off-plane faces group
        while (!off_plane_faces.empty()) {
            std::size_t off_plane_face_index = (*off_plane_faces.begin());
            none_plane_face_groups.push_back(FaceGroup());

            // find connected faces
            find_connected_face_group(mesh.m_ff_adjacency, off_plane_faces,
                                      off_plane_face_index, none_plane_face_groups.back());
            for (auto f_index : none_plane_face_groups.back()) {
                off_plane_faces.erase(f_index);
            }
        }
        LOG_INFO(" - {} non-planar face groups created", none_plane_face_groups.size());
    }

    if (write_intermedia) {
//    IO::repair_non_manifold(mesh);
        IO::save_mesh_plane_segments(out_mesh_path, mesh);
    }

    LOG_INFO("###### TextureRemeshing ------ Load Texture images");
    std::map<std::string, mve::ByteImage::Ptr> material_image_map;
    {
        util::WallTimer timer;
        LOG_INFO(" - load model materials ... ");

        for (auto it = _material_map.begin(); it != _material_map.end(); it++) {
            const std::string image_file_name = it->second;
            material_image_map[it->first] = mve::image::load_file(image_file_name);
        }

        LOG_INFO(" - {} materials loaded", material_image_map.size());
    }

    LOG_INFO("###### TextureRemeshing ------ Create Texture Patches");
    std::vector<MvsTexturing::Base::TexturePatch::Ptr> texture_patches;
    {
        LOG_INFO(" - create plane patches ... ");
        util::WallTimer timer;
        bool ret = false;
        ret = TextureRemeshing::Utils::create_plane_patches(mesh, mesh_texcoords, mesh_texcoord_ids,
                                                            face_materials, material_image_map,
                                                            &texture_patches, TEXTURE_PADDING, PLANE_DENSITY);

        if (!ret) {
            LOG_ERROR(" - plane patches create failed. ");
            return 0;
        }

        std::size_t plane_patches = texture_patches.size();
        LOG_INFO(" - done, {} plane patches created", plane_patches);

        LOG_INFO(" - create irregular patches ...");
        LOG_DEBUG(" - mesh texture coordinate matrix size: {0} x {1}", mesh_texcoords.rows(), mesh_texcoords.cols());
        ret = TextureRemeshing::Utils::create_irregular_patches(mesh, mesh_texcoords, mesh_texcoord_ids,
                                                                none_plane_face_groups, face_materials,
                                                                material_image_map, &texture_patches,
                                                                TEXTURE_PADDING, PLANE_DENSITY);

        if (!ret) {
            LOG_WARN(" - irregular patches create failed. ");
        }

        std::size_t irregular_patches = texture_patches.size() - plane_patches;
        LOG_INFO(" - done, {} irregular patches created", irregular_patches);
    }

    LOG_INFO("###### TextureRemeshing ------ Generate Texture Atlases");
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        LOG_INFO(" - generating atlases ... ");
        MvsTexturing::AtlasMapper::generate_texture_atlases(&texture_patches, &texture_atlases);
        LOG_INFO(" - generating atlases done, {} created", texture_atlases.size());
    }

    LOG_INFO("###### TextureRemeshing ------ Write obj model");
    {
        LOG_INFO(" - writing model ...");
        util::WallTimer timer;
        mve::TriangleMesh::Ptr temp_mesh = TextureRemeshing::Utils::triMesh_to_mveMesh(mesh);
        MvsTexturing::IO::MVE::save_obj_mesh(out_mesh_path, temp_mesh, texture_atlases);
        LOG_INFO(" - writing obj model done");
    }
    LOG_INFO("Texture remeshing done. ");
    return 0;
}

void find_connected_face_group(const IndexMatrix &ff_adjacency, const std::set<std::size_t> &outlier_faces,
                               const int face_index, FaceGroup &face_group) {
    /**
    * first level classification
    * divide faces by off-plane criteria
    */

    if (face_index == -1 ||
        (outlier_faces.find(std::size_t(face_index)) == outlier_faces.end())) {
        return;
    }

    std::queue<std::size_t> q;
    std::set<std::size_t> visited;
    q.push(face_index);
    while (!q.empty()) {
        int next_f_index = q.front();
        q.pop();

        face_group.insert(std::size_t(next_f_index));
        visited.insert(next_f_index);
        // find adjacent face index
        for (int i = 0; i < 3; i++) {
            int adj_f_index = ff_adjacency(next_f_index, i);
            if (adj_f_index == -1) {
                continue;
            }

            if (visited.find(adj_f_index) != visited.end()) {
                continue;
            }

            if (outlier_faces.find(adj_f_index) == outlier_faces.end()) {
                continue;
            }

            q.push(adj_f_index);
        }
    }
};