//
// Created by Storm Phoenix on 2021/10/22.
//
#include <iostream>
#include <fstream>

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
#include <TextureMapper/AtlasMapper.h>

#include <util/timer.h>

#include "RemeshingUtils.h"

#define PLANE_DENSITY 300

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> AttributeMatrix;
typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;

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
             "options for write intermedia files, default false");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        std::cout << "undefine options in command lines.\n";
        return 0;
    }

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();
    const bool write_intermedia = vm["write_intermedia"].as<bool>();

    AttributeMatrix mesh_vertices, mesh_normals, mesh_texcoords;
    IndexMatrix mesh_faces, mesh_normal_ids, mesh_texcoord_ids;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> _material_map;

    std::string extension = "";
    {
        std::size_t dotpos = in_mesh_path.find_last_of('.');
        if (dotpos != std::string::npos) {
            extension = in_mesh_path.substr(dotpos, in_mesh_path.size());
        }
    }

    if (extension == ".ply") {
        MvsTexturing::IO::load_mesh_from_ply(in_mesh_path, mesh_vertices, mesh_faces);
    } else if (extension == ".obj") {
        MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, mesh_vertices, mesh_normals, mesh_texcoords, mesh_faces,
                                             mesh_normal_ids, mesh_texcoord_ids, face_materials, _material_map);
    }

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
    std::cout << "Mesh vertices: " << mesh.m_vertices.rows() << " faces: " << mesh.m_faces.rows() << std::endl;

    using namespace MeshPolyRefinement;
    // Texture remeshing pipeline
    std::cout << "\n### MeshPolyRefinement------Detecting Planes" << std::endl;
    // detect planes on mesh using region-growing
    PlaneEstimation::region_growing_plane_estimate(mesh, vm["planar_score"].as<double>(),
                                                   vm["angle"].as<double>(), vm["ratio"].as<double>(),
                                                   vm["plane_size"].as<int>());

    //expand plane segment regions
    PlaneEstimation::plane_region_expand(mesh, 50.0, 45.0);

    //filter small non-plane regions
    PlaneEstimation::plane_region_refine(mesh);

    //merge parallel adjacent plane segments
    PlaneEstimation::plane_region_merge(mesh);

    if (write_intermedia) {
//    IO::repair_non_manifold(mesh);
        IO::save_mesh_plane_segments(out_mesh_path, mesh);
    }


    std::cout << "\n### TextureRemeshing------Load Texture images" << std::endl;
    std::map<std::string, mve::ByteImage::Ptr> material_image_map;
    {
        util::WallTimer timer;
        std::cout << "\tLoad images...";

        for (auto it = _material_map.begin(); it != _material_map.end(); it++) {
            const std::string image_file_name = it->second;
            material_image_map[it->first] = mve::image::load_file(image_file_name);
        }

        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    std::cout << "\n### TextureRemeshing------Generate Texture Patches" << std::endl;
    std::vector<MvsTexturing::Base::TexturePatch::Ptr> texture_patches;
    {
        std::cout << "\tGenerating..." << std::flush;
        util::WallTimer timer;
        TextureRemeshing::Utils::remeshing_from_plane_groups(mesh, mesh_texcoords,
                                                             mesh_texcoord_ids, face_materials, material_image_map,
                                                             &texture_patches, 10, PLANE_DENSITY);
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    std::cout << "\n### TextureRemeshing------Generate Texture Atlases" << std::endl;
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        MvsTexturing::AtlasMapper::generate_texture_atlases(&texture_patches, &texture_atlases);
    }

    std::cout << "\n### TextureRemeshing------Write obj model" << std::endl;
    {
        std::cout << "\tWriting ..." << std::flush;
        util::WallTimer timer;
        mve::TriangleMesh::Ptr temp_mesh = TextureRemeshing::Utils::triMesh_to_mveMesh(mesh);
        MvsTexturing::IO::MVE::save_obj_mesh(out_mesh_path, temp_mesh, texture_atlases);

        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }
    std::cout << "Texture remeshing done. " << std::endl;
    return 0;
}