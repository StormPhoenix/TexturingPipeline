//
// Created by Storm Phoenix on 2021/10/11.
//
#include <iostream>
#include <set>

#include <mve/mesh_info.h>
#include <util/system.h>
#include <util/file_system.h>

#include <IO/IO.h>
#include <Base/View.h>
#include <Base/LabelGraph.h>
#include <Utils/Timer.h>
#include <TextureMapper/SceneBuilder.h>
#include <TextureMapper/ViewSelection.h>

#include "Arguments.h"

int main(int argc, char **argv) {
    util::system::print_build_timestamp(argv[0]);
    util::system::register_segfault_handler();

    Arguments conf;
    try {
        conf = parse_args(argc, argv);
    } catch (std::invalid_argument & ia) {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    using namespace MvsTexturing;
    // Read mesh files
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = IO::MVE::load_ply_mesh(conf.in_mesh);
    } catch (std::exception &e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    assert(mesh->get_faces().size() % 3 == 0);

    // Build 3d scene
    mve::MeshInfo mesh_info(mesh);
    Builder::MVE::prepare_mesh(&mesh_info, mesh);

    // Create temporary directory
    const std::string output_dir = util::fs::dirname(conf.out_prefix);
    const std::string temp_dir = util::fs::join_path(output_dir, "tmp");
    if (!util::fs::dir_exists(output_dir.c_str())) {
        util::fs::mkdir(output_dir.c_str());
    }

    if (!util::fs::dir_exists(temp_dir.c_str())) {
        util::fs::mkdir(temp_dir.c_str());
    }

    // Read camera images
    std::cout << "### Read camera images " << std::endl;
    std::vector<Base::TextureView> texture_views;
    Builder::build_scene(conf.in_scene, &texture_views, temp_dir);

    // Build adjacency graph
    std::size_t const n_faces = mesh->get_faces().size() / 3;
    Base::LabelGraph graph(n_faces);
    Builder::MVE::build_adjacency_graph(mesh, mesh_info, &graph);

    std::cout << "### View selection " << std::endl;
    {
        // Running projection algorithms
        std::cout << "\tRunning projection algorithm ... " << std::endl;
        const std::vector<unsigned int> &faces = mesh->get_faces();
        const std::vector<math::Vec3f> &vertices = mesh->get_vertices();

        std::vector<std::set<std::size_t>> face_visibility_sets(faces.size() / 3);
        {
            util::WallTimer compute_visibility_timer;
            std::cout << "\tCompute face visibility " << faces.size() / 3 << " faces, " << texture_views.size()
                      << " cameras ... " << std::flush;
            ViewSelection::compute_face_camera_photometric(mesh, texture_views, face_visibility_sets, conf.settings);
            std::cout << "done. (Took: " << compute_visibility_timer.get_elapsed_sec() << " s)" << std::endl;
        }
    }

    return 0;
}