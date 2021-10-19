//
// Created by Storm Phoenix on 2021/10/11.
//
#include <iostream>

#include <mve/mesh_info.h>

#include <Base/View.h>
#include <IO/IO.h>
#include <TextureMapper/SceneBuilder.h>

#include <mve/image_io.h>

int main() {
    const std::string in_mesh_path = "../../../resource/model/anyuanmen.ply";
    const std::string out_mesh_path = "../../../resource/model/anyuanmen-segment.ply";
    const std::string nvm_file_path = "/Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/TexturingPipeline/resource/visualSFM/nvm_anyuanmen.nvm";
    const std::string output_dir = "../../../resource/output/";
    const std::string temp_dir = "../../../resource/output/tmp";

    using namespace MvsTexturing;
    // Read mesh files
    mve::TriangleMesh::Ptr mesh;
    try {
        mesh = IO::MVE::load_ply_mesh(in_mesh_path);
    } catch (std::exception &e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    assert(mesh->get_faces().size() % 3 == 0);

    // Build 3d scene
    mve::MeshInfo mesh_info(mesh);
    Builder::MVE::prepare_mesh(&mesh_info, mesh);

    // Read camera images
    std::vector<Base::TextureView> texture_views;
    Builder::build_scene(nvm_file_path, &texture_views, temp_dir);

    return 0;
}