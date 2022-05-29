//
// Created by Storm Phoenix on 2021/10/25.
//
#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/program_options.hpp>

#include <IO/IO.h>
#include <Repair/MeshSubdivision.h>
#include <Repair/MeshSimplification.h>

namespace bpo = boost::program_options;

void parse_args(bpo::variables_map &vm, int argc, char **argv);

using namespace MvsTexturing;

int main(int argc, char **argv) {
    bpo::variables_map vm;
    parse_args(vm, argc, argv);

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();

    using namespace MvsTexturing;
    MeshSubdivision::AttributeMatrix V, N, T;
    MeshSubdivision::IndexMatrix F, FN, FT;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> material_map;
//    MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, V, N, T, F, FN, FT, face_materials, material_map);
    {
        MvsTexturing::IO::load_mesh_from_ply(in_mesh_path, V, F);

        std::size_t origin_faces = F.rows();
        MeshRepair::remove_duplicate_faces(F);
        std::size_t removed_faces = F.rows();
        std::cout << "remove duplicated faces : " << origin_faces - removed_faces << std::endl;
    }

    MeshSubdivision::AttributeMatrix out_V;
    MeshSubdivision::IndexMatrix out_F;

    std::cout << "MakeDense origin model - faces: " << F.rows() << " vertices: " << V.rows() << std::endl;
    MeshSubdivision::AttributeMatrix FC, out_FC;
    MeshSubdivision::make_mesh_dense(V, F, out_V, out_F, FC, out_FC, 0.05, 982513);
    std::cout << "MakeDense result model - faces: " << out_F.rows() << " vertices: " << out_V.rows() << std::endl;

    if (!MvsTexturing::IO::save_ply_mesh(out_mesh_path, out_V, out_F)) {
        std::cout << "Densed ply save test case failed. \n";
    }

    if (!MvsTexturing::IO::save_ply_mesh("./colored_F.ply", V, F, FC)) {
        std::cout << "Colored ply save test case failed. \n";
        std::cout << "V F FC rows: " << V.rows() << " " << F.rows() << " " << FC.rows() << std::endl;
    }

    if (!MvsTexturing::IO::save_ply_mesh("./colored_densed_F.ply", out_V, out_F, out_FC)) {
        std::cout << "Densed Colored ply save test case failed. \n";
        std::cout << "out_V out_F out_FC rows: " << out_V.rows() << " " << out_F.rows() << " " << out_FC.rows()
                  << std::endl;
    }

    return 0;
}

void parse_args(bpo::variables_map &vm, int argc, char **argv) {
    bpo::options_description opts("Example test MeshRemeshing options");
    opts.add_options()
            ("help", "produce help message")
            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
            ("output_mesh", bpo::value<std::string>()->default_value("res.ply"), "path of output simplified mesh");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        throw std::runtime_error("undefine options in command lines.\n");
    }
}