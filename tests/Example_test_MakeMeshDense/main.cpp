//
// Created by Storm Phoenix on 2021/10/25.
//
#include <iostream>
#include <boost/program_options.hpp>

#include <IO/IO.h>
#include <MvsTexturing.h>

namespace bpo = boost::program_options;

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

int main(int argc, char **argv) {

    bpo::variables_map vm;
    parse_args(vm, argc, argv);

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();

    using namespace MvsTexturing;
    Base::AttributeMatrix V, N, T;
    Base::IndexMatrix F, FN, FT;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> material_map;
//    MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, V, N, T, F, FN, FT, face_materials, material_map);
    MvsTexturing::IO::load_mesh_from_ply(in_mesh_path, V, F);
    MvsTexturing::IO::save_mesh(out_mesh_path, V, F);
    return 0;
}