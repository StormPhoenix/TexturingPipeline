//
// Created by Storm Phoenix on 2022/4/22.
//

#include <iostream>
#include <common.h>
#include <boost/program_options.hpp>

#include "TextureRemoval.h"

void preprocessing(int argc, char **argv);

int main(int argc, char **argv) {
    preprocessing(argc, argv);

    std::string mesh_file = "";
    std::string output_prefix = "";
    {
        // parse args
        namespace bpo = boost::program_options;

        bpo::options_description opts("Example test-MvsTexturing options");
        bpo::variables_map vm;
        opts.add_options()("help", "produce help message")
                ("input_mesh", bpo::value<std::string>()->default_value("mesh.ply"),
                 "The mesh that you want to remove redundant texture.")
                ("output_prefix", bpo::value<std::string>()->default_value("removal"),
                 "The path that you want to store result.");
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
        mesh_file = vm["input_mesh"].as<std::string>();
        output_prefix = vm["output_prefix"].as<std::string>();
    }

    TextureRemoval removal;
    if (!removal.loadObjMesh(mesh_file)) {
        LOG_ERROR("RedundantTextureRemoval - Model load failed");
        return 0;
    }

    removal.removeRedundantTextures(output_prefix);

    LOG_INFO("RedundantTextureRemoval - DONE");
}

void preprocessing(int argc, char **argv) {
    // print program build time
    std::cout << argv[0] << " (built on " << __DATE__ << ", " << __TIME__ << ")" << std::endl;
}
