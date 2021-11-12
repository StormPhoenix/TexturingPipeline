//
// Created by Storm Phoenix on 2021/10/11.
//
#include <set>
#include <iostream>
#include <boost/program_options.hpp>

#include <MvsTexturing.h>
#include <Base/SparseTable.h>
#include <Mapper/AtlasMapper.h>
#include <Mapper/SeamSmoother.h>
#include <Mapper/ViewSelection.h>
#include <Parameter.h>
#include <Utils/Utils.h>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

void parse_args(int argc, char **argv, MvsTexturing::Parameter &param);

void preprocessing(int argc, char **argv);

void run_mrf_method(const MvsTexturing::MeshPtr mesh, const BVHTree &bvh_tree,
                    const MvsTexturing::Parameter &param, MvsTexturing::Base::LabelGraph &graph,
                    std::vector<MvsTexturing::Base::TextureView> &texture_views);

int main(int argc, char **argv) {
    preprocessing(argc, argv);

    MvsTexturing::Parameter param;
    try {
        parse_args(argc, argv, param);
    } catch (std::invalid_argument &ia) {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Read mesh files
    std::cout << "\n### MvsTexturing------Load mesh " << std::endl;
    using namespace MvsTexturing;
    MeshPtr input_mesh;
    try {
        input_mesh = IO::MVE::load_ply_mesh(param.input_mesh);
        assert(input_mesh->get_faces().size() % 3 == 0);
    } catch (std::exception &e) {
        std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    MeshInfo mesh_info(input_mesh);
    Builder::MVE::prepare_mesh(&mesh_info, input_mesh);

    const std::string output_dir = util::fs::dirname(param.output_prefix);
    const std::string temp_dir = util::fs::join_path(output_dir, "tmp");
    {
        // Create temporary directory
        if (!util::fs::dir_exists(output_dir.c_str())) {
            util::fs::mkdir(output_dir.c_str());
        }

        if (!util::fs::dir_exists(temp_dir.c_str())) {
            util::fs::mkdir(temp_dir.c_str());
        }
    }

    // Read camera images
    std::cout << "\n### MvsTexturing------Read camera images " << std::endl;
    std::vector<Base::TextureView> texture_views;
    Builder::build_scene(param.scene_file, &texture_views, temp_dir);

    // Build adjacency graph
    std::cout << "\n### MvsTexturing------Build adjacency graph " << std::endl;
    std::size_t const n_faces = input_mesh->get_faces().size() / 3;
    Base::LabelGraph graph(n_faces);
    Builder::MVE::build_adjacency_graph(input_mesh, mesh_info, &graph);

    std::cout << "\n### MvsTexturing------View selection " << std::endl;
    {
        if (param.labeling_file.empty()) {
            util::WallTimer timer;
            namespace VS = ViewSelection;

            // build bvh tree
            std::cout << "\tBuilding BVH from " << n_faces << " faces... " << std::flush;
            BVHTree bvh_tree(input_mesh->get_faces(), input_mesh->get_vertices());
            std::cout << "done. (Took: " << timer.get_elapsed() << " ms)" << std::endl;

            if (param.method_type == "mrf") {
                std::cout << "\tRunning MRF-algorithm ... " << std::endl;
                timer.reset();
                run_mrf_method(input_mesh, bvh_tree, param, graph, texture_views);
                std::cout << "\n\tMRF optimization done. (Took: " << timer.get_elapsed_sec() << " s)\n";
            } else if (param.method_type == "projection") {
                std::cout << "\tRunning Projection-algorithm ... " << std::endl;
                timer.reset();
                VS::Projection::solve_projection_problem(input_mesh, bvh_tree, graph, texture_views, param);
                std::cout << "\n\tProjection method done. (Took: " << timer.get_elapsed_sec() << " s)\n";
            } else {
                std::cout << "\tView selection method not supported: " << param.method_type << std::endl;
                return 0;
            }
        } else {
            std::cout << "Loading labeling from file... " << std::flush;

            /* Load labeling from file. */
            std::vector<std::size_t> labeling = Utils::vector_from_file<std::size_t>(param.labeling_file);
            if (labeling.size() != graph.num_nodes()) {
                std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
                std::exit(EXIT_FAILURE);
            }

            /* Transfer labeling to graph. */
            for (std::size_t i = 0; i < labeling.size(); ++i) {
                const std::size_t label = labeling[i];
                if (label > texture_views.size()) {
                    std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                graph.set_label(i, label);
            }

            std::cout << "done." << std::endl;
        }
        std::cout << "\tView selection done. " << std::endl;
    }

    std::cout << "\n### MvsTexturing------Generating Patches " << std::endl;
    std::vector<MvsTexturing::Base::TexturePatch::Ptr> texture_patches;
    {
        using namespace MvsTexturing;
        util::WallTimer timer;
        // Create texture patches and adjust them
        std::vector<std::vector<Base::VertexProjectionInfo>> vertex_projection_infos;
        AtlasMapper::generate_texture_patches(graph, input_mesh, mesh_info, &texture_views,
                                              param, &vertex_projection_infos, &texture_patches);

        if (!param.skip_global_seam_leveling) {
            std::cout << "\tRunning global seam leveling... ";
            timer.reset();
            SeamSmoother::global_seam_leveling(graph, input_mesh, mesh_info, vertex_projection_infos, &texture_patches);
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        } else {
            timer.reset();
            std::cout << "\tCalculating validity masks for texture patches... ";
#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches.size(); ++i) {
                Base::TexturePatch::Ptr texture_patch = texture_patches[i];
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        }

        if (!param.skip_local_seam_leveling) {
            std::cout << "\tRunning local seam leveling... " << std::endl;
            timer.reset();
            SeamSmoother::local_seam_leveling(graph, input_mesh, vertex_projection_infos, &texture_patches);
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        }
    }

    std::cout << "\n### MvsTexturing------Generating Atlases " << std::endl;
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        util::WallTimer timer;
        std::cout << "\tgenerating ... " << std::endl;
        AtlasMapper::generate_texture_atlases(&texture_patches, &texture_atlases,
                                              param.tone_mapping == Tone_Mapping_Gamma);
        std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
    }

    std::cout << "\n### MvsTexturing------Write obj model" << std::endl;
    {
        std::cout << "\tWriting ..." << std::flush;
        util::WallTimer timer;
        MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, input_mesh, texture_atlases);
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }
    std::cout << "\nMvsTexturing done. " << std::endl;
    return 0;
}

void parse_args(int argc, char **argv, MvsTexturing::Parameter &param) {
    namespace bpo = boost::program_options;

    bpo::options_description opts("Example test-MvsTexturing options");
    bpo::variables_map vm;
    opts.add_options()
            ("help", "produce help message")
            ("scene_file", bpo::value<std::string>()->default_value("model.nvm"),
             "scene_file := (SCENE_FOLDER | BUNDLE_FILE | MVE_SCENE::EMBEDDING)")
            ("input_mesh", bpo::value<std::string>()->default_value("mesh.ply"),
             "The mesh that you want to texture and which needs to be in the same coordinate frame as the camera parameters.")
            ("output_prefix", bpo::value<std::string>()->default_value("./texture_mesh"),
             "A path and name for the output files, e.g. <path>/<to>/my_textured_mesh")
            ("data_cost_file", bpo::value<std::string>()->default_value(""),
             "Skip calculation of data costs and use the ones provided in the given file")
            ("labeling_file", bpo::value<std::string>()->default_value(""),
             "Skip view selection and use the labeling provided in the given file")
            ("method_type", bpo::value<std::string>()->default_value("mrf"), "Texture mapping method [mrf]")
            ("data_term", bpo::value<std::string>()->default_value("area"), "MRF data cost term [area]")
            ("smoothness_term", bpo::value<std::string>()->default_value("potts"), "MRF smooth cost term [potts]")
            ("outlier_removal", bpo::value<std::string>()->default_value("none"),
             "Photometric outlier removal method [none]")
            ("view_selection_model", bpo::value<bool>()->default_value(false), "Write out view selection model [false]")
            ("skip_geometric_visibility_test", bpo::value<bool>()->default_value(false),
             "Skip geometric visibility test based on ray intersection [false]")
            ("skip_global_seam_leveling", bpo::value<bool>()->default_value(false), "Skip global seam leveling [false]")
            ("skip_local_seam_leveling", bpo::value<bool>()->default_value(false),
             "Skip local seam leveling (Poisson editing) [false]")
            ("skip_hole_filling", bpo::value<bool>()->default_value(false), "Skip hole filling [false]")
            ("keep_unseen_faces", bpo::value<bool>()->default_value(false), "Keep unseen faces [false]")
            ("write_intermediate_results", bpo::value<bool>()->default_value(false),
             "write intermediate results [false]")
            ("tone_mapping", bpo::value<std::string>()->default_value("none"),
             "Tone mapping method: {none, gamma} [none]")
            ("mrf_call_lib", bpo::value<std::string>()->default_value("mapmap"),
             "MRF call library: {mapmap, openmvs} [mapmap]");
    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);

    param.scene_file = vm["scene_file"].as<std::string>();
    param.input_mesh = vm["input_mesh"].as<std::string>();
    param.output_prefix = util::fs::sanitize_path(vm["output_prefix"].as<std::string>());
    param.data_cost_file = vm["data_cost_file"].as<std::string>();
    param.labeling_file = vm["labeling_file"].as<std::string>();
    param.method_type = vm["method_type"].as<std::string>();
    param.data_term = vm["data_term"].as<std::string>();
    param.smoothness_term = vm["smoothness_term"].as<std::string>();
    param.outlier_removal = vm["outlier_removal"].as<std::string>();
    param.mrf_call_lib = vm["mrf_call_lib"].as<std::string>();
    param.view_selection_model = vm["view_selection_model"].as<bool>();
    param.skip_geometric_visibility_test = vm["skip_geometric_visibility_test"].as<bool>();
    param.skip_global_seam_leveling = vm["skip_global_seam_leveling"].as<bool>();
    param.skip_local_seam_leveling = vm["skip_local_seam_leveling"].as<bool>();
    param.skip_hole_filling = vm["skip_hole_filling"].as<bool>();
    param.keep_unseen_faces = vm["keep_unseen_faces"].as<bool>();
    param.write_intermediate_results = vm["write_intermediate_results"].as<bool>();

    param.viewing_angle_threshold = 85.0f;
    param.tone_mapping = vm["tone_mapping"].as<std::string>();

    param.planar_score = 0.3;
    param.angle_threshold = 30.0;
    param.ratio_threshold = 20.0;
    param.min_plane_size = 5;
}

void preprocessing(int argc, char **argv) {
    // print program build time
    std::cout << argv[0] << " (built on " << __DATE__ << ", " << __TIME__ << ")" << std::endl;
}

void run_mrf_method(const MvsTexturing::MeshPtr mesh,
                    const BVHTree &bvh_tree,
                    const MvsTexturing::Parameter &param,
                    MvsTexturing::Base::LabelGraph &graph,
                    std::vector<MvsTexturing::Base::TextureView> &texture_views) {
    using namespace MvsTexturing;
    namespace VS = ViewSelection;
    typedef Base::SparseTable<std::uint32_t, std::uint16_t, double> DataCosts;

    std::size_t n_faces = mesh->get_faces().size() / 3;
    DataCosts data_costs(n_faces, texture_views.size());
    if (param.data_cost_file.empty()) {
        VS::Mrf::calculate_data_costs(mesh, bvh_tree, texture_views, param, &data_costs);

        if (param.write_intermediate_results) {
            std::cout << "\tWriting data cost file... " << std::flush;
            DataCosts::save_to_file(data_costs, param.output_prefix + "_data_costs.spt");
            std::cout << "done." << std::endl;
        }
    } else {
        std::cout << "\tLoading data cost file... " << std::flush;
        try {
            DataCosts::load_from_file(param.data_cost_file, &data_costs);
        } catch (util::FileException e) {
            std::cout << "failed!" << std::endl;
            std::cerr << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "done." << std::endl;
    }
    VS::Mrf::solve_mrf_problem(data_costs, graph, param);
}