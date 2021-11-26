//
// Created by Storm Phoenix on 2021/10/11.
//
#include <set>
#include <iostream>
#include <boost/program_options.hpp>

#include <mve/image_tools.h>

#include <MvsTexturing.h>
#include <Base/SparseTable.h>
#include <Mapper/AtlasMapper.h>
#include <Mapper/SeamSmoother.h>
#include <Mapper/ViewSelection.h>
#include <Parameter.h>
#include <Utils/Utils.h>
#include <Utils/MeshAdapter.h>

#include <PlaneEstimation/RegionExpand.h>
#include <PlaneEstimation/RegionGrowing.h>

#include <MeshSubdivision.h>
#include <MeshSimplification.h>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

void parse_args(int argc, char **argv, MvsTexturing::Parameter &param);

void preprocessing(int argc, char **argv);

void run_mrf_method(const MvsTexturing::MeshPtr mesh, const BVHTree &bvh_tree,
                    const MvsTexturing::Parameter &param, MvsTexturing::Base::LabelGraph &graph,
                    std::vector<MvsTexturing::Base::TextureView> &texture_views);

using MeshPtr = MvsTexturing::MeshPtr;
using MeshInfo = MvsTexturing::MeshInfo;
using LabelGraph = MvsTexturing::Base::LabelGraph;
using TextureView = MvsTexturing::Base::TextureView;
using TextureViews = std::vector<TextureView>;
using TexturePatch = MvsTexturing::Base::TexturePatch;
using TexturePatches = std::vector<MvsTexturing::Base::TexturePatch::Ptr>;
using Parameter = MvsTexturing::Parameter;
using ByteImagePtr = mve::ByteImage::Ptr;
using FloatImagePtr = mve::FloatImage::Ptr;
using FloatImageConstPtr = mve::FloatImage::ConstPtr;
using FaceGroup = MeshSimplification::FaceGroup;
using TriMesh = MeshPolyRefinement::Base::TriMesh;

namespace __inner__ {
    using namespace MvsTexturing;

    class Mesh {
    public:
        Mesh() {}

    public:
        Base::AttributeMatrix m_vertices;
        Base::IndexMatrix m_faces;

        Base::AttributeMatrix m_face_colors;

        bool m_has_face_color = false;
    };

    class Color {
    public:
        std::size_t m_r, m_g, m_b;

        Color() : m_r(0), m_g(0), m_b(0) {}

        Color(std::size_t r, std::size_t g, std::size_t b) :
                m_r(r), m_g(g), m_b(b) {}

        bool operator<(const Color &other) const {
            if (m_r < other.m_r) {
                return true;
            } else if (m_r == other.m_r) {
                if (m_g < other.m_g) {
                    return true;
                } else if (m_g == other.m_g) {
                    return m_b < other.m_b;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
    };
}

bool dense_texture_to_sparse(const __inner__::Mesh &sparse_mesh, const __inner__::Mesh &dense_mesh,
                             const TexturePatches &texture_patches, const Parameter &param,
                             const std::vector<FaceGroup> &planar_group, std::vector<TexturePatch::Ptr> *final_patches);

bool map_textures(MeshPtr input_mesh, MeshInfo &mesh_info, TextureViews &texture_views,
                  const Parameter &param, std::vector<FaceGroup> &planar_groups,
                  const __inner__::Mesh &origin_mesh, const __inner__::Mesh &dense_mesh);

int main(int argc, char **argv) {
    preprocessing(argc, argv);

    MvsTexturing::Parameter param;
    try {
        parse_args(argc, argv, param);
    } catch (std::invalid_argument &ia) {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    using namespace MvsTexturing;

    util::WallTimer whole_timer;

    // Read mesh files
    std::cout << "\n### MvsTexturing------Load mesh " << std::endl;
    __inner__::Mesh origin_mesh, dense_mesh;
    std::vector<FaceGroup> planar_groups;
    MeshPtr input_mesh;
    {
        util::WallTimer IO_timer;
        if (!MvsTexturing::IO::load_mesh_from_ply(param.input_mesh, origin_mesh.m_vertices, origin_mesh.m_faces,
                                                  origin_mesh.m_face_colors)) {
            std::cout << "\tmesh load failed. " << std::endl;
            return 0;
        }

        std::cout << "\treading mesh, vertices: " << origin_mesh.m_vertices.rows()
                  << ", faces: " << origin_mesh.m_faces.rows() << " ... (Took: " << IO_timer.get_elapsed_sec()
                  << " s) " << std::endl;

        if (param.sparse_model) {
            {
                // check if face color is exists
                if (origin_mesh.m_face_colors.rows() <= 0 ||
                    (origin_mesh.m_face_colors.rows() != origin_mesh.m_faces.rows())) {
                    std::cout << "\t WARNING !!! --- the sparse model dosen't have face color attribute. \n";
                    origin_mesh.m_has_face_color = false;
                }

                origin_mesh.m_has_face_color = true;
            }

            // TODO if model is colored, then take colors into consideration in remove duplicated faces step
            // TODO if not, only removing duplicated faces step was executed.

            {
                // remove duplicated faces
                // TODO Take color into consideration
                std::size_t origin_faces = origin_mesh.m_faces.rows();
                MeshSimplification::remove_duplicate_faces(origin_mesh.m_faces, origin_mesh.m_face_colors);
                std::size_t removed_faces = origin_mesh.m_faces.rows();
                std::cout << "\tremove duplicated faces : " << origin_faces - removed_faces << std::endl;
            }

            {
                // detect planes and init face group
                if (origin_mesh.m_has_face_color) {
                    // if face color exists, the plane groups have been computed
                    std::map<__inner__::Color, std::size_t> group_id_map;

                    for (std::size_t r = 0; r < origin_mesh.m_face_colors.rows(); r++) {
                        __inner__::Color color(origin_mesh.m_face_colors(r, 0),
                                               origin_mesh.m_face_colors(r, 1),
                                               origin_mesh.m_face_colors(r, 2));

                        if (color.m_r == 0 &&
                            color.m_g == 0 &&
                            color.m_b == 0) {
                            // TODO
                            continue;
                        }

                        if (group_id_map.find(color) == group_id_map.end()) {
                            planar_groups.push_back(FaceGroup());
                            group_id_map[color] = planar_groups.size() - 1;
                        } else {
                            planar_groups[group_id_map[color]].m_face_indices.push_back(r);
                        }
                    }

                    for (FaceGroup &group : planar_groups) {
                        MeshSimplification::fit_face_group_plane(origin_mesh.m_vertices, origin_mesh.m_faces, group);
                    }
                } else {
                    TriMesh sparse_tri_mesh;
                    MvsTexturing::Utils::eigenMesh_to_TriMesh(origin_mesh.m_vertices, origin_mesh.m_faces,
                                                              sparse_tri_mesh);

                    using namespace MeshPolyRefinement;
                    // detect planes on mesh using region-growing
                    PlaneEstimation::region_growing_plane_estimate(sparse_tri_mesh, param.planar_score,
                                                                   param.angle_threshold, param.ratio_threshold,
                                                                   param.min_plane_size);

                    //expand plane segment regions
                    PlaneEstimation::plane_region_expand(sparse_tri_mesh, 50.0, 45.0);

                    //filter small non-plane regions
                    PlaneEstimation::plane_region_refine(sparse_tri_mesh);

                    //merge parallel adjacent plane segments
                    PlaneEstimation::plane_region_merge(sparse_tri_mesh);

                    planar_groups.resize(sparse_tri_mesh.m_plane_groups.size());
                    for (std::size_t group_idx = 0; group_idx < sparse_tri_mesh.m_plane_groups.size(); group_idx++) {
                        const MeshPolyRefinement::Base::PlaneGroup &group = sparse_tri_mesh.m_plane_groups[group_idx];
                        FaceGroup &dest_group = planar_groups[group_idx];

                        for (std::size_t group_face_idx : group.m_indices) {
                            dest_group.m_face_indices.push_back(group_face_idx);
                        }

                        dest_group.m_plane_normal = group.m_plane_normal;
                        dest_group.m_plane_center = group.m_plane_center;
                        dest_group.m_x_axis = group.m_x_axis;
                        dest_group.m_y_axis = group.m_y_axis;
                    }
                }
            }

            IO_timer.reset();
            MeshSubdivision::make_mesh_dense(origin_mesh.m_vertices, origin_mesh.m_faces, dense_mesh.m_vertices,
                                             dense_mesh.m_faces, origin_mesh.m_face_colors, dense_mesh.m_face_colors);
            std::cout << "\tmodel is too sparse, make the mesh dense ... vertices: " << dense_mesh.m_vertices.rows()
                      << ", faces: " << dense_mesh.m_faces.rows() << " ... (Took: " << IO_timer.get_elapsed_sec()
                      << " s) " << std::endl;

            if (param.debug_mode) {
                IO::save_ply_mesh(Utils::str_prefix(param.output_prefix) + "_Debug_dense.ply",
                                  dense_mesh.m_vertices, dense_mesh.m_faces);
            }

            input_mesh = Utils::eigenMesh_to_mveMesh(dense_mesh.m_vertices, dense_mesh.m_faces);
        } else {
            input_mesh = Utils::eigenMesh_to_mveMesh(origin_mesh.m_vertices, origin_mesh.m_faces);
        }

        if (input_mesh == nullptr || (input_mesh->get_faces().size() % 3 != 0)) {
            std::cerr << "\tcould not load mesh. " << std::endl;
            return 0;
        }
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
    std::vector<TextureView> texture_views;
    {
        util::WallTimer IO_timer;
        std::cout << "\tread camera poses and images ... ";
        Builder::build_scene(param.scene_file, &texture_views, temp_dir);
        std::cout << "(Took: " << IO_timer.get_elapsed_sec() << " s)" << std::endl;
    }

    if (!map_textures(input_mesh, mesh_info, texture_views, param, planar_groups, origin_mesh, dense_mesh)) {
        std::cout << "\nMvsTexturing failed. (Took: " << whole_timer.get_elapsed_sec() << " s)" << std::endl;
        return 0;
    }

    std::cout << "\nMvsTexturing done. (Took: " << whole_timer.get_elapsed_sec() / double(60) << " min)" << std::endl;
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
            ("sparse_model", bpo::value<bool>()->default_value(false),
             "whether the mesh is sparse {true, false} [false]")
            ("tone_mapping", bpo::value<std::string>()->default_value("none"),
             "Tone mapping method: {none, gamma} [none]")
            ("mrf_call_lib", bpo::value<std::string>()->default_value("mapmap"),
             "MRF call library: {mapmap, openmvs} [mapmap]")
            ("debug_mode", bpo::value<bool>()->default_value(false), "Debug mode: {true, false} [false]");
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

    param.sparse_model = vm["sparse_model"].as<bool>();
    param.debug_mode = vm["debug_mode"].as<bool>();
}

void preprocessing(int argc, char **argv) {
    // print program build time
    std::cout << argv[0] << " (built on " << __DATE__ << ", " << __TIME__ << ")" << std::endl;
}

void run_mrf_method(const MeshPtr mesh, const BVHTree &bvh_tree,
                    const Parameter &param, LabelGraph &graph,
                    TextureViews &texture_views) {
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

bool map_textures(MeshPtr input_mesh, MeshInfo &mesh_info, TextureViews &texture_views,
                  const Parameter &param, std::vector<FaceGroup> &planar_groups,
                  const __inner__::Mesh &origin_mesh, const __inner__::Mesh &dense_mesh) {
    // Build adjacency graph
    std::cout << "\n### MvsTexturing------Build adjacency graph " << std::endl;
    std::size_t const n_faces = input_mesh->get_faces().size() / 3;
    LabelGraph graph(n_faces);
    MvsTexturing::Builder::MVE::build_adjacency_graph(input_mesh, mesh_info, &graph);

    std::cout << "\n### MvsTexturing------View selection " << std::endl;
    {
        if (param.labeling_file.empty()) {
            util::WallTimer timer;
            namespace VS = MvsTexturing::ViewSelection;

            // build bvh tree
            std::cout << "\tBuilding BVH from " << n_faces << " faces... " << std::flush;
            BVHTree bvh_tree(input_mesh->get_faces(), input_mesh->get_vertices());
            std::cout << "done. (Took: " << timer.get_elapsed() << " ms)" << std::endl;

            if (param.method_type == "mrf") {
                std::cout << "\trunning MRF-algorithm ... " << std::endl;
                timer.reset();
                run_mrf_method(input_mesh, bvh_tree, param, graph, texture_views);
                std::cout << "\n\tmrf optimization done. (Took: " << timer.get_elapsed_sec() << " s)\n";
            } else if (param.method_type == "projection") {
                std::cout << "\tRunning Projection-algorithm ... " << std::endl;
                timer.reset();
                VS::Projection::solve_projection_problem(input_mesh, mesh_info, bvh_tree, graph, texture_views, param);
                std::cout << "\n\tProjection method done. (Took: " << timer.get_elapsed_sec() << " s)\n";
            } else {
                std::cout << "\tView selection method not supported: " << param.method_type << std::endl;
                return false;
            }
        } else {
            std::cout << "Loading labeling from file... " << std::flush;

            /* Load labeling from file. */
            std::vector<std::size_t> labeling = MvsTexturing::Utils::vector_from_file<std::size_t>(param.labeling_file);
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
    std::vector<TexturePatch::Ptr> texture_patches;
    {
        using namespace MvsTexturing;
        util::WallTimer timer;
        // Create texture patches and adjust them
        std::vector<std::vector<Base::VertexProjectionInfo>> vertex_projection_infos;
        std::cout << "\tgenerate primary texture patches ... ";
        AtlasMapper::generate_texture_patches(graph, input_mesh, mesh_info, &texture_views,
                                              param, &vertex_projection_infos, &texture_patches);
        std::cout << texture_patches.size() << " patches. (Took: " << timer.get_elapsed_sec() << "s)\n";

        if (!param.skip_global_seam_leveling) {
            std::cout << "\trunning global seam leveling... ";
            timer.reset();
            SeamSmoother::global_seam_leveling(graph, input_mesh, mesh_info, vertex_projection_infos, &texture_patches);
            std::cout << "\tdone. (Took: " << timer.get_elapsed_sec() << " s)\n";
        } else {
            timer.reset();
            std::cout << "\tcalculating validity masks for texture patches... ";
#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches.size(); ++i) {
                Base::TexturePatch::Ptr texture_patch = texture_patches[i];
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        }

        if (!param.skip_local_seam_leveling) {
            std::cout << "\trunning local seam leveling ... ";
            timer.reset();
            SeamSmoother::local_seam_leveling(graph, input_mesh, vertex_projection_infos, &texture_patches);
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        }

        if (param.sparse_model) {
            std::vector<TexturePatch::Ptr> final_texture_patches;
            std::cout << "\tretrieve sparse model texture from dense model ... ";
            dense_texture_to_sparse(origin_mesh, dense_mesh, texture_patches, param, planar_groups,
                                    &final_texture_patches);
            texture_patches.swap(final_texture_patches);
            std::cout << "done. (Took: " << timer.get_elapsed_sec() << " s)\n";
        }
    }

    std::cout << "\n### MvsTexturing------Generating Atlases " << std::endl;
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        using namespace MvsTexturing;
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
        if (!param.sparse_model) {
            MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, input_mesh, texture_atlases);
        } else {
            input_mesh = MvsTexturing::Utils::eigenMesh_to_mveMesh(origin_mesh.m_vertices, origin_mesh.m_faces);
            MvsTexturing::IO::save_ply_mesh(
                    MvsTexturing::Utils::str_prefix(param.output_prefix) + "_Debug_dense_final.ply",
                    origin_mesh.m_vertices, origin_mesh.m_faces);
            MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, input_mesh, texture_atlases);
        }
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    return true;
}

bool dense_texture_to_sparse(const __inner__::Mesh &sparse_mesh, const __inner__::Mesh &dense_mesh,
                             const TexturePatches &texture_patches, const Parameter &param,
                             const std::vector<FaceGroup> &planar_groups,
                             std::vector<TexturePatch::Ptr> *final_patches) {

    // build relations between sparse and dense model
    std::vector<std::vector<std::size_t>> face_subdivisions;
    face_subdivisions.resize(sparse_mesh.m_faces.rows());
    {
        std::map<__inner__::Color, std::size_t> tmp_color_face_map;
        for (int i = 0; i < sparse_mesh.m_face_colors.rows(); i++) {
            const __inner__::Color face_color(
                    sparse_mesh.m_face_colors(i, 0),
                    sparse_mesh.m_face_colors(i, 1),
                    sparse_mesh.m_face_colors(i, 2));
            tmp_color_face_map[face_color] = i;
        }

        for (int dense_face_id = 0; dense_face_id < dense_mesh.m_face_colors.rows(); dense_face_id++) {
            const __inner__::Color dense_face_color(
                    dense_mesh.m_face_colors(dense_face_id, 0),
                    dense_mesh.m_face_colors(dense_face_id, 1),
                    dense_mesh.m_face_colors(dense_face_id, 2));

            if (tmp_color_face_map.find(dense_face_color) != tmp_color_face_map.end()) {
                std::size_t sparse_face_idx = tmp_color_face_map[dense_face_color];
                face_subdivisions[sparse_face_idx].push_back(dense_face_id);
            }
        }
    }

    using namespace MvsTexturing;
    // init texture coords and face materials
    std::vector<FloatImageConstPtr> dense_mesh_face_materials;
    std::vector<math::Vec2f> dense_mesh_face_texture_coords;
    {
        std::size_t n_dense_faces = dense_mesh.m_faces.rows();
        dense_mesh_face_materials.resize(n_dense_faces);
        dense_mesh_face_texture_coords.resize(n_dense_faces * 3);

        for (auto it = texture_patches.begin(); it != texture_patches.end(); it++) {
            const std::vector<std::size_t> &faces_in_patch = (*it)->get_faces();
            const std::vector<math::Vec2f> &texture_coords_in_patch = (*it)->get_texcoords();
            if (faces_in_patch.size() * 3 != texture_coords_in_patch.size()) {
                std::cout << "size of faces and texcoords in TexturePatch is not match. \n";
                return false;
            }

            for (std::size_t i = 0; i < faces_in_patch.size(); i++) {
                std::size_t face_id = faces_in_patch[i];
                dense_mesh_face_materials[face_id] = (*it)->get_image();

                for (int c = 0; c < 3; c++) {
                    dense_mesh_face_texture_coords[face_id * 3 + c] = texture_coords_in_patch[i * 3 + c];
                }
            }
        }
    }

    return MeshSimplification::simplify_mesh_texture(sparse_mesh.m_vertices, sparse_mesh.m_faces, planar_groups,

                                                     dense_mesh.m_vertices, dense_mesh.m_faces,
                                                     dense_mesh_face_texture_coords,
                                                     dense_mesh_face_materials,

                                                     face_subdivisions, final_patches);
}