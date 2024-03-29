//
// Created by Storm Phoenix on 2021/10/11.
//
#include <set>
#include <vector>
#include <sstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <common.h>

#include <mve/image.h>
#include <mve/image_io.h>
#include <mve/image_tools.h>

#include <Parameter.h>
#include <MvsTexturing.h>
#include <Base/SparseTable.h>
#include <Mapper/AtlasMapper.h>
#include <Mapper/SeamSmoother.h>
#include <Mapper/ViewSelection.h>
#include <Repair/MeshSubdivision.h>
#include <Repair/MeshSimplification.h>
#include <Utils/Utils.h>
#include <Utils/MeshAdapter.h>

#include <PlaneEstimation/RegionExpand.h>
#include <PlaneEstimation/RegionGrowing.h>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

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
using FaceGroup = MvsTexturing::MeshRepair::FaceGroup;
using FaceSubdivisions = std::vector<std::vector<std::size_t>>;
using TriMesh = MeshPolyRefinement::Base::TriMesh;

void parse_args(int argc, char **argv, MvsTexturing::Parameter &param);

void preprocessing(int argc, char **argv);

void run_mrf_method(const MvsTexturing::MeshPtr mesh, const BVHTree &bvh_tree,
                    const MvsTexturing::Parameter &param, MvsTexturing::Base::LabelGraph &graph,
                    std::vector<MvsTexturing::Base::TextureView> &texture_views);

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

void plane_detection(const MvsTexturing::MeshPtr mesh, const Parameter &param, std::vector<FaceGroup> &planar_groups) {
    using namespace MvsTexturing;
    using PlaneGroup = MeshPolyRefinement::Base::PlaneGroup;

    TriMesh tri_mesh;
    Utils::mveMesh_to_triMesh(mesh, tri_mesh);
    {
        using namespace MeshPolyRefinement;
        // detect planes on mesh using region-growing
        PlaneEstimation::region_growing_plane_estimate(tri_mesh, param.planar_score,
                                                       param.angle_threshold,
                                                       param.ratio_threshold, param.min_plane_size);

        // expand plane segment regions
        PlaneEstimation::plane_region_expand(tri_mesh, 50.0, 45.0);

        // filter small non-plane regions
        PlaneEstimation::plane_region_refine(tri_mesh);

        // merge parallel adjacent plane segments
        PlaneEstimation::plane_region_merge(tri_mesh);

        for (std::size_t group_id = 0; group_id < tri_mesh.m_plane_groups.size(); group_id++) {
            PlaneGroup &group = tri_mesh.m_plane_groups[group_id];
            planar_groups.push_back(FaceGroup());

            for (std::size_t f_index: group.m_indices) {
                planar_groups.back().m_face_indices.push_back(f_index);
            }
            planar_groups.back().m_plane_center = group.m_plane_center;
            planar_groups.back().m_plane_normal = group.m_plane_normal;
        }
    }
}

bool texture_from_dense_to_sparse_model(
        const Parameter &param, const __inner__::Mesh &sparse_mesh, const __inner__::Mesh &dense_mesh,
        const TexturePatches &texture_patches, const std::vector<FaceGroup> &planar_groups,
        std::set<std::size_t> &irregular_patch_faces, const FaceSubdivisions &face_subdivisions,
        std::vector<TexturePatch::Ptr> *final_patches);

bool map_textures(MeshPtr input_mesh, MeshInfo &mesh_info, TextureViews &texture_views, const Parameter &param,
                  std::vector<FaceGroup> &origin_planar_groups,
                  std::set<std::size_t> &irregular_patch_faces,
                  const FaceSubdivisions &face_subdivisions,
                  const __inner__::Mesh &origin_mesh, const __inner__::Mesh &dense_mesh);

void evaluate_make_dense_configurations(const __inner__::Mesh &mesh, double &len_threshold, int &max_dense_faces) {
    using namespace MvsTexturing;
    double _min_x, _min_y, _min_z;
    double _max_x, _max_y, _max_z;

    {
        Base::AttributeMatrix p = mesh.m_vertices(0, Eigen::all);
        _min_x = p(0, 0);
        _min_y = p(0, 1);
        _min_z = p(0, 2);

        _max_x = p(0, 0);
        _max_y = p(0, 1);
        _max_z = p(0, 2);
    }

    for (int i = 0; i < mesh.m_faces.rows(); i++) {
        Base::AttributeMatrix p3 = mesh.m_vertices(mesh.m_faces.row(i), Eigen::all);

        for (int var_j = 0; var_j < 3; var_j++) {
            _min_x = std::min(_min_x, p3(var_j, 0));
            _min_y = std::min(_min_y, p3(var_j, 1));
            _min_z = std::min(_min_z, p3(var_j, 2));

            _max_x = std::max(_max_x, p3(var_j, 0));
            _max_y = std::max(_max_y, p3(var_j, 1));
            _max_z = std::max(_max_z, p3(var_j, 2));
        }
    }

    double split_count = 300;
    double x_max_range = (_max_x - _min_x);
    double y_max_range = (_max_y - _min_y);
    double z_max_range = (_max_z - _min_z);
    double max_range = std::max(std::max(x_max_range, y_max_range), z_max_range);

    len_threshold = max_range / split_count;
    max_dense_faces = 2500000;
}

int main(int argc, char **argv) {
    preprocessing(argc, argv);

    MvsTexturing::Parameter param;
    try {
        parse_args(argc, argv, param);
    } catch (std::invalid_argument &ia) {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (param.debug_mode) {
        spdlog::set_level(spdlog::level::debug);
    }
    LOG_INFO("###### MvsTexturing ------ args, texture_quality: {}", param.textureQuality);

    using namespace MvsTexturing;
    util::WallTimer whole_timer;

    // Read mesh files
    LOG_INFO("###### MvsTexturing ------ Load mesh ");
    __inner__::Mesh origin_mesh, dense_mesh;
    std::vector<FaceGroup> origin_planar_groups;
    std::set<std::size_t> irregular_patch_faces;
    std::vector<std::vector<std::size_t>> face_subdivisions;
    MeshPtr input_mesh;
    {
        util::WallTimer IO_timer;
        if (!MvsTexturing::IO::load_mesh_from_ply(param.input_mesh, origin_mesh.m_vertices, origin_mesh.m_faces,
                                                  origin_mesh.m_face_colors)) {
            LOG_ERROR(" - load failed");
            return 0;
        }

        LOG_INFO(" - mesh loaded, vertices: {0}, faces: {1}", origin_mesh.m_vertices.rows(),
                 origin_mesh.m_faces.rows());

        if (param.sparse_model) {
            {
                // check if face color is exists
                if (origin_mesh.m_face_colors.rows() <= 0 ||
                    (origin_mesh.m_face_colors.rows() != origin_mesh.m_faces.rows())) {
                    LOG_WARN(" - WARNING !!! the sparse model dose not have face color attribute");
                    origin_mesh.m_has_face_color = false;
                } else {
                    origin_mesh.m_has_face_color = true;
                }
            }

            // if model is colored, then take colors into consideration in remove duplicated faces step
            // else, only removing duplicated faces step was executed.

            // remove duplicated faces
            {
                std::size_t origin_faces = origin_mesh.m_faces.rows();
                MeshRepair::remove_duplicate_faces(origin_mesh.m_faces, origin_mesh.m_face_colors);
                std::size_t removed_faces = origin_mesh.m_faces.rows();
                LOG_INFO(" - remove duplicated faces : {}", (origin_faces - removed_faces));
            }

            // detect planes and init face group
            {
                if (origin_mesh.m_has_face_color) {
                    LOG_INFO(" - loading precomputed plane patches ... ");
                    IO_timer.reset();
                    // if face color exists, the plane groups have been computed
                    std::map<__inner__::Color, std::size_t> group_id_map;

                    for (std::size_t r = 0; r < origin_mesh.m_face_colors.rows(); r++) {
                        __inner__::Color color(origin_mesh.m_face_colors(r, 0),
                                               origin_mesh.m_face_colors(r, 1),
                                               origin_mesh.m_face_colors(r, 2));
                        if (color.m_r == 0 &&
                            color.m_g == 0 &&
                            color.m_b == 0) {
                            irregular_patch_faces.insert(r);
                            continue;
                        }

                        if (group_id_map.find(color) == group_id_map.end()) {
                            origin_planar_groups.push_back(FaceGroup());
                            origin_planar_groups.back().m_face_indices.push_back(r);
                            group_id_map[color] = origin_planar_groups.size() - 1;
                        } else {
                            origin_planar_groups[group_id_map[color]].m_face_indices.push_back(r);
                        }
                    }

                    for (FaceGroup &group: origin_planar_groups) {
                        MeshRepair::fit_face_group_plane(origin_mesh.m_vertices, origin_mesh.m_faces, group);
                    }
                    LOG_INFO(" - done. {} plane patches, {} irregular faces loaded", origin_planar_groups.size(),
                             irregular_patch_faces.size());
                } else {
                    LOG_INFO(" - computing face group ... ");
                    IO_timer.reset();

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

                    origin_planar_groups.resize(sparse_tri_mesh.m_plane_groups.size());
                    for (std::size_t group_idx = 0; group_idx < sparse_tri_mesh.m_plane_groups.size(); group_idx++) {
                        const MeshPolyRefinement::Base::PlaneGroup &group = sparse_tri_mesh.m_plane_groups[group_idx];
                        FaceGroup &dest_group = origin_planar_groups[group_idx];

                        for (std::size_t group_face_idx: group.m_indices) {
                            dest_group.m_face_indices.push_back(group_face_idx);
                        }

                        dest_group.m_plane_normal = group.m_plane_normal;
                        dest_group.m_plane_center = group.m_plane_center;
                        dest_group.m_x_axis = group.m_x_axis;
                        dest_group.m_y_axis = group.m_y_axis;
                    }

                    for (std::size_t r = 0; r < sparse_tri_mesh.m_face_plane_index.rows(); r++) {
                        int plane_index = sparse_tri_mesh.m_face_plane_index(r, 0);
                        if (plane_index < 0) {
                            irregular_patch_faces.insert(r);
                        }
                    }

                    LOG_INFO(" - done. {} plane patches is computed", origin_planar_groups.size());
                }
            }

            // make mesh dense
            {
                double len_threshold = 0.05;
                int max_dense_faces = 2000000;
                evaluate_make_dense_configurations(origin_mesh, len_threshold, max_dense_faces);

                IO_timer.reset();
                LOG_INFO(" - model is too sparse, make the model dense ...");
                LOG_DEBUG(" - make dense configurations, edge length threshold: {}, max dense faces: {}",
                          len_threshold, max_dense_faces);
                MeshSubdivision::make_mesh_dense(origin_mesh.m_vertices, origin_mesh.m_faces, dense_mesh.m_vertices,
                                                 dense_mesh.m_faces, origin_mesh.m_face_colors,
                                                 dense_mesh.m_face_colors, len_threshold, max_dense_faces);
                LOG_INFO(" - done, get the dense model, vertices: {}, faces: {}",
                         dense_mesh.m_vertices.rows(), dense_mesh.m_faces.rows());
            }

            LOG_INFO(" - computing mappings between origin and dense models ... ");
            // compute the relations between origin and dense mesh
            {
                face_subdivisions.resize(origin_mesh.m_faces.rows());
                std::map<__inner__::Color, std::size_t> tmp_color_face_map;
                for (int i = 0; i < origin_mesh.m_face_colors.rows(); i++) {
                    const __inner__::Color face_color(
                            origin_mesh.m_face_colors(i, 0),
                            origin_mesh.m_face_colors(i, 1),
                            origin_mesh.m_face_colors(i, 2));
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

                // filter spilt faces
                for (auto it = irregular_patch_faces.begin(); it != irregular_patch_faces.end();) {
                    std::size_t face_index = (*it);
                    if (face_subdivisions[face_index].size() > 1) {
                        origin_planar_groups.push_back(FaceGroup());
                        origin_planar_groups.back().m_face_indices.push_back(face_index);
                        MeshRepair::fit_face_group_plane(origin_mesh.m_vertices, origin_mesh.m_faces,
                                                         origin_planar_groups.back());
                        irregular_patch_faces.erase(it++);
                    } else {
                        it++;
                    }
                }
            }
            LOG_INFO(" - compute mappings done");

            if (param.debug_mode) {
                const std::string DebugMode_DenseMesh_Path =
                        Utils::str_prefix(param.output_prefix) + "_DebugMode_dense-model.ply";
                IO::save_ply_mesh(DebugMode_DenseMesh_Path, dense_mesh.m_vertices, dense_mesh.m_faces);
                LOG_DEBUG(" - save the dense model: {}", DebugMode_DenseMesh_Path);
            }

            input_mesh = Utils::eigenMesh_to_mveMesh(dense_mesh.m_vertices, dense_mesh.m_faces);
        } else {
            input_mesh = Utils::eigenMesh_to_mveMesh(origin_mesh.m_vertices, origin_mesh.m_faces);
        }

        if (input_mesh == nullptr || (input_mesh->get_faces().size() % 3 != 0)) {
            LOG_ERROR(" - could not load mesh ");
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

    if (param.debug_mode) {
        param.debug_dir = util::fs::join_path(output_dir, "debug_mode");
        param.debug_primary_patch_dir = util::fs::join_path(param.debug_dir, "primary_patch");
        param.debug_remapping_patch_dir = util::fs::join_path(param.debug_dir, "remapping_patch");

        if (!util::fs::dir_exists(param.debug_dir.c_str())) {
            util::fs::mkdir(param.debug_dir.c_str());
        }

        if (!util::fs::dir_exists(param.debug_primary_patch_dir.c_str())) {
            util::fs::mkdir(param.debug_primary_patch_dir.c_str());
        }

        if (!util::fs::dir_exists(param.debug_remapping_patch_dir.c_str())) {
            util::fs::mkdir(param.debug_remapping_patch_dir.c_str());
        }
    }


    // Read camera images
    LOG_INFO("###### MvsTexturing ------ Load scene ");
    std::vector<TextureView> texture_views;
    {
        util::WallTimer IO_timer;
        LOG_INFO(" - read camera poses and images from {} ... ", param.scene_file);
        Builder::build_scene(param.scene_file, &texture_views, temp_dir);
        LOG_INFO(" - done. {} images loaded", texture_views.size());
    }

    if (!map_textures(input_mesh, mesh_info, texture_views, param, origin_planar_groups,
                      irregular_patch_faces, face_subdivisions, origin_mesh, dense_mesh)) {
        LOG_ERROR(" - texture mapping failed");
        return 0;
    }

    LOG_INFO("MvsTexturing done");
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
            ("texture_quality", bpo::value<float>()->default_value(1.0), "texture quality [1.0]")
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
    param.textureQuality = vm["texture_quality"].as<float>();
    if (param.textureQuality <= 0) {
        param.textureQuality = 0;
    } else if (param.textureQuality > 1.0) {
        param.textureQuality = 1.0;
    }

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
                  const Parameter &param, std::vector<FaceGroup> &origin_planar_groups,
                  std::set<std::size_t> &irregular_patch_faces,
                  const FaceSubdivisions &face_subdivisions,
                  const __inner__::Mesh &origin_mesh, const __inner__::Mesh &dense_mesh) {
    // Build adjacency graph
    LOG_INFO("###### MvsTexturing ------ Build adjacency graph ");
    LOG_INFO(" - computing adjacency graph ... ");
    std::size_t const n_faces = input_mesh->get_faces().size() / 3;
    LabelGraph graph(n_faces);
    MvsTexturing::Builder::MVE::build_adjacency_graph(input_mesh, mesh_info, &graph);
    LOG_INFO(" - done. adjacency graph info: {0} edges, {1} faces", graph.num_edges(), graph.num_nodes());

    LOG_INFO("###### MvsTexturing ------ View selection ");
    {
        if (param.labeling_file.empty()) {
            namespace VS = MvsTexturing::ViewSelection;

            // build bvh tree
            LOG_INFO(" - building BVH from {} faces ... ", n_faces);
            BVHTree bvh_tree(input_mesh->get_faces(), input_mesh->get_vertices());
            LOG_INFO(" - BVH tree done.");

            if (param.method_type == "mrf") {
                LOG_INFO(" - MRF algorithm started ... ");
                run_mrf_method(input_mesh, bvh_tree, param, graph, texture_views);
                LOG_INFO(" - MRF optimization done");
            } else if (param.method_type == "projection") {
                LOG_INFO(" - projection algorithm started ... ");
                // run projection method
                {
                    if (param.sparse_model) {
                        std::vector<FaceGroup> mesh_planar_groups;
                        for (const FaceGroup &origin_group: origin_planar_groups) {
                            mesh_planar_groups.push_back(FaceGroup());
                            FaceGroup &dense_group = mesh_planar_groups.back();

                            for (std::size_t origin_f_index: origin_group.m_face_indices) {
                                for (std::size_t dense_f_index: face_subdivisions[origin_f_index]) {
                                    dense_group.m_face_indices.push_back(dense_f_index);
                                }
                            }

                            dense_group.m_plane_center = origin_group.m_plane_center;
                            dense_group.m_plane_normal = origin_group.m_plane_normal;
                        }
                        // 额外的纹理参数：mesh_planar_groups，表明输入的纹理经过均匀细分
                        VS::Projection::solve_projection_problem(input_mesh, mesh_info, bvh_tree, mesh_planar_groups,
                                                                 graph, texture_views, param);
                    } else {
                        VS::Projection::solve_projection_problem(input_mesh, mesh_info, bvh_tree,
                                                                 graph, texture_views, param);
                    }
                }
                LOG_INFO(" - projection optimization done");
            } else if (param.method_type == "region_growing") {
                LOG_INFO(" - region growing algorithm started ... ");
                VS::RegionGrowing::solve_RegionGrowing_problem(input_mesh, bvh_tree, param, texture_views, graph);
                LOG_INFO(" - region growing algorithm done");
            } else if (param.method_type == "optimal") {
                LOG_INFO(" - run optimal algorithm started ... ");
                VS::Projection::solve_OptimalPerFace(input_mesh, mesh_info, bvh_tree, graph, texture_views, param);
                LOG_INFO(" - run optimal algorithm done");
            } else if (param.method_type == "submrf") {
                LOG_INFO(" - Sub-MRF algorithm started ... ");
                if (param.sparse_model) {
                    std::vector<FaceGroup> mesh_planar_groups;
                    for (const FaceGroup &origin_group: origin_planar_groups) {
                        mesh_planar_groups.push_back(FaceGroup());
                        FaceGroup &dense_group = mesh_planar_groups.back();

                        for (std::size_t origin_f_index: origin_group.m_face_indices) {
                            for (std::size_t dense_f_index: face_subdivisions[origin_f_index]) {
                                dense_group.m_face_indices.push_back(dense_f_index);
                            }
                        }

                        dense_group.m_plane_center = origin_group.m_plane_center;
                        dense_group.m_plane_normal = origin_group.m_plane_normal;
                    }
                    VS::Mrf::solveMultipleMrfProblem(input_mesh, mesh_info, param, bvh_tree,
                                                     graph, texture_views, mesh_planar_groups);
                } else {
                    if (origin_planar_groups.size() > 0) {
                        VS::Mrf::solveMultipleMrfProblem(input_mesh, mesh_info, param, bvh_tree,
                                                         graph, texture_views, origin_planar_groups);
                    } else {
                        std::vector<FaceGroup> planar_groups;
                        plane_detection(input_mesh, param, planar_groups);
                        VS::Mrf::solveMultipleMrfProblem(input_mesh, mesh_info, param, bvh_tree,
                                                         graph, texture_views, planar_groups);
                    }
                }
                LOG_INFO(" - Sub-MRF optimization done");
            } else {
                LOG_ERROR(" - view selection method not supported: {}", param.method_type);
                return false;
            }
        } else {
            std::cout << "Loading labeling from file... " << std::flush;

            /* Load labeling from file. */
            std::vector<std::size_t> labeling = MvsTexturing::Utils::vector_from_file<std::size_t>(
                    param.labeling_file);
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
    }

    LOG_INFO("###### MvsTexturing ------ Generating Patches ");
    std::vector<TexturePatch::Ptr> texture_patches;
    {
        using namespace MvsTexturing;
        // Create texture patches and adjust them
        std::vector<std::vector<Base::VertexProjectionInfo>> vertex_projection_infos;
        LOG_INFO(" - generate primary texture patches ... ");
        AtlasMapper::generate_texture_patches(graph, input_mesh, mesh_info, &texture_views,
                                              param, &vertex_projection_infos, &texture_patches);
        LOG_INFO(" - {} patches created", texture_patches.size());

        if (texture_patches.size() <= 0) {
            LOG_WARN(" - no primary texture generated");
        }

        if (param.method_type == "projection" || param.sparse_model == true) {
            for (TexturePatch::Ptr patch: texture_patches) {
                // Image sharpen
//                patch->get_image()->sharpen();
            }
        }

        if (param.debug_mode) {
            if (texture_patches.size() > 0) {
                TexturePatch::Ptr first_texture_patch = texture_patches[0];
                const std::string output_name = util::fs::join_path(param.debug_primary_patch_dir, "0_.png");
                mve::image::save_png_file(mve::image::float_to_byte_image(first_texture_patch->get_image()),
                                          output_name);
                LOG_DEBUG(" - save first primary patch: {}", output_name);
            }
        }

        bool global_seam_leveling_success = false;
        if (!param.skip_global_seam_leveling) {
            LOG_INFO(" - global seam leveling started ... ");
            global_seam_leveling_success = SeamSmoother::global_seam_leveling(
                    graph, input_mesh, mesh_info, vertex_projection_infos, &texture_patches);
            LOG_INFO(" - global seam leveling done");
        }

        if ((!global_seam_leveling_success) || param.skip_global_seam_leveling) {
            LOG_INFO(" - calculating validity masks for texture patches... ");
#pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches.size(); ++i) {
                Base::TexturePatch::Ptr texture_patch = texture_patches[i];
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
            }
            LOG_INFO(" - calculate validity masks done");
        }

        if (param.debug_mode) {
            if (texture_patches.size() > 0) {
                TexturePatch::Ptr first_texture_patch = texture_patches[0];
                const std::string output_name = util::fs::join_path(param.debug_primary_patch_dir,
                                                                    "0_AdjustColor_.png");
                mve::image::save_png_file(mve::image::float_to_byte_image(first_texture_patch->get_image()),
                                          output_name);
                LOG_DEBUG(" - save first adjusted color patch: {}", output_name);
            }
        }

        if (!param.skip_local_seam_leveling) {
            LOG_INFO(" - local seam leveling started ... ");
            SeamSmoother::local_seam_leveling(graph, input_mesh, vertex_projection_infos, &texture_patches);
            LOG_INFO(" - local seam leveling done");

            if (param.debug_mode) {
                if (texture_patches.size() > 0) {
                    TexturePatch::Ptr first_texture_patch = texture_patches[0];
                    const std::string output_name = util::fs::join_path(param.debug_primary_patch_dir,
                                                                        "0_SeamLeveling_.png");
                    mve::image::save_png_file(mve::image::float_to_byte_image(first_texture_patch->get_image()),
                                              output_name);
                    LOG_DEBUG(" - save first seam leveling patch: {}", output_name);
                }
            }
        }

        if (param.sparse_model) {
            std::vector<TexturePatch::Ptr> final_texture_patches;
            LOG_INFO(" - retrieve sparse model texture from dense model ... ");
            bool ret = texture_from_dense_to_sparse_model(
                    param, origin_mesh, dense_mesh, texture_patches, origin_planar_groups,
                    irregular_patch_faces, face_subdivisions, &final_texture_patches);

            if (!ret) {
                LOG_ERROR(" - map_textures() : texture simplify failed");
                return false;
            }
            texture_patches.swap(final_texture_patches);
            LOG_INFO(" - done. {} sparse texture patches generated.", texture_patches.size());

            if (param.debug_mode) {
                if (texture_patches.size() > 0) {
                    int patch_index = 0;
                    int max_area = 0;
                    for (int i = 0; i < texture_patches.size(); i++) {
                        TexturePatch::Ptr patch = texture_patches[i];
                        const int area = patch->get_width() * patch->get_height();
                        if (max_area < area) {
                            max_area = area;
                            patch_index = i;
                        }
                    }
                    std::stringstream ss;
                    ss << patch_index << "_.png";

                    std::string output_name;
                    ss >> output_name;
                    output_name = util::fs::join_path(param.debug_remapping_patch_dir, output_name);

                    mve::image::save_png_file(
                            mve::image::float_to_byte_image(texture_patches[patch_index]->get_image()), output_name);
                    LOG_DEBUG(" - save remapping patch: {}", output_name);
                }
            }
        }
    }

    LOG_INFO("###### MvsTexturing ------ Generating Atlases ");
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        using namespace MvsTexturing;
        util::WallTimer timer;
        LOG_INFO(" - atlases are generating ... ");
        AtlasMapper::generate_texture_atlases(param, &texture_patches, &texture_atlases,
                                              param.tone_mapping == Tone_Mapping_Gamma);
        LOG_INFO(" - done, {} atlases generated.", texture_atlases.size());
    }

    LOG_INFO("###### MvsTexturing ------ Write obj model");
    {
        LOG_INFO(" - writing model ...");
        util::WallTimer timer;
        if (!param.sparse_model) {
            MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, input_mesh, texture_atlases);
        } else {
            input_mesh = MvsTexturing::Utils::eigenMesh_to_mveMesh(origin_mesh.m_vertices, origin_mesh.m_faces);
            MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, input_mesh, texture_atlases);
        }
        LOG_INFO(" - done: {}.obj", param.output_prefix);
    }

    return true;
}

bool texture_from_dense_to_sparse_model(
        const Parameter &param, const __inner__::Mesh &sparse_mesh, const __inner__::Mesh &dense_mesh,
        const TexturePatches &texture_patches, const std::vector<FaceGroup> &planar_groups,
        std::set<std::size_t> &irregular_patch_faces, const FaceSubdivisions &face_subdivisions,
        std::vector<TexturePatch::Ptr> *final_patches) {
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
                LOG_ERROR(" - size of faces and texcoords in texture patch is not match");
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

    int ret = MvsTexturing::MeshRepair::create_plane_patches_on_sparse_mesh(
            param, sparse_mesh.m_vertices, sparse_mesh.m_faces, planar_groups,
            dense_mesh.m_vertices, dense_mesh.m_faces, dense_mesh_face_texture_coords, dense_mesh_face_materials,
            face_subdivisions, final_patches, Base::TexturePatch::kTexturePatchPadding,
            param.textureQuality);

    if (!ret) {
        LOG_ERROR(" - texture_from_dense_to_sparse_model() : create plane patches failed");
        return false;
    }
    const int n_plane_patches_size = final_patches->size();
    LOG_DEBUG(" - done, {} plane patches created. ", final_patches->size());

    ret = MeshRepair::create_irregular_patches_on_sparse_mesh(
            sparse_mesh.m_vertices, sparse_mesh.m_faces, irregular_patch_faces,
            dense_mesh_face_texture_coords, dense_mesh_face_materials,
            face_subdivisions, final_patches, Base::TexturePatch::kTexturePatchPadding);
    if (!ret) {
        LOG_ERROR(" - texture_from_dense_to_sparse_model() : create irregular patches failed");
        return false;
    }
    LOG_DEBUG(" - done, {} irregular patches created. ", final_patches->size() - n_plane_patches_size);
    return true;
}
