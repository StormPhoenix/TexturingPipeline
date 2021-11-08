//
// Created by Storm Phoenix on 2021/10/22.
//
#include <iostream>
#include <sstream>

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
#include <TextureMapper/SceneBuilder.h>
#include <Base/TexturePatch.h>
#include <Base/TextureAtlas.h>
#include <TextureMapper/AtlasMapper.h>

#include <util/timer.h>

#include "PlaneMergeUtils.h"

#define PLANE_DENSITY 650

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> AttributeMatrix;
typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
using Vec3 = Eigen::Matrix<double, 1, 3>;

template<typename T>
struct NopeFunctor {
    NopeFunctor() {}

    void operator()(T a, T b) {
    }

    void operator()() {
    }
};

template<typename T>
struct DifferenceCounterFunctor {
    double _m_total_score = 0.0;
    std::size_t _m_counter = 0;

    DifferenceCounterFunctor() : _m_total_score(0.0), _m_counter(0) {}

    void operator()(T a, T b) {
        if (a == 0 || b == 0) {
            return;
        }
        int difference = std::abs((int(a) - int(b)));
        _m_total_score += difference;
        _m_counter++;
    }

    void operator()() {
        if (_m_counter == 0) {
            std::cout << "Total score: none counter.\n" << std::endl;
        } else {
            std::cout << "Total score: " << (_m_total_score / _m_counter) << std::endl << std::endl;
        }
    }
};

template<typename F>
void compute_difference_img(const int test_camera_id_1_, const int test_camera_id_2_, F func) {
    std::string proj_image_1_name;
    std::string proj_image_2_name;
    {
        // calculate input image names
        std::stringstream ss;
        ss << "./Output_PlaneMerge/Debug_Group_9_Camera_" << test_camera_id_1_ << "_.png";

        ss >> proj_image_1_name;
        std::cout << "\tRead: " << proj_image_1_name << std::endl;

        ss.clear();
        ss << "./Output_PlaneMerge/Debug_Group_9_Camera_" << test_camera_id_2_ << "_.png";
        ss >> proj_image_2_name;
        std::cout << "\tRead: " << proj_image_2_name << std::endl;
    }


    auto _1_img = mve::image::load_png_file(proj_image_1_name);
    auto _2_img = mve::image::load_png_file(proj_image_2_name);
    assert(_1_img != nullptr && _2_img != nullptr);

    const std::size_t image_width = _1_img->width();
    const std::size_t image_height = _1_img->height();
    assert(image_width == _2_img->width() && image_height == _2_img->height());

    // calculate difference image
    auto final_img = mve::ByteImage::create(image_width, image_height, 3);
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < image_height; y++) {
            for (int c = 0; c < 3; c++) {
                final_img->at(x, y, c) = (unsigned char) std::abs(
                        int(_1_img->at(x, y, c)) - int(_2_img->at(x, y, c)));
                func(_1_img->at(x, y, c), _2_img->at(x, y, c));
            }
        }
    }

    std::string output_file_name;
    {
        // calculate output image names
        std::stringstream ss;
        ss << "./Output_PlaneMerge/Group_" << test_camera_id_1_ << "_" << test_camera_id_2_ << "_sub_.png";
        ss >> output_file_name;
    }
    mve::image::save_png_file(final_img, output_file_name);
    func();
}

int main(int argc, char **argv) {
    namespace bpo = boost::program_options;
    bpo::options_description opts("Example test MeshRemeshing options");
    bpo::variables_map vm;

    opts.add_options()
            ("help", "produce help message")
            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
            ("output_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of output dense mesh")
            ("scene_file", bpo::value<std::string>()->default_value("model.nvm"), "path of scene file (*.nvm)")
            ("distort_dir", bpo::value<std::string>()->default_value("tmp"), "path of temporary director")
            ("planar_score", bpo::value<double>()->default_value(0.3),
             "minimal planar score of face during plane estimation, default 0.8")
            ("angle", bpo::value<double>()->default_value(30.0),
             "maximal angle in degree that plane segment should accept face, default 15.0")
            ("ratio", bpo::value<double>()->default_value(20.0),
             "maximal distance ratio that plane segment should accept face, default 10.0")
            ("plane_size", bpo::value<int>()->default_value(10),
             "minimal number of facets in plane segment, default 10");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        std::cout << "undefine options in command lines.\n";
        return 0;
    }

    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();
    const std::string scene_file = vm["scene_file"].as<std::string>();
    const std::string distort_dir = vm["distort_dir"].as<std::string>();

    /*
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


//    IO::repair_non_manifold(mesh);
    IO::save_mesh_plane_segments(out_mesh_path, mesh);

    std::cout << "\n### PlaneTextureMerge------Load Texture images" << std::endl;
    std::vector<MvsTexturing::Base::TextureView> camera_views;
    {
        util::WallTimer timer;
        std::cout << "\tLoad images...";

        using namespace MvsTexturing;
        Builder::build_scene(scene_file, &camera_views, distort_dir);

        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    std::cout << "\n### PlaneTextureMerge------Generate Texture Patches" << std::endl;
    std::vector<MvsTexturing::Base::TexturePatch> texture_patch;
    {
        util::WallTimer timer;
        std::cout << "\tGenerate texture patches...";
        PlaneTexMerge::Utils::generate_plane_projection_patch(mesh, camera_views, &texture_patch, 10, 100);
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }
     */

    std::cout << "\n### PlaneTextureMerge------Patch image substract" << std::endl;
    const int test_camera_id_1_ = 36;
    for (int test_camera_id_2_ = 37; test_camera_id_2_ <= 60; test_camera_id_2_++) {
        compute_difference_img(test_camera_id_1_, test_camera_id_2_, DifferenceCounterFunctor<unsigned char>());
    }

    std::cout << "PlaneTextureMerge------done. " << std::endl;
    return 0;
}