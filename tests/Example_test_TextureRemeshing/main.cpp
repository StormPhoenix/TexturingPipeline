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
#include <IO/ObjModel.h>
#include <Base/TexturePatch.h>
#include <Base/TextureAtlas.h>
#include <TextureMapper/AtlasMapper.h>

#include <util/timer.h>

#include "RemeshingUtils.h"

#define PLANE_DENSITY 300

// TODO delete
#include <util/file_system.h>

// TODO Rename
bool texture_patches_from_group(const MeshPolyRefinement::Base::TriMesh &mesh,
                                std::vector<MvsTexturing::Base::TexturePatch::Ptr> &texture_patches,
                                std::vector<math::Vec2f> &global_texcoords,
                                std::vector<std::size_t> &global_texcoord_ids,
                                std::vector<std::string> &face_materials,
                                std::map<std::string, mve::ByteImage::Ptr> &material_image_map,
                                int padding_pixels = 10) {
    using namespace MeshPolyRefinement;
    for (std::size_t g_idx = 0; g_idx < mesh.m_plane_groups.size(); g_idx++) {
        const Base::PlaneGroup &group = mesh.m_plane_groups[g_idx];

        // Compute 3D uv
        Base::Scalar max_dx, min_dx = 0;
        Base::Scalar max_dy, min_dy = 0;
        const std::size_t n_faces = group.m_indices.size();
        std::vector<math::Vec2f> texcoords;
        texcoords.resize(n_faces * 3);

//#pragma omp parallel for schedule(dynamic)
        for (std::size_t f_i = 0; f_i < n_faces; f_i++) {
            std::size_t face_idx = group.m_indices[f_i];
            Base::AttributeMatrix points_3 = mesh.m_vertices(mesh.m_faces.row(face_idx), Eigen::all);

//            std::cout << "Debug points 3: " << points_3 << std::endl;
            Base::Vec3 d0 = points_3.row(0) - group.m_plane_center;
            Base::Vec3 d1 = points_3.row(1) - group.m_plane_center;
            Base::Vec3 d2 = points_3.row(2) - group.m_plane_center;

            Base::Scalar dx0 = d0.dot(group.m_x_axis);

            Base::Scalar dx1 = d1.dot(group.m_x_axis);
            Base::Scalar dx2 = d2.dot(group.m_x_axis);

            Base::Scalar dy0 = d0.dot(group.m_y_axis);
            Base::Scalar dy1 = d1.dot(group.m_y_axis);
            Base::Scalar dy2 = d2.dot(group.m_y_axis);

            max_dx = std::max(max_dx, std::max(dx0, std::max(dx1, dx2)));
            min_dx = std::min(min_dx, std::min(dx0, std::min(dx1, dx2)));

            max_dy = std::max(max_dy, std::max(dy0, std::max(dy1, dy2)));
            min_dy = std::min(min_dy, std::min(dy0, std::min(dy1, dy2)));

            texcoords[f_i * 3 + 0] = {dx0, dy0};
            texcoords[f_i * 3 + 1] = {dx1, dy1};
            texcoords[f_i * 3 + 2] = {dx2, dy2};
        }

        Base::Scalar d_width = max_dx - min_dx;
        Base::Scalar d_height = max_dy - min_dy;

        // Create images
        std::size_t image_width = d_width * PLANE_DENSITY + 2 * padding_pixels;
        std::size_t image_height = d_height * PLANE_DENSITY + 2 * padding_pixels;
        mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

        // TODO delete
        float random_color[3];
        Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
        random_color[0] = double(color(0));
        random_color[1] = double(color(1));
        random_color[2] = double(color(2));

        patch_image->fill_color(random_color);

        // Re-scale texture coords
        double padding = double(padding_pixels) / PLANE_DENSITY;
#pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < texcoords.size(); i++) {
            if ((texcoords[i][0] - min_dx + padding) < 0 || (texcoords[i][1] - min_dy + padding) < 0 ){
                std::cout << "Debug coord: \n\t" << texcoords[i][0] << ", " << texcoords[i][1] << std::endl;
                std::cout << "\t" << min_dx << ", " << min_dy << std::endl;
                std::cout << "\t" << texcoords[i][0] - min_dx << ", " << texcoords[i][1] - min_dy << std::endl;
                std::cout << "\t" << texcoords[i][0] - min_dx + padding << ", " << texcoords[i][1] - min_dy + padding << std::endl;
            }

            texcoords[i][0] = (texcoords[i][0] - min_dx + padding) * PLANE_DENSITY;
            texcoords[i][1] = (texcoords[i][1] - min_dy + padding) * PLANE_DENSITY;

            if (texcoords[i][0] < 0 || texcoords[i][1] < 0) {
                std::cout << "Debug: vt0 " << texcoords[i][0] << " vt1 " << texcoords[i][1] << std::endl;
            }
        }

        // Copy src images
        for (std::size_t i = 0; i < texcoords.size(); i += 3) {
            std::size_t f_i = i / 3;
            std::size_t f_idx = group.m_indices[f_i];

            mve::ByteImage::Ptr src_image = material_image_map[face_materials[f_idx]];
            const int src_width = src_image->width();
            const int src_height = src_image->height();

            math::Vec2f src_v1 = global_texcoords[global_texcoord_ids[f_idx * 3 + 0]];
            src_v1[0] *= src_width;
            src_v1[1] *= src_height;

            math::Vec2f src_v2 = global_texcoords[global_texcoord_ids[f_idx * 3 + 1]];
            src_v2[0] *= src_width;
            src_v2[1] *= src_height;

            math::Vec2f src_v3 = global_texcoords[global_texcoord_ids[f_idx * 3 + 2]];
            src_v3[0] *= src_width;
            src_v3[1] *= src_height;

            using namespace MvsTexturing;

            math::Vec2f v1 = texcoords[i];
            math::Vec2f v2 = texcoords[i + 1];
            math::Vec2f v3 = texcoords[i + 2];
            Math::Tri2D tri(v1, v2, v3);
            float area = tri.get_area();
            if (area < std::numeric_limits<float>::epsilon()) { continue; }

            Math::Rect2D<float> aabb = tri.get_aabb();
            int const min_tri_x = static_cast<int>(std::floor(aabb.min_x));
            int const min_tri_y = static_cast<int>(std::floor(aabb.min_y));
            int const max_tri_x = static_cast<int>(std::ceil(aabb.max_x));
            int const max_tri_y = static_cast<int>(std::ceil(aabb.max_y));

#pragma omp parallel for schedule(dynamic)
            for (std::size_t x = min_tri_x; x <= max_tri_x; x++) {
                for (std::size_t y = min_tri_y; y <= max_tri_y; y++) {
                    math::Vec3f bcoords = tri.get_barycentric_coords(x, y);
                    if (bcoords.minimum() >= 0.0f) {
                        math::Vec2f src_coord = {
                                src_v1[0] * bcoords[0] + src_v2[0] * bcoords[1] + src_v3[0] * bcoords[2],
                                src_v1[1] * bcoords[0] + src_v2[1] * bcoords[1] + src_v3[1] * bcoords[2]
                        };
                        for (int c = 0; c < 3; c++) {
                            unsigned char src_color = src_image->at(src_coord[0], src_coord[1], c);
                            patch_image->at(x, y, c) = std::min(1.0f, std::max(0.0f, ((float) src_color) / 255.0f));
                        }
                    } else {
                        continue;
                    }
                }
            }
        }

        // Create texture patch
        MvsTexturing::Base::TexturePatch::Ptr patch = MvsTexturing::Base::TexturePatch::create(0, group.m_indices,
                                                                                               texcoords, patch_image);
        texture_patches.push_back(patch);
    }
    return true;
}

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
             "minimal number of facets in plane segment, default 10");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        std::cout << "undefine options in command lines.\n";
        return 0;
    }
    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();


    std::vector<math::Vec3f> mesh_vertices;
    std::vector<math::Vec3f> mesh_normals;
    std::vector<math::Vec2f> mesh_texcoords;
    std::vector<std::size_t> mesh_faces;
    std::vector<std::size_t> mesh_normal_ids, mesh_texcoord_ids;
    std::vector<std::string> face_materials;
    std::map<std::string, std::string> material_map;

    MvsTexturing::IO::load_mesh_from_obj(in_mesh_path, mesh_vertices, mesh_normals, mesh_texcoords,
                                         mesh_faces, mesh_normal_ids, mesh_texcoord_ids,
                                         face_materials, material_map);

    MeshPolyRefinement::Base::TriMesh mesh;
    {
        std::vector<double> tmp_vertices(mesh_vertices.size() * 3);
        for (int i = 0; i < mesh_vertices.size(); i++) {
            for (int j = 0; j < 3; j++) {
                tmp_vertices[i * 3 + j] = mesh_vertices[i][j];
            }
        }

        MeshPolyRefinement::IO::read_mesh_from_memory(tmp_vertices, mesh_faces, mesh);
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

    // TODO delete
//    IO::repair_non_manifold(mesh);
//    IO::save_mesh_plane_segments(out_mesh_path, mesh);

    std::cout << "\n### TextureRemeshing------Load Texture images" << std::endl;
    std::map<std::string, mve::ByteImage::Ptr> material_image_map;
    {
        util::WallTimer timer;
        std::cout << "\tLoad images...";

        for (auto it = material_map.begin(); it != material_map.end(); it++) {
            const std::string image_file_name = it->second;
            material_image_map[it->first] = mve::image::load_file(image_file_name);
        }

        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    std::cout << "\n### TextureRemeshing------Generate Texture Patches" << std::endl;
    std::vector<MvsTexturing::Base::TexturePatch::Ptr> texture_patches;
    {
        std::cout << "\tGenerating texture patches..." << std::flush;
        util::WallTimer timer;
        // TODO texture_patch 的生成做并行化
        texture_patches_from_group(mesh, texture_patches,
                                   mesh_texcoords, mesh_texcoord_ids,
                                   face_materials, material_image_map);
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;

        timer.reset();
        std::cout << "\tCalculating texture patch validity mask...";
#pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < texture_patches.size(); ++i) {
            MvsTexturing::Base::TexturePatch::Ptr texture_patch = texture_patches[i];
            std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
            texture_patch->adjust_colors(patch_adjust_values);
        }
        std::cout << " done. (Took: " << timer.get_elapsed_sec() << "s)" << std::endl;
    }

    std::cout << "\n### TextureRemeshing------Generate Texture Atlases" << std::endl;
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> texture_atlases;
    {
        MvsTexturing::AtlasMapper::generate_texture_atlases(&texture_patches, &texture_atlases);
    }

    std::cout << "\n### TextureRemeshing------Write obj model" << std::endl;
    {
        // TODO *.obj 文件写入暂时不添加 vertex_normal
        mve::TriangleMesh::Ptr temp_mesh = TextureRemeshing::Utils::triMesh_to_mveMesh(mesh);
        MvsTexturing::IO::MVE::save_obj_mesh(out_mesh_path, temp_mesh, texture_atlases);
    }
    std::cout << "Texture remeshing done. " << std::endl;
    return 0;
}