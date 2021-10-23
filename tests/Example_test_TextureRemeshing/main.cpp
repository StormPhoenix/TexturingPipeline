//
// Created by Storm Phoenix on 2021/10/22.
//
#include <iostream>

#include <boost/program_options.hpp>

#include <mve/mesh_io_ply.h>
#include <mve/image.h>

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

// TODO Rename
bool texture_patches_from_group(const MeshPolyRefinement::Base::TriMesh &mesh,
                                std::vector<MvsTexturing::Base::TexturePatch::Ptr> &texture_patches,
                                int padding_pixels = 10) {
    using namespace MeshPolyRefinement;
    for (std::size_t g_idx = 0; g_idx < mesh.m_plane_groups.size(); g_idx++) {
        const Base::PlaneGroup &group = mesh.m_plane_groups[g_idx];

//        std::cout << "Debug m_x_axis: " << group.m_x_axis << std::endl;
//        std::cout << "Debug m_x_axis len: " <<
//        group.m_x_axis[0] * group.m_x_axis[0]
//        + group.m_x_axis[1] * group.m_x_axis[1]
//        + group.m_x_axis[2] * group.m_x_axis[2] << std::endl;

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
//            std::cout << "Debug p0: " << p0 << std::endl;
//            std::cout << "Debug p1: " << p1 << std::endl;
//            std::cout << "Debug p2: " << p2 << std::endl;

            Base::Scalar dx0 = d0.dot(group.m_x_axis);

            /*
            std::cout << "Debug d0       : " << d0 << std::endl;
            std::cout << "Debug m_x_axis : " << group.m_x_axis << std::endl;
            std::cout << "Debug manual   : \n" << d0(0, 0) * group.m_x_axis(0, 0) +
                                                  d0(0, 1) * group.m_x_axis(0, 1) +
                                                  d0(0, 2) * group.m_x_axis(0, 2) << std::endl;
            std::cout << "Debug dot  - " << d0.dot(group.m_x_axis.transpose()) << std::endl;
            std::cout << "Debug dot T- " << d0.dot(group.m_x_axis) << std::endl;
            std::cout << "Debug T *  - " << d0 * group.m_x_axis.transpose() << std::endl;
             */
//            std::cout << "Debug T    - " << d0 * group.m_x_axis << std::endl;

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

//        std::cout << "Debug dxy: \n" << "min_dx: " << min_dx << " max_dx: " << max_dx << std::endl;
//        std::cout << "min_dy: " << min_dy << " max_dy: " << max_dy << std::endl;
        Base::Scalar d_width = max_dx - min_dx;
        Base::Scalar d_height = max_dy - min_dy;
        // TODO debug comment
//        std::cout << "Debug plane size: d_width: " << d_width << " , d_height: " << d_height << std::endl;

        // Create images
        std::size_t image_width = d_width * PLANE_DENSITY + 2 * padding_pixels;
        std::size_t image_height = d_height * PLANE_DENSITY + 2 * padding_pixels;
        mve::FloatImage::Ptr patch_image = mve::FloatImage::create(image_width, image_height, 3);

        float random_color[3];
        Eigen::RowVector3d color = 0.5 * Eigen::RowVector3d::Random() + Eigen::RowVector3d(0.5, 0.5, 0.5);
        random_color[0] = double(color(0));
        random_color[1] = double(color(1));
        random_color[2] = double(color(2));

        patch_image->fill_color(random_color);

        // Re-scale texture coords
        double padding = double(padding_pixels) / PLANE_DENSITY;
        // TODO uncomment
#pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < texcoords.size(); i++) {
            if ((texcoords[i][0] - min_dx + padding) < 0 || (texcoords[i][1] - min_dy + padding) < 0 ){
//                (texcoords[f_i * 3 + 1][0] - min_dx) < 0 || (texcoords[f_i * 3 + 1][1] - min_dy) < 0 ||
//                (texcoords[f_i * 3 + 2][0] - min_dx) < 0 || (texcoords[f_i * 3 + 2][1] - min_dy) < 0) {
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
            /*
            texcoords[i][0] -= min_dx;
            texcoords[i][1] -= min_dy;
            if (texcoords[i][0] < 0 || texcoords[i][1] < 0) {
                std::cout << "Debug: vt0 " << texcoords[i][0] << " vt1 " << texcoords[i][1] << std::endl;
            }
             */
        }

        // Create texture patch
        MvsTexturing::Base::TexturePatch::Ptr patch = MvsTexturing::Base::TexturePatch::create(0, group.m_indices,
                                                                                               texcoords, patch_image);
        texture_patches.push_back(patch);
        /*
        Base::Vec3 bottom_left_p = group.m_plane_center + (group.m_x_axis * min_dx)
                                   + (group.m_y_axis * min_dy);

        std::vector<math::Vec2d> texcoords;
        texcoords.resize(group.m_indices.size() * 3);
        for (std::size_t f_i = 0; f_i < group.m_indices.size(); f_i++) {
            std::size_t face_idx = group.m_indices[f_i];
            Base::AttributeMatrix points_3 = mesh.m_vertices(mesh.m_faces.row(face_idx), Eigen::all);
            Base::AttributeMatrix rel_points3 = points_3.rowwise() - bottom_left_p;
            Base::AttributeMatrix offset_x = rel_points3 * group.m_x_axis.transpose();
            Base::AttributeMatrix offset_y = rel_points3 * group.m_y_axis.transpose();

            // TODO 检查 Eigen 数组操作
            texcoords[f_i * 3 + 0] = {offset_x(0, 0), offset_y(0, 0)};
            texcoords[f_i * 3 + 1] = {offset_x(1, 0), offset_y(1, 0)};
            texcoords[f_i * 3 + 2] = {offset_x(2, 0), offset_y(2, 0)};

//            std::cout << "Debug offset_x: " << offset_x << std::endl;
//            std::cout << "Debug offset_y: " << offset_y << std::endl;

            // TODO Debug 检查 mvs-texturing 的 TexturePatch 的 Faces\Texcoords 的大小是否匹配（匹配）
            // TODO Debug 检查 TexturePatch 的 Faces 保存的是不是 face_idx（是）
            // TODO Debug 检查 TexturePatch 的 texcoords 保存的坐标是什么坐标系的（纹理图像坐标系）
            //      TexturePatch 数值上是像素坐标系；被插入到 TextureAtlas 后被转化到 [0, 1] 区间

//            std::cout << "Debug points_3: \n" << points_3 << std::endl;
//            std::cout << "Debug x_axis: \n" << group.m_x_axis << std::endl;
//            std::cout << "Debug (points_3 - x_axis): \n"
//                      << (points_3.rowwise() - group.m_x_axis) << std::endl;
//
//            std::cout << "Debug (points_3 - m_x_axis) * (x_axis.transpose): \n"
//                      << (points_3.rowwise() - group.m_x_axis) * (group.m_x_axis.transpose())
//                      << std::endl;
//
//            std::cout << "Debug (points_3 - m_x_axis) dot (x_axis.transpose): \n"
//                      << (points_3.rowwise() - group.m_x_axis).dot(group.m_x_axis.transpose())
//                      << std::endl;

            // Check correction
//            Base::AttributeMatrix offset = (points_3.rowwise() - group.m_plane_center);
//            std::cout << "Debug multiplication correction: \n"
//                      << (offset(1, 0) * group.m_x_axis(0, 0)
//                          + offset(1, 1) * group.m_x_axis(0, 1)
//                          + offset(1, 2) * group.m_x_axis(0, 2)) << std::endl;
        }
         */
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

    // TODO delete
    std::cout << in_mesh_path << std::endl;

    using namespace MeshPolyRefinement;
    Base::TriMesh mesh;
    if (!IO::read_mesh_from_ply(in_mesh_path, mesh)) {
        return 0;
    } else {
        std::cout << "Mesh vertices: " << mesh.m_vertices.rows() << " faces: " << mesh.m_faces.rows() << std::endl;
    }

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
    IO::repair_non_manifold(mesh);
//    IO::save_mesh_plane_segments(out_mesh_path, mesh);

/*
    {
        // TODO delete
        // Test mve saving
        mve::TriangleMesh::Ptr temp_mesh = TextureRemeshing::Utils::triMesh_to_mveMesh(mesh);
        mve::geom::SavePLYOptions options;
        options.format_binary = false;
        options.write_vertex_colors = false;
        options.write_vertex_normals = true;
        options.write_face_colors = false;
        mve::geom::save_ply_mesh(temp_mesh, out_mesh_path, options);
    }
    */

    std::cout << "\n### TextureRemeshing------Generate Texture Patches" << std::endl;
    std::vector<MvsTexturing::Base::TexturePatch::Ptr> texture_patches;
    {
        util::WallTimer timer;
        // TODO texture_patch 的生成做并行化
        std::cout << "\tGenerating texture patches...";
        texture_patches_from_group(mesh, texture_patches);
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

    // 0. Region growing
    //      如果有区域未增长到怎么办 ？（未增长到的区域暂时不管，uv 用 0 代替）
    // 1. 确定 plane 形状（plane 里面所有点和 XY-axis 做 cosine 解出 uv）
    //      区域增长过大，一张图片放不下怎么办？（暂时不考虑。在代码里加上这个 case 判断）
    //
    // 2. 生成 plane 对应 texture patch
    //      face、uv、validity_mask
    //      mvs-texturing 中的 uv 坐标范围
    // 3. 合成 texture mask（直接调用 adjust_color() 就好）

    std::cout << "Texture remeshing done. " << std::endl;
    return 0;
}