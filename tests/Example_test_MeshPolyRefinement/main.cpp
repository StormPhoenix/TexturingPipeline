#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <iostream>

#include <igl/decimate.h>
#include <igl/collapse_edge.h>
#include <igl/cotmatrix.h>

#include <boost/container/flat_set.hpp>

#include <Base/TriMesh.h>
#include <PlaneEstimation/RegionGrowing.h>
#include <PlaneEstimation/RegionExpand.h>
#include <DataIO/Repair.h>
#include <DataIO/IO.h>

#define TINYPLY_IMPLEMENTATION

#include <tinyply.h>

void write_ply_example(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXi &C) {
    struct float3 {
        float x, y, z;
    };

    struct int3 {
        int32_t x, y, z;
    };

    struct uchar3 {
        u_char r, g, b;

        uchar3() {
            r = g = b = 0;;
        }

        uchar3(u_char r_, u_char g_, u_char b_) :
                r(r_), g(g_), b(b_) {}
    };

    struct geometry {
        std::vector<float3> vertices;
        std::vector<int3> faces;
        std::vector<uchar3> face_colors;
    };

    geometry geom;
    for (const auto &v : V.rowwise()) {
        geom.vertices.push_back({(float) v[0], (float) v[1], (float) v[2]});
    }

    for (const auto &f : F.rowwise()) {
        geom.faces.push_back({f[0], f[1], f[2]});
    }

    for (const auto &c : C.rowwise()) {
        geom.face_colors.push_back({(u_char) c[0], (u_char) c[1], (u_char) c[2]});
    }

    std::string filename = "test_tinyply";
    std::filebuf fb_binary;
    fb_binary.open(filename + "-binary.ply", std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) {
        throw std::runtime_error("failed to open " + filename);
    }

    std::filebuf fb_ascii;
    fb_ascii.open(filename + "-ascii.ply", std::ios::out | std::ios::out);
    std::ostream outstream_ascii(&fb_ascii);
    if (outstream_ascii.fail()) {
        throw std::runtime_error("failed to open " + filename);
    }

    tinyply::PlyFile out_file;
    out_file.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, geom.vertices.size(),
                                       reinterpret_cast<uint8_t *>(geom.vertices.data()), tinyply::Type::INVALID, 0);

    out_file.add_properties_to_element("face", {"vertex_indices"}, tinyply::Type::INT32, geom.faces.size(),
                                       reinterpret_cast<uint8_t *>(geom.faces.data()), tinyply::Type::UINT8, 3);

    out_file.add_properties_to_element("face", {"red", "green", "blue"}, tinyply::Type::UINT8, geom.face_colors.size(),
                                       reinterpret_cast<uint8_t *>(geom.face_colors.data()), tinyply::Type::INVALID, 0);

    out_file.write(outstream_binary, true);
    out_file.write(outstream_ascii, false);
}

int main() {
    const std::string in_mesh_path = "../../../resource/model/anyuanmen.ply";
    const std::string out_mesh_path = "../../../resource/model/anyuanmen-segment.ply";

    // params
    double planar_score = 0.80000000000000004;
    double angle = 15;
    double ratio = 10;
    int min_plane_size = 10;

    using namespace MeshPolyRefinement;
    Base::TriMesh mesh;
    if (!IO::read_mesh_from_ply(in_mesh_path, mesh)) {
        return 0;
    } else {
        std::cout << "Mesh vertices: " << mesh.m_vertices.rows() << " faces: " << mesh.m_faces.rows() << std::endl;
    }

    std::cout << "MeshPolyRefinement------Detecting Planes" << std::endl;
    // detect planes on mesh using region-growing
    PlaneEstimation::region_growing_plane_estimate(mesh, planar_score, angle, ratio, min_plane_size);

    //expand plane segment regions
    PlaneEstimation::plane_region_expand(mesh,50.0,45.0);

    //filter small non-plane regions
    PlaneEstimation::plane_region_refine(mesh);

    //merge parallel adjacent plane segments
    PlaneEstimation::plane_region_merge(mesh);

    IO::repair_non_manifold(mesh);
    IO::save_mesh_plane_segments(out_mesh_path,mesh);
    return 0;
}
