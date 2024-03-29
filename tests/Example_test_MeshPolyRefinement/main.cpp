
#include <iostream>

#include <boost/program_options.hpp>

#define TINYPLY_IMPLEMENTATION
#include <Base/TriMesh.h>

#include <DataIO/Repair.h>
#include <DataIO/IO.h>
#include <PlaneEstimation/RegionGrowing.h>
#include <PlaneEstimation/RegionExpand.h>
#include <PlaneDecimation/PlaneEdgeDecimator.h>
#include <PlaneOptimization/Optimize.h>
#include <PlaneOptimization/MeshSubdivide.h>
#include <PlaneDecimation/DelaunayRemesh2D.h>

int main(int argc, char *argv[]) {
    namespace bpo = boost::program_options;
    bpo::options_description opts("Example test MeshRefinement options");
    bpo::variables_map vm;

//    opts.add_options()
//            ("help", "produce help message")
//            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
//            ("output_mesh", bpo::value<std::string>()->default_value("res.ply"), "path of output simplified mesh")
//            ("planar_score", bpo::value<double>()->default_value(0.97),
//             "minimal planar score of face during plane estimation, default 0.97")
//            ("angle", bpo::value<double>()->default_value(10.0),
//             "maximal angle in degree that plane segment should accept face, default 10.0")
//            ("ratio", bpo::value<double>()->default_value(5.0),
//             "maximal distance ratio that plane segment should accept face, default 5.0")
//            ("plane_size", bpo::value<int>()->default_value(10),
//             "minimal number of facets in plane segment, default 10");

    opts.add_options()
            ("help", "produce help message")
            ("input_mesh", bpo::value<std::string>()->default_value("semantic_mesh.ply"), "path of input dense mesh")
            ("output_mesh", bpo::value<std::string>()->default_value("res.ply"), "path of output simplified mesh")
            ("planar_score", bpo::value<double>()->default_value(0.3),
             "minimal planar score of face during plane estimation, default 0.97")
            ("angle", bpo::value<double>()->default_value(30.0),
             "maximal angle in degree that plane segment should accept face, default 10.0")
            ("ratio", bpo::value<double>()->default_value(20.0),
             "maximal distance ratio that plane segment should accept face, default 5.0")
            ("plane_size", bpo::value<int>()->default_value(5),
             "minimal number of facets in plane segment, default 10");

    try {
        bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    } catch (...) {
        std::cout << "undefine options in command lines.\n";
        return 0;
    }
    const std::string in_mesh_path = vm["input_mesh"].as<std::string>();
    const std::string out_mesh_path = vm["output_mesh"].as<std::string>();

    using namespace MeshPolyRefinement;
    Base::TriMesh mesh;
    if (!IO::read_mesh_from_ply(in_mesh_path, mesh)) {
        return 0;
    } else {
        std::cout << "Mesh vertices: " << mesh.m_vertices.rows() << " faces: " << mesh.m_faces.rows() << std::endl;
    }

    std::cout << "MeshPolyRefinement------Detecting Planes" << std::endl;
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

    //decimate facets inside plane segments, plane boundary edges remain unchanged
    PlaneDecimation::Decimator decimator(mesh);
    std::cout << "MeshPolyRefinement------Plane Decimating" << std::endl;
    decimator.decimate();

    //split edges that may cause degeneration during optimization
    PlaneOptimization::MeshSubdivide subdivide(mesh);
    subdivide.split();

    //optimize vertex positions
    std::cout << "MeshPolyRefinement------Optimizing" << std::endl;
    PlaneOptimization::optimize_mesh(mesh,100);

    //decimate facets, edges between two plane segments are collapsed
    std::cout << "MeshPolyRefinement------Plane Edge Decimating" << std::endl;
    PlaneDecimation::PlaneEdgeDecimator edge_decimator(mesh);
    edge_decimator.decimate();

    //remesh plane facets to remove fold facets
    PlaneDecimation::DelaunayRemesh2D delaunay_remesh_2d(mesh);
    delaunay_remesh_2d.remesh();

    IO::repair_non_manifold(mesh);
    IO::save_mesh_plane_segments(out_mesh_path, mesh);

    std::cout << "testMeshPolyRefinement done. " << std::endl;
    return 0;
}