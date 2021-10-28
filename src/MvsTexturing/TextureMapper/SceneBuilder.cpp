//
// Created by Storm Phoenix on 2021/10/15.
//
#include <cstring>
#include <string>
#include <vector>

#include <util/timer.h>
#include <util/tokenizer.h>
#include <util/file_system.h>
#include <mve/image_io.h>
#include <mve/mesh_io_ply.h>
#include <mve/mesh_info.h>
#include <mve/image_tools.h>
#include <mve/bundle_io.h>
#include <mve/scene.h>
#include <mve/mesh.h>

#include "Base/View.h"
#include "Base/LabelGraph.h"
#include "Utils/ProgressCounter.h"

namespace MvsTexturing {
    namespace Builder {
        void build_from_nvm_scene(const std::string &nvm_file, std::vector<Base::TextureView> *texture_views,
                                  const std::string &distortion_img_dir) {
            std::vector<mve::AdditionalCameraInfo> nvm_cams;
            mve::Bundle::Ptr bundle = mve::load_nvm_bundle(nvm_file, &nvm_cams);
            mve::Bundle::Cameras &cameras = bundle->get_cameras();

            using namespace Utils;
            ProgressCounter view_counter("\tLoading", cameras.size());
#pragma omp parallel for
            for (std::size_t i = 0; i < cameras.size(); ++i) {
                view_counter.progress<SIMPLE>();
                mve::CameraInfo &mve_cam = cameras[i];
                mve::AdditionalCameraInfo const &nvm_cam = nvm_cams[i];

                mve::ByteImage::Ptr image = mve::image::load_file(nvm_cam.filename);

                int const maxdim = std::max(image->width(), image->height());
                mve_cam.flen = mve_cam.flen / static_cast<float>(maxdim);

                image = mve::image::image_undistort_vsfm<uint8_t>
                        (image, mve_cam.flen, nvm_cam.radial_distortion);

                const std::string image_file = util::fs::join_path(
                        distortion_img_dir,
                        util::fs::replace_extension(
                                util::fs::basename(nvm_cam.filename),
                                "png"
                        )
                );
                mve::image::save_png_file(image, image_file);

#pragma omp critical
                texture_views->push_back(Base::TextureView(i, mve_cam, image_file));
                view_counter.inc();
            }
        }

        void from_nvm_scene(const std::string &nvm_file, std::vector<Base::TextureView> *texture_views,
                            const std::string &distortion_img_dir) {
            std::vector<mve::AdditionalCameraInfo> nvm_cams;
            mve::Bundle::Ptr bundle = mve::load_nvm_bundle(nvm_file, &nvm_cams);
            mve::Bundle::Cameras &cameras = bundle->get_cameras();

            using namespace Utils;
            ProgressCounter view_counter("\tLoading", cameras.size());
#pragma omp parallel for
            for (std::size_t i = 0; i < cameras.size(); ++i) {
                view_counter.progress<SIMPLE>();
                mve::CameraInfo &mve_cam = cameras[i];
                const mve::AdditionalCameraInfo &nvm_cam = nvm_cams[i];

                mve::ByteImage::Ptr image = mve::image::load_file(nvm_cam.filename);

                int const maxdim = std::max(image->width(), image->height());
                mve_cam.flen = mve_cam.flen / static_cast<float>(maxdim);

                image = mve::image::image_undistort_vsfm<uint8_t>
                        (image, mve_cam.flen, nvm_cam.radial_distortion);

                const std::string image_file = util::fs::join_path(
                        distortion_img_dir,
                        util::fs::replace_extension(
                                util::fs::basename(nvm_cam.filename), "png")
                );
                mve::image::save_png_file(image, image_file);

#pragma omp critical
                texture_views->push_back(Base::TextureView(i, mve_cam, image_file));
                view_counter.inc();
            }
        }

        void from_images_and_camera_files(const std::string &path, std::vector<Base::TextureView> *texture_views,
                                          const std::string &distortion_img_dir) {
            util::fs::Directory dir(path);
            std::sort(dir.begin(), dir.end());
            std::vector<std::string> files;
            for (std::size_t i = 0; i < dir.size(); ++i) {
                util::fs::File const &cam_file = dir[i];
                if (cam_file.is_dir) continue;

                std::string cam_file_ext = util::string::uppercase(util::string::right(cam_file.name, 4));
                if (cam_file_ext != ".CAM") continue;

                std::string prefix = util::string::left(cam_file.name, cam_file.name.size() - 4);
                if (prefix.empty()) continue;

                /* Find corresponding image file. */
                int step = 1;
                for (std::size_t j = i + 1; j < dir.size(); j += step) {
                    util::fs::File const &img_file = dir[j];

                    /* Since the files are sorted we can break - no more files with the same prefix exist. */
                    if (util::string::left(img_file.name, prefix.size()) != prefix) {
                        if (step == 1) {
                            j = i;
                            step = -1;
                            continue;
                        } else {
                            break;
                        }
                    }

                    /* Image file (based on extension)? */
                    std::string img_file_ext = util::string::uppercase(util::string::right(img_file.name, 4));
                    if (img_file_ext != ".PNG" && img_file_ext != ".JPG" &&
                        img_file_ext != "TIFF" && img_file_ext != "JPEG")
                        continue;

                    files.push_back(cam_file.get_absolute_name());
                    files.push_back(img_file.get_absolute_name());
                    break;
                }
            }

            using namespace Utils;
            ProgressCounter view_counter("\tLoading", files.size() / 2);
#pragma omp parallel for
            for (std::size_t i = 0; i < files.size(); i += 2) {
                view_counter.progress<SIMPLE>();
                const std::string cam_file = files[i];
                const std::string img_file = files[i + 1];

                /* Read CAM file. */
                std::ifstream infile(cam_file.c_str(), std::ios::binary);
                if (!infile.good()) {
                    throw util::FileException(util::fs::basename(cam_file), std::strerror(errno));
                }
                std::string cam_int_str, cam_ext_str;
                std::getline(infile, cam_ext_str);
                std::getline(infile, cam_int_str);
                util::Tokenizer tok_ext, tok_int;
                tok_ext.split(cam_ext_str);
                tok_int.split(cam_int_str);
#pragma omp critical
                if (tok_ext.size() != 12 || tok_int.size() < 1) {
                    std::cerr << "Invalid CAM file: " << util::fs::basename(cam_file) << std::endl;
                    std::exit(EXIT_FAILURE);
                }

                /* Create cam_info and eventually undistort image. */
                mve::CameraInfo cam_info;
                cam_info.set_translation_from_string(tok_ext.concat(0, 3));
                cam_info.set_rotation_from_string(tok_ext.concat(3, 0));

                std::stringstream ss(cam_int_str);
                ss >> cam_info.flen;
                if (ss.peek() && !ss.eof())
                    ss >> cam_info.dist[0];
                if (ss.peek() && !ss.eof())
                    ss >> cam_info.dist[1];
                if (ss.peek() && !ss.eof())
                    ss >> cam_info.paspect;
                if (ss.peek() && !ss.eof())
                    ss >> cam_info.ppoint[0];
                if (ss.peek() && !ss.eof())
                    ss >> cam_info.ppoint[1];

                std::string image_file = util::fs::abspath(img_file);
                if (cam_info.dist[0] != 0.0f) {
                    mve::ByteImage::Ptr image = mve::image::load_file(img_file);
                    if (cam_info.dist[1] != 0.0f) {
                        image = mve::image::image_undistort_k2k4<uint8_t>(image,
                                                                          cam_info.flen, cam_info.dist[0],
                                                                          cam_info.dist[1]);
                    } else {
                        image = mve::image::image_undistort_vsfm<uint8_t>(image,
                                                                          cam_info.flen, cam_info.dist[0]);
                    }

                    image_file = util::fs::join_path(
                            distortion_img_dir,
                            util::fs::replace_extension(util::fs::basename(img_file), "png")
                    );
                    mve::image::save_png_file(image, image_file);
                }

#pragma omp critical
                texture_views->push_back(Base::TextureView(i / 2, cam_info, image_file));
                view_counter.inc();
            }
        }

        void from_mve_scene(const std::string &scene_dir, std::string const &image_name,
                            std::vector<Base::TextureView> *texture_views) {

            mve::Scene::Ptr scene;
            try {
                scene = mve::Scene::create(scene_dir);
            } catch (std::exception &e) {
                std::cerr << "Could not open scene: " << e.what() << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::size_t num_views = scene->get_views().size();
            texture_views->reserve(num_views);

            using namespace Utils;
            ProgressCounter view_counter("\tLoading", num_views);
            for (std::size_t i = 0; i < num_views; ++i) {
                view_counter.progress<SIMPLE>();

                mve::View::Ptr view = scene->get_view_by_id(i);
                if (view == NULL) {
                    view_counter.inc();
                    continue;
                }

                if (!view->has_image(image_name, mve::IMAGE_TYPE_UINT8)) {
                    std::cout << "Warning: View " << view->get_name() << " has no byte image "
                              << image_name << std::endl;
                    continue;
                }

                mve::View::ImageProxy const *image_proxy = view->get_image_proxy(image_name);

                if (image_proxy->channels < 3) {
                    std::cerr << "Image " << image_name << " of view " <<
                              view->get_name() << " is not a color image!" << std::endl;
                    exit(EXIT_FAILURE);
                }

                texture_views->push_back(Base::TextureView(view->get_id(), view->get_camera(), util::fs::abspath(
                        util::fs::join_path(view->get_directory(), image_proxy->filename))));
                view_counter.inc();
            }
        }

        void build_scene(const std::string &in_scene, std::vector<Base::TextureView> *texture_views,
                         const std::string &distortion_img_dir) {
            // Bundle file
            if (util::fs::file_exists(in_scene.c_str())) {
                const std::string &file = in_scene;
                const std::string extension = util::string::uppercase(util::string::right(file, 3));
                if (extension == "NVM") {
                    from_nvm_scene(file, texture_views, distortion_img_dir);
                }
            }

            // Scene folder
            if (util::fs::dir_exists(in_scene.c_str())) {
                from_images_and_camera_files(in_scene, texture_views, distortion_img_dir);
            }

            // MVE_SCENE::EMBEDDING
            std::size_t pos = in_scene.rfind("::");
            if (pos != std::string::npos) {
                std::string scene_dir = in_scene.substr(0, pos);
                std::string image_name = in_scene.substr(pos + 2, in_scene.size());
                from_mve_scene(scene_dir, image_name, texture_views);
            }

            std::sort(texture_views->begin(), texture_views->end(),
                      [](Base::TextureView const &l, Base::TextureView const &r) -> bool {
                          return l.get_id() < r.get_id();
                      }
            );

            std::size_t num_views = texture_views->size();
            if (num_views == 0) {
                std::cerr
                        << "No proper input scene descriptor given.\n"
                        << "A input descriptor can be:\n"
                        << "BUNDLE_FILE - a bundle file (currently onle .nvm files are supported)\n"
                        << "SCENE_FOLDER - a folder containing images and .cam files\n"
                        << "MVE_SCENE::EMBEDDING - a mve scene and embedding\n"
                        << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        namespace MVE {
            typedef mve::TriangleMesh::Ptr MeshPtr;
            typedef mve::TriangleMesh::ConstPtr MeshConstPtr;

            std::size_t remove_redundant_faces(const mve::MeshInfo &mesh_info, MeshPtr mesh) {
                mve::TriangleMesh::FaceList &faces = mesh->get_faces();
                mve::TriangleMesh::FaceList new_faces;
                new_faces.reserve(faces.size());

                std::size_t num_redundant = 0;
                for (std::size_t i = 0; i < faces.size(); i += 3) {
                    std::size_t face_id = i / 3;
                    bool redundant = false;
                    for (std::size_t j = 0; !redundant && j < 3; ++j) {
                        mve::MeshInfo::AdjacentFaces const &adj_faces = mesh_info[faces[i + j]].faces;
                        for (std::size_t k = 0; !redundant && k < adj_faces.size(); ++k) {
                            std::size_t adj_face_id = adj_faces[k];

                            /* Remove only the redundant face with smaller id. */
                            if (face_id < adj_face_id) {
                                bool identical = true;
                                /* Faces are considered identical if they consist of the same vertices. */
                                for (std::size_t l = 0; l < 3; ++l) {
                                    std::size_t vertex = faces[adj_face_id * 3 + l];
                                    if (std::find(&faces[i], &faces[i + 3], vertex) == &faces[i + 3]) {
                                        identical = false;
                                        break;
                                    }
                                }

                                redundant = identical;
                            }
                        }
                    }

                    if (redundant) {
                        ++num_redundant;
                    } else {
                        new_faces.insert(new_faces.end(), faces.cbegin() + i, faces.cbegin() + i + 3);
                    }
                }

                faces.swap(new_faces);

                return num_redundant;
            }

            void prepare_mesh(mve::MeshInfo *mesh_info, MeshPtr mesh) {
                std::size_t num_redundant = remove_redundant_faces(*mesh_info, mesh);
                if (num_redundant > 0) {
                    std::cout << "\tRemoved " << num_redundant << " redundant faces." << std::endl;
                }

                /* Ensure face and vertex normals. */
                mesh->ensure_normals(true, true);

                /* Update vertex infos. */
                mesh_info->clear();
                mesh_info->initialize(mesh);
            }

            void build_adjacency_graph(MeshConstPtr mesh, const mve::MeshInfo &mesh_info, Base::LabelGraph *graph) {
                mve::TriangleMesh::FaceList const &faces = mesh->get_faces();
                std::size_t const num_faces = faces.size() / 3;

                using namespace Utils;
                ProgressCounter face_counter("\tAdding edges", num_faces);
                for (std::size_t i = 0; i < faces.size(); i += 3) {
                    face_counter.progress<SIMPLE>();

                    std::size_t v1 = faces[i];
                    std::size_t v2 = faces[i + 1];
                    std::size_t v3 = faces[i + 2];

                    std::vector<std::size_t> adj_faces;
                    mesh_info.get_faces_for_edge(v1, v2, &adj_faces);
                    mesh_info.get_faces_for_edge(v2, v3, &adj_faces);
                    mesh_info.get_faces_for_edge(v3, v1, &adj_faces);

                    for (std::size_t j = 0; j < adj_faces.size(); ++j) {
                        /* Face id vs. face position. */
                        std::size_t face = i / 3;
                        std::size_t adj_face = adj_faces[j];

                        /* Avoid self referencing. */
                        if (face != adj_face) {
                            /* Edge not already in graph? */
                            if (!graph->has_edge(face, adj_face)) {
                                graph->add_edge(face, adj_face);
                            }
                        }
                    }
                    face_counter.inc();
                }
                std::cout << "\t" << graph->num_edges() << " total edges." << std::endl;
            }
        }
    }
}