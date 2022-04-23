//
// Created by Storm Phoenix on 2022/4/22.
//

#include "TextureRemoval.h"

#include <map>
#include <set>
#include <IO/IO.h>
#include <common.h>
#include <queue>
#include <mve/image_io.h>
#include <mve/image_tools.h>
#include <Base/TexturePatch.h>
#include <Mapper/AtlasMapper.h>

struct Edge {
    int index0;
    int index1;

    explicit Edge(int i, int j) {
        assert(i != j);

        if (i == j) {
            LOG_ERROR("Edge Creation Failed - Vertex is equal");
        }

        if (i <= j) {
            index0 = i;
            index1 = j;
        } else {
            index0 = j;
            index1 = i;
        }
    }

    bool operator<(const Edge &other) const {
        if (index0 < other.index0) {
            return true;
        } else if (index0 == other.index0) {
            return index1 < other.index1;
        } else {
            return false;
        }
    }

    bool operator==(const Edge &other) const {
        return (index0 == other.index0) && (index1 == other.index1);
    }
};

#define TEXTURE_BORDER 1


using MeshPtr = mve::TriangleMesh::Ptr;

MeshPtr eigenMesh_to_mveMesh(const TextureRemoval::AttributeMatrix &V,
                             const TextureRemoval::IndexMatrix &F,
                             const TextureRemoval::AttributeMatrix &FC) {
    if (V.rows() <= 0 || F.rows() <= 0) {
        return nullptr;
    }

    if (FC.rows() > 0 && FC.rows() != F.rows()) {
        return nullptr;
    }

    mve::TriangleMesh::Ptr ret = mve::TriangleMesh::create();

    mve::TriangleMesh::VertexList &vertices_list = ret->get_vertices();
    mve::TriangleMesh::FaceList &faces_list = ret->get_faces();
    mve::TriangleMesh::ColorList &face_colors_list = ret->get_face_colors();

    // copy vertices
    Eigen::Index n_vertices = V.rows();
    vertices_list.resize(n_vertices);
    for (int i = 0; i < n_vertices; i++) {
        math::Vec3f &v = vertices_list[i];
        v[0] = V(i, 0);
        v[1] = V(i, 1);
        v[2] = V(i, 2);
    }

    // copy faces
    Eigen::Index n_faces = F.rows();
    faces_list.resize(3 * n_faces);
    for (int i = 0; i < n_faces * 3; i += 3) {
        faces_list[i + 0] = F(i / 3, 0);
        faces_list[i + 1] = F(i / 3, 1);
        faces_list[i + 2] = F(i / 3, 2);
    }

    // copy face colors
    Eigen::Index n_face_colors = FC.rows();
    face_colors_list.resize(n_face_colors);
    for (int i = 0; i < n_face_colors; i++) {
        math::Vec4f &f_color = face_colors_list[i];
        for (int c = 0; c < FC.cols(); c++) {
            f_color[c] = FC(i, c);
        }
    }

    mve::MeshInfo mesh_info(ret);
    MvsTexturing::Builder::MVE::prepare_mesh(&mesh_info, ret);

    return ret;
}

MeshPtr eigenMesh_to_mveMesh(const TextureRemoval::AttributeMatrix &V,
                             const TextureRemoval::IndexMatrix &F) {
    TextureRemoval::AttributeMatrix FC;
    return eigenMesh_to_mveMesh(V, F, FC);
}

void computeAdjacency(const TextureRemoval::IndexMatrix &indices, TextureRemoval::Adjacency &adjacency) {
    std::map<Edge, std::vector<unsigned int>> edgeFaceMap;
    unsigned int nFaces = indices.rows();
    for (unsigned int faceID = 0; faceID < nFaces; faceID++) {
        for (int j = 0; j < 3; j++) {

            int firstVertexID = indices(faceID, j);
            int secondVertexID = indices(faceID, (j + 1) % 3);
            if (firstVertexID == secondVertexID) {
                LOG_WARN("Adjacency Computation Warning - Two same vertex ID");
                continue;
            }

            Edge edge(firstVertexID, secondVertexID);
            if (edgeFaceMap.find(edge) == edgeFaceMap.end()) {
                edgeFaceMap[edge] = std::vector<unsigned int>();
            }
            edgeFaceMap[edge].push_back(faceID);
        }
    }

    for (unsigned int i = 0; i < nFaces; i++) {
        adjacency.push_back(std::set<int>());
    }

    for (std::map<Edge, std::vector<unsigned int>>::iterator it = edgeFaceMap.begin(); it != edgeFaceMap.end(); it++) {
        std::vector<unsigned int> &faceIDs = it->second;
        if (faceIDs.size() > 2) {
            LOG_WARN("Adjacency Computation Error - More than 3 triangles attach to one edge: {}", faceIDs.size());
        } else if (faceIDs.size() == 2) {
            adjacency[faceIDs[0]].insert(faceIDs[1]);
            adjacency[faceIDs[1]].insert(faceIDs[0]);
        }
    }
}

void findAdjacentFaceIDs(const TextureRemoval::Adjacency &faceAdjacency, const TextureRemoval::Adjacency &texAdjacency,
                         const unsigned int faceID, std::vector<unsigned int> &adjFaceIDs) {
    const std::set<int> &adjFaceIDsInMeshSpace = faceAdjacency[faceID];
    const std::set<int> &adjFaceIDsInTexSpace = texAdjacency[faceID];

    for (const int adjFaceID: adjFaceIDsInMeshSpace) {
        if (adjFaceID == faceID) {
            continue;
        }

        if (adjFaceIDsInTexSpace.find(adjFaceID) != adjFaceIDsInTexSpace.end()) {
            adjFaceIDs.push_back(adjFaceID);
        }
    }
}

void findWholePatch(unsigned int faceID, const std::set<unsigned int> &totalFaceIDs,
                    const TextureRemoval::Adjacency &faceAdjacency, const TextureRemoval::Adjacency &texAdjacency,
                    std::set<unsigned int> &result) {
    std::set<unsigned int> visited;
    std::queue<unsigned int> q;
    q.push(faceID);
    visited.insert(faceID);

    while (!q.empty()) {
        unsigned int currFaceID = q.front();
        q.pop();

        std::vector<unsigned int> nextFaceIDs;
        findAdjacentFaceIDs(faceAdjacency, texAdjacency, currFaceID, nextFaceIDs);
        for (unsigned int nextFaceID: nextFaceIDs) {
            if ((totalFaceIDs.find(nextFaceID) == totalFaceIDs.end()) || (visited.find(nextFaceID) != visited.end())) {
                continue;
            }
            q.push(nextFaceID);
            visited.insert(nextFaceID);
        }
        result.insert(currFaceID);
    }
}

bool TextureRemoval::loadObjMesh(std::string file) {
    bool ret = MvsTexturing::IO::load_mesh_from_obj(
            file, meshVertices, meshNormals, meshTexcoords, meshFaces,
            meshNormalIDs, meshTexcoordIDs, faceMaterials, materialNameMap);

    if (ret) {
        computeAdjacency(meshFaces, faceAdjacency);
        computeAdjacency(meshTexcoordIDs, texAdjacency);
    }
    return ret;
}

void TextureRemoval::searchTexSpacePatches() {
    std::set<unsigned int> totalFaceIDs;
    for (unsigned int faceID = 0; faceID < meshFaces.rows(); faceID++) {
        totalFaceIDs.insert(faceID);
    }

    while (totalFaceIDs.size() > 0) {
        // BFS search
        unsigned int seedFaceID = (*totalFaceIDs.begin());
        patchesInTexSpace.push_back(std::set<unsigned int>());

        // store bfs search result
        std::set<unsigned int> &lastPatch = patchesInTexSpace[patchesInTexSpace.size() - 1];
        findWholePatch(seedFaceID, totalFaceIDs, faceAdjacency, texAdjacency, lastPatch);

        for (unsigned int patchFaceID: lastPatch) {
            totalFaceIDs.erase(patchFaceID);
        }
    }
}

double getCoord(int faceID, int vertexID, int axis, const TextureRemoval::IndexMatrix &meshTexcoordIDs,
                const TextureRemoval::AttributeMatrix &meshTexcoords) {
    unsigned int texcoordID = meshTexcoordIDs(faceID, vertexID);
    return meshTexcoords(texcoordID, axis);
}

void updateCoordRange(int axis, int faceID, const TextureRemoval::IndexMatrix &meshTexcoordIDs,
                      const TextureRemoval::AttributeMatrix &meshTexcoords, double &minCoord, double &maxCoord) {
    for (int i = 0; i < 3; i++) {
        double coord = getCoord(faceID, i, axis, meshTexcoordIDs, meshTexcoords);
        if (minCoord > coord) {
            minCoord = coord;
        }

        if (maxCoord < coord) {
            maxCoord = coord;
        }
    }
}

void TextureRemoval::readMaterials() {
    for (std::map<std::string, std::string>::iterator it = materialNameMap.begin();
         it != materialNameMap.end(); it++) {
        materialImage[it->first] = mve::image::load_file(it->second);
    }
}

using namespace MvsTexturing;

Base::TexturePatch::Ptr generateTexturePatch(TextureRemoval &removal, unsigned int index) {
    if ((index >= removal.getPatches().size()) ||
        (removal.getPatches()[index].size() <= 0)) {
        return nullptr;
    }

    mve::FloatImage::Ptr imageOfPatch = nullptr;
    std::vector<std::size_t> facesOfPatch;
    std::vector<math::Vec2f> texcoordsOfPatch;

    const TextureRemoval::IndexMatrix &meshTexcoordIDs = removal.getMeshTexcoordIDs();
    const TextureRemoval::AttributeMatrix &meshTexcoords = removal.getMeshTexcoords();
    std::map<std::string, mve::ByteImage::Ptr> &materialImage = removal.getMaterialImage();
    const std::vector<std::string> &faceMaterials = removal.getFaceMaterials();


    // check material
    const std::set<unsigned int> &patch = removal.getPatches()[index];
    std::string materialID = removal.getFaceMaterials()[(*patch.begin())];
    mve::ByteImage::Ptr image = materialImage[materialID];
    int materialWidth = image->width();
    int materialHeight = image->height();

    double minXCoord, maxXCoord;
    double minYCoord, maxYCoord;

    minXCoord = minYCoord = 1.0f;
    maxXCoord = maxYCoord = 0.0f;


    updateCoordRange(0, (*patch.begin()), meshTexcoordIDs, meshTexcoords, minXCoord, maxXCoord);
    updateCoordRange(1, (*patch.begin()), meshTexcoordIDs, meshTexcoords, minYCoord, maxYCoord);

    for (unsigned int faceID: patch) {
        if (materialID != faceMaterials[faceID]) {
            LOG_ERROR("generateTexturePatch ERROR - Faces in the patch have different materials");
            return nullptr;
        }
        facesOfPatch.push_back(faceID);
        updateCoordRange(0, faceID, meshTexcoordIDs, meshTexcoords, minXCoord, maxXCoord);
        updateCoordRange(1, faceID, meshTexcoordIDs, meshTexcoords, minYCoord, maxYCoord);

        for (int vertexID = 0; vertexID < 3; vertexID++) {
            double coordX = getCoord(faceID, vertexID, 0, meshTexcoordIDs, meshTexcoords);
            double coordY = getCoord(faceID, vertexID, 1, meshTexcoordIDs, meshTexcoords);
            math::Vec2f coord = math::Vec2f(coordX * materialWidth, coordY * materialHeight);
            texcoordsOfPatch.push_back(coord);
        }
    }

    int subWidth = (int(maxXCoord * materialWidth) - int(minXCoord * materialWidth)) + 1;
    int subHeight = (int(maxYCoord * materialHeight) - int(minYCoord * materialHeight)) + 1;
    int minX = minXCoord * image->width();
    int minY = minYCoord * image->height();

    subWidth += 2 * TEXTURE_BORDER;
    subHeight += 2 * TEXTURE_BORDER;
    minX -= TEXTURE_BORDER;
    minY -= TEXTURE_BORDER;

    math::Vec2f min(minX, minY);
    for (std::size_t i = 0; i < texcoordsOfPatch.size(); i++) {
        texcoordsOfPatch[i] = texcoordsOfPatch[i] - min;
    }

    mve::ByteImage::Ptr subImage = mve::image::crop(image, subWidth, subHeight, minX, minY, *math::Vec3uc(255, 0, 255));
    imageOfPatch = mve::image::byte_to_float_image(subImage);

    Base::TexturePatch::Ptr final = Base::TexturePatch::create(0, facesOfPatch, texcoordsOfPatch, imageOfPatch);
    std::vector<math::Vec3f> patch_adjust_values(final->get_faces().size() * 3, math::Vec3f(0.0f));
    final->adjust_colors(patch_adjust_values);

    return final;
    // TODO check MaskImage
    // 参考 AtlasMapper.cpp 648 ~ 657 行
}

void TextureRemoval::testSavePatchExample(unsigned int index) {
    readMaterials();
    Base::TexturePatch::Ptr patch = generateTexturePatch(*this, index);
    mve::image::save_png_file(mve::image::float_to_byte_image(patch->get_image()), "./0.png");

    // TODO check MaskImage
    // 参考 AtlasMapper.cpp 648 ~ 657 行
}

void TextureRemoval::removeRedundantTextures(std::string outputDir) {
    searchTexSpacePatches();
    readMaterials();

    std::vector<Base::TexturePatch::Ptr> patches;
    for (int i = 0; i < patchesInTexSpace.size(); i++) {
        patches.push_back(generateTexturePatch(*this, i));
    }

    MvsTexturing::Parameter param;
    param.output_prefix = outputDir;
    param.debug_mode = false;
    std::vector<MvsTexturing::Base::TextureAtlas::Ptr> textureAtlases;

    MvsTexturing::AtlasMapper::generate_texture_atlases(param, &patches, &textureAtlases, false);
    MeshPtr mesh = eigenMesh_to_mveMesh(meshVertices, meshFaces);
    MvsTexturing::IO::MVE::save_obj_mesh(param.output_prefix, mesh, textureAtlases);
}