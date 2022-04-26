//
// Created by Storm Phoenix on 2022/4/22.
//

#ifndef TEXTURINGPIPELINE_TEXTUREREMOVAL_H
#define TEXTURINGPIPELINE_TEXTUREREMOVAL_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <mve/image.h>

#include <vector>
#include <map>
#include <set>

class TextureRemoval {
public:
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> AttributeMatrix;
    typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> IndexMatrix;
    typedef std::vector<std::set<int>> Adjacency;

public:
    bool loadObjMesh(std::string file);

    void removeRedundantTextures(std::string outputDir);

    const std::vector<std::set<unsigned int>> &getPatches() {
        return patchesInTexSpace;
    }

    const std::vector<std::string> &getFaceMaterials() {
        return faceMaterials;
    }

    std::map<std::string, mve::ByteImage::Ptr> &getMaterialImage() {
        return materialImage;
    }

    const IndexMatrix &getMeshTexcoordIDs() {
        return meshTexcoordIDs;
    }

    const AttributeMatrix &getMeshTexcoords() {
        return meshTexcoords;
    }

private:
    void readMaterials();

    void searchTexSpacePatches();

private:
    AttributeMatrix meshVertices;
    AttributeMatrix meshNormals;
    AttributeMatrix meshTexcoords;
    IndexMatrix meshFaces;
    IndexMatrix meshNormalIDs;
    IndexMatrix meshTexcoordIDs;

    std::vector<std::string> faceMaterials;
    std::map<std::string, std::string> materialNameMap;
    std::map<std::string, mve::ByteImage::Ptr> materialImage;

    Adjacency faceAdjacency;
    Adjacency texAdjacency;

    std::vector<std::set<unsigned int>> patchesInTexSpace;
};


#endif //TEXTURINGPIPELINE_TEXTUREREMOVAL_H
