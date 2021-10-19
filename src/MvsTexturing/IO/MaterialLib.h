//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_MATERIALLIB_H
#define TEXTURINGPIPELINE_MATERIALLIB_H

#include <vector>
#include <mve/image.h>

namespace MvsTexturing {
    namespace IO {
        namespace Obj {
            struct Material {
                std::string name;
                mve::ByteImage::ConstPtr diffuse_map;
            };

            /**
            * Class representing a material lib of and obj model.
            */
            class MaterialLib : public std::vector<Material> {
            public:
                /** Saves the material lib to an .mtl file and all maps of its
                  * materials with the given prefix.
                  */
                void save_to_files(std::string const &prefix) const;
            };
        }
    }
}

#endif //TEXTURINGPIPELINE_MATERIALLIB_H
