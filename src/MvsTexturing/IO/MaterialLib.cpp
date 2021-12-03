//
// Created by Storm Phoenix on 2021/10/17.
//

#include <fstream>
#include <cstring>
#include <cerrno>

#include <util/exception.h>
#include <util/file_system.h>
#include <mve/image_io.h>

#include "MaterialLib.h"

namespace MvsTexturing {
    namespace IO {
        namespace Obj {
            void MaterialLib::save_to_files(const std::string &prefix) const {
                std::string filename = prefix + ".mtl";
                std::ofstream out(filename.c_str());
                if (!out.good())
                    throw util::FileException(filename, std::strerror(errno));

                std::string const name = util::fs::basename(prefix);

                for (Material const &material : *this) {
                    std::string diffuse_map_postfix = "_" + material.name + "_map_Kd.png";
                    out << "newmtl " << material.name << '\n'
                        << "Ka 1.000000 1.000000 1.000000" << '\n'
                        << "Kd 1.000000 1.000000 1.000000" << '\n'
                        << "Ks 0.000000 0.000000 0.000000" << '\n'
                        //<< "Tr 0.000000" << '\n' // *Tr*ansparancy vs. *d*issolve: Tr = 1.0 - d
                        << "illum 1" << '\n'
                        << "Ns 1.000000" << '\n'
                        << "map_Kd " << name + diffuse_map_postfix << std::endl;
                }
                out.close();

                /*
                for (Material const &material : *this) {
                    std::string filename = prefix + "_" + material.name + "_map_Kd.png";
//                    mve::image::save_png_file(material.diffuse_map, filename);
                }
                 */
            }
        }
    }
}