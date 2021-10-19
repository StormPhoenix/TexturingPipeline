//
// Created by Storm Phoenix on 2021/10/15.
//

#ifndef TEXTURINGPIPELINE_SETTINGS_H
#define TEXTURINGPIPELINE_SETTINGS_H

#include <sstream>
#include <vector>
#include <string>

namespace MvsTexturing {
    namespace Base {
        template<typename T>
        const std::vector<std::string> choice_strings();

        template<typename T>
        inline
        const std::string choice_string(T i) {
            return choice_strings<T>()[static_cast<std::size_t>(i)];
        }

        template<typename T>
        inline
        const std::string choices() {
            const std::vector<std::string> strings = choice_strings<T>();
            const std::size_t n = strings.size();
            std::stringstream ss;
            for (std::size_t i = 0; i < n; ++i) {
                ss << strings[i];
                if (i != n - 1) ss << ", ";
            }
            return ss.str();
        }

        template<typename T>
        inline
        T parse_choice(std::string s) {
            const std::vector<std::string> strings = choice_strings<T>();
            const std::size_t n = strings.size();
            for (std::size_t i = 0; i < n; ++i) {
                if (s == strings[i]) {
                    return static_cast<T>(i);
                }
            }

            std::stringstream ss;
            ss << "Invalid choice: " << s << " (Available choices: ";
            ss << choices<T>() << ")";

            throw std::invalid_argument(ss.str());
        }

        /** Enum representing a data term. */
        enum DataTerm {
            DATA_TERM_AREA = 0,
            DATA_TERM_GMI = 1
        };

        enum MethodType {
            MRF = 0,
            Projection = 1
        };

        /** Enum representing a smoothness term. */
        enum SmoothnessTerm {
            SMOOTHNESS_TERM_POTTS = 0
        };

        /** Enum representing outlier removal choice. */
        enum OutlierRemoval {
            OUTLIER_REMOVAL_NONE = 0,
            OUTLIER_REMOVAL_GAUSS_DAMPING = 1,
            OUTLIER_REMOVAL_GAUSS_CLAMPING = 2
        };

        /** Enum representing tone mapping choice. */
        enum ToneMapping {
            TONE_MAPPING_NONE = 0,
            TONE_MAPPING_GAMMA = 1
        };

        struct Settings {
            bool verbose = false;

            DataTerm data_term = DATA_TERM_GMI;
            MethodType method_type = MRF;
            SmoothnessTerm smoothness_term = SMOOTHNESS_TERM_POTTS;
            OutlierRemoval outlier_removal = OUTLIER_REMOVAL_NONE;
            ToneMapping tone_mapping = TONE_MAPPING_NONE;

            bool geometric_visibility_test = true;
            bool global_seam_leveling = true;
            bool local_seam_leveling = true;
            bool hole_filling = true;
            bool keep_unseen_faces = false;

            float viewing_angle_threshold = 85.0f;
        };

        template<>
        inline
        const std::vector<std::string> choice_strings<DataTerm>() {
            return {"area", "gmi"};
        }

        template<>
        inline
        const std::vector<std::string> choice_strings<MethodType>() {
            return {"mrf", "projection"};
        }

        template<>
        inline
        const std::vector<std::string> choice_strings<SmoothnessTerm>() {
            return {"potts"};
        }

        template<>
        inline
        const std::vector<std::string> choice_strings<OutlierRemoval>() {
            return {"none", "gauss_damping", "gauss_clamping"};
        }

        template<>
        inline
        const std::vector<std::string> choice_strings<ToneMapping>() {
            return {"none", "gamma"};
        }
    }
}

#endif //TEXTURINGPIPELINE_SETTINGS_H
