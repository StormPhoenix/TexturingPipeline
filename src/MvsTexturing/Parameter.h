//
// Created by Storm Phoenix on 2021/11/8.
//

#ifndef TEXTURINGPIPELINE_PARAMETER_H
#define TEXTURINGPIPELINE_PARAMETER_H

#include <string>

namespace MvsTexturing {
    const std::string Data_Term_Area = "area";
    const std::string Data_Term_GMI = "gmi";

    const std::string Outlier_Removal_None = "none";
    const std::string Outlier_Removal_Gauss_Clamping = "gauss_clamping";
    const std::string Outlier_Removal_Gauss_Damping = "gauss_damping";

    const std::string Tone_Mapping_None = "none";
    const std::string Tone_Mapping_Gamma = "gamma";

    const std::string Mrf_Call_Lib_MapMap = "mapmap";
    const std::string Mrf_Call_Lib_OpenMVS = "openmvs";

    struct Parameter {
        std::string scene_file;
        std::string input_mesh;
        std::string output_prefix;
        std::string data_cost_file;
        std::string labeling_file;
        std::string method_type;
        std::string data_term;
        std::string smoothness_term;
        std::string outlier_removal;
        bool view_selection_model;
        bool skip_geometric_visibility_test;
        bool skip_global_seam_leveling;
        bool skip_local_seam_leveling;
        bool skip_hole_filling;
        bool keep_unseen_faces;
        bool write_intermediate_results;

        double viewing_angle_threshold = 85.0f;
        std::string tone_mapping;
        std::string mrf_call_lib = Mrf_Call_Lib_MapMap;

        double planar_score;
        double angle_threshold;
        double ratio_threshold;
        double min_plane_size;
    };
}

#endif //TEXTURINGPIPELINE_PARAMETER_H
