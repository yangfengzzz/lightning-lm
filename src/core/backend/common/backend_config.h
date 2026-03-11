#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace lightning::backend {

struct LoopProposalOptions {
    int loop_kf_gap = 20;
    int min_id_interval = 20;
    int closest_id_th = 50;
    double max_range = 30.0;
};

struct LoopRegistrationOptions {
    int submap_idx_range = 40;
    std::vector<double> ndt_resolutions{10.0, 5.0, 2.0, 1.0};
    double voxel_size_ratio = 0.1;
    double transformation_epsilon = 0.05;
    double step_size = 0.7;
    int max_iterations = 40;
};

struct LoopFilterOptions {
    double ndt_score_th = 1.0;
    std::string robust_kernel = "cauchy";
    double robust_kernel_delta = 5.2 / 5.0;
    bool with_height = true;
    double height_noise = 0.1;
};

struct PoseGraphOptions {
    double motion_trans_noise = 0.1;
    double motion_rot_noise = 3.0 * M_PI / 180.0;
    double loop_trans_noise = 0.2;
    double loop_rot_noise = 3.0 * M_PI / 180.0;
    int optimize_iterations = 20;
};

struct LoopClosingBackendConfig {
    std::string variant_name = "baseline_current";
    LoopProposalOptions proposal;
    LoopRegistrationOptions registration;
    LoopFilterOptions filter;
    PoseGraphOptions pose_graph;
};

LoopClosingBackendConfig LoadLoopClosingBackendConfig(const std::string& yaml_path);

}  // namespace lightning::backend
