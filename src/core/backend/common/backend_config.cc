#include "core/backend/common/backend_config.h"

#include <yaml-cpp/yaml.h>

namespace lightning::backend {
namespace {

template <typename T>
void AssignIfPresent(const YAML::Node& node, const char* key, T& target) {
    if (node && node[key]) {
        target = node[key].as<T>();
    }
}

}  // namespace

LoopClosingBackendConfig LoadLoopClosingBackendConfig(const std::string& yaml_path) {
    LoopClosingBackendConfig cfg;
    if (yaml_path.empty()) {
        return cfg;
    }

    YAML::Node yaml = YAML::LoadFile(yaml_path);
    if (yaml["system"] && yaml["system"]["backend_variant"]) {
        cfg.variant_name = yaml["system"]["backend_variant"].as<std::string>();
    }

    const YAML::Node legacy = yaml["loop_closing"];
    AssignIfPresent(legacy, "loop_kf_gap", cfg.proposal.loop_kf_gap);
    AssignIfPresent(legacy, "min_id_interval", cfg.proposal.min_id_interval);
    AssignIfPresent(legacy, "closest_id_th", cfg.proposal.closest_id_th);
    AssignIfPresent(legacy, "max_range", cfg.proposal.max_range);
    AssignIfPresent(legacy, "ndt_score_th", cfg.filter.ndt_score_th);
    AssignIfPresent(legacy, "with_height", cfg.filter.with_height);

    const YAML::Node backend = yaml["backend"];
    const YAML::Node proposal = backend["loop_proposal"];
    AssignIfPresent(proposal, "loop_kf_gap", cfg.proposal.loop_kf_gap);
    AssignIfPresent(proposal, "min_id_interval", cfg.proposal.min_id_interval);
    AssignIfPresent(proposal, "closest_id_th", cfg.proposal.closest_id_th);
    AssignIfPresent(proposal, "max_range", cfg.proposal.max_range);

    const YAML::Node registration = backend["loop_registration"];
    AssignIfPresent(registration, "submap_idx_range", cfg.registration.submap_idx_range);
    AssignIfPresent(registration, "voxel_size_ratio", cfg.registration.voxel_size_ratio);
    AssignIfPresent(registration, "transformation_epsilon", cfg.registration.transformation_epsilon);
    AssignIfPresent(registration, "step_size", cfg.registration.step_size);
    AssignIfPresent(registration, "max_iterations", cfg.registration.max_iterations);
    if (registration && registration["ndt_resolutions"]) {
        cfg.registration.ndt_resolutions = registration["ndt_resolutions"].as<std::vector<double>>();
    }

    const YAML::Node filter = backend["loop_filter"];
    AssignIfPresent(filter, "ndt_score_th", cfg.filter.ndt_score_th);
    AssignIfPresent(filter, "robust_kernel", cfg.filter.robust_kernel);
    AssignIfPresent(filter, "robust_kernel_delta", cfg.filter.robust_kernel_delta);
    AssignIfPresent(filter, "with_height", cfg.filter.with_height);
    AssignIfPresent(filter, "height_noise", cfg.filter.height_noise);

    const YAML::Node pose_graph = backend["pose_graph"];
    AssignIfPresent(pose_graph, "motion_trans_noise", cfg.pose_graph.motion_trans_noise);
    AssignIfPresent(pose_graph, "motion_rot_noise", cfg.pose_graph.motion_rot_noise);
    AssignIfPresent(pose_graph, "loop_trans_noise", cfg.pose_graph.loop_trans_noise);
    AssignIfPresent(pose_graph, "loop_rot_noise", cfg.pose_graph.loop_rot_noise);
    AssignIfPresent(pose_graph, "optimize_iterations", cfg.pose_graph.optimize_iterations);

    if (cfg.variant_name == "robust_loops_v1" && (!filter || !filter["robust_kernel"])) {
        cfg.filter.robust_kernel = "dcs";
    }

    return cfg;
}

}  // namespace lightning::backend
