#pragma once

#include <memory>
#include <vector>

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/backend/common/backend_config.h"
#include "core/backend/common/backend_diagnostics.h"
#include "core/backend/loop_filter/loop_filter_strategy.h"
#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"

namespace lightning::backend {

class LoopPoseGraph {
   public:
    explicit LoopPoseGraph(PoseGraphOptions options);

    void Init(const LoopClosingBackendConfig& config);
    int Optimize(Keyframe::Ptr current_kf, const std::vector<Keyframe::Ptr>& all_keyframes,
                 const std::vector<LoopCandidate>& accepted_candidates, const LoopFilterStrategy& filter,
                 LoopClosingDiagnostics& diagnostics);

   private:
    PoseGraphOptions options_;
    LoopFilterOptions filter_options_;
    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;
    Mat6d info_motion_ = Mat6d::Identity();
    Mat6d info_loops_ = Mat6d::Identity();
    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vertices_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> loop_edges_;
};

}  // namespace lightning::backend
