#pragma once

#include <vector>

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/backend/common/backend_config.h"

namespace lightning::backend {

class SpatialRadiusLoopProposal {
   public:
    explicit SpatialRadiusLoopProposal(LoopProposalOptions options) : options_(options) {}

    std::vector<LoopCandidate> Detect(const std::vector<Keyframe::Ptr>& all_keyframes, Keyframe::Ptr current_kf,
                                      Keyframe::Ptr last_loop_kf, bool& updated_last_loop) const;

   private:
    LoopProposalOptions options_;
};

}  // namespace lightning::backend
