#pragma once

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/backend/common/backend_config.h"

namespace lightning::backend {

class MultiResolutionNDTLoopRegistration {
   public:
    explicit MultiResolutionNDTLoopRegistration(LoopRegistrationOptions options) : options_(options) {}

    void Compute(LoopCandidate& candidate, const std::vector<Keyframe::Ptr>& all_keyframes) const;

   private:
    LoopRegistrationOptions options_;
};

}  // namespace lightning::backend
