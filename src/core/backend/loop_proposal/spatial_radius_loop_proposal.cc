#include "core/backend/loop_proposal/spatial_radius_loop_proposal.h"

namespace lightning::backend {

std::vector<LoopCandidate> SpatialRadiusLoopProposal::Detect(const std::vector<Keyframe::Ptr>& all_keyframes,
                                                             Keyframe::Ptr current_kf, Keyframe::Ptr last_loop_kf,
                                                             bool& updated_last_loop) const {
    updated_last_loop = false;
    std::vector<LoopCandidate> candidates;
    if (current_kf == nullptr) {
        return candidates;
    }

    Keyframe::Ptr check_first = nullptr;
    if (last_loop_kf != nullptr && (current_kf->GetID() - last_loop_kf->GetID()) <= options_.loop_kf_gap) {
        return candidates;
    }

    for (auto kf : all_keyframes) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval) {
            continue;
        }

        if (abs(int(kf->GetID() - current_kf->GetID())) < options_.closest_id_th) {
            break;
        }

        Vec3d dt = kf->GetOptPose().translation() - current_kf->GetOptPose().translation();
        if (dt.head<2>().norm() < options_.max_range) {
            LoopCandidate c(kf->GetID(), current_kf->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * current_kf->GetLIOPose();
            candidates.emplace_back(c);
            check_first = kf;
        }
    }

    updated_last_loop = !candidates.empty();
    return candidates;
}

}  // namespace lightning::backend
