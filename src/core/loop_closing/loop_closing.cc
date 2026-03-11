//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"

#include <chrono>

#include "core/backend/common/backend_diagnostics.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    backend_config_ = backend::LoadLoopClosingBackendConfig(yaml_path);
    diagnostics_.variant_name = backend_config_.variant_name;

    proposal_ = std::make_unique<backend::SpatialRadiusLoopProposal>(backend_config_.proposal);
    registration_ = std::make_unique<backend::MultiResolutionNDTLoopRegistration>(backend_config_.registration);
    filter_ = backend::MakeLoopFilterStrategy(backend_config_);
    pose_graph_ = std::make_unique<backend::LoopPoseGraph>(backend_config_.pose_graph);
    pose_graph_->Init(backend_config_);

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    DetectLoopCandidates();
    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size()
                  << " backend_variant=" << backend_config_.variant_name;
    }

    ComputeLoopCandidates();
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();
    diagnostics_.optimization_trigger_count++;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        diagnostics_.candidates_proposed = 0;
        return;
    }

    bool updated_last_loop = false;
    candidates_ = proposal_->Detect(all_keyframes_, cur_kf_, last_loop_kf_, updated_last_loop);
    diagnostics_.candidates_proposed = static_cast<int>(candidates_.size());
    if (updated_last_loop) {
        last_loop_kf_ = cur_kf_;
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    diagnostics_.accepted_scores.clear();
    diagnostics_.rejected_scores.clear();
    if (candidates_.empty()) {
        diagnostics_.candidates_accepted = 0;
        diagnostics_.candidates_rejected = 0;
        return;
    }

    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });

    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (filter_->AcceptCandidate(lc)) {
            succ_candidates.emplace_back(lc);
            diagnostics_.accepted_scores.push_back(lc.ndt_score_);
        } else {
            diagnostics_.rejected_scores.push_back(lc.ndt_score_);
        }
    }

    diagnostics_.candidates_accepted = static_cast<int>(succ_candidates.size());
    diagnostics_.candidates_rejected = static_cast<int>(candidates_.size() - succ_candidates.size());

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    registration_->Compute(c, all_keyframes_);
}

void LoopClosing::PoseOptimization() {
    auto t0 = std::chrono::steady_clock::now();
    pose_graph_->Optimize(cur_kf_, all_keyframes_, candidates_, *filter_, diagnostics_);
    auto t1 = std::chrono::steady_clock::now();
    diagnostics_.optimizer_runtime_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
    backend::WriteLoopClosingDiagnostics(diagnostics_);

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << diagnostics_.loop_outliers << "/" << diagnostics_.total_loop_edges;
    }

    if (loop_cb_) {
        loop_cb_();
    }

    LOG(INFO) << "optimize finished, loops: " << diagnostics_.total_loop_edges
              << ", backend_variant=" << backend_config_.variant_name;
}

}  // namespace lightning
