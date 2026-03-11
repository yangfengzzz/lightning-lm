//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/backend/common/backend_config.h"
#include "core/backend/common/backend_diagnostics.h"
#include "core/backend/loop_filter/loop_filter_strategy.h"
#include "core/backend/loop_proposal/spatial_radius_loop_proposal.h"
#include "core/backend/loop_registration/multi_resolution_ndt_registration.h"
#include "core/backend/pose_graph/loop_pose_graph.h"
#include "utils/async_message_process.h"

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

   protected:
    void HandleKF(Keyframe::Ptr kf);
    void DetectLoopCandidates();
    void ComputeLoopCandidates();
    void ComputeForCandidate(LoopCandidate& c);
    void PoseOptimization();

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    backend::LoopClosingBackendConfig backend_config_;
    backend::LoopClosingDiagnostics diagnostics_;
    std::unique_ptr<backend::SpatialRadiusLoopProposal> proposal_;
    std::unique_ptr<backend::MultiResolutionNDTLoopRegistration> registration_;
    std::unique_ptr<backend::LoopFilterStrategy> filter_;
    std::unique_ptr<backend::LoopPoseGraph> pose_graph_;

    LoopClosedCallback loop_cb_;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
