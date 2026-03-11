#include "core/backend/pose_graph/loop_pose_graph.h"

#include "core/opti_algo/algo_select.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"

namespace lightning::backend {

LoopPoseGraph::LoopPoseGraph(PoseGraphOptions options) : options_(options) {}

void LoopPoseGraph::Init(const LoopClosingBackendConfig& config) {
    options_ = config.pose_graph;
    filter_options_ = config.filter;

    miao::OptimizerConfig optimizer_config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                           miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    optimizer_config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(optimizer_config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise * options_.motion_trans_noise);
    info_motion_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.motion_rot_noise * options_.motion_rot_noise);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise * options_.loop_trans_noise);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise * options_.loop_rot_noise);
}

int LoopPoseGraph::Optimize(Keyframe::Ptr current_kf, const std::vector<Keyframe::Ptr>& all_keyframes,
                            const std::vector<LoopCandidate>& accepted_candidates, const LoopFilterStrategy& filter,
                            LoopClosingDiagnostics& diagnostics) {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(current_kf->GetID());
    v->SetEstimate(current_kf->GetOptPose());
    optimizer_->AddVertex(v);
    kf_vertices_.emplace_back(v);

    for (int i = 1; i < 3; i++) {
        int id = current_kf->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);
            e->SetMeasurement(last_kf->GetLIOPose().inverse() * current_kf->GetLIOPose());
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
        }
    }

    if (filter_options_.with_height) {
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (filter_options_.height_noise * filter_options_.height_noise));
        optimizer_->AddEdge(e);
    }

    if (diagnostics.variant_name.empty()) {
        diagnostics.variant_name = filter.Name();
    }

    for (const auto& c : accepted_candidates) {
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, optimizer_->GetVertex(c.idx1_));
        e->SetVertex(1, optimizer_->GetVertex(c.idx2_));
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);
        e->SetRobustKernel(filter.CreateLoopKernel());
        optimizer_->AddEdge(e);
        loop_edges_.emplace_back(e);
    }

    if (accepted_candidates.empty()) {
        diagnostics.total_loop_edges = static_cast<int>(loop_edges_.size());
        return 0;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);
    const int iterations = optimizer_->Optimize(options_.optimize_iterations);

    int cnt_outliers = 0;
    diagnostics.loop_edge_chi2.clear();
    for (auto& e : loop_edges_) {
        diagnostics.loop_edge_chi2.push_back(e->Chi2());
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }
        if (filter.MarkAsOutlier(*e)) {
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            e->SetRobustKernel(nullptr);
        }
    }

    for (auto& vert : kf_vertices_) {
        all_keyframes[vert->GetId()]->SetOptPose(vert->Estimate());
    }

    diagnostics.loop_outliers = cnt_outliers;
    diagnostics.total_loop_edges = static_cast<int>(loop_edges_.size());
    diagnostics.optimizer_iterations = iterations;
    return iterations;
}

}  // namespace lightning::backend
