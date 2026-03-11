#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/loop_candidate.h"
#include "core/backend/common/backend_config.h"
#include "core/graph/edge.h"
#include "core/robust_kernel/robust_kernel.h"

namespace lightning::backend {

class LoopFilterStrategy {
   public:
    explicit LoopFilterStrategy(LoopFilterOptions options) : options_(options) {}
    virtual ~LoopFilterStrategy() = default;

    virtual bool AcceptCandidate(const LoopCandidate& candidate) const { return candidate.ndt_score_ > options_.ndt_score_th; }
    virtual std::shared_ptr<miao::RobustKernel> CreateLoopKernel() const = 0;
    virtual bool MarkAsOutlier(const miao::Edge& edge) const { return edge.Chi2() > options_.robust_kernel_delta; }
    virtual std::string Name() const = 0;

   protected:
    LoopFilterOptions options_;
};

std::unique_ptr<LoopFilterStrategy> MakeLoopFilterStrategy(const LoopClosingBackendConfig& config);

}  // namespace lightning::backend
