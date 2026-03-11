#include "core/backend/loop_filter/loop_filter_strategy.h"

#include "core/robust_kernel/cauchy.h"
#include "core/robust_kernel/dcs.h"

namespace lightning::backend {
namespace {

class BaselineLoopFilterStrategy final : public LoopFilterStrategy {
   public:
    explicit BaselineLoopFilterStrategy(LoopFilterOptions options) : LoopFilterStrategy(options) {}
    std::shared_ptr<miao::RobustKernel> CreateLoopKernel() const override {
        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.robust_kernel_delta);
        return rk;
    }
    std::string Name() const override { return "baseline_current"; }
};

class RobustLoopsV1LoopFilterStrategy final : public LoopFilterStrategy {
   public:
    explicit RobustLoopsV1LoopFilterStrategy(LoopFilterOptions options) : LoopFilterStrategy(options) {}
    std::shared_ptr<miao::RobustKernel> CreateLoopKernel() const override {
        auto rk = std::make_shared<miao::RobustKernelDCS>();
        rk->SetDelta(options_.robust_kernel_delta);
        return rk;
    }
    std::string Name() const override { return "robust_loops_v1"; }
};

}  // namespace

std::unique_ptr<LoopFilterStrategy> MakeLoopFilterStrategy(const LoopClosingBackendConfig& config) {
    if (config.variant_name == "robust_loops_v1") {
        return std::make_unique<RobustLoopsV1LoopFilterStrategy>(config.filter);
    }
    return std::make_unique<BaselineLoopFilterStrategy>(config.filter);
}

}  // namespace lightning::backend
