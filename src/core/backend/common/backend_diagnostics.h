#pragma once

#include <string>
#include <vector>

namespace lightning::backend {

struct LoopClosingDiagnostics {
    std::string variant_name = "baseline_current";
    int optimization_trigger_count = 0;
    int candidates_proposed = 0;
    int candidates_accepted = 0;
    int candidates_rejected = 0;
    int loop_outliers = 0;
    int total_loop_edges = 0;
    int optimizer_iterations = 0;
    double optimizer_runtime_ms = 0.0;
    std::vector<double> accepted_scores;
    std::vector<double> rejected_scores;
    std::vector<double> loop_edge_chi2;
};

void WriteLoopClosingDiagnostics(const LoopClosingDiagnostics& diagnostics);
void WriteLocalizationPgoDiagnostics(const std::string& payload_json);

}  // namespace lightning::backend
