#include "core/backend/common/backend_diagnostics.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace lightning::backend {
namespace {

std::string JsonArray(const std::vector<double>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << std::fixed << std::setprecision(6) << values[i];
    }
    oss << "]";
    return oss.str();
}

}  // namespace

void WriteLoopClosingDiagnostics(const LoopClosingDiagnostics& diagnostics) {
    const char* path = std::getenv("LIGHTNING_BACKEND_DIAG_PATH");
    if (path == nullptr || *path == '\0') {
        return;
    }
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "{\n"
        << "  \"variant_name\": \"" << diagnostics.variant_name << "\",\n"
        << "  \"optimization_trigger_count\": " << diagnostics.optimization_trigger_count << ",\n"
        << "  \"candidates_proposed\": " << diagnostics.candidates_proposed << ",\n"
        << "  \"candidates_accepted\": " << diagnostics.candidates_accepted << ",\n"
        << "  \"candidates_rejected\": " << diagnostics.candidates_rejected << ",\n"
        << "  \"loop_outliers\": " << diagnostics.loop_outliers << ",\n"
        << "  \"total_loop_edges\": " << diagnostics.total_loop_edges << ",\n"
        << "  \"optimizer_iterations\": " << diagnostics.optimizer_iterations << ",\n"
        << "  \"optimizer_runtime_ms\": " << std::fixed << std::setprecision(3) << diagnostics.optimizer_runtime_ms
        << ",\n"
        << "  \"accepted_scores\": " << JsonArray(diagnostics.accepted_scores) << ",\n"
        << "  \"rejected_scores\": " << JsonArray(diagnostics.rejected_scores) << ",\n"
        << "  \"loop_edge_chi2\": " << JsonArray(diagnostics.loop_edge_chi2) << "\n"
        << "}\n";
}

void WriteLocalizationPgoDiagnostics(const std::string& payload_json) {
    const char* path = std::getenv("LIGHTNING_PGO_DIAG_PATH");
    if (path == nullptr || *path == '\0') {
        return;
    }
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << payload_json;
}

}  // namespace lightning::backend
