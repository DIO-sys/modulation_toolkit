#include "measurement/ber_analyzer.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm>

void BERAnalyzer::record(float snr_db, float ber) {
    results_.push_back({snr_db, ber});
}

void BERAnalyzer::write_csv(const std::string& path) const {
    std::ofstream f(path);
    if (!f)
        throw std::runtime_error("failed to open " + path);
    f << "snr_db,ber\n";
    for (const auto& [snr, ber] : results_)
        f << snr << "," << ber << "\n";
}

float BERAnalyzer::compute_ber(const std::vector<uint8_t>& tx,
                                const std::vector<uint8_t>& rx) {
    size_t n = std::min(tx.size(), rx.size());
    size_t errors = 0;
    for (size_t i = 0; i < n; ++i)
        if (tx[i] != rx[i]) ++errors;
    return static_cast<float>(errors) / static_cast<float>(n);
}