#pragma once
#include <vector>
#include <string>
#include <cstdint>

class BERAnalyzer {
public:
    // log one SNR/BER point
    void record(float snr_db, float ber);

    // write all recorded points to a CSV file
    void write_csv(const std::string& path) const;

    // compute BER from tx and rx bit vectors
    static float compute_ber(const std::vector<uint8_t>& tx,
                             const std::vector<uint8_t>& rx);
private:
    std::vector<std::pair<float, float>> results_;
};