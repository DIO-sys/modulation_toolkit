#pragma once
#include "modulation/demodulator_base.hpp"

class QPSKDemodulator : public DemodulatorBase {
public:
    explicit QPSKDemodulator(int sps, float rolloff = 0.35f, int span = 8);

    std::vector<uint8_t>
        demodulate(const std::vector<std::complex<float>>& samples) override;

    std::vector<float>
        demodulate_soft(const std::vector<std::complex<float>>& samples) override;

    int bits_per_symbol() const override { return 2; }

private:
    std::vector<std::complex<float>>
        matched_filter_and_downsample(const std::vector<std::complex<float>>& samples) const;
};