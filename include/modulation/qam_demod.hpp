#pragma once
#include "modulation/demodulator_base.hpp"
#include <cmath>

template <int N>
class QAMDemodulator : public DemodulatorBase {
    static_assert(N == 16 || N == 64, "QAMDemodulator only supports 16 or 64");
    static constexpr int BITS = (N == 16) ? 4 : 6;
    static constexpr int M    = (N == 16) ? 4 : 8;

public:
    explicit QAMDemodulator(int sps, float rolloff = 0.35f, int span = 8);

    std::vector<uint8_t>
        demodulate(const std::vector<std::complex<float>>& samples) override;

    std::vector<float>
        demodulate_soft(const std::vector<std::complex<float>>& samples) override;

    int bits_per_symbol() const override { return BITS; }

private:
    std::vector<std::complex<float>>
        filter_and_sync_symbols(const std::vector<std::complex<float>>& samples);

    // returns binary index of nearest constellation point on one axis
    int slice_axis(float val) const;

    // gray encode binary index → gray index
    int gray_encode(int binary) const;

    // gray decode gray index → binary index
    int gray_decode(int gray) const;

    // soft LLRs for one axis
    std::vector<float> soft_bits_axis(float val) const;

    float norm_;
};