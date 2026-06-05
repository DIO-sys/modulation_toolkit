#pragma once
#include "modulation/modulator_base.hpp"
#include <cmath>
#include <stdexcept>

template <int N>
class QAMModulator : public ModulatorBase {
    static_assert(N == 16 || N == 64, "QAMModulator only supports 16 or 64");
    //total bits per symbol 
    static constexpr int BITS = (N == 16) ? 4 : 6;
    //points per axis in the constellation for each i and q 
    static constexpr int M    = (N == 16) ? 4 : 8; // points per axis

public:
    explicit QAMModulator(int sps, float rolloff = 0.35f, int span = 8);

    std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) override;

    int bits_per_symbol() const override { return BITS; }

private:
    std::vector<std::complex<float>>
        map_bits(const std::vector<uint8_t>& bits) const;

    std::vector<std::complex<float>>
        upsample(const std::vector<std::complex<float>>& symbols) const;

    // gray code an integer index to axis position
    float axis_value(int gray_index) const;

    // precomputed normalization factor so average symbol power = 1
    float norm_;
};