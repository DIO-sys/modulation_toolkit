#pragma once
#include "modulation/modulator_base.hpp"

class FMModulator : public ModulatorBase {
public:
    // freq_deviation: how far the carrier shifts per bit in normalized units
    FMModulator(int sps, float freq_deviation = 0.25f,
                float rolloff = 0.35f, int span = 8);

    std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) override;

    int bits_per_symbol() const override { return 1; }

private:
    float freq_dev_;
};