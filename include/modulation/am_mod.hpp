#pragma once
#include "modulation/modulator_base.hpp"

class AMModulator : public ModulatorBase {
public:
    // modulation_index: depth of amplitude modulation 0.0-1.0
    AMModulator(int sps, float modulation_index = 0.5f,
                float rolloff = 0.35f, int span = 8);

    std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) override;

    int bits_per_symbol() const override { return 1; }

private:
    float mod_index_;
};