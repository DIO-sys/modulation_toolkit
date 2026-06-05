#pragma once
#include "modulation/modulator_base.hpp"

class QPSKModulator : public ModulatorBase {
public:
    explicit QPSKModulator(int sps, float rolloff = 0.35f, int span = 8);

    std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) override;

    int bits_per_symbol() const override { return 2; }

private:
    std::vector<std::complex<float>>
        map_bits(const std::vector<uint8_t>& bits) const;

    std::vector<std::complex<float>>
        upsample(const std::vector<std::complex<float>>& symbols) const;
};