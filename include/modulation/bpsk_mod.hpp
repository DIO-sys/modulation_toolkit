#pragma once
#include "modulation/modulator_base.hpp"


class BPSKModulator : public ModulatorBase {
public:
    explicit BPSKModulator(int sps, float rolloff = 0.35f, int span = 8);

    std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) override;

    int bits_per_symbol() const override { return 1; }

private:
    //every very 0 bit becomes +1+0j, every 1 bit becomes -1+0j
    std::vector<std::complex<float>>
        map_bits(const std::vector<uint8_t>& bits) const;
    //stretches the symbol vector by sps, inserting zeros between symbols so the RRC filter has samples to fill in.
    std::vector<std::complex<float>>
        upsample(const std::vector<std::complex<float>>& symbols) const;
};
