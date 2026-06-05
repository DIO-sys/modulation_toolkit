#include "modulation/am_mod.hpp"
#include <cmath>

AMModulator::AMModulator(int sps, float modulation_index,
                         float rolloff, int span)
    : ModulatorBase(sps, rolloff, span)
    , mod_index_(modulation_index)
{}

std::vector<std::complex<float>>
AMModulator::modulate(const std::vector<uint8_t>& bits) {
    std::vector<std::complex<float>> symbols;
    symbols.reserve(bits.size());

    for (uint8_t bit : bits) {
        // AM: carrier amplitude scaled by message signal
        // bit 0 → high amplitude (1 + mod_index)
        // bit 1 → low amplitude  (1 - mod_index)
        float amplitude = (bit == 0) ? (1.0f + mod_index_)
                                     : (1.0f - mod_index_);
        symbols.push_back({amplitude, 0.0f});
    }

    auto upsampled = std::vector<std::complex<float>>(
        symbols.size() * sps_, {0.0f, 0.0f});
    for (size_t i = 0; i < symbols.size(); ++i)
        upsampled[i * sps_] = symbols[i];

    return apply_rrc(upsampled);
}