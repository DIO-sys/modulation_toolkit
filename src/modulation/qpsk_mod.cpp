#include "modulation/qpsk_mod.hpp"
#include <cmath>
#include <stdexcept>

static constexpr float INV_SQRT2 = 0.70710678118f;

QPSKModulator::QPSKModulator(int sps, float rolloff, int span)
    : ModulatorBase(sps, rolloff, span)
{}

std::vector<std::complex<float>>
QPSKModulator::modulate(const std::vector<uint8_t>& bits) {
    if (bits.size() % 2 != 0)
        throw std::invalid_argument("QPSK requires even number of bits");
    auto symbols   = map_bits(bits);
    auto upsampled = upsample(symbols);
    return apply_rrc(upsampled);
}

std::vector<std::complex<float>>
QPSKModulator::map_bits(const std::vector<uint8_t>& bits) const {
    std::vector<std::complex<float>> symbols;
    symbols.reserve(bits.size() / 2);

    for (size_t i = 0; i < bits.size(); i += 2) {
        uint8_t b0 = bits[i];
        uint8_t b1 = bits[i + 1];

        // Gray coded QPSK constellation:
        // 00 → +1+1j  (45°)
        // 01 → -1+1j  (135°)
        // 11 → -1-1j  (225°)
        // 10 → +1-1j  (315°)
        float re = (b0 == 0) ?  INV_SQRT2 : -INV_SQRT2;
        float im = (b1 == 0) ?  INV_SQRT2 : -INV_SQRT2;
        symbols.push_back({re, im});
    }
    return symbols;
}

std::vector<std::complex<float>>
QPSKModulator::upsample(const std::vector<std::complex<float>>& symbols) const {
    std::vector<std::complex<float>> out(symbols.size() * sps_, {0.0f, 0.0f});
    for (size_t i = 0; i < symbols.size(); ++i)
        out[i * sps_] = symbols[i];
    return out;
}