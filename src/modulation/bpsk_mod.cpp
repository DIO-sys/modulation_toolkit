#include "modulation/bpsk_mod.hpp"

BPSKModulator::BPSKModulator(int sps, float rolloff, int span)
    : ModulatorBase(sps, rolloff, span)
{}

//turn symbols bits into symbols upsample those symbols and appy rrc to those symbols
std::vector<std::complex<float>>
BPSKModulator::modulate(const std::vector<uint8_t>& bits) {
    auto symbols   = map_bits(bits);
    auto upsampled = upsample(symbols);
    return apply_rrc(upsampled);
}

//turn each one and 0 into a float. imaginary is always 0,
std::vector<std::complex<float>>
BPSKModulator::map_bits(const std::vector<uint8_t>& bits) const {
    std::vector<std::complex<float>> symbols;
    symbols.reserve(bits.size());

    for (uint8_t bit : bits) {
        // BPSK: 0 → +1, 1 → -1 on the real axis, imaginary is always 0
        float val = (bit == 0) ? 1.0f : -1.0f;
        symbols.push_back({val, 0.0f});
    }
    return symbols;
}

//turn samples and add sps-1 zeroes between each 
std::vector<std::complex<float>>
BPSKModulator::upsample(const std::vector<std::complex<float>>& symbols) const {
    // insert sps-1 zeros after each symbol
    std::vector<std::complex<float>> out(symbols.size() * sps_, {0.0f, 0.0f});
    for (size_t i = 0; i < symbols.size(); ++i)
        out[i * sps_] = symbols[i];
    return out;
}