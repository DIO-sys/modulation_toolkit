#include "modulation/qpsk_demod.hpp"

QPSKDemodulator::QPSKDemodulator(int sps, float rolloff, int span)
    : DemodulatorBase(sps, rolloff, span)
{}

std::vector<uint8_t>
QPSKDemodulator::demodulate(const std::vector<std::complex<float>>& samples) {
    auto filtered = matched_filter_and_downsample(samples);

    std::vector<uint8_t> bits;
    bits.reserve(filtered.size() * 2);

    for (const auto& sym : filtered) {
        // hard decision on each axis independently
        // real axis → b0, imag axis → b1
        bits.push_back(sym.real() >= 0.0f ? 0 : 1);
        bits.push_back(sym.imag() >= 0.0f ? 0 : 1);
    }
    return bits;
}

std::vector<float>
QPSKDemodulator::demodulate_soft(const std::vector<std::complex<float>>& samples) {
    auto filtered = matched_filter_and_downsample(samples);

    std::vector<float> llrs;
    llrs.reserve(filtered.size() * 2);

    for (const auto& sym : filtered) {
        // real part → LLR for b0, imag part → LLR for b1
        llrs.push_back(sym.real());
        llrs.push_back(sym.imag());
    }
    return llrs;
}

std::vector<std::complex<float>>
QPSKDemodulator::matched_filter_and_downsample(
    const std::vector<std::complex<float>>& samples) const
{
    auto filtered = apply_rrc(samples);

    std::vector<std::complex<float>> symbols;
    int offset = rrc_.delay() * 2;
    for (int i = offset; i < static_cast<int>(filtered.size()); i += sps_)
        symbols.push_back(filtered[i]);

    return symbols;
}