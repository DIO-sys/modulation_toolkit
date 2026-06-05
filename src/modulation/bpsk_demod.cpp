#include "modulation/bpsk_demod.hpp"

BPSKDemodulator::BPSKDemodulator(int sps, float rolloff, int span)
    : DemodulatorBase(sps, rolloff, span)
{}

std::vector<uint8_t>
BPSKDemodulator::demodulate(const std::vector<std::complex<float>>& samples) {
    auto filtered = matched_filter_and_downsample(samples);

    std::vector<uint8_t> bits;
    bits.reserve(filtered.size());

    for (const auto& sym : filtered) {
        // hard decision on real axis — imaginary is always ~0 for BPSK
        bits.push_back(sym.real() >= 0.0f ? 0 : 1);
    }
    return bits;
}

std::vector<float>
BPSKDemodulator::demodulate_soft(const std::vector<std::complex<float>>& samples) {
    auto filtered = matched_filter_and_downsample(samples);

    std::vector<float> llrs;
    llrs.reserve(filtered.size());

    for (const auto& sym : filtered) {
        // LLR for BPSK: positive = likely 0, negative = likely 1
        // real part is the natural soft metric for BPSK
        llrs.push_back(sym.real());
    }
    return llrs;
}

//for this function maximize snr by correlating received signals against pulse shape 
//then it takes all samples that aren't upsample 0s and adds themm to a vector
std::vector<std::complex<float>>
BPSKDemodulator::matched_filter_and_downsample(
    const std::vector<std::complex<float>>& samples) const
{
    auto filtered = apply_rrc(samples);

    std::vector<std::complex<float>> symbols;
    int offset = rrc_.delay() * 2; 
    for (int i = offset; i < static_cast<int>(filtered.size()); i += sps_)
        symbols.push_back(filtered[i]);

    return symbols;
}