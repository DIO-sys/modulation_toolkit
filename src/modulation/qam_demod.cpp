#include "modulation/qam_demod.hpp"

template <int N>
QAMDemodulator<N>::QAMDemodulator(int sps, float rolloff, int span)
    : DemodulatorBase(sps, rolloff, span)
{
    float m = static_cast<float>(M);
    norm_ = std::sqrt(2.0f * (m * m - 1.0f) / 3.0f);
}

template <int N>
std::vector<uint8_t>
QAMDemodulator<N>::demodulate(const std::vector<std::complex<float>>& samples) {
    auto filtered = filter_and_sync(samples);

    std::vector<uint8_t> bits;
    bits.reserve(filtered.size() * BITS);

    int half_bits = BITS / 2;

    for (const auto& sym : filtered) {
        // slice → binary index → gray encode → unpack bits
        int i_binary = slice_axis(sym.real());
        int q_binary = slice_axis(sym.imag());

        int i_gray = gray_encode(i_binary);
        int q_gray = gray_encode(q_binary);

        for (int b = half_bits - 1; b >= 0; --b)
            bits.push_back((i_gray >> b) & 1);
        for (int b = half_bits - 1; b >= 0; --b)
            bits.push_back((q_gray >> b) & 1);
    }
    return bits;
}

template <int N>
std::vector<float>
QAMDemodulator<N>::demodulate_soft(const std::vector<std::complex<float>>& samples) {
    auto filtered = filter_and_sync(samples);

    std::vector<float> llrs;
    llrs.reserve(filtered.size() * BITS);

    for (const auto& sym : filtered) {
        auto i_llrs = soft_bits_axis(sym.real());
        auto q_llrs = soft_bits_axis(sym.imag());
        for (float l : i_llrs) llrs.push_back(l);
        for (float l : q_llrs) llrs.push_back(l);
    }
    return llrs;
}

template <int N>
std::vector<std::complex<float>>
QAMDemodulator<N>::filter_and_sync_symbols(
    const std::vector<std::complex<float>>& samples)
{
    return filter_and_sync(samples);
}

template <int N>
int QAMDemodulator<N>::slice_axis(float val) const {
    // scale back to unnormalized grid [-M+1, M-1]
    float scaled = val * norm_;

    // clamp to valid range
    float min_pos = -(static_cast<float>(M) - 1.0f);
    float max_pos =   static_cast<float>(M) - 1.0f;
    scaled = std::max(min_pos, std::min(max_pos, scaled));

    // map to binary index 0..M-1
    int binary = static_cast<int>(
        (scaled + static_cast<float>(M) - 1.0f) / 2.0f + 0.5f);
    binary = std::max(0, std::min(M - 1, binary));

    return binary;
}

template <int N>
int QAMDemodulator<N>::gray_encode(int binary) const {
    return binary ^ (binary >> 1);
}

template <int N>
int QAMDemodulator<N>::gray_decode(int gray) const {
    int binary = gray;
    for (int mask = gray >> 1; mask != 0; mask >>= 1)
        binary ^= mask;
    return binary;
}

template <int N>
std::vector<float>
QAMDemodulator<N>::soft_bits_axis(float val) const {
    int half_bits = BITS / 2;
    std::vector<float> llrs(half_bits);

    float scaled = val * norm_;

    for (int b = 0; b < half_bits; ++b) {
        int   step     = 1 << (half_bits - 1 - b);
        float boundary = static_cast<float>(step) * 2.0f -
                         static_cast<float>(M);
        llrs[b] = scaled - boundary;
    }
    return llrs;
}

template class QAMDemodulator<16>;
template class QAMDemodulator<64>;