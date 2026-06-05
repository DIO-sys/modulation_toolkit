#include "modulation/qam_mod.hpp"

template <int N>
QAMModulator<N>::QAMModulator(int sps, float rolloff, int span)
    : ModulatorBase(sps, rolloff, span)
{
    // average power normalization for square QAM
    // E[|s|^2] = (2/3)(M^2 - 1) for M-ary PAM on each axis
    float m = static_cast<float>(M);
    norm_ = std::sqrt(2.0f * (m * m - 1.0f) / 3.0f);
}

template <int N>
std::vector<std::complex<float>>
QAMModulator<N>::modulate(const std::vector<uint8_t>& bits) {
    if (bits.size() % BITS != 0)
        throw std::invalid_argument("bit count must be multiple of bits_per_symbol");
    auto symbols   = map_bits(bits);
    auto upsampled = upsample(symbols);
    return apply_rrc(upsampled);
}

template <int N>
float QAMModulator<N>::axis_value(int gray_index) const {
    // gray decode: convert gray code back to binary index
    int binary = gray_index;
    for (int mask = gray_index >> 1; mask != 0; mask >>= 1)
        binary ^= mask;

    // map binary index to constellation axis positions
    // e.g. 16-QAM: 0,1,2,3 → -3,-1,+1,+3 normalized
    float pos = static_cast<float>(2 * binary - (M - 1));
    return pos / norm_;
}

template <int N>
std::vector<std::complex<float>>
QAMModulator<N>::map_bits(const std::vector<uint8_t>& bits) const {
    std::vector<std::complex<float>> symbols;
    symbols.reserve(bits.size() / BITS);

    int half_bits = BITS / 2;  // bits per axis

    for (size_t i = 0; i < bits.size(); i += BITS) {
        // pack bits into I and Q gray indices
        int i_idx = 0;
        int q_idx = 0;

        for (int b = 0; b < half_bits; ++b) {
            i_idx = (i_idx << 1) | bits[i + b];
            q_idx = (q_idx << 1) | bits[i + half_bits + b];
        }

        float re = axis_value(i_idx);
        float im = axis_value(q_idx);
        symbols.push_back({re, im});
    }
    return symbols;
}

template <int N>
std::vector<std::complex<float>>
QAMModulator<N>::upsample(const std::vector<std::complex<float>>& symbols) const {
    std::vector<std::complex<float>> out(symbols.size() * sps_, {0.0f, 0.0f});
    for (size_t i = 0; i < symbols.size(); ++i)
        out[i * sps_] = symbols[i];
    return out;
}

// explicit instantiations
template class QAMModulator<16>;
template class QAMModulator<64>;