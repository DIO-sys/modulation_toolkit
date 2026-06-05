#include "modulation/demodulator_base.hpp"
#include <stdexcept>

DemodulatorBase::DemodulatorBase(int sps, float rolloff, int span)
    : sps_(sps)
    , rrc_(rolloff, sps, span)
    , pll_(sps)
{}

std::vector<std::complex<float>>
DemodulatorBase::apply_rrc(const std::vector<std::complex<float>>& samples) const {
    return rrc_.apply(samples);
}

std::vector<std::complex<float>>
DemodulatorBase::filter_and_sync(const std::vector<std::complex<float>>& samples) {
    // matched filter first — always
    auto filtered = rrc_.apply(samples);

    if (timing_recovery_) {
        // PLL replaces fixed downsample — advances through buffer at omega_ rate
        return pll_.process(filtered);
    }

    // fixed downsample — assumes perfect clock alignment
    std::vector<std::complex<float>> symbols;
    int offset = rrc_.delay() * 2;
    for (int i = offset; i < static_cast<int>(filtered.size()); i += sps_)
        symbols.push_back(filtered[i]);
    return symbols;
}

std::vector<float>
DemodulatorBase::demodulate_soft(const std::vector<std::complex<float>>&) {
    throw std::runtime_error(
        "demodulate_soft not implemented for this scheme");
}