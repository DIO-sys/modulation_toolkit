#include "modulation/demodulator_base.hpp"
#include <stdexcept>

DemodulatorBase::DemodulatorBase(int sps, float rolloff, int span)
    : sps_(sps)
    , rrc_(rolloff, sps, span)
{}

std::vector<std::complex<float>>
DemodulatorBase::apply_rrc(const std::vector<std::complex<float>>& samples) const {
    return rrc_.apply(samples);
}

std::vector<float>
DemodulatorBase::demodulate_soft(const std::vector<std::complex<float>>&) {
    throw std::runtime_error(
        "demodulate_soft not implemented for this scheme");
}