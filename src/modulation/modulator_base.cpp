#include "modulation/modulator_base.hpp"

ModulatorBase::ModulatorBase(int sps, float rolloff, int span)
    : sps_(sps)
    , rrc_(rolloff, sps, span)
{}

std::vector<std::complex<float>>
ModulatorBase::apply_rrc(const std::vector<std::complex<float>>& symbols) const {
    return rrc_.apply(symbols);
}