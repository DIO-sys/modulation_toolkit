#pragma once
#include "modulation/rrc_filter.hpp"
#include <complex>
#include <vector>
#include <cstdint>

class DemodulatorBase {
public:
    DemodulatorBase(int sps, float rolloff = 0.35f, int span = 8);
    virtual ~DemodulatorBase() = default;

    virtual std::vector<uint8_t>
        demodulate(const std::vector<std::complex<float>>& samples) = 0;

    virtual std::vector<float>
        demodulate_soft(const std::vector<std::complex<float>>& samples);

    virtual int samples_per_symbol() const { return sps_; }
    virtual int bits_per_symbol()    const = 0;

protected:
    // subclasses call this on received samples before making decisions
    std::vector<std::complex<float>>
        apply_rrc(const std::vector<std::complex<float>>& samples) const;

    int       sps_;
    RRCFilter rrc_;
};