#pragma once
#include "modulation/rrc_filter.hpp"
#include "modulation/mm_timing_pll.hpp"
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

    // off by default — fixed downsample used until enabled
    void enable_timing_recovery(bool on) { timing_recovery_ = on; }

protected:
    // matched filter only — used internally
    std::vector<std::complex<float>>
        apply_rrc(const std::vector<std::complex<float>>& samples) const;

    // matched filter then either PLL or fixed downsample
    // all demodulators call this instead of apply_rrc directly
    std::vector<std::complex<float>>
        filter_and_sync(const std::vector<std::complex<float>>& samples);

    int               sps_;
    RRCFilter         rrc_;
    MuellerMuellerPLL pll_;
    bool              timing_recovery_{ false };
};