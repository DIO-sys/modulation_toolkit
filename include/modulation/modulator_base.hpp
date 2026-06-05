#pragma once
#include "modulation/rrc_filter.hpp"  // remove the modulator_base.hpp include
#include <complex>
#include <vector>
#include <cstdint>
//modulate (apply bits to symbols) then apply rrc  
class ModulatorBase {
public:
    // sps: samples per symbol
    // rolloff: RRC alpha
    // span: RRC filter span in symbols
    ModulatorBase(int sps, float rolloff = 0.35f, int span = 8);
    virtual ~ModulatorBase() = default;

    virtual std::vector<std::complex<float>>
        modulate(const std::vector<uint8_t>& bits) = 0;
    //design choice in time how many samples go in and out of the hardware. higher = faster transmission = more demanding
    virtual int samples_per_symbol() const { return sps_; }
    //fixed by constellation  
    virtual int bits_per_symbol()    const = 0;

protected:
    // subclasses call this after mapping bits to symbols
    std::vector<std::complex<float>>
        apply_rrc(const std::vector<std::complex<float>>& symbols) const;

    int       sps_;
    RRCFilter rrc_;
};