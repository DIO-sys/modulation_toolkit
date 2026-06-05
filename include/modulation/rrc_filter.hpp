#pragma once
#include <vector>
#include <complex>

class RRCFilter {
public:
    // rolloff: alpha 0.0-1.0
    // sps: samples per symbol
    // span: number of symbols the filter spans (typically 6-10)
    RRCFilter(float rolloff, int sps, int span);

    // apply filter to a block of complex samples — returns filtered output
    std::vector<std::complex<float>>
        apply(const std::vector<std::complex<float>>& in) const;

    const std::vector<float>& coefficients() const { return taps_; }
    int delay() const { return delay_; }

private:
    void compute_taps();
    float rolloff_;
    int   sps_;
    int   span_;
    int   delay_;               // group delay in samples = span * sps / 2
    std::vector<float> taps_;   // filter coefficients
};