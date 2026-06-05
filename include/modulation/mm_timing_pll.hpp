#pragma once
#include <complex>
#include <vector>

class MuellerMuellerPLL {
public:
    // sps: samples per symbol
    // loop_bw: loop bandwidth — controls tracking speed vs noise
    MuellerMuellerPLL(int sps, float loop_bw = 0.01f);

    // process received samples — returns symbols at corrected sampling instants
    std::vector<std::complex<float>>
        process(const std::vector<std::complex<float>>& samples);

    void reset();

private:
    // hard decision on a complex sample — BPSK/QPSK/QAM agnostic
    std::complex<float> decide(const std::complex<float>& sample) const;

    // linear interpolation between samples
    std::complex<float> interpolate(const std::vector<std::complex<float>>& samples,
                                    float offset, int index) const;

    int   sps_;
    float loop_bw_;

    // loop filter coefficients
    float alpha_;   // proportional gain
    float beta_;    // integral gain

    // loop state
    float mu_;          // fractional timing offset [0, 1)
    float omega_;       // current samples per symbol estimate
    float omega_mid_;   // nominal sps
    float omega_rel_;   // max deviation from nominal

    std::complex<float> prev_sample_;
    std::complex<float> prev_decision_;
};