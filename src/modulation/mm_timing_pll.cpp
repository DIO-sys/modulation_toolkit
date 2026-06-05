#include "modulation/mm_timing_pll.hpp"
#include <cmath>

MuellerMuellerPLL::MuellerMuellerPLL(int sps, float loop_bw)
    : sps_(sps)
    , loop_bw_(loop_bw)
    , mu_(0.0f)
    , omega_(static_cast<float>(sps))
    , omega_mid_(static_cast<float>(sps))
    , omega_rel_(0.005f * static_cast<float>(sps))  // allow 0.5% clock deviation
    , prev_sample_(0.0f, 0.0f)
    , prev_decision_(0.0f, 0.0f)
{
    // second order loop filter coefficients from loop bandwidth
    float denom = 1.0f + 2.0f * loop_bw + loop_bw * loop_bw;
    alpha_ = (4.0f * loop_bw * loop_bw) / denom;
    beta_  = (4.0f * loop_bw)           / denom;
}

std::vector<std::complex<float>>
MuellerMuellerPLL::process(const std::vector<std::complex<float>>& samples) {
    std::vector<std::complex<float>> out;
    out.reserve(samples.size() / sps_);

    int n = static_cast<int>(samples.size());
    float strobe = mu_;   // fractional offset into current symbol period

    for (int i = sps_; i < n - sps_; ) {
        // interpolate sample at current timing estimate
        std::complex<float> sample = interpolate(samples, strobe, i);

        // hard decision
        std::complex<float> decision = decide(sample);

        // Mueller-Müller timing error detector
        // e = Re{ decision[n] * conj(prev_sample) - prev_decision * conj(sample) }
        float error = (decision.real()      * prev_sample_.real() +
                       decision.imag()      * prev_sample_.imag()) -
                      (prev_decision_.real() * sample.real() +
                       prev_decision_.imag() * sample.imag());

        // clamp error to prevent runaway
        error = std::max(-1.0f, std::min(1.0f, error));

        // loop filter — update omega and mu
        omega_ += beta_ * error;
        omega_  = std::max(omega_mid_ - omega_rel_,
                  std::min(omega_mid_ + omega_rel_, omega_));

        mu_    += omega_ + alpha_ * error;

        // advance by integer part of mu
        int advance = static_cast<int>(std::floor(mu_));
        mu_ -= static_cast<float>(advance);
        i   += advance;

        prev_sample_   = sample;
        prev_decision_ = decision;

        out.push_back(decision);
    }

    return out;
}

void MuellerMuellerPLL::reset() {
    mu_            = 0.0f;
    omega_         = omega_mid_;
    prev_sample_   = {0.0f, 0.0f};
    prev_decision_ = {0.0f, 0.0f};
}

std::complex<float>
MuellerMuellerPLL::decide(const std::complex<float>& sample) const {
    // BPSK/QPSK/QAM agnostic slicer — nearest unit constellation point
    float re = sample.real() >= 0.0f ? 1.0f : -1.0f;
    float im = sample.imag() >= 0.0f ? 1.0f : -1.0f;
    return {re, im};
}

std::complex<float>
MuellerMuellerPLL::interpolate(const std::vector<std::complex<float>>& samples,
                                float offset, int index) const {
    // linear interpolation between sample[index] and sample[index+1]
    int i0 = index;
    int i1 = std::min(index + 1, static_cast<int>(samples.size()) - 1);
    return samples[i0] * (1.0f - offset) + samples[i1] * offset;
}