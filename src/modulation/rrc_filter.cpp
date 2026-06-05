#include "modulation/rrc_filter.hpp"
#include <cmath>
#include <stdexcept>

RRCFilter::RRCFilter(float rolloff, int sps, int span)
    : rolloff_(rolloff)
    , sps_(sps)
    , span_(span)
    , delay_(span * sps / 2)
{
    if (rolloff < 0.0f || rolloff > 1.0f)
        throw std::invalid_argument("rolloff must be in [0, 1]");
    if (sps < 1)
        throw std::invalid_argument("sps must be >= 1");
    compute_taps();
}

void RRCFilter::compute_taps() {
    int num_taps = span_ * sps_ + 1;
    taps_.resize(num_taps);

    float Ts    = 1.0f;
    float alpha = rolloff_;

    for (int i = 0; i < num_taps; ++i) {
        float t = static_cast<float>(i - delay_) / static_cast<float>(sps_);

        float h;

        if (t == 0.0f) {
            h = (1.0f / std::sqrt(static_cast<float>(sps_))) *
                (1.0f + alpha * (4.0f / M_PI - 1.0f));

        } else if (alpha != 0.0f &&
                   std::abs(t) == Ts / (4.0f * alpha)) {
            h = (alpha / std::sqrt(2.0f * static_cast<float>(sps_))) *
                ((1.0f + 2.0f / M_PI) * std::sin(M_PI / (4.0f * alpha)) +
                 (1.0f - 2.0f / M_PI) * std::cos(M_PI / (4.0f * alpha)));

        } else {
            float num = std::sin(M_PI * t * (1.0f - alpha)) +
                        4.0f * alpha * t * std::cos(M_PI * t * (1.0f + alpha));
            float den = M_PI * t *
                        (1.0f - std::pow(4.0f * alpha * t, 2.0f));
            h = (1.0f / std::sqrt(static_cast<float>(sps_))) * (num / den);
        }

        taps_[i] = h;
    }

    // normalize taps to unit energy so two cascaded RRC filters
    // don't attenuate the signal
    float energy = 0.0f;
    for (float t : taps_)
        energy += t * t;
    float norm = std::sqrt(energy);
    for (float& t : taps_)
        t /= norm;
}

std::vector<std::complex<float>>
RRCFilter::apply(const std::vector<std::complex<float>>& in) const {
    int taps  = static_cast<int>(taps_.size());
    int n_out = static_cast<int>(in.size());
    std::vector<std::complex<float>> out(n_out, {0.0f, 0.0f});

    for (int n = 0; n < n_out; ++n) {
        std::complex<float> acc = {0.0f, 0.0f};
        for (int k = 0; k < taps; ++k) {
            int idx = n - k;
            if (idx >= 0 && idx < n_out)
                acc += taps_[k] * in[idx];
        }
        out[n] = acc;
    }
    return out;
}