#include "channel/multipath_channel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <random>
#include <numeric>

MultipathChannel::MultipathChannel(
    std::vector<int>                 tap_delays,
    std::vector<std::complex<float>> tap_gains,
    float                            snr_db)
    : tap_delays_(tap_delays)
    , tap_gains_(tap_gains)
    , snr_db_(snr_db)
    , rng_(std::random_device{}())
    , dist_(0.0f, 1.0f)
{
    if (tap_delays.size() != tap_gains.size())
        throw std::invalid_argument(
            "tap_delays and tap_gains must have equal length");
    if (tap_delays.empty())
        throw std::invalid_argument("at least one tap required");

    max_delay_ = *std::max_element(tap_delays.begin(), tap_delays.end());
}

std::vector<std::complex<float>>
MultipathChannel::apply(const std::vector<std::complex<float>>& in) const
{
    int N     = static_cast<int>(in.size());
    float sigma = compute_noise_sigma();

    std::vector<std::complex<float>> out(N, {0.0f, 0.0f});

    // apply each tap — delayed and scaled copy of input
    for (size_t t = 0; t < tap_delays_.size(); ++t) {
        int   delay = tap_delays_[t];
        auto  gain  = tap_gains_[t];

        for (int n = 0; n < N; ++n) {
            int src = n - delay;
            if (src >= 0 && src < N)
                out[n] += gain * in[src];
        }
    }

    // add AWGN on top of multipath
    for (int n = 0; n < N; ++n) {
        float ni = dist_(rng_) * sigma;
        float nq = dist_(rng_) * sigma;
        out[n] += std::complex<float>(ni, nq);
    }

    return out;
}

float MultipathChannel::compute_noise_sigma() const {
    float linear_snr = std::pow(10.0f, snr_db_ / 10.0f);
    return std::sqrt(signal_power_ / (2.0f * linear_snr));
}