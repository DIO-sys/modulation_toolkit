#include "channel/awgn_channel.hpp"
#include <cmath>

AWGNChannel::AWGNChannel(float snr_db)
    : snr_db_(snr_db)
    , rng_(std::random_device{}())
    , dist_(0.0f, 1.0f)
{
}

//each sample gets different noise since rng reruns on sample
std::vector<std::complex<float>>
AWGNChannel::apply(const std::vector<std::complex<float>>& in) const {
    float sigma = compute_noise_sigma();

    std::vector<std::complex<float>> out;
    out.reserve(in.size());

    for (const auto& sample : in) {
        // independent gaussian noise on I and Q
        float ni = dist_(rng_) * sigma;
        float nq = dist_(rng_) * sigma;
        out.push_back({sample.real() + ni, sample.imag() + nq});
    }
    return out;
}

void AWGNChannel::set_snr(float snr_db) {
    snr_db_ = snr_db;
}

//standard deviation from noise per i and q componenet
float AWGNChannel::compute_noise_sigma() const {
    // SNR = signal_power / noise_power
    // noise_power = sigma^2
    // sigma = sqrt(signal_power / linear_snr)
    float linear_snr = std::pow(10.0f, snr_db_ / 10.0f);
    return std::sqrt(signal_power_ / (2.0f * linear_snr));
    // divide by 2 because noise is split across I and Q
}