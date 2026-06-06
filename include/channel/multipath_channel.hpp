#pragma once
#include <vector>
#include <complex>
#include <random>

class MultipathChannel {
public:
    // tap_delays: sample delays for each path
    // tap_gains: complex gain for each path
    // snr_db: AWGN noise level on top of multipath
    MultipathChannel(std::vector<int>          tap_delays,
                     std::vector<std::complex<float>> tap_gains,
                     float snr_db);

    std::vector<std::complex<float>>
        apply(const std::vector<std::complex<float>>& in) const;

    void set_snr(float snr_db) { snr_db_ = snr_db; }
    float snr_db() const { return snr_db_; }
    int max_delay() const { return max_delay_; }

    void set_signal_power(float power) { signal_power_ = power; }

private:
    float compute_noise_sigma() const;

    std::vector<int>                 tap_delays_;
    std::vector<std::complex<float>> tap_gains_;
    float                            snr_db_;
    float                            signal_power_{ 1.0f };
    int                              max_delay_;

    mutable std::mt19937                    rng_;
    mutable std::normal_distribution<float> dist_;
};