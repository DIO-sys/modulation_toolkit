#pragma once
#include <complex>
#include <vector>
#include <random>

class AWGNChannel {
public:
    explicit AWGNChannel(float snr_db);

    std::vector<std::complex<float>>
        apply(const std::vector<std::complex<float>>& in) const;

    void set_snr(float snr_db);
    void set_signal_power(float power) { signal_power_ = power; }
    float snr_db() const { return snr_db_; }

private:
    float compute_noise_sigma() const;

    float snr_db_;
    float signal_power_{ 1.0f };

    mutable std::mt19937                    rng_;
    mutable std::normal_distribution<float> dist_;
};