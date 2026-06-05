#pragma once
#include <complex>
#include <vector>
#include <random>

class AWGNChannel {
public:
    // snr_db: signal to noise ratio in dB
    explicit AWGNChannel(float snr_db);

    std::vector<std::complex<float>>
        apply(const std::vector<std::complex<float>>& in) const;

    void set_snr(float snr_db);
    float snr_db() const { return snr_db_; }

private:
    float compute_noise_sigma() const;

    float snr_db_;
    float signal_power_{ 1.0f };  // normalized — BPSK symbols are +/-1
    
    //mutables lets const variable modify variable. int rng is basically a random number generator from a range all with the same selection probability 
    mutable std::mt19937                     rng_;
    //transformation on top of the rng output used to pick a point on a normal bell curve with 99% of values between -3 and 3 
    mutable std::normal_distribution<float>  dist_;
};