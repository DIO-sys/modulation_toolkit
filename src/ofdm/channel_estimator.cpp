#include "ofdm/channel_estimator.hpp"
#include <stdexcept>
#include <cmath>

ChannelEstimator::ChannelEstimator(const ResourceGrid& grid,
                                    std::complex<float> pilot_value)
    : grid_(grid)
    , pilot_value_(pilot_value)
{}

std::vector<std::complex<float>>
ChannelEstimator::estimate(const std::vector<std::complex<float>>& rx_grid) const
{
    int num_sc  = grid_.num_subcarriers();
    int num_sym = grid_.num_symbols();

    // H[symbol * num_sc + subcarrier]
    std::vector<std::complex<float>> H(num_sc * num_sym, {1.0f, 0.0f});

    std::vector<std::complex<float>> prev_H(num_sc, {1.0f, 0.0f});

    for (int s = 0; s < num_sym; ++s) {
        auto pilot_scs = grid_.pilot_subcarriers(s);

        if (!pilot_scs.empty()) {
            // LS estimate at each pilot subcarrier
            // H[k] = Y[k] / X[k] where X[k] = pilot_value
            std::vector<std::complex<float>> pilot_H;
            pilot_H.reserve(pilot_scs.size());

            for (int sc : pilot_scs) {
                std::complex<float> Y = rx_grid[s * num_sc + sc];
                // LS: H = Y / X
                std::complex<float> h = Y / pilot_value_;
                pilot_H.push_back(h);
            }

            // interpolate to all subcarriers
            auto H_sym = interpolate(pilot_H, pilot_scs, num_sc);

            for (int k = 0; k < num_sc; ++k)
                H[s * num_sc + k] = H_sym[k];

            prev_H = H_sym;
        } else {
            // no pilots in this symbol — use previous estimate
            for (int k = 0; k < num_sc; ++k)
                H[s * num_sc + k] = prev_H[k];
        }
    }
    return H;
}

std::vector<std::complex<float>>
ChannelEstimator::equalize(const std::vector<std::complex<float>>& rx_grid,
                             const std::vector<std::complex<float>>& H) const
{
    int N = static_cast<int>(rx_grid.size());
    std::vector<std::complex<float>> equalized(N);

    for (int i = 0; i < N; ++i) {
        // one-tap equalizer: divide by channel estimate
        // avoid division by near-zero
        float mag_sq = H[i].real() * H[i].real() +
                       H[i].imag() * H[i].imag();

        if (mag_sq > 1e-10f)
            equalized[i] = rx_grid[i] / H[i];
        else
            equalized[i] = rx_grid[i];
    }
    return equalized;
}

std::vector<std::complex<float>>
ChannelEstimator::interpolate(
    const std::vector<std::complex<float>>& pilot_H,
    const std::vector<int>& pilot_indices,
    int num_subcarriers) const
{
    std::vector<std::complex<float>> H(num_subcarriers, {1.0f, 0.0f});

    if (pilot_H.empty()) return H;

    int num_pilots = static_cast<int>(pilot_indices.size());

    for (int k = 0; k < num_subcarriers; ++k) {
        // find surrounding pilot indices
        int left_idx  = 0;
        int right_idx = num_pilots - 1;

        for (int p = 0; p < num_pilots - 1; ++p) {
            if (pilot_indices[p] <= k && pilot_indices[p+1] >= k) {
                left_idx  = p;
                right_idx = p + 1;
                break;
            }
        }

        int left_sc  = pilot_indices[left_idx];
        int right_sc = pilot_indices[right_idx];

        if (left_sc == right_sc) {
            H[k] = pilot_H[left_idx];
        } else {
            // linear interpolation
            float t = static_cast<float>(k - left_sc) /
                      static_cast<float>(right_sc - left_sc);
            H[k] = pilot_H[left_idx] * (1.0f - t) +
                   pilot_H[right_idx] * t;
        }
    }
    return H;
}