#pragma once
#include "ofdm/resource_grid.hpp"
#include <vector>
#include <complex>

class ChannelEstimator {
public:
    // grid: resource grid with pilot positions
    // pilot_value: known transmitted pilot symbol
    ChannelEstimator(const ResourceGrid& grid,
                     std::complex<float> pilot_value = {1.0f, 0.0f});

    // least-squares channel estimation using received pilots
    // returns per-subcarrier channel response H[k] for each symbol
    // output: flat vector [symbol * num_subcarriers + subcarrier]
    std::vector<std::complex<float>>
        estimate(const std::vector<std::complex<float>>& rx_grid) const;

    // one-tap equalizer — divide each subcarrier by H[k] also corrects channel distotion
    std::vector<std::complex<float>>
        equalize(const std::vector<std::complex<float>>& rx_grid,
                 const std::vector<std::complex<float>>& H) const;

private:
    // interpolate channel estimates between pilot subcarriers
    // fills non-pilot subcarriers by linear interpolation
    std::vector<std::complex<float>>
        interpolate(const std::vector<std::complex<float>>& pilot_estimates,
                    const std::vector<int>& pilot_indices,
                    int num_subcarriers) const;

    const ResourceGrid&  grid_;
    std::complex<float>  pilot_value_;
};