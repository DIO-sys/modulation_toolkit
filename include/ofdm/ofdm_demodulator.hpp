#pragma once
#include "ofdm/resource_grid.hpp"
#include "ofdm/channel_estimator.hpp"
#include <vector>
#include <complex>

class OFDMDemodulator {
public:
    OFDMDemodulator(int num_subcarriers,
                    int cp_len,
                    ResourceGrid grid);

    // takes time-domain samples
    // strips CP, FFT, channel estimate, equalize, unmap
    // returns QAM symbols
    std::vector<std::complex<float>>
        demodulate(const std::vector<std::complex<float>>& samples) const;

    int num_subcarriers() const { return num_sc_; }
    int cp_len()          const { return cp_len_; }
    int symbol_len()      const { return num_sc_ + cp_len_; }

private:
    // run FFT on one OFDM symbol — time to frequency domain
    std::vector<std::complex<float>>
        fft(const std::vector<std::complex<float>>& time_domain) const;

    // strip cyclic prefix — drop first cp_len samples
    std::vector<std::complex<float>>
        remove_cyclic_prefix(const std::vector<std::complex<float>>& symbol) const;

    int          num_sc_;
    int          cp_len_;
    ResourceGrid grid_;
};