#pragma once
#include "ofdm/resource_grid.hpp"
#include <vector>
#include <complex>
#include <cstdint>

class OFDMModulator {
public:
    // num_subcarriers: FFT size
    // cp_len: cyclic prefix length in samples
    // grid: resource grid for pilot/data mapping
    OFDMModulator(int num_subcarriers,
                  int cp_len,
                  ResourceGrid grid);

    // takes QAM symbols, maps to grid, IFFT, prepends CP
    // returns time-domain samples
    std::vector<std::complex<float>>
        modulate(const std::vector<std::complex<float>>& qam_symbols) const;

    int num_subcarriers() const { return num_sc_; }
    int cp_len()          const { return cp_len_; }
    int symbol_len()      const { return num_sc_ + cp_len_; }

private:
    // run IFFT on one OFDM symbol — frequency to time domain
    std::vector<std::complex<float>>
        ifft(const std::vector<std::complex<float>>& freq_domain) const;

    // prepend cyclic prefix — copy last cp_len samples to front
    std::vector<std::complex<float>>
        add_cyclic_prefix(const std::vector<std::complex<float>>& symbol) const;

    int          num_sc_;
    int          cp_len_;
    ResourceGrid grid_;
};