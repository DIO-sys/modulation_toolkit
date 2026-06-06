#pragma once
#include <vector>
#include <complex>
#include <cstdint>

// subcarriers arranged in frequency, OFDM symbols in time
// pilot symbols inserted at fixed subcarrier/symbol positions
// data symbols fill remaining positions

class ResourceGrid {
public:
    // num_subcarriers: total subcarriers (e.g. 64)
    // num_symbols: OFDM symbols per frame
    // pilot_spacing: every Nth subcarrier is a pilot
    // pilot_symbol_period: every Nth OFDM symbol carries pilots
    ResourceGrid(int num_subcarriers,
                 int num_symbols,
                 int pilot_spacing,
                 int pilot_symbol_period);

    // map QAM symbols onto grid — fills data positions
    // returns total grid as flat vector [symbol * num_subcarriers + subcarrier]
    std::vector<std::complex<float>>
        map(const std::vector<std::complex<float>>& data_symbols,
            const std::complex<float>& pilot_value = {1.0f, 0.0f}) const;

    // extract data symbols from received grid — strips pilots
    std::vector<std::complex<float>>
        unmap(const std::vector<std::complex<float>>& grid) const;

    // get pilot positions — (symbol_index, subcarrier_index) pairs
    std::vector<std::pair<int,int>> pilot_positions() const;

    // get pilot subcarrier indices for a given symbol
    std::vector<int> pilot_subcarriers(int symbol_idx) const;

    int num_subcarriers()     const { return num_sc_; }
    int num_symbols()         const { return num_sym_; }
    int num_data_per_symbol() const { return data_per_sym_; }
    int num_pilots_per_symbol() const { return pilots_per_sym_; }
    int total_data_symbols()  const { return data_per_sym_ * num_sym_; }

private:
    bool is_pilot(int symbol_idx, int subcarrier_idx) const;

    int num_sc_;
    int num_sym_;
    int pilot_spacing_;
    int pilot_sym_period_;
    int data_per_sym_;
    int pilots_per_sym_;
};