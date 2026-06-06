#include "ofdm/resource_grid.hpp"
#include <stdexcept>

ResourceGrid::ResourceGrid(int num_subcarriers,
                            int num_symbols,
                            int pilot_spacing,
                            int pilot_symbol_period)
    : num_sc_(num_subcarriers)
    , num_sym_(num_symbols)
    , pilot_spacing_(pilot_spacing)
    , pilot_sym_period_(pilot_symbol_period)
{
    if (num_subcarriers < 1 || num_symbols < 1)
        throw std::invalid_argument("subcarriers and symbols must be >= 1");
    if (pilot_spacing < 1)
        throw std::invalid_argument("pilot spacing must be >= 1");

    // count pilots per symbol — every pilot_spacing-th subcarrier
    pilots_per_sym_ = num_sc_ / pilot_spacing_;
    data_per_sym_   = num_sc_ - pilots_per_sym_;
}

bool ResourceGrid::is_pilot(int symbol_idx, int subcarrier_idx) const {
    // pilots appear on pilot symbol periods at every pilot_spacing subcarrier
    bool pilot_sym = (symbol_idx % pilot_sym_period_ == 0);
    bool pilot_sc  = (subcarrier_idx % pilot_spacing_ == 0);
    return pilot_sym && pilot_sc;
}

std::vector<std::complex<float>>
ResourceGrid::map(const std::vector<std::complex<float>>& data_symbols,
                   const std::complex<float>& pilot_value) const
{
    int grid_size = num_sc_ * num_sym_;
    std::vector<std::complex<float>> grid(grid_size, {0.0f, 0.0f});

    int data_idx = 0;
    for (int sym = 0; sym < num_sym_; ++sym) {
        for (int sc = 0; sc < num_sc_; ++sc) {
            int grid_idx = sym * num_sc_ + sc;
            if (is_pilot(sym, sc)) {
                grid[grid_idx] = pilot_value;
            } else {
                if (data_idx < static_cast<int>(data_symbols.size()))
                    grid[grid_idx] = data_symbols[data_idx++];
            }
        }
    }
    return grid;
}

std::vector<std::complex<float>>
ResourceGrid::unmap(const std::vector<std::complex<float>>& grid) const {
    std::vector<std::complex<float>> data;
    data.reserve(total_data_symbols());

    for (int sym = 0; sym < num_sym_; ++sym)
        for (int sc = 0; sc < num_sc_; ++sc)
            if (!is_pilot(sym, sc))
                data.push_back(grid[sym * num_sc_ + sc]);

    return data;
}

std::vector<std::pair<int,int>> ResourceGrid::pilot_positions() const {
    std::vector<std::pair<int,int>> positions;
    for (int sym = 0; sym < num_sym_; ++sym)
        for (int sc = 0; sc < num_sc_; ++sc)
            if (is_pilot(sym, sc))
                positions.push_back({sym, sc});
    return positions;
}

std::vector<int> ResourceGrid::pilot_subcarriers(int symbol_idx) const {
    std::vector<int> pilots;
    if (symbol_idx % pilot_sym_period_ == 0)
        for (int sc = 0; sc < num_sc_; ++sc)
            if (sc % pilot_spacing_ == 0)
                pilots.push_back(sc);
    return pilots;
}