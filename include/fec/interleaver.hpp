#pragma once
#include <vector>
#include <cstdint>
#include <stdexcept>

class Interleaver {
public:
    // rows x cols defines the interleaver depth
    // write bits row by row, read out column by column
    Interleaver(int rows, int cols);

    std::vector<uint8_t>
        interleave(const std::vector<uint8_t>& bits) const;

    std::vector<uint8_t>
        deinterleave(const std::vector<uint8_t>& bits) const;

    int size() const { return rows_ * cols_; }

private:
    int rows_;
    int cols_;
};