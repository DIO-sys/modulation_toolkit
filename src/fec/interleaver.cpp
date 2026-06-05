#include "fec/interleaver.hpp"

Interleaver::Interleaver(int rows, int cols)
    : rows_(rows)
    , cols_(cols)
{
    if (rows < 1 || cols < 1)
        throw std::invalid_argument("rows and cols must be >= 1");
}

std::vector<uint8_t>
Interleaver::interleave(const std::vector<uint8_t>& bits) const {
    // pad input to multiple of rows*cols
    int block = rows_ * cols_;
    std::vector<uint8_t> padded = bits;
    while (static_cast<int>(padded.size()) % block != 0)
        padded.push_back(0);

    std::vector<uint8_t> out(padded.size());

    // write row by row, read column by column
    // out[col * rows + row] = in[row * cols + col]
    for (int i = 0; i < static_cast<int>(padded.size()); i += block) {
        for (int row = 0; row < rows_; ++row)
            for (int col = 0; col < cols_; ++col)
                out[i + col * rows_ + row] = padded[i + row * cols_ + col];
    }
    return out;
}

std::vector<uint8_t>
Interleaver::deinterleave(const std::vector<uint8_t>& bits) const {
    int block = rows_ * cols_;
    std::vector<uint8_t> out(bits.size());

    // reverse — write column by column, read row by row
    for (int i = 0; i < static_cast<int>(bits.size()); i += block) {
        for (int row = 0; row < rows_; ++row)
            for (int col = 0; col < cols_; ++col)
                out[i + row * cols_ + col] = bits[i + col * rows_ + row];
    }
    return out;
}