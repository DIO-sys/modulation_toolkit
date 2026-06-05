#include "fec/conv_encoder.hpp"
#include <stdexcept>

ConvolutionalEncoder::ConvolutionalEncoder(int rate_denom,
                                           int constraint_length,
                                           std::vector<uint8_t> polynomials)
    : rate_denom_(rate_denom)
    , K_(constraint_length)
    , polynomials_(polynomials)
    , reg_mask_((1u << constraint_length) - 1)
{
    if (polynomials.size() != static_cast<size_t>(rate_denom))
        throw std::invalid_argument(
            "number of polynomials must equal rate denominator");
    if (constraint_length < 2 || constraint_length > 32)
        throw std::invalid_argument(
            "constraint length must be between 2 and 32");
}

std::vector<uint8_t>
ConvolutionalEncoder::encode(const std::vector<uint8_t>& bits) const {
    std::vector<uint8_t> coded;
    coded.reserve((bits.size() + K_ - 1) * rate_denom_);

    uint32_t shift_reg = 0;

    // encode input bits
    for (uint8_t bit : bits) {
        // shift register right, insert new bit at MSB
        shift_reg >>= 1;
        shift_reg  |= (static_cast<uint32_t>(bit) << (K_ - 1));
        shift_reg  &= reg_mask_;

        // one output bit per polynomial
        for (uint8_t poly : polynomials_)
            coded.push_back(compute_output(shift_reg, poly));
    }

    // flush K-1 tail bits — drive shift register back to zero
    for (int i = 0; i < K_ - 1; ++i) {
        shift_reg >>= 1;
        shift_reg  &= reg_mask_;

        for (uint8_t poly : polynomials_)
            coded.push_back(compute_output(shift_reg, poly));
    }

    return coded;
}

uint8_t ConvolutionalEncoder::compute_output(uint32_t shift_reg,
                                              uint8_t polynomial) const {
    // AND register with polynomial mask — select tapped positions
    uint32_t masked = shift_reg & static_cast<uint32_t>(polynomial);

    // XOR all selected bits together — popcount mod 2
    uint8_t parity = 0;
    while (masked) {
        parity ^= masked & 1;
        masked >>= 1;
    }
    return parity;
}