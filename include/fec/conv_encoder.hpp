#pragma once
#include <vector>
#include <cstdint>

class ConvolutionalEncoder {
public:
    // rate: 1/2 or 1/3
    // constraint_length: K — number of shift register stages
    // polynomials: generator polynomials in octal
    //   rate 1/2 K=7 standard: {0133, 0171}
    //   rate 1/3 K=7 standard: {0133, 0145, 0175}
    ConvolutionalEncoder(int rate_denom,
                         int constraint_length,
                         std::vector<uint8_t> polynomials);

    // encode a bit stream — returns coded bits
    // output length = input length * rate_denom
    std::vector<uint8_t>
        encode(const std::vector<uint8_t>& bits) const;

    int rate_denom()         const { return rate_denom_; }
    int constraint_length()  const { return K_; }

private:
    // compute one output bit from current shift register state
    uint8_t compute_output(uint32_t shift_reg, uint8_t polynomial) const;

    int                  rate_denom_;
    int                  K_;
    std::vector<uint8_t> polynomials_;
    uint32_t             reg_mask_;   // K-1 bit mask
};