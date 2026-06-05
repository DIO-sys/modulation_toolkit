#pragma once
#include <vector>
#include <array>
#include <cstdint>
#include <limits>

class ViterbiDecoder {
public:
    ViterbiDecoder(int rate_denom,
                   int constraint_length,
                   std::vector<uint8_t> polynomials);

    // hard decision — input is coded bits (0 or 1)
    std::vector<uint8_t>
        decode_hard(const std::vector<uint8_t>& coded_bits) const;

    // soft decision — input is LLRs (positive = likely 0, negative = likely 1)
    std::vector<uint8_t>
        decode_soft(const std::vector<float>& llrs) const;

    int rate_denom()        const { return rate_denom_; }
    int constraint_length() const { return K_; }
    int num_states()        const { return num_states_; }

private:
    void build_trellis();

    uint8_t compute_output(uint32_t shift_reg, int poly_idx) const;

    template <typename Received, typename BranchMetricFn>
    std::vector<uint8_t> decode_core(const Received& received,
                                      BranchMetricFn branch_metric) const;

    int                  rate_denom_;
    int                  K_;
    int                  num_states_;
    std::vector<uint8_t> polynomials_;

    std::vector<std::array<uint32_t, 2>>               next_state_;
    std::vector<std::array<std::vector<uint8_t>, 2>>   output_bits_;
};