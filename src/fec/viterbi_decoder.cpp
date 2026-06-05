#include "fec/viterbi_decoder.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>

static constexpr float INF = std::numeric_limits<float>::infinity();

ViterbiDecoder::ViterbiDecoder(int rate_denom,
                                int constraint_length,
                                std::vector<uint8_t> polynomials)
    : rate_denom_(rate_denom)
    , K_(constraint_length)
    , num_states_(1 << (constraint_length - 1))
    , polynomials_(polynomials)
{
    if (polynomials.size() != static_cast<size_t>(rate_denom))
        throw std::invalid_argument(
            "number of polynomials must equal rate denominator");
    build_trellis();
}

void ViterbiDecoder::build_trellis() {
    next_state_.resize(num_states_);
    output_bits_.resize(num_states_);

    for (int state = 0; state < num_states_; ++state) {
        for (int input = 0; input < 2; ++input) {
            uint32_t shift_reg = (static_cast<uint32_t>(input) << (K_ - 1))
                                 | static_cast<uint32_t>(state);

            next_state_[state][input] =
                static_cast<uint32_t>(shift_reg >> 1) & (num_states_ - 1);

            output_bits_[state][input].resize(rate_denom_);
            for (int p = 0; p < rate_denom_; ++p)
                output_bits_[state][input][p] = compute_output(shift_reg, p);
        }
    }
}

uint8_t ViterbiDecoder::compute_output(uint32_t shift_reg, int poly_idx) const {
    uint32_t masked = shift_reg & static_cast<uint32_t>(polynomials_[poly_idx]);
    uint8_t parity = 0;
    while (masked) {
        parity ^= masked & 1;
        masked >>= 1;
    }
    return parity;
}

template <typename Received, typename BranchMetricFn>
std::vector<uint8_t>
ViterbiDecoder::decode_core(const Received& received,
                             BranchMetricFn branch_metric) const
{
    int num_symbols = static_cast<int>(received.size()) / rate_denom_;

    std::vector<float> path_metric(num_states_, INF);
    path_metric[0] = 0.0f;

    // store both previous state and input bit at each trellis stage
    std::vector<std::vector<std::pair<uint32_t, uint8_t>>> survivor(
        num_symbols,
        std::vector<std::pair<uint32_t, uint8_t>>(num_states_, {0, 0}));

    // forward pass — ACS
    for (int t = 0; t < num_symbols; ++t) {
        std::vector<float> new_metric(num_states_, INF);

        for (int state = 0; state < num_states_; ++state) {
            if (path_metric[state] == INF) continue;

            for (int input = 0; input < 2; ++input) {
                uint32_t next        = next_state_[state][input];
                const auto& expected = output_bits_[state][input];

                float bm = 0.0f;
                for (int p = 0; p < rate_denom_; ++p)
                    bm += branch_metric(expected[p],
                                        received[t * rate_denom_ + p]);

                float candidate = path_metric[state] + bm;

                if (candidate < new_metric[next]) {
                    new_metric[next]  = candidate;
                    // store previous state AND input bit
                    survivor[t][next] = {static_cast<uint32_t>(state),
                                         static_cast<uint8_t>(input)};
                }
            }
        }
        path_metric = new_metric;
    }

    // traceback — follow previous states back to start
    uint32_t state = static_cast<uint32_t>(
        std::min_element(path_metric.begin(), path_metric.end())
        - path_metric.begin());

    std::vector<uint8_t> decoded(num_symbols);

    for (int t = num_symbols - 1; t >= 0; --t) {
        auto [prev_state, input_bit] = survivor[t][state];
        decoded[t] = input_bit;
        state      = prev_state;
    }

    // strip K-1 tail bits
    decoded.resize(num_symbols - (K_ - 1));
    return decoded;
}
std::vector<uint8_t>
ViterbiDecoder::decode_hard(const std::vector<uint8_t>& coded_bits) const {
    return decode_core(coded_bits,
        [](uint8_t expected, uint8_t received) -> float {
            return (expected != received) ? 1.0f : 0.0f;
        });
}

std::vector<uint8_t>
ViterbiDecoder::decode_soft(const std::vector<float>& llrs) const {
    return decode_core(llrs,
        [](uint8_t expected, float llr) -> float {
            float sign = (expected == 0) ? 1.0f : -1.0f;
            return -sign * llr;
        });
}