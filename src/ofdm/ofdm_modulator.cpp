#include "ofdm/ofdm_modulator.hpp"
#include <fftw3.h>
#include <stdexcept>
#include <cstring>

OFDMModulator::OFDMModulator(int num_subcarriers,
                              int cp_len,
                              ResourceGrid grid)
    : num_sc_(num_subcarriers)
    , cp_len_(cp_len)
    , grid_(grid)
{
    if (num_subcarriers < 1)
        throw std::invalid_argument("num_subcarriers must be >= 1");
    if (cp_len < 0)
        throw std::invalid_argument("cp_len must be >= 0");
}

std::vector<std::complex<float>>
OFDMModulator::modulate(const std::vector<std::complex<float>>& qam_symbols) const
{
    // map QAM symbols onto resource grid
    auto grid = grid_.map(qam_symbols);

    int num_sym   = grid_.num_symbols();
    int total_len = num_sym * symbol_len();

    std::vector<std::complex<float>> output;
    output.reserve(total_len);

    for (int s = 0; s < num_sym; ++s) {
        // extract one OFDM symbol from grid
        std::vector<std::complex<float>> freq_sym(
            grid.begin() + s * num_sc_,
            grid.begin() + (s + 1) * num_sc_);

        // IFFT — frequency to time domain
        auto time_sym = ifft(freq_sym);

        // prepend cyclic prefix
        auto with_cp = add_cyclic_prefix(time_sym);

        output.insert(output.end(), with_cp.begin(), with_cp.end());
    }
    return output;
}

std::vector<std::complex<float>>
OFDMModulator::ifft(const std::vector<std::complex<float>>& freq) const
{
    int N = static_cast<int>(freq.size());
    std::vector<std::complex<float>> out(N);

    fftwf_complex* in_buf  = fftwf_alloc_complex(N);
    fftwf_complex* out_buf = fftwf_alloc_complex(N);

    for (int i = 0; i < N; ++i) {
        in_buf[i][0] = freq[i].real();
        in_buf[i][1] = freq[i].imag();
    }

    fftwf_plan plan = fftwf_plan_dft_1d(N, in_buf, out_buf,
                                         FFTW_BACKWARD, FFTW_ESTIMATE);
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);

    // normalize by 1/N
    float norm = 1.0f / static_cast<float>(N);
    for (int i = 0; i < N; ++i)
        out[i] = std::complex<float>(out_buf[i][0] * norm,
                                      out_buf[i][1] * norm);

    fftwf_free(in_buf);
    fftwf_free(out_buf);
    return out;
}

std::vector<std::complex<float>>
OFDMModulator::add_cyclic_prefix(
    const std::vector<std::complex<float>>& symbol) const
{
    int N = static_cast<int>(symbol.size());
    std::vector<std::complex<float>> out(N + cp_len_);

    // copy last cp_len samples to front
    for (int i = 0; i < cp_len_; ++i)
        out[i] = symbol[N - cp_len_ + i];

    // copy full symbol after CP
    for (int i = 0; i < N; ++i)
        out[cp_len_ + i] = symbol[i];

    return out;
}