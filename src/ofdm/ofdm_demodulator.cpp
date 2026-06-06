#include "ofdm/ofdm_demodulator.hpp"
#include "ofdm/channel_estimator.hpp"
#include <fftw3.h>
#include <stdexcept>

OFDMDemodulator::OFDMDemodulator(int num_subcarriers,
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
OFDMDemodulator::demodulate(const std::vector<std::complex<float>>& samples) const
{
    int num_sym    = grid_.num_symbols();
    int sym_len    = symbol_len();


    // output grid — frequency domain symbols after FFT
    std::vector<std::complex<float>> rx_grid(num_sc_ * num_sym);

    for (int s = 0; s < num_sym; ++s) {
        // extract one symbol from sample stream
        std::vector<std::complex<float>> rx_sym(
            samples.begin() + s * sym_len,
            samples.begin() + (s + 1) * sym_len);

        // strip cyclic prefix
        auto no_cp = remove_cyclic_prefix(rx_sym);

        // FFT — time to frequency domain
        auto freq = fft(no_cp);

        // store in grid
        for (int k = 0; k < num_sc_; ++k)
            rx_grid[s * num_sc_ + k] = freq[k];
    }

    // channel estimation + equalization
    ChannelEstimator estimator(grid_, {1.0f, 0.0f});
    auto H = estimator.estimate(rx_grid);
    auto equalized = estimator.equalize(rx_grid, H);

    // extract data symbols — strip pilots
    return grid_.unmap(equalized);
}

std::vector<std::complex<float>>
OFDMDemodulator::fft(const std::vector<std::complex<float>>& time) const
{
    int N = static_cast<int>(time.size());
    std::vector<std::complex<float>> out(N);

    fftwf_complex* in_buf  = fftwf_alloc_complex(N);
    fftwf_complex* out_buf = fftwf_alloc_complex(N);

    for (int i = 0; i < N; ++i) {
        in_buf[i][0] = time[i].real();
        in_buf[i][1] = time[i].imag();
    }

    fftwf_plan plan = fftwf_plan_dft_1d(N, in_buf, out_buf,
                                         FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);

    for (int i = 0; i < N; ++i)
        out[i] = std::complex<float>(out_buf[i][0], out_buf[i][1]);

    fftwf_free(in_buf);
    fftwf_free(out_buf);
    return out;
}

std::vector<std::complex<float>>
OFDMDemodulator::remove_cyclic_prefix(
    const std::vector<std::complex<float>>& symbol) const
{
    // drop first cp_len samples
    return std::vector<std::complex<float>>(
        symbol.begin() + cp_len_,
        symbol.end());
}