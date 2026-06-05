#include "modulation/bpsk_mod.hpp"
#include "modulation/bpsk_demod.hpp"
#include "modulation/qpsk_mod.hpp"
#include "modulation/qpsk_demod.hpp"
#include "modulation/qam_mod.hpp"
#include "modulation/qam_demod.hpp"
#include "channel/awgn_channel.hpp"
#include "measurement/ber_analyzer.hpp"

#include <iostream>
#include <vector>
#include <cstdint>
#include <random>
#include <string>
#include <cmath>

std::vector<uint8_t> random_bits(size_t n, uint32_t seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 1);
    std::vector<uint8_t> bits(n);
    for (auto& b : bits)
        b = static_cast<uint8_t>(dist(rng));
    return bits;
}

float measure_power(const std::vector<std::complex<float>>& samples) {
    float power = 0.0f;
    for (const auto& s : samples)
        power += s.real() * s.real() + s.imag() * s.imag();
    return power / static_cast<float>(samples.size());
}

struct SchemeResult {
    std::string name;
    int         bits_per_symbol;
    std::vector<std::pair<float, float>> points;
};

template <typename Mod, typename Demod>
SchemeResult run_ber_sweep(const std::string& name,
                           Mod& mod, Demod& demod,
                           const std::vector<float>& snr_points,
                           size_t num_bits)
{
    SchemeResult result;
    result.name           = name;
    result.bits_per_symbol = mod.bits_per_symbol();

    for (float snr_db : snr_points) {
        size_t bps    = mod.bits_per_symbol();
        size_t padded = ((num_bits + bps - 1) / bps) * bps;

        auto tx_bits    = random_bits(padded);
        auto tx_samples = mod.modulate(tx_bits);

        float sig_power = measure_power(tx_samples);
        AWGNChannel channel(snr_db);
        channel.set_signal_power(sig_power);

        auto rx_samples = channel.apply(tx_samples);
        auto rx_bits    = demod.demodulate(rx_samples);

        float ber = BERAnalyzer::compute_ber(tx_bits, rx_bits);
        result.points.push_back({snr_db, ber});

        std::printf("  [%-8s] SNR %5.1f dB | BER %.6f\n",
                    name.c_str(), snr_db, ber);
    }
    return result;
}

int main() {
    constexpr int    SPS      = 4;
    constexpr float  ROLLOFF  = 0.35f;
    constexpr int    SPAN     = 8;
    constexpr size_t NUM_BITS = 10000;

    std::vector<float> snr_points;
    for (float snr = -2.0f; snr <= 14.0f; snr += 0.5f)
        snr_points.push_back(snr);

    BPSKModulator      bpsk_mod    (SPS, ROLLOFF, SPAN);
    BPSKDemodulator    bpsk_demod  (SPS, ROLLOFF, SPAN);
    QPSKModulator      qpsk_mod    (SPS, ROLLOFF, SPAN);
    QPSKDemodulator    qpsk_demod  (SPS, ROLLOFF, SPAN);
    QAMModulator<16>   qam16_mod   (SPS, ROLLOFF, SPAN);
    QAMDemodulator<16> qam16_demod (SPS, ROLLOFF, SPAN);
    QAMModulator<64>   qam64_mod   (SPS, ROLLOFF, SPAN);
    QAMDemodulator<64> qam64_demod (SPS, ROLLOFF, SPAN);

    std::cout << "[loopback] BER sweep — all schemes\n\n";

    auto bpsk_result  = run_ber_sweep("BPSK",   bpsk_mod,   bpsk_demod,   snr_points, NUM_BITS);
    auto qpsk_result  = run_ber_sweep("QPSK",   qpsk_mod,   qpsk_demod,   snr_points, NUM_BITS);
    auto qam16_result = run_ber_sweep("16-QAM", qam16_mod,  qam16_demod,  snr_points, NUM_BITS);
    auto qam64_result = run_ber_sweep("64-QAM", qam64_mod,  qam64_demod,  snr_points, NUM_BITS);

    BERAnalyzer analyzer;
    auto write = [&](const SchemeResult& r, const std::string& path) {
        for (auto& [snr, ber] : r.points)
            analyzer.record(snr, ber);
        analyzer.write_csv(path);
        analyzer = BERAnalyzer();
    };

    write(bpsk_result,  "../results/bpsk_ber.csv");
    write(qpsk_result,  "../results/qpsk_ber.csv");
    write(qam16_result, "../results/qam16_ber.csv");
    write(qam64_result, "../results/qam64_ber.csv");

    std::cout << "\n[loopback] results written to results/\n";
    return 0;
}