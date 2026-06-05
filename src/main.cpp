#include "modulation/bpsk_mod.hpp"
#include "modulation/bpsk_demod.hpp"
#include "channel/awgn_channel.hpp"
#include "measurement/ber_analyzer.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <random>

// generate random bit stream of length n
std::vector<uint8_t> random_bits(size_t n) {
    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 1);
    std::vector<uint8_t> bits(n);
    for (auto& b : bits)
        b = static_cast<uint8_t>(dist(rng));
    return bits;
}

// count bit errors between tx and rx bit vectors
size_t count_errors(const std::vector<uint8_t>& tx,
                    const std::vector<uint8_t>& rx) {
    size_t errors = 0;
    size_t n = std::min(tx.size(), rx.size());
    for (size_t i = 0; i < n; ++i)
        if (tx[i] != rx[i]) ++errors;
    return errors;
}

int main() {
    constexpr int    SPS        = 4;
    constexpr float  ROLLOFF    = 0.35f;
    constexpr int    SPAN       = 8;
    constexpr size_t NUM_BITS   = 10000;  // bits per SNR point

    // SNR sweep: -2 dB to 10 dB in 0.5 dB steps
    std::vector<float> snr_points;
    for (float snr = -2.0f; snr <= 10.0f; snr += 0.5f)
        snr_points.push_back(snr);

    BPSKModulator   mod  (SPS, ROLLOFF, SPAN);
    BPSKDemodulator demod(SPS, ROLLOFF, SPAN);

    std::ofstream csv("../results/bpsk_ber.csv");
    csv << "snr_db,ber\n";

    std::cout << "[loopback] BPSK BER sweep\n";
    std::cout << "SNR (dB) | BER\n";
    std::cout << "---------|----------\n";

    for (float snr_db : snr_points) {
        AWGNChannel channel(snr_db);

        auto tx_bits    = random_bits(NUM_BITS);
        auto tx_samples = mod.modulate(tx_bits);
        auto rx_samples = channel.apply(tx_samples);
        auto rx_bits    = demod.demodulate(rx_samples);

        size_t errors = count_errors(tx_bits, rx_bits);
        float  ber    = static_cast<float>(errors) /
                        static_cast<float>(tx_bits.size());

        csv << snr_db << "," << ber << "\n";

        std::printf("  %6.1f   | %.6f\n", snr_db, ber);
    }

    csv.close();
    std::cout << "[loopback] results written to results/bpsk_ber.csv\n";
    return 0;
}