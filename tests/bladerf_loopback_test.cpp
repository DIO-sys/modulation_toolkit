#include "hw/bladerf_device.hpp"
#include "hw/circular_buffer.hpp"
#include "hw/app_state.hpp"
#include "modulation/bpsk_mod.hpp"
#include "modulation/bpsk_demod.hpp"
#include "modulation/qpsk_mod.hpp"
#include "modulation/qpsk_demod.hpp"
#include "modulation/qam_mod.hpp"
#include "modulation/qam_demod.hpp"
#include "fec/conv_encoder.hpp"
#include "fec/viterbi_decoder.hpp"
#include "fec/interleaver.hpp"
#include "ofdm/resource_grid.hpp"
#include "ofdm/ofdm_modulator.hpp"
#include "ofdm/ofdm_demodulator.hpp"
#include "measurement/ber_analyzer.hpp"
#include "measurement/evm_meter.hpp"

#include <iostream>
#include <vector>
#include <complex>
#include <cstdint>
#include <random>
#include <cmath>
#include <fstream>
#include <string>
#include <numeric>
#include <thread>
#include <chrono>

// ─── helpers ─────────────────────────────────────────────────────────────

std::vector<uint8_t> random_bits(size_t n, uint32_t seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 1);
    std::vector<uint8_t> bits(n);
    for (auto& b : bits)
        b = static_cast<uint8_t>(dist(rng));
    return bits;
}

float compute_evm(const std::vector<std::complex<float>>& tx,
                  const std::vector<std::complex<float>>& rx)
{
    float err_power = 0.0f;
    float sig_power = 0.0f;
    size_t n = std::min(tx.size(), rx.size());
    for (size_t i = 0; i < n; ++i) {
        auto err = rx[i] - tx[i];
        err_power += err.real() * err.real() + err.imag() * err.imag();
        sig_power += tx[i].real() * tx[i].real() + tx[i].imag() * tx[i].imag();
    }
    return 100.0f * std::sqrt(err_power / sig_power);
}

std::vector<float> deinterleave_llrs(const std::vector<float>& llrs,
                                      int rows, int cols)
{
    int block = rows * cols;
    std::vector<float> out(llrs.size());
    for (int i = 0; i < static_cast<int>(llrs.size()); i += block)
        for (int row = 0; row < rows; ++row)
            for (int col = 0; col < cols; ++col)
                out[i + row * cols + col] = llrs[i + col * rows + row];
    return out;
}

void write_csv(const std::string& path,
               const std::vector<std::tuple<int,float,float>>& points)
{
    std::ofstream f(path);
    f << "tx_gain_db,ber,evm_pct\n";
    for (auto& [gain, ber, evm] : points)
        f << gain << "," << ber << "," << evm << "\n";
}

// ─── TX loop buffer ───────────────────────────────────────────────────────

std::vector<std::complex<float>>
make_tx_loop(const std::vector<std::complex<float>>& buf)
{
    constexpr size_t CHUNK = 4096;
    auto loop = buf;
    while (loop.size() % CHUNK != 0)
        loop.insert(loop.end(), buf.begin(), buf.end());
    return loop;
}

// ─── frame sync ───────────────────────────────────────────────────────────

constexpr int SYNC_LEN = 631;  // ZC length — prime number required

std::vector<std::complex<float>> make_sync_word() {
    // Zadoff-Chu sequence — root u=25, length 631 (prime)
    // ideal periodic autocorrelation — used in LTE PSS/SSS
    constexpr int u = 25;
    constexpr int N = SYNC_LEN;
    std::vector<std::complex<float>> zc(N);
    for (int n = 0; n < N; ++n) {
        float phase = -M_PI * u * n * (n + 1) / static_cast<float>(N);
        zc[n] = std::complex<float>(std::cos(phase), std::sin(phase));
    }
    return zc;
}
int find_frame_start(const std::vector<std::complex<float>>& rx,
                     const std::vector<std::complex<float>>& sync_word,
                     int search_len = 16384)
{
    int   best_idx = 0;
    float best_val = 0.0f;
    int   n = std::min(search_len,
                       static_cast<int>(rx.size()) - SYNC_LEN);

    for (int i = 0; i < n; ++i) {
        float corr = 0.0f;
        for (int j = 0; j < SYNC_LEN; ++j)
            corr += rx[i + j].real() * sync_word[j].real()
                  + rx[i + j].imag() * sync_word[j].imag();
        corr = std::abs(corr);
        if (corr > best_val) {
            best_val = corr;
            best_idx = i;
        }
    }
    return best_idx;
}

// ─── collect RX samples ───────────────────────────────────────────────────

std::vector<std::complex<float>>
collect_rx(CircularBuffer<std::complex<float>>& rx_buf, size_t num_samples)
{
    constexpr size_t CHUNK = 4096;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<std::complex<float>> tmp(CHUNK);
    while (rx_buf.available() >= CHUNK)
        rx_buf.pop_batch(tmp.data(), CHUNK);

    while (rx_buf.available() < num_samples)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::vector<std::complex<float>> rx(num_samples);
    size_t got = 0;
    while (got < num_samples) {
        size_t batch = std::min(CHUNK, num_samples - got);
        if (rx_buf.available() >= batch) {
            rx_buf.pop_batch(rx.data() + got, batch);
            got += batch;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return rx;
}

// ─── BPSK hardware BER sweep ──────────────────────────────────────────────

void run_bpsk_sweep(BladeRFDevice& dev,
                    CircularBuffer<std::complex<float>>& rx_buf,
                    const std::vector<int>& tx_gains)
{
    constexpr int    SPS      = 4;
    constexpr float  ROLLOFF  = 0.35f;
    constexpr int    SPAN     = 8;
    constexpr size_t NUM_BITS = 5000;

    BPSKModulator   mod(SPS, ROLLOFF, SPAN);
    BPSKDemodulator demod(SPS, ROLLOFF, SPAN);

    auto sync_word  = make_sync_word();
    auto tx_bits    = random_bits(NUM_BITS);
    auto tx_payload = mod.modulate(tx_bits);

    // TX frame = sync_word + payload
    std::vector<std::complex<float>> tx_frame;
    tx_frame.insert(tx_frame.end(), sync_word.begin(),   sync_word.end());
    tx_frame.insert(tx_frame.end(), tx_payload.begin(),  tx_payload.end());

    auto tx_loop = make_tx_loop(tx_frame);
    std::printf("[bpsk] frame=%zu  loop=%zu  remainder=%zu\n",
                tx_frame.size(), tx_loop.size(), tx_loop.size() % 4096);

    std::vector<std::tuple<int,float,float>> results;

    std::cout << "\n[BPSK HW] TX gain sweep\n";
    std::printf("  %-12s | %-8s | %-10s | %-10s\n",
                "TX gain (dB)", "sync@", "BER", "EVM (%)");
    std::printf("  %s\n", std::string(48, '-').c_str());

    dev.start_tx_stream(tx_loop);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    dev.start_rx_stream();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for (int gain : tx_gains) {
        dev.set_tx_gain(gain);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // collect two full loops so sync search has enough range
        auto rx_raw = collect_rx(rx_buf, tx_loop.size() * 2);

        // find frame start via correlation
        int offset = find_frame_start(rx_raw, sync_word);
        int payload_start = offset + SYNC_LEN;

        // guard against running off the end
        if (payload_start + static_cast<int>(tx_payload.size())
                > static_cast<int>(rx_raw.size())) {
            std::printf("  %-12d | %-8d | [sync out of range]\n", gain, offset);
            results.push_back({gain, 0.5f, 0.0f});
            continue;
        }

        std::vector<std::complex<float>> rx_payload(
            rx_raw.begin() + payload_start,
            rx_raw.begin() + payload_start + tx_payload.size());

        auto rx_bits = demod.demodulate(rx_payload);
        size_t cmp   = std::min(tx_bits.size(), rx_bits.size());

        float ber = BERAnalyzer::compute_ber(
            std::vector<uint8_t>(tx_bits.begin(), tx_bits.begin() + cmp),
            std::vector<uint8_t>(rx_bits.begin(), rx_bits.begin() + cmp));

        // EVM at symbol sampling instants
        std::vector<std::complex<float>> tx_sym, rx_sym;
        int off = SPS * SPAN;
        size_t pay_len = std::min(tx_payload.size(), rx_payload.size());
        for (int i = off; i < static_cast<int>(pay_len); i += SPS) {
            tx_sym.push_back(tx_payload[i]);
            rx_sym.push_back(rx_payload[i]);
        }
        float evm = compute_evm(tx_sym, rx_sym);

        results.push_back({gain, ber, evm});
        std::printf("  %-12d | %-8d | %-10.6f | %-10.2f\n",
                    gain, offset, ber, evm);
    }

    dev.stop_rx_stream();
    dev.stop_tx_stream();

    write_csv("../results/hw_bpsk_ber.csv", results);
    std::cout << "[BPSK HW] written to results/hw_bpsk_ber.csv\n";
}

// ─── coded OFDM hardware BER sweep ───────────────────────────────────────

void run_ofdm_sweep(BladeRFDevice& dev,
                    CircularBuffer<std::complex<float>>& rx_buf,
                    const std::vector<int>& tx_gains)
{
    constexpr int    NUM_SC       = 64;
    constexpr int    CP_LEN       = 16;
    constexpr int    NUM_SYM      = 14;
    constexpr int    PILOT_SP     = 4;
    constexpr int    PILOT_PERIOD = 2;
    constexpr int    IL_ROWS      = 20;
    constexpr int    IL_COLS      = 10;
    constexpr int    BITS_PER_SYM = 2;
    constexpr float  INV_SQRT2    = 0.70710678118f;
    constexpr size_t NUM_BITS     = 5000;

    ResourceGrid    grid(NUM_SC, NUM_SYM, PILOT_SP, PILOT_PERIOD);
    OFDMModulator   ofdm_mod(NUM_SC, CP_LEN, grid);
    OFDMDemodulator ofdm_demod(NUM_SC, CP_LEN, grid);
    ConvolutionalEncoder enc(2, 7, {0133, 0171});
    ViterbiDecoder       dec(2, 7, {0133, 0171});
    Interleaver          il(IL_ROWS, IL_COLS);

    int data_per_frame = grid.total_data_symbols() * BITS_PER_SYM;
    int il_block       = IL_ROWS * IL_COLS;
    int lcm_size       = (data_per_frame * il_block) /
                          std::gcd(data_per_frame, il_block);

    auto tx_bits      = random_bits(NUM_BITS);
    auto coded        = enc.encode(tx_bits);
    auto interleaved  = il.interleave(coded);
    size_t padded_len = ((interleaved.size() + lcm_size - 1)
                         / lcm_size) * lcm_size;
    interleaved.resize(padded_len, 0);

    std::vector<std::complex<float>> qam_syms;
    qam_syms.reserve(interleaved.size() / 2);
    for (size_t i = 0; i + 1 < interleaved.size(); i += 2) {
        float re = (interleaved[i]   == 0) ?  INV_SQRT2 : -INV_SQRT2;
        float im = (interleaved[i+1] == 0) ?  INV_SQRT2 : -INV_SQRT2;
        qam_syms.push_back({re, im});
    }

    auto tx_payload = ofdm_mod.modulate(qam_syms);
    auto sync_word  = make_sync_word();

    // TX frame = sync_word + OFDM payload
    std::vector<std::complex<float>> tx_frame;
    tx_frame.insert(tx_frame.end(), sync_word.begin(),  sync_word.end());
    tx_frame.insert(tx_frame.end(), tx_payload.begin(), tx_payload.end());

    auto tx_loop = make_tx_loop(tx_frame);
    std::printf("[ofdm] frame=%zu  loop=%zu  remainder=%zu\n",
                tx_frame.size(), tx_loop.size(), tx_loop.size() % 4096);

    std::vector<std::tuple<int,float,float>> results;

    std::cout << "\n[OFDM HW] TX gain sweep\n";
    std::printf("  %-12s | %-8s | %-10s | %-10s\n",
                "TX gain (dB)", "sync@", "BER", "EVM (%)");
    std::printf("  %s\n", std::string(48, '-').c_str());

    dev.start_tx_stream(tx_loop);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    dev.start_rx_stream();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for (int gain : tx_gains) {
        dev.set_tx_gain(gain);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto rx_raw = collect_rx(rx_buf, tx_loop.size() * 2);

        int offset        = find_frame_start(rx_raw, sync_word);
        int payload_start = offset + SYNC_LEN;

        if (payload_start + static_cast<int>(tx_payload.size())
                > static_cast<int>(rx_raw.size())) {
            std::printf("  %-12d | %-8d | [sync out of range]\n", gain, offset);
            results.push_back({gain, 0.5f, 0.0f});
            continue;
        }

        std::vector<std::complex<float>> rx_payload(
            rx_raw.begin() + payload_start,
            rx_raw.begin() + payload_start + tx_payload.size());

        auto rx_syms = ofdm_demod.demodulate(rx_payload);

        std::vector<float> llrs;
        llrs.reserve(rx_syms.size() * 2);
        for (const auto& s : rx_syms) {
            llrs.push_back(s.real());
            llrs.push_back(s.imag());
        }

        if (llrs.size() > padded_len) llrs.resize(padded_len);
        size_t safe = (llrs.size() / il_block) * il_block;
        llrs.resize(safe);
        auto llrs_dint = deinterleave_llrs(llrs, IL_ROWS, IL_COLS);
        auto decoded   = dec.decode_soft(llrs_dint);

        size_t cmp = std::min(tx_bits.size(), decoded.size());
        float ber = BERAnalyzer::compute_ber(
            std::vector<uint8_t>(tx_bits.begin(), tx_bits.begin() + cmp),
            std::vector<uint8_t>(decoded.begin(), decoded.begin() + cmp));

        float evm = compute_evm(qam_syms, rx_syms);

        results.push_back({gain, ber, evm});
        std::printf("  %-12d | %-8d | %-10.6f | %-10.2f\n",
                    gain, offset, ber, evm);
    }

    dev.stop_rx_stream();
    dev.stop_tx_stream();

    write_csv("../results/hw_ofdm_ber.csv", results);
    std::cout << "[OFDM HW] written to results/hw_ofdm_ber.csv\n";
}

// ─── main ────────────────────────────────────────────────────────────────

int main() {
    constexpr uint64_t CENTER_FREQ_HZ = 915'000'000ULL;
    constexpr int      RX_GAIN        = 30;

    std::vector<int> tx_gains = {
        0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60
    };

    AppState state;
    state.center_freq_hz.store(CENTER_FREQ_HZ);
    state.rx_gain_db.store(RX_GAIN);
    state.tx_gain_db.store(0);

    CircularBuffer<std::complex<float>> rx_buf(1 << 20);

    BladeRFDevice dev(rx_buf, state);

    std::cout << "[hw loopback] opening BladeRF\n";
    if (!dev.open()) {
        std::cerr << "[hw loopback] failed to open device\n";
        return 1;
    }

    if (!dev.configure()) {
        std::cerr << "[hw loopback] failed to configure device\n";
        return 1;
    }

    std::cout << "[hw loopback] RF cable loopback — TX1 → RX1\n";
    std::cout << "[hw loopback] press enter to begin sweep\n";
    std::cin.get();

    run_bpsk_sweep(dev, rx_buf, tx_gains);
    run_ofdm_sweep(dev, rx_buf, tx_gains);

    std::cout << "\n[hw loopback] all results written to results/\n";
    return 0;
}