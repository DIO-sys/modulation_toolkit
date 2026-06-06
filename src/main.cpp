#include "modulation/bpsk_mod.hpp"
#include "modulation/bpsk_demod.hpp"
#include "modulation/qpsk_mod.hpp"
#include "modulation/qpsk_demod.hpp"
#include "modulation/qam_mod.hpp"
#include "modulation/qam_demod.hpp"
#include "channel/awgn_channel.hpp"
#include "channel/multipath_channel.hpp"
#include "fec/conv_encoder.hpp"
#include "fec/viterbi_decoder.hpp"
#include "fec/interleaver.hpp"
#include "ofdm/resource_grid.hpp"
#include "ofdm/ofdm_modulator.hpp"
#include "ofdm/ofdm_demodulator.hpp"
#include "cv2x/cv2x_sidelink.hpp"
#include "measurement/ber_analyzer.hpp"

#include <iostream>
#include <vector>
#include <cstdint>
#include <random>
#include <string>
#include <cmath>
#include <fstream>
#include <numeric>

// ─── helpers ─────────────────────────────────────────────────────────────

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
               const std::vector<std::pair<float,float>>& points)
{
    std::ofstream f(path);
    f << "snr_db,ber\n";
    for (auto& [snr, ber] : points)
        f << snr << "," << ber << "\n";
}

// ─── baseband BER sweep ───────────────────────────────────────────────────

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
    result.name            = name;
    result.bits_per_symbol = mod.bits_per_symbol();

    for (float snr_db : snr_points) {
        size_t bps    = mod.bits_per_symbol();
        size_t padded = ((num_bits + bps - 1) / bps) * bps;

        auto tx_bits    = random_bits(padded);
        auto tx_samples = mod.modulate(tx_bits);

        AWGNChannel channel(snr_db);
        channel.set_signal_power(measure_power(tx_samples));

        auto rx_samples = channel.apply(tx_samples);
        auto rx_bits    = demod.demodulate(rx_samples);

        float ber = BERAnalyzer::compute_ber(tx_bits, rx_bits);
        result.points.push_back({snr_db, ber});

        std::printf("  [%-8s] SNR %5.1f dB | BER %.6f\n",
                    name.c_str(), snr_db, ber);
    }
    return result;
}

// ─── OFDM coded BER sweep ─────────────────────────────────────────────────

std::vector<std::pair<float,float>>
run_ofdm_ber_sweep(const std::string& label,
                   const std::vector<float>& snr_points,
                   size_t num_bits,
                   bool use_multipath)
{
    constexpr int   NUM_SC       = 64;
    constexpr int   CP_LEN       = 16;
    constexpr int   NUM_SYM      = 14;
    constexpr int   PILOT_SP     = 4;
    constexpr int   PILOT_PERIOD = 2;
    constexpr int   IL_ROWS      = 20;
    constexpr int   IL_COLS      = 10;
    constexpr int   BITS_PER_SYM = 2;
    constexpr float INV_SQRT2    = 0.70710678118f;

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

    std::vector<std::pair<float,float>> results;

    for (float snr_db : snr_points) {
        auto tx_bits     = random_bits(num_bits);
        auto coded       = enc.encode(tx_bits);
        auto interleaved = il.interleave(coded);

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

        auto tx_samples = ofdm_mod.modulate(qam_syms);

        std::vector<std::complex<float>> rx_samples;
        if (use_multipath) {
            MultipathChannel ch(
                {0, 4},
                {{1.0f, 0.0f}, {0.3f, 0.1f}},
                snr_db);
            ch.set_signal_power(measure_power(tx_samples));
            rx_samples = ch.apply(tx_samples);
        } else {
            AWGNChannel ch(snr_db);
            ch.set_signal_power(measure_power(tx_samples));
            rx_samples = ch.apply(tx_samples);
        }

        auto rx_syms = ofdm_demod.demodulate(rx_samples);

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
        std::vector<uint8_t> tx_trim(tx_bits.begin(),
                                      tx_bits.begin() + cmp);
        std::vector<uint8_t> rx_trim(decoded.begin(),
                                      decoded.begin() + cmp);

        float ber = BERAnalyzer::compute_ber(tx_trim, rx_trim);
        results.push_back({snr_db, ber});
        std::printf("  [%-14s] SNR %5.1f dB | BER %.6f\n",
                    label.c_str(), snr_db, ber);
    }
    return results;
}

// ─── C-V2X PC5 sidelink demo ─────────────────────────────────────────────

void run_cv2x_demo() {
    constexpr int   NUM_SC       = 64;
    constexpr int   CP_LEN       = 16;
    constexpr int   NUM_SYM      = 14;
    constexpr int   PILOT_SP     = 4;
    constexpr int   PILOT_PERIOD = 2;
    constexpr float INV_SQRT2    = 0.70710678118f;
    constexpr float SNR_DB       = 6.0f;

    std::cout << "\n[C-V2X] PC5 sidelink demo\n";
    std::cout << std::string(52, '-') << "\n\n";

    // ── construct BSM — vehicle on Pennsylvania Ave NW
    BSM tx_bsm;
    tx_bsm.vehicle_id   = 0xDEADBEEF;
    tx_bsm.latitude     = static_cast<int32_t>( 38.8977 * 1e7);
    tx_bsm.longitude    = static_cast<int32_t>(-77.0365 * 1e7);
    tx_bsm.speed_cms    = static_cast<uint16_t>(15.0f * 44.704f);
    tx_bsm.heading      = static_cast<uint16_t>(90.0f / 0.0125f);
    tx_bsm.brake_status = 0x00;

    // ── SCI format 1 — priority 3, MCS 5, broadcast
    SCI sci;
    sci.priority          = 3;
    sci.resource_interval = 2;
    sci.mcs               = 5;
    sci.retx_index        = 0;
    sci.resource_block    = 0;
    sci.group_dst_id      = 0xFF;

    std::cout << "[C-V2X] TX BSM: " << tx_bsm.to_string() << "\n";
    std::cout << "[C-V2X] TX SCI: " << sci.to_string()    << "\n\n";

    // ── encode BSM into PC5 frame (serialize + FEC)
    auto frame = PC5Frame::encode(tx_bsm, sci);
    std::printf("[C-V2X] encoded %zu payload bits (rate 1/2 FEC)\n\n",
                frame.payload_bits.size());

    // ── map payload bits through OFDM chain
    ResourceGrid    grid(NUM_SC, NUM_SYM, PILOT_SP, PILOT_PERIOD);
    OFDMModulator   ofdm_mod(NUM_SC, CP_LEN, grid);
    OFDMDemodulator ofdm_demod(NUM_SC, CP_LEN, grid);

    int data_per_frame = grid.total_data_symbols() * 2;

    // pad to frame boundary
    auto bits = frame.payload_bits;
    while (static_cast<int>(bits.size()) % data_per_frame != 0)
        bits.push_back(0);

    // QPSK map
    std::vector<std::complex<float>> syms;
    syms.reserve(bits.size() / 2);
    for (size_t i = 0; i + 1 < bits.size(); i += 2) {
        float re = (bits[i]   == 0) ?  INV_SQRT2 : -INV_SQRT2;
        float im = (bits[i+1] == 0) ?  INV_SQRT2 : -INV_SQRT2;
        syms.push_back({re, im});
    }

    // OFDM modulate
    auto tx_samples = ofdm_mod.modulate(syms);

    // AWGN channel at SNR_DB
    AWGNChannel ch(SNR_DB);
    ch.set_signal_power(measure_power(tx_samples));
    auto rx_samples = ch.apply(tx_samples);

    std::printf("[C-V2X] channel: AWGN %.1f dB SNR\n\n", SNR_DB);

    // OFDM demodulate
    auto rx_syms = ofdm_demod.demodulate(rx_samples);

    // QPSK hard demap
    std::vector<uint8_t> rx_bits;
    rx_bits.reserve(rx_syms.size() * 2);
    for (const auto& s : rx_syms) {
        rx_bits.push_back(s.real() >= 0.0f ? 0 : 1);
        rx_bits.push_back(s.imag() >= 0.0f ? 0 : 1);
    }

    // trim to original payload size
    rx_bits.resize(frame.payload_bits.size());
    frame.payload_bits = rx_bits;

    // ── decode PC5 frame back to BSM
    auto rx_bsm = PC5Frame::decode(frame);

    std::cout << "[C-V2X] RX BSM: " << rx_bsm.to_string() << "\n\n";

    // ── verify
    bool id_ok  = (tx_bsm.vehicle_id == rx_bsm.vehicle_id);
    bool lat_ok = (tx_bsm.latitude   == rx_bsm.latitude);
    bool lon_ok = (tx_bsm.longitude  == rx_bsm.longitude);
    bool spd_ok = (tx_bsm.speed_cms  == rx_bsm.speed_cms);
    bool hdg_ok = (tx_bsm.heading    == rx_bsm.heading);

    std::printf("  vehicle_id : %s\n", id_ok  ? "PASS" : "FAIL");
    std::printf("  latitude   : %s\n", lat_ok ? "PASS" : "FAIL");
    std::printf("  longitude  : %s\n", lon_ok ? "PASS" : "FAIL");
    std::printf("  speed      : %s\n", spd_ok ? "PASS" : "FAIL");
    std::printf("  heading    : %s\n", hdg_ok ? "PASS" : "FAIL");

    bool all_ok = id_ok && lat_ok && lon_ok && spd_ok && hdg_ok;
    std::cout << "\n[C-V2X] "
              << (all_ok ? "ALL FIELDS DECODED CORRECTLY"
                         : "DECODE FAILURES DETECTED")
              << "\n";
}

// ─── main ─────────────────────────────────────────────────────────────────

int main() {
    constexpr int    SPS      = 4;
    constexpr float  ROLLOFF  = 0.35f;
    constexpr int    SPAN     = 8;
    constexpr size_t NUM_BITS = 10000;

    // ── baseband uncoded BER sweep
    std::vector<float> snr_bb;
    for (float snr = -2.0f; snr <= 14.0f; snr += 0.5f)
        snr_bb.push_back(snr);

    BPSKModulator      bpsk_mod    (SPS, ROLLOFF, SPAN);
    BPSKDemodulator    bpsk_demod  (SPS, ROLLOFF, SPAN);
    QPSKModulator      qpsk_mod    (SPS, ROLLOFF, SPAN);
    QPSKDemodulator    qpsk_demod  (SPS, ROLLOFF, SPAN);
    QAMModulator<16>   qam16_mod   (SPS, ROLLOFF, SPAN);
    QAMDemodulator<16> qam16_demod (SPS, ROLLOFF, SPAN);
    QAMModulator<64>   qam64_mod   (SPS, ROLLOFF, SPAN);
    QAMDemodulator<64> qam64_demod (SPS, ROLLOFF, SPAN);

    std::cout << "[baseband] BER sweep — all schemes\n\n";

    auto bpsk_r  = run_ber_sweep("BPSK",   bpsk_mod,   bpsk_demod,   snr_bb, NUM_BITS);
    auto qpsk_r  = run_ber_sweep("QPSK",   qpsk_mod,   qpsk_demod,   snr_bb, NUM_BITS);
    auto qam16_r = run_ber_sweep("16-QAM", qam16_mod,  qam16_demod,  snr_bb, NUM_BITS);
    auto qam64_r = run_ber_sweep("64-QAM", qam64_mod,  qam64_demod,  snr_bb, NUM_BITS);

    write_csv("../results/bpsk_ber.csv",  bpsk_r.points);
    write_csv("../results/qpsk_ber.csv",  qpsk_r.points);
    write_csv("../results/qam16_ber.csv", qam16_r.points);
    write_csv("../results/qam64_ber.csv", qam64_r.points);

    // ── coded OFDM BER sweep
    std::vector<float> snr_ofdm;
    for (float snr = -4.0f; snr <= 14.0f; snr += 0.5f)
        snr_ofdm.push_back(snr);

    std::cout << "\n[OFDM] coded BER sweep — AWGN\n\n";
    auto ofdm_awgn = run_ofdm_ber_sweep("OFDM AWGN",
                                         snr_ofdm, NUM_BITS, false);

    std::cout << "\n[OFDM] coded BER sweep — multipath\n\n";
    auto ofdm_mp   = run_ofdm_ber_sweep("OFDM multipath",
                                         snr_ofdm, NUM_BITS, true);

    write_csv("../results/ofdm_awgn_ber.csv",      ofdm_awgn);
    write_csv("../results/ofdm_multipath_ber.csv", ofdm_mp);

    // ── C-V2X PC5 sidelink demo
    run_cv2x_demo();

    std::cout << "\n[main] all results written to results/\n";
    return 0;
}