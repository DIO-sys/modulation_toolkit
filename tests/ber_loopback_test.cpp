#include "fec/conv_encoder.hpp"
#include "fec/viterbi_decoder.hpp"
#include "fec/interleaver.hpp"
#include "channel/awgn_channel.hpp"
#include "modulation/bpsk_mod.hpp"
#include "modulation/bpsk_demod.hpp"
#include "modulation/qam_mod.hpp"
#include "modulation/qam_demod.hpp"
#include "ofdm/resource_grid.hpp"
#include "ofdm/ofdm_modulator.hpp"
#include "ofdm/ofdm_demodulator.hpp"
#include "measurement/ber_analyzer.hpp"
#include <liquid/liquid.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <random>
#include <cmath>
#include <fstream>
#include <string>

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

// ─── Encoder validation ───────────────────────────────────────────────────

bool validate_encoder(int rate_denom, const std::vector<uint8_t>& polys,
                      const std::vector<uint8_t>& bits)
{
    ConvolutionalEncoder our_enc(rate_denom, 7, polys);
    auto our_coded = our_enc.encode(bits);

    fec_scheme scheme = (rate_denom == 2) ? LIQUID_FEC_CONV_V27
                                          : LIQUID_FEC_CONV_V39;

    unsigned int msg_len = static_cast<unsigned int>(bits.size());
    unsigned int enc_len = fec_get_enc_msg_length(scheme, msg_len);

    std::vector<uint8_t> msg_bytes((msg_len + 7) / 8, 0);
    for (size_t i = 0; i < bits.size(); ++i)
        if (bits[i]) msg_bytes[i / 8] |= (1 << (7 - (i % 8)));

    std::vector<uint8_t> enc_bytes(enc_len, 0);
    fec q = fec_create(scheme, nullptr);
    fec_encode(q, msg_len, msg_bytes.data(), enc_bytes.data());
    fec_destroy(q);

    std::vector<uint8_t> liquid_coded;
    liquid_coded.reserve(enc_len * 8);
    for (uint8_t byte : enc_bytes)
        for (int b = 7; b >= 0; --b)
            liquid_coded.push_back((byte >> b) & 1);

    size_t compare_len = std::min(our_coded.size(), liquid_coded.size());
    int mismatches = 0;
    for (size_t i = 0; i < compare_len; ++i) {
        if (our_coded[i] != liquid_coded[i]) {
            std::printf("  mismatch at bit %zu: ours=%d liquid=%d\n",
                        i, our_coded[i], liquid_coded[i]);
            if (++mismatches > 10) {
                std::cout << "  ... too many mismatches, stopping\n";
                break;
            }
        }
    }

    if (mismatches == 0) {
        std::printf("  [PASS] %zu bits match liquid-dsp exactly\n", compare_len);
        return true;
    }
    std::printf("  [FAIL] %d mismatches in %zu bits\n", mismatches, compare_len);
    return false;
}

// ─── Viterbi validation ───────────────────────────────────────────────────

bool validate_viterbi_hard(const std::vector<uint8_t>& bits) {
    ConvolutionalEncoder enc(2, 7, {0133, 0171});
    ViterbiDecoder       dec(2, 7, {0133, 0171});

    auto coded   = enc.encode(bits);
    auto decoded = dec.decode_hard(coded);

    int mismatches = 0;
    size_t n = std::min(bits.size(), decoded.size());
    for (size_t i = 0; i < n; ++i)
        if (bits[i] != decoded[i]) ++mismatches;

    if (mismatches == 0) {
        std::printf("  [PASS] hard decision: %zu bits recovered exactly\n", n);
        return true;
    }
    std::printf("  [FAIL] hard decision: %d mismatches in %zu bits\n",
                mismatches, n);
    return false;
}

bool validate_viterbi_soft(const std::vector<uint8_t>& bits) {
    constexpr int   SPS     = 4;
    constexpr float ROLLOFF = 0.35f;
    constexpr int   SPAN    = 8;

    ConvolutionalEncoder enc(2, 7, {0133, 0171});
    ViterbiDecoder       dec(2, 7, {0133, 0171});
    BPSKModulator        mod(SPS, ROLLOFF, SPAN);
    BPSKDemodulator      demod(SPS, ROLLOFF, SPAN);

    auto coded      = enc.encode(bits);
    auto tx_samples = mod.modulate(coded);
    auto llrs       = demod.demodulate_soft(tx_samples);
    auto decoded    = dec.decode_soft(llrs);

    int mismatches = 0;
    size_t n = std::min(bits.size(), decoded.size());
    for (size_t i = 0; i < n; ++i)
        if (bits[i] != decoded[i]) ++mismatches;

    if (mismatches == 0) {
        std::printf("  [PASS] soft decision: %zu bits recovered exactly\n", n);
        return true;
    }
    std::printf("  [FAIL] soft decision: %d mismatches in %zu bits\n",
                mismatches, n);
    return false;
}

// ─── OFDM noiseless loopback validation ──────────────────────────────────

bool validate_ofdm_loopback() {
    constexpr int NUM_SC       = 64;
    constexpr int CP_LEN       = 16;
    constexpr int NUM_SYM      = 14;
    constexpr int PILOT_SP     = 4;
    constexpr int PILOT_PERIOD = 2;

    ResourceGrid    grid(NUM_SC, NUM_SYM, PILOT_SP, PILOT_PERIOD);
    OFDMModulator   mod(NUM_SC, CP_LEN, grid);
    OFDMDemodulator demod(NUM_SC, CP_LEN, grid);

    // generate known QAM symbols — all +1+0j for simplicity
    int num_data = grid.total_data_symbols();
    std::vector<std::complex<float>> tx_syms(num_data, {1.0f, 0.0f});

    // TX: map to grid, IFFT, add CP
    auto tx_samples = mod.modulate(tx_syms);

    // RX: strip CP, FFT, channel estimate, equalize, unmap
    auto rx_syms = demod.demodulate(tx_samples);

    // check symbols match
    int mismatches = 0;
    size_t n = std::min(tx_syms.size(), rx_syms.size());
    for (size_t i = 0; i < n; ++i) {
        float err = std::abs(tx_syms[i] - rx_syms[i]);
        if (err > 0.01f) {
            if (++mismatches <= 5)
                std::printf("  sym[%zu]: tx=(%.3f,%.3f) rx=(%.3f,%.3f) err=%.4f\n",
                            i,
                            tx_syms[i].real(), tx_syms[i].imag(),
                            rx_syms[i].real(), rx_syms[i].imag(), err);
        }
    }

    if (mismatches == 0) {
        std::printf("  [PASS] OFDM noiseless loopback: %zu symbols match\n", n);
        return true;
    }
    std::printf("  [FAIL] OFDM noiseless loopback: %d mismatches in %zu symbols\n",
                mismatches, n);
    return false;
}

// ─── BER sweep ───────────────────────────────────────────────────────────

struct BERPoint { float snr; float ber; };

std::vector<BERPoint> ber_sweep_uncoded(const std::vector<float>& snr_points,
                                         size_t num_bits)
{
    constexpr int   SPS     = 4;
    constexpr float ROLLOFF = 0.35f;
    constexpr int   SPAN    = 8;

    BPSKModulator   mod(SPS, ROLLOFF, SPAN);
    BPSKDemodulator demod(SPS, ROLLOFF, SPAN);

    std::vector<BERPoint> results;
    for (float snr_db : snr_points) {
        auto tx_bits    = random_bits(num_bits);
        auto tx_samples = mod.modulate(tx_bits);
        AWGNChannel ch(snr_db);
        ch.set_signal_power(measure_power(tx_samples));
        auto rx_bits = demod.demodulate(ch.apply(tx_samples));
        float ber    = BERAnalyzer::compute_ber(tx_bits, rx_bits);
        results.push_back({snr_db, ber});
        std::printf("  [uncoded ] SNR %5.1f dB | BER %.6f\n", snr_db, ber);
    }
    return results;
}

std::vector<BERPoint> ber_sweep_coded(const std::vector<float>& snr_points,
                                       size_t num_bits, bool soft,
                                       int rate_denom,
                                       const std::vector<uint8_t>& polys)
{
    constexpr int   SPS     = 4;
    constexpr float ROLLOFF = 0.35f;
    constexpr int   SPAN    = 8;
    constexpr int   IL_ROWS = 20;
    constexpr int   IL_COLS = 10;

    BPSKModulator        mod(SPS, ROLLOFF, SPAN);
    BPSKDemodulator      demod(SPS, ROLLOFF, SPAN);
    ConvolutionalEncoder enc(rate_denom, 7, polys);
    ViterbiDecoder       dec(rate_denom, 7, polys);
    Interleaver          il(IL_ROWS, IL_COLS);

    std::vector<BERPoint> results;
    for (float snr_db : snr_points) {
        auto tx_bits     = random_bits(num_bits);
        auto coded       = enc.encode(tx_bits);
        auto interleaved = il.interleave(coded);
        size_t il_size   = interleaved.size();
        auto tx_samples  = mod.modulate(interleaved);

        AWGNChannel ch(snr_db);
        ch.set_signal_power(measure_power(tx_samples));
        auto rx_samples = ch.apply(tx_samples);

        std::vector<uint8_t> decoded;
        if (soft) {
            auto llrs = demod.demodulate_soft(rx_samples);
            if (llrs.size() > il_size) llrs.resize(il_size);
            while (llrs.size() < il_size) llrs.push_back(0.0f);
            size_t safe = (llrs.size() / (IL_ROWS * IL_COLS)) * (IL_ROWS * IL_COLS);
            llrs.resize(safe);
            auto llrs_dint = deinterleave_llrs(llrs, IL_ROWS, IL_COLS);
            decoded        = dec.decode_soft(llrs_dint);
        } else {
            auto rx_coded = demod.demodulate(rx_samples);
            if (rx_coded.size() > il_size) rx_coded.resize(il_size);
            while (rx_coded.size() < il_size) rx_coded.push_back(0);
            auto dint = il.deinterleave(rx_coded);
            decoded   = dec.decode_hard(dint);
        }

        size_t cmp = std::min(tx_bits.size(), decoded.size());
        std::vector<uint8_t> tx_trim(tx_bits.begin(), tx_bits.begin() + cmp);
        std::vector<uint8_t> rx_trim(decoded.begin(), decoded.begin() + cmp);

        float ber = BERAnalyzer::compute_ber(tx_trim, rx_trim);
        results.push_back({snr_db, ber});
        std::printf("  [%-8s] SNR %5.1f dB | BER %.6f\n",
                    soft ? "soft" : "hard", snr_db, ber);
    }
    return results;
}

void write_csv(const std::string& path, const std::vector<BERPoint>& points) {
    std::ofstream f(path);
    f << "snr_db,ber\n";
    for (auto& p : points)
        f << p.snr << "," << p.ber << "\n";
}

// ─── main ────────────────────────────────────────────────────────────────

int main() {
    auto bits_100  = random_bits(100);
    auto bits_1000 = random_bits(1000);

    std::cout << "\n[validation] encoder vs liquid-dsp\n";
    std::cout << "rate 1/2 K=7 — 100 bits:\n";
    bool e1 = validate_encoder(2, {0133, 0171}, bits_100);
    std::cout << "rate 1/2 K=7 — 1000 bits:\n";
    bool e2 = validate_encoder(2, {0133, 0171}, bits_1000);

    std::cout << "\n[validation] Viterbi decoder — noiseless loopback\n";
    bool v1 = validate_viterbi_hard(bits_1000);
    bool v2 = validate_viterbi_soft(bits_1000);

    std::cout << "\n[validation] OFDM noiseless loopback\n";
    bool o1 = validate_ofdm_loopback();

    bool all_passed = e1 && e2 && v1 && v2 && o1;
    std::cout << "\n[validation] "
              << (all_passed ? "ALL PASSED" : "FAILURES DETECTED") << "\n";

    if (!all_passed) return 1;

    std::cout << "\n[BER sweep] uncoded vs hard vs soft — BPSK\n\n";

    std::vector<float> snr_uncoded;
    for (float snr = -2.0f; snr <= 10.0f; snr += 0.5f)
        snr_uncoded.push_back(snr);

    std::vector<float> snr_coded;
    for (float snr = -8.0f; snr <= 6.0f; snr += 0.5f)
        snr_coded.push_back(snr);

    constexpr size_t NUM_BITS = 100000;

    std::cout << "uncoded:\n";
    auto uncoded = ber_sweep_uncoded(snr_uncoded, NUM_BITS);

    std::cout << "\nrate 1/2 hard Viterbi:\n";
    auto hard_half = ber_sweep_coded(snr_coded, NUM_BITS, false,
                                      2, {0133, 0171});

    std::cout << "\nrate 1/2 soft Viterbi:\n";
    auto soft_half = ber_sweep_coded(snr_coded, NUM_BITS, true,
                                      2, {0133, 0171});

    std::cout << "\nrate 1/3 hard Viterbi:\n";
    auto hard_third = ber_sweep_coded(snr_coded, NUM_BITS, false,
                                       3, {0133, 0145, 0175});

    std::cout << "\nrate 1/3 soft Viterbi:\n";
    auto soft_third = ber_sweep_coded(snr_coded, NUM_BITS, true,
                                       3, {0133, 0145, 0175});

    write_csv("../results/bpsk_uncoded_ber.csv",         uncoded);
    write_csv("../results/bpsk_hard_viterbi_ber.csv",    hard_half);
    write_csv("../results/bpsk_soft_viterbi_ber.csv",    soft_half);
    write_csv("../results/bpsk_hard_viterbi_13_ber.csv", hard_third);
    write_csv("../results/bpsk_soft_viterbi_13_ber.csv", soft_third);

    std::cout << "\n[BER sweep] results written to results/\n";
    return 0;
}