#pragma once
#include "hw/app_state.hpp"
#include "hw/circular_buffer.hpp"
#include <complex>
#include <cstdint>
#include <vector>

struct bladerf;

class BladeRFDevice {
public:
    BladeRFDevice(CircularBuffer<std::complex<float>>& rx_buf, AppState& state);
    ~BladeRFDevice();

    bool open();
    bool configure();
    void close();

    // RX — blocking, fills rx_buf_ with TRANSFER_SAMPLES normalized IQ samples
    void receive();

    // TX — blocking, transmits count samples from buf
    void transmit(const std::complex<float>* buf, size_t count);

    // frequency and gain — callable from main thread while streaming
    bool set_frequency(uint64_t freq_hz);
    bool set_rx_gain(int gain_db);
    bool set_tx_gain(int gain_db);

private:
    bool configure_rx();
    bool configure_tx();
    void scale_and_push(const int16_t* raw, unsigned int num_samples);
    void log_error(const char* context, int status);

    CircularBuffer<std::complex<float>>& rx_buf_;
    AppState&                            state_;
    struct bladerf*                      dev_{ nullptr };

    std::vector<int16_t> rx_raw_buf_;
    std::vector<int16_t> tx_raw_buf_;

    static constexpr unsigned int TRANSFER_SAMPLES = 4096;
    static constexpr unsigned int SAMPLE_RATE_HZ   = 40'000'000;
    static constexpr unsigned int BANDWIDTH_HZ     = 40'000'000;
};