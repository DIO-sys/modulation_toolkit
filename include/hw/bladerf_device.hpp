#pragma once
#include "hw/app_state.hpp"
#include "hw/circular_buffer.hpp"
#include <complex>
#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>

struct bladerf;

class BladeRFDevice {
public:
    BladeRFDevice(CircularBuffer<std::complex<float>>& rx_buf, AppState& state);
    ~BladeRFDevice();

    bool open();
    bool configure();
    void close();

    // single-shot blocking TX/RX — used for one-off transfers
    void receive();
    void transmit(const std::complex<float>* buf, size_t count);

    // streaming TX — continuously repeats tx_buf in a background thread
    // call start_tx_stream() before start_rx_stream()
    void start_tx_stream(const std::vector<std::complex<float>>& tx_buf);
    void stop_tx_stream();

    // streaming RX — continuously pushes into rx_buf_ in a background thread
    void start_rx_stream();
    void stop_rx_stream();

    bool set_frequency(uint64_t freq_hz);
    bool set_rx_gain(int gain_db);
    bool set_tx_gain(int gain_db);

private:
    bool configure_rx();
    bool configure_tx();
    void scale_and_push(const int16_t* raw, unsigned int num_samples);
    void log_error(const char* context, int status);

    void tx_thread_fn(std::vector<std::complex<float>> tx_buf);
    void rx_thread_fn();

    CircularBuffer<std::complex<float>>& rx_buf_;
    AppState&                            state_;
    struct bladerf*                      dev_{ nullptr };

    std::vector<int16_t> rx_raw_buf_;
    std::vector<int16_t> tx_raw_buf_;

    std::thread      tx_thread_;
    std::thread      rx_thread_;
    std::atomic<bool> tx_running_{ false };
    std::atomic<bool> rx_running_{ false };

    static constexpr unsigned int TRANSFER_SAMPLES = 4096;
    static constexpr unsigned int SAMPLE_RATE_HZ   = 40'000'000;
    static constexpr unsigned int BANDWIDTH_HZ     = 40'000'000;
};