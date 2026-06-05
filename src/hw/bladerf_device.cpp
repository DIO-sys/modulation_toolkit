#include "hw/bladerf_device.hpp"
#include <libbladeRF.h>
#include <iostream>
using namespace std;

BladeRFDevice::BladeRFDevice(CircularBuffer<complex<float>>& rx_buf, AppState& state)
    : rx_buf_(rx_buf)
    , state_(state)
    , rx_raw_buf_(TRANSFER_SAMPLES * 2)
    , tx_raw_buf_(TRANSFER_SAMPLES * 2)
{}

BladeRFDevice::~BladeRFDevice() {
    close();
}

bool BladeRFDevice::open() {
    int status = bladerf_open(&dev_, nullptr);
    if (status != 0) {
        log_error("bladerf_open", status);
        return false;
    }
    cout << "[bladerf] device opened\n";
    return true;
}

bool BladeRFDevice::configure() {
    return configure_rx() && configure_tx();
}

bool BladeRFDevice::configure_rx() {
    int status;
    uint32_t actual_rate = 0;

    status = bladerf_set_sample_rate(dev_, BLADERF_CHANNEL_RX(0),
                                     SAMPLE_RATE_HZ, &actual_rate);
    if (status != 0) { log_error("set_sample_rate RX", status); return false; }
    cout << "[bladerf] RX sample rate: " << actual_rate << " sps\n";

    uint32_t actual_bw = 0;
    status = bladerf_set_bandwidth(dev_, BLADERF_CHANNEL_RX(0),
                                   BANDWIDTH_HZ, &actual_bw);
    if (status != 0) { log_error("set_bandwidth RX", status); return false; }

    status = bladerf_set_frequency(dev_, BLADERF_CHANNEL_RX(0),
                                   state_.center_freq_hz.load());
    if (status != 0) { log_error("set_frequency RX", status); return false; }

    status = bladerf_set_gain(dev_, BLADERF_CHANNEL_RX(0),
                              state_.rx_gain_db.load());
    if (status != 0) { log_error("set_gain RX", status); return false; }

    status = bladerf_sync_config(dev_, BLADERF_RX_X1,
                                 BLADERF_FORMAT_SC16_Q11,
                                 32, TRANSFER_SAMPLES, 4, 5000);
    if (status != 0) { log_error("sync_config RX", status); return false; }

    // DC offset correction — same as spectrum_analyzer
    bladerf_set_correction(dev_, BLADERF_CHANNEL_RX(0), BLADERF_CORR_DCOFF_I, 0);
    bladerf_set_correction(dev_, BLADERF_CHANNEL_RX(0), BLADERF_CORR_DCOFF_Q, 0);

    status = bladerf_enable_module(dev_, BLADERF_CHANNEL_RX(0), true);
    if (status != 0) { log_error("enable_module RX", status); return false; }

    cout << "[bladerf] RX configured\n";
    return true;
}

bool BladeRFDevice::configure_tx() {
    int status;
    uint32_t actual_rate = 0;

    status = bladerf_set_sample_rate(dev_, BLADERF_CHANNEL_TX(0),
                                     SAMPLE_RATE_HZ, &actual_rate);
    if (status != 0) { log_error("set_sample_rate TX", status); return false; }

    uint32_t actual_bw = 0;
    status = bladerf_set_bandwidth(dev_, BLADERF_CHANNEL_TX(0),
                                   BANDWIDTH_HZ, &actual_bw);
    if (status != 0) { log_error("set_bandwidth TX", status); return false; }

    status = bladerf_set_frequency(dev_, BLADERF_CHANNEL_TX(0),
                                   state_.center_freq_hz.load());
    if (status != 0) { log_error("set_frequency TX", status); return false; }

    status = bladerf_set_gain(dev_, BLADERF_CHANNEL_TX(0),
                              state_.tx_gain_db.load());
    if (status != 0) { log_error("set_gain TX", status); return false; }

    status = bladerf_sync_config(dev_, BLADERF_TX_X1,
                                 BLADERF_FORMAT_SC16_Q11,
                                 32, TRANSFER_SAMPLES, 4, 5000);
    if (status != 0) { log_error("sync_config TX", status); return false; }

    status = bladerf_enable_module(dev_, BLADERF_CHANNEL_TX(0), true);
    if (status != 0) { log_error("enable_module TX", status); return false; }

    cout << "[bladerf] TX configured\n";
    return true;
}

void BladeRFDevice::close() {
    if (dev_) {
        bladerf_enable_module(dev_, BLADERF_CHANNEL_RX(0), false);
        bladerf_enable_module(dev_, BLADERF_CHANNEL_TX(0), false);
        bladerf_close(dev_);
        dev_ = nullptr;
        cout << "[bladerf] device closed\n";
    }
}

void BladeRFDevice::receive() {
    int status = bladerf_sync_rx(dev_, rx_raw_buf_.data(), TRANSFER_SAMPLES,
                                 nullptr, 5000);
    if (status != 0) {
        log_error("bladerf_sync_rx", status);
        state_.running.store(false);
        return;
    }
    scale_and_push(rx_raw_buf_.data(), TRANSFER_SAMPLES);
}

void BladeRFDevice::transmit(const complex<float>* buf, size_t count) {
    // normalize float [-1,1] back to SC16_Q11 range [-2048, 2047]
    for (size_t i = 0; i < count; ++i) {
        tx_raw_buf_[i * 2]     = static_cast<int16_t>(buf[i].real() * 2047.0f);
        tx_raw_buf_[i * 2 + 1] = static_cast<int16_t>(buf[i].imag() * 2047.0f);
    }
    int status = bladerf_sync_tx(dev_, tx_raw_buf_.data(),
                                 static_cast<unsigned int>(count),
                                 nullptr, 5000);
    if (status != 0) {
        log_error("bladerf_sync_tx", status);
        state_.running.store(false);
    }
}

bool BladeRFDevice::set_frequency(uint64_t freq_hz) {
    int status = bladerf_set_frequency(dev_, BLADERF_CHANNEL_RX(0), freq_hz);
    if (status != 0) { log_error("set_frequency RX", status); return false; }
    status = bladerf_set_frequency(dev_, BLADERF_CHANNEL_TX(0), freq_hz);
    if (status != 0) { log_error("set_frequency TX", status); return false; }
    state_.center_freq_hz.store(freq_hz);
    return true;
}

bool BladeRFDevice::set_rx_gain(int gain_db) {
    int status = bladerf_set_gain(dev_, BLADERF_CHANNEL_RX(0), gain_db);
    if (status != 0) { log_error("set_gain RX", status); return false; }
    state_.rx_gain_db.store(gain_db);
    return true;
}

bool BladeRFDevice::set_tx_gain(int gain_db) {
    int status = bladerf_set_gain(dev_, BLADERF_CHANNEL_TX(0), gain_db);
    if (status != 0) { log_error("set_gain TX", status); return false; }
    state_.tx_gain_db.store(gain_db);
    return true;
}

void BladeRFDevice::scale_and_push(const int16_t* raw, unsigned int num_samples) {
    for (unsigned int i = 0; i < num_samples; ++i) {
        float i_val = raw[i * 2]     / 2048.0f;
        float q_val = raw[i * 2 + 1] / 2048.0f;
        rx_buf_.push(complex<float>(i_val, q_val));
    }
}

void BladeRFDevice::log_error(const char* context, int status) {
    cerr << "[bladerf] " << context
         << " failed: " << bladerf_strerror(status) << "\n";
}