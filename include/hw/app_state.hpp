#pragma once
#include <atomic>
#include <cstdint>

struct AppState {
    // device config — UI or main writes, threads read
    std::atomic<uint64_t> center_freq_hz { 915'000'000ULL };
    std::atomic<int>      rx_gain_db     { 30 };
    std::atomic<int>      tx_gain_db     { 0 };
    std::atomic<bool>     running        { true };

    // loopback control — set by main before thread launch
    std::atomic<bool>     loopback_mode  { false };

    AppState()                           = default;
    AppState(const AppState&)            = delete;
    AppState& operator=(const AppState&) = delete;
};