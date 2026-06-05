#include "modulation/fm_mod.hpp"
#include <cmath>

FMModulator::FMModulator(int sps, float freq_deviation,
                         float rolloff, int span)
    : ModulatorBase(sps, rolloff, span)
    , freq_dev_(freq_deviation)
{}

std::vector<std::complex<float>>
FMModulator::modulate(const std::vector<uint8_t>& bits) {
    std::vector<std::complex<float>> samples;
    samples.reserve(bits.size() * sps_);

    float phase = 0.0f;

    for (uint8_t bit : bits) {
        // FM: frequency shift based on bit value
        // bit 0 → positive frequency shift
        // bit 1 → negative frequency shift
        float freq = (bit == 0) ? freq_dev_ : -freq_dev_;

        for (int s = 0; s < sps_; ++s) {
            phase += 2.0f * static_cast<float>(M_PI) * freq;
            // wrap phase to [-pi, pi] to prevent float overflow
            if (phase >  static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
            if (phase < -static_cast<float>(M_PI)) phase += 2.0f * static_cast<float>(M_PI);
            samples.push_back({std::cos(phase), std::sin(phase)});
        }
    }

    return samples;
}