# Architecture

## Overview

This stack was built with one goal: understand what actually happens between a transmitter and a receiver at for things like link budget analysis. Every layer was validated independently before integration. BER curves against theoretical Q-function bounds. Encoder output bit-exact against liquid-dsp. OFDM noiseless loopback before any channel noise was introduced. The hardware layer uses the same modulation and FEC components as the simulation without modification, which means any bug found in simulation is also a bug in hardware.

---

## Modulation layer

BPSK, QPSK, and QAM share a common base class that owns the RRC filter. This was a deliberate choice rather than instantiating a filter per scheme. The tap computation runs once at construction and the coefficients are identical across all schemes for the same `sps`, `rolloff`, and `span`. No reason to recompute them.

RRC pulse shaping runs at `sps=4`, rolloff 0.35, span 8. The 0.35 rolloff is the LTE standard value and it sits at a reasonable balance between spectral containment and timing sensitivity. Go lower and you tighten the spectrum but the timing recovery loop becomes more sensitive to phase error. Go higher and the pulse decays faster but you use more bandwidth. 0.35 is the answer the industry settled on.

The matched filter introduces a group delay of `span * sps / 2` samples on each side. Because both the TX and RX filters contribute this delay, the correct downsample offset is `rrc_.delay() * 2`, not `rrc_.delay()`. This was the first real bug in the project. With the wrong offset the demodulator samples between symbols rather than at symbol centers and BER sits at 0.5 regardless of how much SNR you add.

Gray coding in QAM is implemented with a clean separation of concerns. `axis_value()` maps a Gray index to a normalized constellation position. `slice_axis()` maps a received float back to the nearest binary index. `demodulate()` Gray-encodes that index before unpacking bits. An earlier version had `slice_axis()` returning an already-encoded index, which caused double encoding and a broken BER floor. The fix was keeping `slice_axis()` in binary space throughout and letting `demodulate()` handle the Gray encoding in one place.

The Mueller-Muller PLL tracks symbol timing using the error term `e = d[n] * s[n-1] - d[n-1] * s[n]` where `d` is the hard decision and `s` is the interpolated sample. The second-order loop filter updates both the fractional offset `mu_` and the samples-per-symbol estimate `omega_`. In simulation the PLL is off since fixed-stride downsampling at the known `sps` is exact. On hardware, where transmitter and receiver clocks are independent, it has to be on.

---

## FEC layer

The convolutional encoder uses a shift register of length K with generator polynomials applied via XOR masking. Standard polynomials `{0133, 0171}` for rate 1/2 and `{0133, 0145, 0175}` for rate 1/3, constraint length K=7. The encoder was validated bit-exact against liquid-dsp before the Viterbi decoder was written. That ordering matters. Catching a polynomial endian error at the encoder stage is a five minute fix. Finding it six weeks later as a mysterious BER floor is not.

The Viterbi decoder builds the full trellis at construction rather than on each decode call. `build_trellis()` precomputes `next_state_[state][input]` and `output_bits_[state][input]` for all 64 states and both input bits. The forward ACS pass stores `{prev_state, input_bit}` pairs in the survivor table. The key decision here is storing the previous state explicitly rather than scanning `next_state_` forward during traceback. Multiple states can transition to the same next state, which makes forward scanning ambiguous. Following stored `prev_state` pointers backward is unambiguous and runs in O(T) time. This was the bug that caused 503 bit errors out of 1000 in noiseless loopback before the fix.

Soft-decision Viterbi uses LLR correlation as the branch metric: `-sign(expected_bit) * llr`. This weights uncertain bits less when computing path metrics and gives roughly 2.5 dB gain over hard-decision at the same BER. Same decoder, same trellis, different branch metric function. The gain is free.

The block interleaver writes bits row-by-row and reads column-by-column. Depth `20 * 10 = 200` bits means a burst error spanning 20 consecutive coded bits gets spread across 20 different codewords after deinterleaving, each codeword seeing only one error. For a Qualcomm or L3Harris production system the depth would be sized to the coherence time of the actual fading channel rather than a fixed value.

One alignment problem took real time to track down. The interleaver pads input to a multiple of `rows * cols` and the OFDM frame carries a fixed number of bits. If those two numbers are not jointly divisible, the LLR vector after demodulation gets truncated to a size that is not a multiple of the interleaver block and the deinterleave index mapping writes out of bounds. The fix is padding to the LCM of `data_per_frame` and `il_block` rather than either independently.

---

## OFDM layer

The resource grid follows the LTE pilot pattern with pilots on every `pilot_spacing`-th subcarrier on every `pilot_period`-th symbol. With 64 subcarriers and `PILOT_SP=4`, there are 16 pilots per pilot symbol. This density satisfies the Nyquist criterion for channel estimation: the channel must be sampled at least twice per coherence bandwidth. With a two-tap multipath channel at 4-sample delay and 40 MSPS sample rate, the coherence bandwidth is roughly 10 MHz. At 40 MHz total bandwidth, 2.5 MHz between pilots is well within that requirement.

The IFFT normalizes by `1/N` to preserve signal power across the transform. The FFT at the receiver does not normalize. This is intentional. The IFFT converts frequency-domain symbols to time-domain samples, and the normalization ensures the time-domain signal has the same power as the input. The FFT undoes the transform without rescaling so the received symbols land at the correct amplitude for the QAM decision boundaries. Getting this backwards produces an unexplained SNR penalty that is genuinely annoying to find.

The cyclic prefix is the last `cp_len` samples of the IFFT output prepended to the front. This converts linear convolution with the channel into circular convolution, which in the frequency domain is a per-subcarrier multiplication: `Y[k] = H[k] * X[k]`. The one-tap equalizer divides by the channel estimate `H[k]` to undo it. This works exactly as long as `cp_len` exceeds the channel's maximum delay spread. With `CP_LEN=16` at 40 MSPS the protected delay spread is 400 ns, which covers the two-tap test channel and most indoor multipath environments.

Channel estimation uses least-squares at pilot subcarriers: `H[k] = Y[k] / X[k]` where `X[k]` is the known pilot value. Between pilots the estimate is linearly interpolated. This is the right approach for a static or slowly-varying channel. For fast fading you would need higher-order interpolation or a Wiener filter estimator. Linear interpolation assumes the channel is approximately constant between pilots and that assumption breaks when the coherence time drops below the pilot period.

---

## Channel models

The AWGN channel computes noise sigma as `sqrt(signal_power / (2 * linear_snr))`. The factor of 2 splits noise power equally between I and Q. Each component receives independent Gaussian noise with variance `sigma^2`, giving total noise power `2 * sigma^2 = signal_power / linear_snr`. The RNG is marked `mutable` because `apply()` is const. The channel model itself does not change state, only the internal noise generator advances.

The multipath channel applies each tap as a delayed and scaled copy of the input: `out[n] += gain[t] * in[n - delay[t]]`. The two-tap model with delays `{0, 4}` and gains `{1.0, 0.3}` represents a direct path plus a 30% reflection arriving 4 samples later, which is 100 ns at 40 MSPS. AWGN is added on top of the multipath output. The result is a frequency-selective fading channel where some subcarriers see constructive interference and others see destructive interference. The channel estimator has to correct this subcarrier by subcarrier.

---

## Hardware layer

`BladeRFDevice` owns two background threads. A TX thread that continuously loops the modulated buffer through `bladerf_sync_tx` and an RX thread that continuously calls `bladerf_sync_rx` and pushes samples into the `CircularBuffer`. Both threads have to run simultaneously because `bladerf_sync_tx` and `bladerf_sync_rx` are blocking calls that stall if the other direction is not also actively streaming. Calling them sequentially causes the inactive direction to timeout, which was the first hardware failure.

The `CircularBuffer` is a lock-free SPSC ring buffer with power-of-two capacity. `push()` uses `memory_order_release` when updating `write_pos_` and `pop_batch()` uses `memory_order_acquire` when reading it. This ordering guarantees the consumer sees all writes to `buf_[write & mask_]` that happened before the producer's store to `write_pos_`. Without the acquire-release pair the consumer can read stale buffer contents even when `write_pos_` appears updated. This is the kind of bug that reproduces intermittently and takes a long time to find without knowing what to look for.

The TX buffer is padded to a multiple of `TRANSFER_SAMPLES=4096` by repeating the frame rather than zero-padding. Zero-padding creates silence gaps between frames. The RX collects samples during the gap and the demodulator receives a constant DC level instead of the modulated signal. Repeating the frame eliminates the gap while keeping the buffer a valid multiple of the USB transfer size. This was found by printing the raw RX sample amplitudes and seeing 20 consecutive identical values.

Hardware BER validation confirmed 14 dB SNR above the noise floor through the RF cable loopback between TX1 and RX1. Full BER curves were not collected and the next step for this project. The AD9361 introduces a carrier frequency offset between TX and RX even at the same nominal frequency setting. Without a carrier recovery loop the received constellation rotates continuously and coherent demodulation is not possible. A PLL-based carrier frequency offset estimator is the remaining component for hardware BER validation. The infrastructure for hardware streaming is complete and verified.

---

## C-V2X sidelink

The PC5 demo runs on top of the OFDM PHY without any modification to the layers below it. The SCI format 1 header packs 32 bits covering priority (3 bits), resource reservation interval (4 bits), MCS index (5 bits), retransmission index (2 bits), starting resource block (10 bits), and destination group ID (8 bits). The BSM serializes to 17 bytes covering vehicle ID, latitude, longitude, speed, heading, and brake status. This is a subset of the SAE J2735 core fields, enough to demonstrate the full encode-transmit-receive-decode chain with real safety message content.

The convolutional code was used instead of the 3GPP turbo code for a practical reason. The convolutional encoder was already validated against liquid-dsp and the purpose of this demo is to show the complete stack working end to end, not to implement every detail of the PC5 spec. The coding gain is real, the BER behavior is correct, and the same decoder that was validated in Phase 3 is running here unchanged. A production implementation targeting a system like Qualcomm's C-V2X chipset would replace the convolutional code with a turbo encoder and add the full SCI CRC.
