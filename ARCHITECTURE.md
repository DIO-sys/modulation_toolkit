# Architecture

## Overview

The stack is organized as a pipeline: modulation → channel → demodulation, with FEC and OFDM wrapping that core chain. Each layer was validated independently before being integrated — BER curves against theoretical Q-function bounds, encoder output bit-exact against liquid-dsp, OFDM noiseless loopback before any channel was added. The hardware layer sits alongside the simulation chain and uses the same modulation and FEC components without modification.

---

## Modulation layer

**BPSK, QPSK, QAM** share a common base class that enforces the modulate/demodulate interface and owns the RRC filter. The decision to share the filter rather than instantiate per scheme was deliberate — the RRC tap computation is expensive at initialization and the filter coefficients are identical across schemes for the same `sps`, `rolloff`, and `span`. The base class computes taps once.

RRC pulse shaping is applied at `sps=4`, `α=0.35`, `span=8`. The rolloff was chosen at 0.35 — the LTE standard value — because it balances spectral containment against timing sensitivity. Higher rolloff would give faster pulse decay but wider occupied bandwidth. Lower rolloff would tighten the spectrum but make the timing recovery loop more sensitive to phase error.

The matched filter introduces a group delay of `span × sps / 2` samples. The critical implementation detail is that the TX and RX filters each contribute this delay, so the correct downsample offset is `rrc_.delay() × 2` not `rrc_.delay()`. Getting this wrong produces flat BER at 0.5 regardless of SNR — the demodulator is sampling between symbols rather than at symbol centers.

Gray coding in QAM maps adjacent constellation points to codewords differing by one bit. The implementation separates this cleanly — `axis_value()` maps a Gray index to a normalized constellation position, `slice_axis()` maps a received float back to the nearest binary index, and `demodulate()` Gray-encodes that index before unpacking bits. An earlier version had `slice_axis()` returning an already-Gray-encoded index, causing double encoding and a broken BER floor. The fix was keeping `slice_axis()` in binary space throughout.

The Mueller-Müller PLL tracks symbol timing by computing the error `e = d[n]·s[n-1] - d[n-1]·s[n]` where `d` is the hard decision and `s` is the interpolated sample. The second-order loop filter updates both the fractional offset `mu_` (proportional path) and the samples-per-symbol estimate `omega_` (integral path). In simulation the PLL is disabled — fixed-stride downsampling at the known `sps` is exact. The PLL is enabled for hardware paths where transmitter and receiver clocks are independent.

---

## FEC layer

The convolutional encoder uses a shift register of length K with generator polynomials applied via XOR masking. Standard NASA/CCSDS polynomials `{0133, 0171}` for rate 1/2 and `{0133, 0145, 0175}` for rate 1/3, constraint length K=7. The encoder was validated bit-exact against liquid-dsp's `LIQUID_FEC_CONV_V27` across 100 and 1000 bit test vectors before the Viterbi decoder was written — this catches polynomial endian errors before they become invisible BER issues.

The Viterbi decoder builds the full trellis at construction time rather than on each decode call. `build_trellis()` precomputes `next_state_[state][input]` and `output_bits_[state][input]` for all 64 states and both input bits. The forward ACS pass stores `{prev_state, input_bit}` pairs in the survivor table. The critical design decision here is storing the previous state explicitly rather than scanning `next_state_` forward during traceback — multiple states can transition to the same next state, making forward scanning ambiguous. Tracing backward through stored `prev_state` pointers is unambiguous and O(T) where T is the number of trellis stages.

Soft-decision Viterbi uses LLR correlation as the branch metric: `-sign(expected_bit) × llr`. This weights uncertain received bits less when computing path metrics, giving approximately 2.5 dB gain over hard-decision Viterbi at the same BER. The gain is free — same decoder, same trellis, different branch metric function.

The block interleaver writes bits row-by-row into a `rows × cols` matrix and reads column-by-column. Interleaver depth `20 × 10 = 200` bits means a burst error spanning 20 consecutive coded bits is spread across 20 different codewords after deinterleaving, each seeing only one error. The depth was chosen to be larger than the expected coherence length of the AWGN channel while keeping latency reasonable. For hardware use, depth would be sized to the coherence time of the actual fading channel.

One non-obvious alignment issue: the interleaver pads input to a multiple of `rows × cols`, and the OFDM frame carries a fixed number of bits per frame. If these two numbers are not jointly aligned, the LLR vector after demodulation gets truncated to a size that isn't a multiple of the interleaver block, and the deinterleave index mapping writes out of bounds. The fix is padding to the LCM of `data_per_frame` and `il_block` rather than either independently.

---

## OFDM layer

The resource grid follows the LTE pilot pattern — pilots on every `pilot_spacing`-th subcarrier on every `pilot_period`-th OFDM symbol. `NUM_SC=64` subcarriers with `PILOT_SP=4` gives 16 pilots per pilot symbol, one pilot every 4 subcarriers. This density was chosen to satisfy the Nyquist sampling theorem for the channel — the channel must be sampled at least twice per coherence bandwidth. With a two-tap multipath channel at 4-sample delay and 40 MSPS sample rate, the coherence bandwidth is roughly 10 MHz, and at 40 MHz total bandwidth the pilot spacing of 4 subcarriers (625 kHz per subcarrier × 4 = 2.5 MHz between pilots) is well within the sampling requirement.

The IFFT normalizes by `1/N` to preserve signal power across the transform. The FFT does not — this is intentional and correct. The IFFT converts frequency-domain symbols to time-domain samples, and the `1/N` normalization ensures the time-domain signal has the same power as the frequency-domain input. The FFT at the receiver undoes the time-to-frequency conversion without rescaling, so the received symbols have the correct amplitude for the QAM decision boundaries.

The cyclic prefix is the last `cp_len` samples of the IFFT output prepended to the front. This converts linear convolution with the channel into circular convolution, which is a per-subcarrier multiplication in the frequency domain — `Y[k] = H[k] × X[k]`. The one-tap equalizer at the receiver simply divides by the channel estimate `H[k]` to undo this multiplication. This works exactly as long as `cp_len` exceeds the channel's maximum delay spread. With `CP_LEN=16` at 40 MSPS the protected delay spread is 400 ns — sufficient for the two-tap 4-sample test channel and most indoor multipath environments.

Channel estimation uses least-squares at pilot subcarriers: `H[k] = Y[k] / X[k]` where `X[k]` is the known pilot value `{1.0f, 0.0f}`. Between pilots, the channel estimate is linearly interpolated. This is optimal for a static or slowly-varying channel. For a fast-fading channel, higher-order interpolation or a Wiener filter estimator would be needed — linear interpolation assumes the channel is approximately constant between pilots, which fails when the coherence time is shorter than the pilot period.

---

## Channel models

The AWGN channel computes noise sigma as `sqrt(signal_power / (2 × linear_snr))`. The factor of 2 divides noise power equally between I and Q components — each receives independent Gaussian noise with variance `sigma²`, giving total noise power `2 × sigma² = signal_power / linear_snr`. The `mutable` RNG is necessary because `apply()` is const — the channel model itself doesn't change, only the internal noise state advances.

The multipath channel applies each tap as a delayed and scaled copy of the input: `out[n] += gain[t] × in[n - delay[t]]`. The two-tap model `{0, 4}` with gains `{1.0, 0.3}` represents a direct path plus a 30% reflection arriving 4 samples later (100 ns at 40 MSPS). AWGN is added on top of the multipath output. The result is a frequency-selective fading channel — some subcarriers see constructive interference, others destructive, creating a non-flat frequency response that the channel estimator must correct.

---

## Hardware layer

The `BladeRFDevice` owns two background threads — a TX thread that continuously loops the modulated buffer through `bladerf_sync_tx`, and an RX thread that continuously calls `bladerf_sync_rx` and pushes samples into the `CircularBuffer`. Both threads must run simultaneously — `bladerf_sync_tx` and `bladerf_sync_rx` are blocking calls that stall if the other direction isn't also actively streaming. Calling them sequentially causes the inactive direction to timeout.

The `CircularBuffer` is a lock-free SPSC ring buffer with power-of-two capacity. `push()` uses `memory_order_release` when updating `write_pos_` and `pop_batch()` uses `memory_order_acquire` when reading it. This ordering guarantees the consumer sees all writes to `buf_[write & mask_]` that happened before the producer's store to `write_pos_`. Without the acquire-release pair, the consumer could read stale buffer contents even though `write_pos_` appeared updated.

The TX buffer is padded to a multiple of `TRANSFER_SAMPLES=4096` by repeating the frame rather than zero-padding. Zero-padding creates silence gaps between frames — the RX collects samples during the gap and the demodulator receives a constant DC level rather than the modulated signal. Repeating the frame eliminates the gap while keeping the buffer a valid multiple of the USB transfer size.

Hardware BER validation confirmed 14 dB SNR above the noise floor through the RF cable loopback. Full BER curve collection was not completed — the AD9361 introduces a carrier frequency offset between TX and RX even at the same nominal frequency setting, and without a carrier recovery loop the received constellation rotates continuously, making coherent demodulation impossible. Implementing a PLL-based carrier frequency offset estimator is the remaining step for hardware BER validation.

---

## C-V2X sidelink

The PC5 demo runs on top of the OFDM PHY without modification. The SCI format 1 header packs 32 bits encoding priority (3 bits), resource reservation interval (4 bits), MCS index (5 bits), retransmission index (2 bits), starting resource block (10 bits), and destination group ID (8 bits). The BSM serializes to 17 bytes covering vehicle ID, latitude, longitude, speed, heading, and brake status — a subset of the SAE J2735 core fields sufficient to demonstrate the full encode-transmit-decode chain.

The choice to use the convolutional code rather than the 3GPP turbo code was deliberate — the convolutional encoder was already validated against liquid-dsp and the demo's purpose is to show the full stack working end to end, not to implement every detail of the PC5 spec. The coding gain demonstrated is real and the BER behavior is correct. A production implementation would replace the convolutional code with a turbo encoder and add the full SCI CRC.