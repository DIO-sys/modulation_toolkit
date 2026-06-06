import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erfc

SPS = 1       # OFDM uses no oversampling
BPS = 2       # QPSK inside OFDM
RATE = 0.5    # rate 1/2 convolutional code

def read_csv(path):
    snr, ber = [], []
    with open(path) as f:
        next(f)
        for row in csv.reader(f):
            snr.append(float(row[0]))
            ber.append(float(row[1]))
    return np.array(snr), np.array(ber)

def to_ebn0(snr_db, bits_per_symbol, sps):
    return snr_db + 10 * np.log10(sps / bits_per_symbol)

def theoretical_qpsk_coded_soft(ebn0_db, rate=0.5):
    ebn0_lin = 10 ** ((ebn0_db + 7.5) / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin * rate))

results_dir = "../results"

files = [
    ("ofdm_awgn_ber.csv",      "Coded OFDM — AWGN",      "blue"),
    ("ofdm_multipath_ber.csv", "Coded OFDM — multipath",  "orange"),
]

fig, ax = plt.subplots(figsize=(10, 7))

ebn0_theory = np.linspace(-4, 14, 400)

ax.semilogy(ebn0_theory, theoretical_qpsk_coded_soft(ebn0_theory),
            '--', color='gray', alpha=0.5,
            label='Soft coded QPSK theoretical')

for fname, label, color in files:
    path = os.path.join(results_dir, fname)
    if not os.path.exists(path):
        print(f"[skip] {path} not found")
        continue
    snr, ber = read_csv(path)
    ebn0 = to_ebn0(snr, BPS, SPS)
    mask = ber > 0
    ax.semilogy(ebn0[mask], ber[mask], '-o', color=color,
                markersize=4, label=f'{label} simulated')

ax.set_xlabel('Eb/N0 (dB)', fontsize=13)
ax.set_ylabel('BER', fontsize=13)
ax.set_title('Coded OFDM BER — AWGN vs multipath (CP length = 16)', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, which='both', linestyle='--', alpha=0.5)
ax.set_xlim(-4, 14)
ax.set_ylim(1e-5, 1.0)

plt.tight_layout()
plt.savefig('../results/ofdm_multipath_comparison.png', dpi=150)
plt.show()
print("[plot] saved to results/ofdm_multipath_comparison.png")