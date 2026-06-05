import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erfc

SPS = 4

def read_csv(path):
    snr, ber = [], []
    with open(path) as f:
        next(f)
        for row in csv.reader(f):
            snr.append(float(row[0]))
            ber.append(float(row[1]))
    return np.array(snr), np.array(ber)

def to_ebn0(snr_db, bits_per_symbol, sps):
    # convert per-sample SNR to Eb/N0
    # Eb/N0 = SNR_sample * sps / bits_per_symbol
    return snr_db + 10 * np.log10(sps / bits_per_symbol)

def theoretical_bpsk(ebn0_db):
    ebn0_lin = 10 ** (ebn0_db / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin))

def theoretical_qpsk(ebn0_db):
    return theoretical_bpsk(ebn0_db)

def theoretical_qam(ebn0_db, M):
    ebn0_lin = 10 ** (ebn0_db / 10.0)
    m = int(np.sqrt(M))
    k = int(np.log2(M))
    return (2 * (1 - 1/m) / k) * erfc(np.sqrt(3 * k * ebn0_lin / (2 * (M - 1))))

results_dir = "../results"

schemes = [
    ("BPSK",   "bpsk_ber.csv",  "blue",   1),
    ("QPSK",   "qpsk_ber.csv",  "green",  2),
    ("16-QAM", "qam16_ber.csv", "orange", 4),
    ("64-QAM", "qam64_ber.csv", "red",    6),
]

fig, ax = plt.subplots(figsize=(10, 7))

ebn0_theory = np.linspace(-2, 18, 300)

ax.semilogy(ebn0_theory, theoretical_bpsk(ebn0_theory),
            '--', color='blue',   alpha=0.5, label='BPSK theoretical')
ax.semilogy(ebn0_theory, theoretical_qpsk(ebn0_theory),
            '--', color='green',  alpha=0.5, label='QPSK theoretical')
ax.semilogy(ebn0_theory, theoretical_qam(ebn0_theory, 16),
            '--', color='orange', alpha=0.5, label='16-QAM theoretical')
ax.semilogy(ebn0_theory, theoretical_qam(ebn0_theory, 64),
            '--', color='red',    alpha=0.5, label='64-QAM theoretical')

for name, fname, color, bps in schemes:
    path = os.path.join(results_dir, fname)
    if not os.path.exists(path):
        print(f"[skip] {path} not found")
        continue
    snr, ber = read_csv(path)
    ebn0 = to_ebn0(snr, bps, SPS)
    mask = ber > 0
    ax.semilogy(ebn0[mask], ber[mask], '-o', color=color,
                markersize=4, label=f'{name} simulated')

ax.set_xlabel('Eb/N0 (dB)', fontsize=13)
ax.set_ylabel('BER', fontsize=13)
ax.set_title('BER vs Eb/N0 — uncoded modulation schemes', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, which='both', linestyle='--', alpha=0.5)
ax.set_xlim(-2, 18)
ax.set_ylim(1e-5, 1.0)

plt.tight_layout()
plt.savefig('../results/ber_curves.png', dpi=150)
plt.show()
print("[plot] saved to results/ber_curves.png")