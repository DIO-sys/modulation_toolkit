import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erfc

SPS = 4
BPS = 1  # BPSK

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

def theoretical_bpsk(ebn0_db):
    ebn0_lin = 10 ** (ebn0_db / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin))

def theoretical_bpsk_coded_hard(ebn0_db, rate=0.5):
    ebn0_lin = 10 ** ((ebn0_db + 5.0) / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin * rate))

def theoretical_bpsk_coded_soft(ebn0_db, rate=0.5):
    ebn0_lin = 10 ** ((ebn0_db + 7.5) / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin * rate))

def theoretical_bpsk_coded_hard_13(ebn0_db):
    ebn0_lin = 10 ** ((ebn0_db + 7.0) / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin * (1/3)))

def theoretical_bpsk_coded_soft_13(ebn0_db):
    ebn0_lin = 10 ** ((ebn0_db + 9.5) / 10.0)
    return 0.5 * erfc(np.sqrt(ebn0_lin * (1/3)))

results_dir = "../results"

files = [
    ("bpsk_uncoded_ber.csv",         "Uncoded BPSK",       "blue"),
    ("bpsk_hard_viterbi_ber.csv",    "Hard Viterbi (1/2)", "orange"),
    ("bpsk_soft_viterbi_ber.csv",    "Soft Viterbi (1/2)", "green"),
    ("bpsk_hard_viterbi_13_ber.csv", "Hard Viterbi (1/3)", "red"),
    ("bpsk_soft_viterbi_13_ber.csv", "Soft Viterbi (1/3)", "purple"),
]

fig, ax = plt.subplots(figsize=(10, 7))

ebn0_theory = np.linspace(-8, 14, 400)

ax.semilogy(ebn0_theory, theoretical_bpsk(ebn0_theory),
            '--', color='blue',   alpha=0.4, label='Uncoded theoretical')
ax.semilogy(ebn0_theory, theoretical_bpsk_coded_hard(ebn0_theory),
            '--', color='orange', alpha=0.4, label='Hard 1/2 theoretical')
ax.semilogy(ebn0_theory, theoretical_bpsk_coded_soft(ebn0_theory),
            '--', color='green',  alpha=0.4, label='Soft 1/2 theoretical')
ax.semilogy(ebn0_theory, theoretical_bpsk_coded_hard_13(ebn0_theory),
            '--', color='red',    alpha=0.4, label='Hard 1/3 theoretical')
ax.semilogy(ebn0_theory, theoretical_bpsk_coded_soft_13(ebn0_theory),
            '--', color='purple', alpha=0.4, label='Soft 1/3 theoretical')

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
ax.set_title('BER vs Eb/N0 — uncoded vs hard vs soft Viterbi (1/2 and 1/3)', fontsize=14)
ax.legend(fontsize=10)
ax.grid(True, which='both', linestyle='--', alpha=0.5)
ax.set_xlim(-8, 14)
ax.set_ylim(1e-6, 1.0)

plt.tight_layout()
plt.savefig('../results/viterbi_comparison.png', dpi=150)
plt.show()
print("[plot] saved to results/viterbi_comparison.png")