"""compare_results.py — Compare Encrypted ILC vs Plaintext ILC results.

Loads all iterations from:
  data/          → Encrypted ILC  (TCP.py + EncILC.go)
  Plaintext/data/ → Plaintext ILC  (TCP_plain.py)

Produces two figures:
  Fig 1 — RMS error convergence over iterations (pitch / yaw / total)
  Fig 2 — Trajectory + error comparison at first and last shared iteration

Saves PDFs to the current directory.
"""

import os, glob
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

BASE_DIR  = os.path.dirname(os.path.abspath(__file__))
ENC_DIR   = os.path.join(BASE_DIR, 'data')
PLAIN_DIR = os.path.join(BASE_DIR, 'Plaintext', 'data')

# ── Helpers ───────────────────────────────────────────────────────────────
def load_all(data_dir):
    files = sorted(
        glob.glob(os.path.join(data_dir, 'data_*.npz')),
        key=lambda f: int(os.path.splitext(os.path.basename(f))[0].split('_')[1])
    )
    return [np.load(f) for f in files]

def rms_deg(E, col):
    return np.rad2deg(np.sqrt(np.mean(E[:, col] ** 2)))

enc_data   = load_all(ENC_DIR)
plain_data = load_all(PLAIN_DIR)

if not enc_data:
    raise FileNotFoundError(f"No encrypted data found in {ENC_DIR}")
if not plain_data:
    raise FileNotFoundError(f"No plaintext data found in {PLAIN_DIR}")

print(f"Encrypted  iterations: {len(enc_data)}")
print(f"Plaintext  iterations: {len(plain_data)}")

# ── Style ─────────────────────────────────────────────────────────────────
mpl.rcParams.update({
    'font.family':       'serif',
    'font.size':         11,
    'axes.labelsize':    11,
    'axes.titlesize':    11,
    'xtick.labelsize':   10,
    'ytick.labelsize':   10,
    'legend.fontsize':   10,
    'legend.framealpha': 0.85,
    'legend.edgecolor':  '0.7',
    'lines.linewidth':   1.5,
    'axes.linewidth':    0.8,
    'grid.linewidth':    0.5,
    'grid.linestyle':    ':',
    'grid.alpha':        0.6,
})

C_ENC   = '#1F77B4'   # blue  — encrypted
C_PLAIN = '#D62728'   # red   — plaintext
C_REF   = '#333333'   # dark grey — reference

# ═══════════════════════════════════════════════════════════════════════════
# Figure 1 — RMS convergence curves
# ═══════════════════════════════════════════════════════════════════════════
def build_rms_series(data_list):
    ks, ep_list, ey_list, et_list = [], [], [], []
    for d in data_list:
        nv = int(d['n_valid'])
        E  = d['E'][:nv]
        ep = rms_deg(E, 0)
        ey = rms_deg(E, 1)
        ks.append(int(d['k']))
        ep_list.append(ep)
        ey_list.append(ey)
        et_list.append(np.sqrt(ep**2 + ey**2))
    return np.array(ks), np.array(ep_list), np.array(ey_list), np.array(et_list)

enc_k,   enc_ep,   enc_ey,   enc_et   = build_rms_series(enc_data)
plain_k, plain_ep, plain_ey, plain_et = build_rms_series(plain_data)

fig1, axes1 = plt.subplots(1, 3, figsize=(11, 3.5))
fig1.suptitle('ILC Convergence: Encrypted vs Plaintext', fontsize=12)
fig1.subplots_adjust(left=0.08, right=0.97, bottom=0.14, top=0.88, wspace=0.35)

titles  = ['(a) Pitch RMS Error', '(b) Yaw RMS Error', '(c) Total RMS Error']
ylabels = [r'Pitch RMS $e_p$ (deg)', r'Yaw RMS $e_y$ (deg)', r'Total RMS $e$ (deg)']
enc_series   = [enc_ep,   enc_ey,   enc_et  ]
plain_series = [plain_ep, plain_ey, plain_et]

for i, ax in enumerate(axes1):
    ax.plot(enc_k,   enc_series[i],   'o-', color=C_ENC,   label='Encrypted', markersize=4)
    ax.plot(plain_k, plain_series[i], 's-', color=C_PLAIN, label='Plaintext', markersize=4)
    ax.set_title(titles[i],  loc='left', pad=4)
    ax.set_xlabel('Iteration $k$')
    ax.set_ylabel(ylabels[i])
    ax.legend()
    ax.grid(True)

out1 = os.path.join(BASE_DIR, 'compare_convergence.pdf')
fig1.savefig(out1, format='pdf', bbox_inches='tight', dpi=300)
print(f"Saved: {out1}")

# ═══════════════════════════════════════════════════════════════════════════
# Figure 2 — Trajectory comparison (last iteration of each)
# ═══════════════════════════════════════════════════════════════════════════
de = enc_data[-1]
dp = plain_data[-1]
Ts = float(de['Ts'])

Ne = int(de['n_valid'])
Np = int(dp['n_valid'])
te = np.arange(Ne) * Ts
tp = np.arange(Np) * Ts

ke = int(de['k'])
kp = int(dp['k'])

fig2, axes2 = plt.subplots(2, 2, figsize=(9, 6))
fig2.suptitle(f'Trajectory Comparison — Enc k={ke}  vs  Plain k={kp}', fontsize=12)
fig2.subplots_adjust(left=0.09, right=0.97, bottom=0.10, top=0.91, hspace=0.45, wspace=0.32)

out_ylabels = [r'Pitch $\theta_p$ (deg)', r'Yaw $\theta_y$ (deg)']
err_ylabels = [r'Pitch Error $e_p$ (deg)', r'Yaw Error $e_y$ (deg)']
out_titles  = ['(a) Pitch Output', '(b) Yaw Output']
err_titles  = ['(c) Pitch Error',  '(d) Yaw Error' ]

for col in range(2):
    # Row 0: output trajectory
    ax = axes2[0, col]
    ax.plot(te, np.rad2deg(de['R'][:Ne, col]),
            color=C_REF,   ls='--', lw=1.2, label='Reference', zorder=3)
    ax.plot(te, np.rad2deg(de['Y'][:Ne, col]),
            color=C_ENC,   label=f'Encrypted (k={ke})', zorder=2)
    ax.plot(tp, np.rad2deg(dp['Y'][:Np, col]),
            color=C_PLAIN, label=f'Plaintext  (k={kp})', zorder=2, ls='-.')
    ax.set_title(out_titles[col], loc='left', pad=4)
    ax.set_ylabel(out_ylabels[col])
    ax.set_xlim([0, max(te[-1], tp[-1])])
    ax.legend(fontsize=9)
    ax.grid(True)

    # Row 1: tracking error
    ax = axes2[1, col]
    ax.axhline(0, color='k', lw=0.7, zorder=1)
    ax.plot(te, np.rad2deg(de['E'][:Ne, col]),
            color=C_ENC,   label=f'Encrypted (k={ke})', zorder=2)
    ax.plot(tp, np.rad2deg(dp['E'][:Np, col]),
            color=C_PLAIN, label=f'Plaintext  (k={kp})', zorder=2, ls='-.')
    ax.set_title(err_titles[col], loc='left', pad=4)
    ax.set_ylabel(err_ylabels[col])
    ax.set_xlabel('Time (s)')
    ax.set_xlim([0, max(te[-1], tp[-1])])
    ax.legend(fontsize=9)
    ax.grid(True)

out2 = os.path.join(BASE_DIR, 'compare_trajectories.pdf')
fig2.savefig(out2, format='pdf', bbox_inches='tight', dpi=300)
print(f"Saved: {out2}")

# ── Print summary table ───────────────────────────────────────────────────
print("\n── RMS Error Summary (last iteration) ─────────────────────────")
print(f"{'':20s}  {'Pitch':>8s}  {'Yaw':>8s}  {'Total':>8s}")
print(f"{'Encrypted   (k='+str(ke)+')':20s}  {enc_ep[-1]:8.3f}°  {enc_ey[-1]:8.3f}°  {enc_et[-1]:8.3f}°")
print(f"{'Plaintext   (k='+str(kp)+')':20s}  {plain_ep[-1]:8.3f}°  {plain_ey[-1]:8.3f}°  {plain_et[-1]:8.3f}°")
diff_p = enc_ep[-1] - plain_ep[-1]
diff_y = enc_ey[-1] - plain_ey[-1]
diff_t = enc_et[-1] - plain_et[-1]
print(f"{'Enc - Plain':20s}  {diff_p:+8.3f}°  {diff_y:+8.3f}°  {diff_t:+8.3f}°")
print("────────────────────────────────────────────────────────────────")

plt.show()
