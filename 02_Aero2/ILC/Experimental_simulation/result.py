"""result.py — Plot first vs last ILC iteration from data/ folder.

Layout (3 rows × 2 cols):
  Row 1: Output trajectories  [ref / k_first / k_last]
  Row 2: Tracking errors      [k_first / k_last]
  Row 3: Control inputs       [k_first / k_last]

Saves a vector-format PDF to data/result_k<first>_k<last>.pdf
"""

import os, glob
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# ── Load first and last data files ────────────────────────────────────────
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
files = sorted(
    glob.glob(os.path.join(DATA_DIR, 'data_*.npz')),
    key=lambda f: int(os.path.splitext(os.path.basename(f))[0].split('_')[1])
)
if len(files) < 1:
    raise FileNotFoundError(f"No data_*.npz files found in {DATA_DIR}")

d1 = np.load(files[0])
dl = np.load(files[-1])

k1 = int(d1['k'])
kl = int(dl['k'])
Ts = float(d1['Ts'])

N1 = int(d1['n_valid'])
Nl = int(dl['n_valid'])
t1 = np.arange(N1) * Ts
tl = np.arange(Nl) * Ts
t_max = max(t1[-1], tl[-1])

print(f"Loaded: k={k1} ({N1} steps)  vs  k={kl} ({Nl} steps),  Ts={Ts}s")

# ── Global style (paper-quality) ─────────────────────────────────────────
mpl.rcParams.update({
    'font.family':        'serif',
    'font.size':          11,
    'axes.labelsize':     11,
    'axes.titlesize':     11,
    'xtick.labelsize':    10,
    'ytick.labelsize':    10,
    'legend.fontsize':    10,
    'legend.framealpha':  0.85,
    'legend.edgecolor':   '0.7',
    'lines.linewidth':    1.4,
    'axes.linewidth':     0.8,
    'grid.linewidth':     0.5,
    'grid.linestyle':     ':',
    'grid.alpha':         0.6,
})

C_REF   = '#333333'   # dark grey  — reference
C_FIRST = '#D62728'   # red        — first iteration
C_LAST  = '#1F77B4'   # blue       — last iteration
LS_REF  = '--'

# ── Figure ────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(7.5, 8.0))
fig.subplots_adjust(
    left=0.10, right=0.97,
    bottom=0.07, top=0.96,
    hspace=0.45, wspace=0.35,
)

# ─── Row 1: Output trajectories ───────────────────────────────────────────
out_ylabels = [r'Pitch $\theta_p$ (deg)', r'Yaw $\theta_y$ (deg)']
out_titles  = ['(a) Pitch Output', '(b) Yaw Output']

for col in range(2):
    ax = axes[0, col]
    ax.plot(t1, np.rad2deg(d1['R'][:N1, col]),
            color=C_REF, ls=LS_REF, lw=1.2, label='Reference', zorder=3)
    ax.plot(t1, np.rad2deg(d1['Y'][:N1, col]),
            color=C_FIRST, label=f'$k = {k1}$', zorder=2)
    ax.plot(tl, np.rad2deg(dl['Y'][:Nl, col]),
            color=C_LAST,  label=f'$k = {kl}$', zorder=2)
    ax.set_title(out_titles[col], loc='left', pad=4)
    ax.set_ylabel(out_ylabels[col])
    ax.set_xlim([0, t_max])
    ax.legend(loc='upper right', ncol=1)
    ax.grid(True)

# ─── Row 2: Tracking errors ───────────────────────────────────────────────
err_ylabels = [r'Pitch Error $e_p$ (deg)', r'Yaw Error $e_y$ (deg)']
err_titles  = ['(c) Pitch Error', '(d) Yaw Error']

for col in range(2):
    ax = axes[1, col]
    ax.axhline(0, color='k', lw=0.7, ls='-', zorder=1)
    ax.plot(t1, np.rad2deg(d1['E'][:N1, col]),
            color=C_FIRST, label=f'$k = {k1}$', zorder=2)
    ax.plot(tl, np.rad2deg(dl['E'][:Nl, col]),
            color=C_LAST,  label=f'$k = {kl}$', zorder=2)
    ax.set_title(err_titles[col], loc='left', pad=4)
    ax.set_ylabel(err_ylabels[col])
    ax.set_xlim([0, t_max])
    ax.legend(loc='upper right')
    ax.grid(True)

# ─── Row 3: Control inputs ────────────────────────────────────────────────
inp_ylabels = [r'$V_\mathrm{main}$ (V)', r'$V_\mathrm{tail}$ (V)']
inp_titles  = ['(e) Main Motor Voltage', '(f) Tail Motor Voltage']

for col in range(2):
    ax = axes[2, col]
    ax.plot(t1, d1['U'][:N1, col],
            color=C_FIRST, label=f'$k = {k1}$', zorder=2)
    ax.plot(tl, dl['U'][:Nl, col],
            color=C_LAST,  label=f'$k = {kl}$', zorder=2)
    ax.set_title(inp_titles[col], loc='left', pad=4)
    ax.set_ylabel(inp_ylabels[col])
    ax.set_xlabel('Time (s)')
    ax.set_xlim([0, t_max])
    ax.legend(loc='upper right')
    ax.grid(True)

# ── Save as PDF (vector) ──────────────────────────────────────────────────
out_path = os.path.join(DATA_DIR, f'result_k{k1}_k{kl}.pdf')
fig.savefig(out_path, format='pdf', bbox_inches='tight', dpi=300)
print(f"Saved: {out_path}")

plt.show()
