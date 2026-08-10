"""result.py — Plot ILC learning progress from data/ folder.

Layout (1 row × 2 cols):
  Pitch / Yaw output trajectories  [ref / all iterations, faded / k_first / k_last]

Intermediate iterations are drawn low-alpha with a color gradient (red→blue,
first→last) so the learning trend is visible without cluttering the plot;
first/last are drawn opaque on top and are the only ones in the legend.

Saves a vector-format PDF to data/result_k<first>_k<last>.pdf
"""

import os, glob
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable

# ── Load all data files ───────────────────────────────────────────────────
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
files = sorted(
    glob.glob(os.path.join(DATA_DIR, 'data_*.npz')),
    key=lambda f: int(os.path.splitext(os.path.basename(f))[0].split('_')[1])
)
if len(files) < 1:
    raise FileNotFoundError(f"No data_*.npz files found in {DATA_DIR}")

trials = [np.load(f) for f in files]
d1, dl = trials[0], trials[-1]

k1 = int(d1['k'])
kl = int(dl['k'])
Ts = float(d1['Ts'])

N1 = int(d1['n_valid'])
Nl = int(dl['n_valid'])
t1 = np.arange(N1) * Ts
tl = np.arange(Nl) * Ts
t_max = max(int(d['n_valid']) for d in trials) * Ts

print(f"Loaded {len(trials)} trials: k={k1} ({N1} steps)  ...  k={kl} ({Nl} steps),  Ts={Ts}s")

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

# Intermediate iterations: faded, color graded red (early) -> blue (late) so
# the learning trend reads at a glance; only k_first/k_last get a legend
# entry and full opacity.
CMAP_MID  = LinearSegmentedColormap.from_list('ilc_progress', [C_FIRST, C_LAST])
K_ALL     = [int(d['k']) for d in trials]
norm      = Normalize(vmin=min(K_ALL), vmax=max(K_ALL))
MID_ALPHA = 0.25
MID_LW    = 0.8
mid_trials = trials[1:-1]  # excludes first and last (drawn separately, opaque)

def plot_mid(ax, col, field):
    for d in mid_trials:
        Nm = int(d['n_valid'])
        tm = np.arange(Nm) * Ts
        ax.plot(tm, np.rad2deg(d[field][:Nm, col]) if field != 'U' else d[field][:Nm, col],
                color=CMAP_MID(norm(int(d['k']))), alpha=MID_ALPHA, lw=MID_LW, zorder=2)

# ── Figure ────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(8.3, 3.2))
fig.subplots_adjust(
    left=0.10, right=0.90,
    bottom=0.16, top=0.90,
    wspace=0.35,
)

cbar_ax = fig.add_axes([0.93, 0.16, 0.02, 0.74])
sm = ScalarMappable(cmap=CMAP_MID, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, cax=cbar_ax)
cbar.set_label('ILC iteration $k$ (intermediate trials)')

# ─── Output trajectories (pitch / yaw) ────────────────────────────────────
out_ylabels = [r'Pitch $\theta_p$ (deg)', r'Yaw $\theta_y$ (deg)']
out_titles  = ['(a) Pitch Output', '(b) Yaw Output']

for col in range(2):
    ax = axes[col]
    plot_mid(ax, col, 'Y')
    ax.plot(t1, np.rad2deg(d1['R'][:N1, col]),
            color=C_REF, ls=LS_REF, lw=1.2, label='Reference', zorder=4)
    ax.plot(t1, np.rad2deg(d1['Y'][:N1, col]),
            color=C_FIRST, label=f'$k = {k1}$', zorder=3)
    ax.plot(tl, np.rad2deg(dl['Y'][:Nl, col]),
            color=C_LAST,  label=f'$k = {kl}$', zorder=3)
    ax.set_title(out_titles[col], loc='left', pad=4)
    ax.set_ylabel(out_ylabels[col])
    ax.set_xlabel('Time (s)')
    ax.set_xlim([0, t_max])
    ax.legend(loc='upper right', ncol=1)
    ax.grid(True)

# ── Save as PDF (vector) ──────────────────────────────────────────────────
out_path = os.path.join(DATA_DIR, f'result_k{k1}_k{kl}.pdf')
fig.savefig(out_path, format='pdf', bbox_inches='tight', dpi=300)
print(f"Saved: {out_path}")

plt.show()
