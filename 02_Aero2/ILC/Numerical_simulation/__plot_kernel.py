"""
plot_kernel.py — Kernel ILC (simulation_kernel.go) result plots
4-panel figure formatted for 2-column IEEE paper (PDF output).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os, glob

BASE = os.path.dirname(os.path.abspath(__file__))

# ── Publication style ──────────────────────────────────────────────────��──────
plt.rcParams.update({
    'font.family':        'serif',
    'font.size':          10,
    'axes.labelsize':     10,
    'axes.titlesize':     11,
    'xtick.labelsize':    9,
    'ytick.labelsize':    9,
    'legend.fontsize':    9,
    'legend.framealpha':  0.7,
    'lines.linewidth':    1.0,
    'axes.linewidth':     0.6,
    'grid.linewidth':     0.4,
    'grid.alpha':         0.4,
    'pdf.fonttype':       42,   # embed TrueType fonts
    'ps.fonttype':        42,
})

FOLDER = os.path.join(BASE, 'result_kernel')
TS_CSV = os.path.join(BASE, 'controller_data', 'Ts.csv')

Ts = float(np.loadtxt(TS_CSV)) if os.path.exists(TS_CSV) else 0.05

def load(name):
    p = os.path.join(FOLDER, name)
    if not os.path.exists(p):
        return None
    d = np.loadtxt(p, delimiter=',')
    return d.reshape(-1, 1) if d.ndim == 1 else d

ref   = load('ref.csv')
base  = load('y_baseline.csv')
V_raw = load('V_history.csv')

if ref is None:
    raise FileNotFoundError(f"ref.csv not found in {FOLDER}/")

ref_flat  = ref.flatten()
base_flat = base.flatten() if base is not None else None
t = np.arange(len(ref_flat)) * Ts

y_files = sorted(glob.glob(os.path.join(FOLDER, 'trial_*_y.csv')))
available = []
for f in y_files:
    bn = os.path.basename(f)
    try:
        available.append(int(bn.split('_')[1]))
    except ValueError:
        pass
available = sorted(set(available))

if not available:
    raise FileNotFoundError(f"No trial_*_y.csv found in {FOLDER}/")

first_tr = available[0]
last_tr  = available[-1]

def load_trial(tr, suffix):
    for fmt in [f'trial_{tr:04d}_{suffix}.csv', f'trial_{tr:02d}_{suffix}.csv']:
        d = load(fmt)
        if d is not None:
            return d
    return None

y_first = load_trial(first_tr, 'y')
y_last  = load_trial(last_tr,  'y')
u_first = load_trial(first_tr, 'u')
u_last  = load_trial(last_tr,  'u')

err_data = load('error_rms.csv')
n_err = len(err_data) if err_data is not None else '?'

print(f"Ts = {Ts} s")
print(f"Comparing : Trial {first_tr} vs Trial {last_tr}  ({n_err} trials total)")

v_first = v_last = t_v = None
if V_raw is not None:
    V       = V_raw.reshape(1, -1) if V_raw.ndim == 1 else V_raw
    n_v     = V.shape[0]
    t_v     = np.arange(V.shape[1] // 2) * Ts
    v_first = V[0]
    v_last  = V[n_v - 1]

# ── Figure layout: 7" wide (2-col IEEE full-width), 4 rows ───────────────────
fig = plt.figure(figsize=(7, 8))
gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.55, wspace=0.32,
                       left=0.09, right=0.97, top=0.97, bottom=0.07)

ax_y  = fig.add_subplot(gs[0, :])
ax_e  = fig.add_subplot(gs[1, :])
ax_v1 = fig.add_subplot(gs[2, 0])
ax_v2 = fig.add_subplot(gs[2, 1])
ax_u1 = fig.add_subplot(gs[3, 0])
ax_u2 = fig.add_subplot(gs[3, 1])

C1 = '#7799CC'
C2 = '#003399'
C3 = '#CC8877'
C4 = '#990000'

# ── Output tracking ───────────────────────────────────────────────────────────
ax_y.plot(t, ref_flat, 'k--', lw=1.8, zorder=0, label='Reference')
if y_first is not None:
    ax_y.plot(t, y_first.flatten(), color=C1, lw=1.0, label=f'Trial {first_tr}')
if y_last is not None:
    ax_y.plot(t, y_last.flatten(),  color=C2, lw=1.2, label=f'Trial {last_tr}')
ax_y.set_ylabel('$y$')
ax_y.set_title('Output Tracking')
ax_y.legend(ncol=3, loc='upper right')
ax_y.grid(True)

# ── Tracking error ────────────────────────────────────────────────────────────
ax_e.axhline(0, color='k', lw=0.5, ls='--')
if y_first is not None:
    e1 = ref_flat - y_first.flatten()
    ax_e.plot(t, e1, color=C1, lw=1.0, label=f'Trial {first_tr}')
if y_last is not None:
    eN = ref_flat - y_last.flatten()
    ax_e.plot(t, eN, color=C2, lw=1.2, label=f'Trial {last_tr}')
ax_e.set_ylabel('$r - y$')
ax_e.set_xlabel('Time (s)')
ax_e.set_title('Tracking Error')
ax_e.legend(ncol=2, loc='upper right')
ax_e.grid(True)

# ── V_ILC feedforward ─────────────────────────────────────────────────────────
if v_first is not None and t_v is not None:
    ax_v1.plot(t_v, v_first[0::2], color=C1, lw=1.0, label=f'Trial {first_tr}')
    ax_v1.plot(t_v, v_last[0::2],  color=C2, lw=1.2, label=f'Trial {last_tr}')
    ax_v2.plot(t_v, v_first[1::2], color=C3, lw=1.0, label=f'Trial {first_tr}')
    ax_v2.plot(t_v, v_last[1::2],  color=C4, lw=1.2, label=f'Trial {last_tr}')
ax_v1.set_ylabel('$v$')
ax_v1.set_xlabel('Time (s)')
ax_v1.set_title('ILC Feedforward (1)')
ax_v1.legend(); ax_v1.grid(True)
ax_v2.set_ylabel('$v$')
ax_v2.set_xlabel('Time (s)')
ax_v2.set_title('ILC Feedforward (2)')
ax_v2.legend(); ax_v2.grid(True)

# ── Control input ─────────────────────────────────────────────────────────────
if u_first is not None:
    ax_u1.plot(t, u_first[:, 0], color=C1, lw=1.0, label=f'Trial {first_tr}')
    ax_u2.plot(t, u_first[:, 1], color=C3, lw=1.0, label=f'Trial {first_tr}')
if u_last is not None:
    ax_u1.plot(t, u_last[:, 0],  color=C2, lw=1.2, label=f'Trial {last_tr}')
    ax_u2.plot(t, u_last[:, 1],  color=C4, lw=1.2, label=f'Trial {last_tr}')
ax_u1.set_ylabel('$u$')
ax_u1.set_xlabel('Time (s)')
ax_u1.set_title('Control Input (1)')
ax_u1.legend(); ax_u1.grid(True)
ax_u2.set_ylabel('$u$')
ax_u2.set_xlabel('Time (s)')
ax_u2.set_title('Control Input (2)')
ax_u2.legend(); ax_u2.grid(True)

out = os.path.join(FOLDER, 'results_kernel.pdf')
plt.savefig(out, format='pdf', bbox_inches='tight')
print(f'Saved: {out}')
plt.show()
