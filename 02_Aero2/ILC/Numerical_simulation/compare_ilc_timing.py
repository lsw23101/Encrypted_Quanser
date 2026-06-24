"""
compare_ilc_timing.py
  Reads ilc_time_ms.csv from simulation_packed and simulation_nopacking results,
  then prints a comparison table: min / max / mean / std per trial (ms).
"""

import os
import numpy as np

BASE          = os.path.dirname(os.path.abspath(__file__))
PACKED_DIR    = os.path.join(BASE, "result_timing_packed")
NOPACKING_DIR = os.path.join(BASE, "result_timing_nopacking")
CSV_NAME      = "ilc_time_ms.csv"


def load_times(directory: str):
    path = os.path.join(directory, CSV_NAME)
    if not os.path.exists(path):
        return None
    data = np.loadtxt(path, delimiter=",")
    return data.flatten()


def stats(arr: np.ndarray) -> dict:
    return {
        "n":    len(arr),
        "min":  arr.min(),
        "max":  arr.max(),
        "mean": arr.mean(),
        "std":  arr.std(ddof=1) if len(arr) > 1 else 0.0,
    }


def main():
    packed    = load_times(PACKED_DIR)
    nopacking = load_times(NOPACKING_DIR)

    missing = []
    if packed    is None: missing.append(PACKED_DIR)
    if nopacking is None: missing.append(NOPACKING_DIR)
    if missing:
        print("Missing ilc_time_ms.csv in:", ", ".join(missing))
        print("Run the Go simulations first:")
        print("  go run simulation_packed.go")
        print("  go run simulation_nopacking.go")
        return

    sp = stats(packed)
    sn = stats(nopacking)

    # Header
    col_w = 18
    label_w = 20
    divider = "-" * (label_w + col_w * 2 + 4)

    print("\n" + divider)
    print(f"{'ILC Update Time (ms)':<{label_w}}  {'Packed (SVD)':<{col_w}}  {'NoPacking':<{col_w}}")
    print(divider)

    rows = [
        ("Trials (N)",   f"{sp['n']}",               f"{sn['n']}"),
        ("Min",          f"{sp['min']:.3f}",          f"{sn['min']:.3f}"),
        ("Max",          f"{sp['max']:.3f}",          f"{sn['max']:.3f}"),
        ("Mean",         f"{sp['mean']:.3f}",         f"{sn['mean']:.3f}"),
        ("Std dev",      f"{sp['std']:.3f}",          f"{sn['std']:.3f}"),
    ]

    for label, pval, nval in rows:
        print(f"  {label:<{label_w - 2}}  {pval:<{col_w}}  {nval:<{col_w}}")

    print(divider)

    # Speedup / slowdown
    if sn["mean"] > 0:
        ratio = sp["mean"] / sn["mean"]
        if ratio >= 1.0:
            print(f"  Packed is {ratio:.2f}x  faster than NoPacking (mean)")
        else:
            print(f"  NoPacking is {1/ratio:.2f}x faster than Packed (mean)")

    print(divider + "\n")

    # Per-trial table
    print("Per-trial breakdown:")
    header = f"  {'Trial':>5}  {'Packed (ms)':>12}  {'NoPacking (ms)':>14}"
    print(header)
    print("  " + "-" * (len(header) - 2))
    n_trials = min(len(packed), len(nopacking))
    for i in range(n_trials):
        print(f"  {i+1:>5}  {packed[i]:>12.1f}  {nopacking[i]:>14.1f}")
    print()


if __name__ == "__main__":
    main()
