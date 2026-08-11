"""
compare_pack.py — Packed vs. No-Packing ILC benchmark report.

Reads timing_results.csv (written by EncILC_packed.go and EncILC_nopack.go while
they run on the real Aero2 hardware) and prints a side-by-side comparison of:

  - Enc   : offline encryption of the ILC gain K_r          (ms, one-off)
  - Eval  : encrypted ILC update  K_r x E  per trial        (ms, per trial)
  - Dec   : decryption of the updated ILC state Theta       (ms, per trial)
  - Size  : serialized ciphertext storage for K_r and Theta (bytes)

CSV columns:
  method,trial,nR,pN,mN,enc_kr_ms,ctKr_bytes,eval_ms,dec_ms,theta_bytes

Usage:
  1) go run EncILC_packed.go   +  python ../Plant_TCP.py   (>=1 trial)
  2) go run EncILC_nopack.go   +  python ../Plant_TCP.py   (>=1 trial)
  3) python compare_pack.py
"""

import os
import csv

BASE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(BASE, "timing_results.csv")

METHODS = ["packed", "nopack"]


def load_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def stats(vals):
    n = len(vals)
    if n == 0:
        return {"n": 0, "min": 0.0, "max": 0.0, "mean": 0.0, "std": 0.0}
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / (n - 1) if n > 1 else 0.0
    return {"n": n, "min": min(vals), "max": max(vals),
            "mean": mean, "std": var ** 0.5}


def human_bytes(b):
    b = float(b)
    for unit in ("B", "KB", "MB", "GB"):
        if b < 1024.0:
            return f"{b:.1f} {unit}"
        b /= 1024.0
    return f"{b:.1f} TB"


def summarize(rows):
    """Per-method summary: constant offline metrics + per-trial eval/dec stats."""
    out = {}
    for m in METHODS:
        mrows = [r for r in rows if r["method"] == m]
        if not mrows:
            out[m] = None
            continue
        eval_ms = [float(r["eval_ms"]) for r in mrows]
        dec_ms = [float(r["dec_ms"]) for r in mrows]
        first = mrows[0]
        out[m] = {
            "nR": int(first["nR"]),
            "pN": int(first["pN"]),
            "mN": int(first["mN"]),
            "enc_kr_ms": float(first["enc_kr_ms"]),
            "ctKr_bytes": int(first["ctKr_bytes"]),
            "theta_bytes": int(first["theta_bytes"]),
            "eval": stats(eval_ms),
            "dec": stats(dec_ms),
        }
    return out


def _row(label, pv, nv, ratio=""):
    return f"  {label:<22}{pv:>18}{nv:>18}{ratio:>16}"


def ratio_str(packed, nopack, unit="x", faster="packed"):
    """nopack/packed ratio, phrased as how much heavier nopack is."""
    if packed <= 0:
        return ""
    r = nopack / packed
    return f"nopack {r:.2f}{unit}"


def main():
    rows = load_rows(CSV_PATH)
    if not rows:
        print(f"No data at {CSV_PATH}")
        print("Run both Go servers with the hardware client first:")
        print("  go run EncILC_packed.go   +  python ../Plant_TCP.py")
        print("  go run EncILC_nopack.go   +  python ../Plant_TCP.py")
        return

    s = summarize(rows)
    p, n = s["packed"], s["nopack"]

    missing = [m for m in METHODS if s[m] is None]
    if missing:
        print(f"[warn] No rows yet for: {', '.join(missing)}")
        print("       Run that variant with the hardware client, then re-run.\n")

    width = 22 + 18 * 2 + 16
    print("\n" + "=" * width)
    print(f"  {'Packed vs. No-Packing ILC (Aero2 hardware)':<{width-2}}")
    print("=" * width)
    print(_row("Metric", "Packed", "NoPacking", "nopack/packed"))
    print("-" * width)

    if p:
        print(_row("nR (rank G_N)", str(p["nR"]), n["nR"] if n else "-", ""))
        print(_row("pN (p*N steps)", str(p["pN"]), n["pN"] if n else "-", ""))
        print("-" * width)

    def fmt_ms(x):
        return f"{x:.2f} ms"

    if p and n:
        # Enc (offline, one-off)
        print(_row("Enc K_r (offline)",
                   fmt_ms(p["enc_kr_ms"]), fmt_ms(n["enc_kr_ms"]),
                   ratio_str(p["enc_kr_ms"], n["enc_kr_ms"])))
        # Eval (per trial)
        print(_row("Eval  mean",
                   fmt_ms(p["eval"]["mean"]), fmt_ms(n["eval"]["mean"]),
                   ratio_str(p["eval"]["mean"], n["eval"]["mean"])))
        print(_row("Eval  min/max",
                   f'{p["eval"]["min"]:.1f}/{p["eval"]["max"]:.1f}',
                   f'{n["eval"]["min"]:.1f}/{n["eval"]["max"]:.1f}', ""))
        # Dec (per trial)
        print(_row("Dec   mean",
                   fmt_ms(p["dec"]["mean"]), fmt_ms(n["dec"]["mean"]),
                   ratio_str(p["dec"]["mean"], n["dec"]["mean"])))
        print("-" * width)
        # Storage
        print(_row("ctKr size",
                   human_bytes(p["ctKr_bytes"]), human_bytes(n["ctKr_bytes"]),
                   ratio_str(p["ctKr_bytes"], n["ctKr_bytes"])))
        print(_row("Theta size",
                   human_bytes(p["theta_bytes"]), human_bytes(n["theta_bytes"]),
                   ratio_str(p["theta_bytes"], n["theta_bytes"])))
        print(_row("trials (N)", str(p["eval"]["n"]), str(n["eval"]["n"]), ""))
    else:
        for m in METHODS:
            if s[m]:
                d = s[m]
                print(f"\n  [{m}]  enc_kr={d['enc_kr_ms']:.1f} ms  "
                      f"eval_mean={d['eval']['mean']:.1f} ms  "
                      f"dec_mean={d['dec']['mean']:.1f} ms  "
                      f"ctKr={human_bytes(d['ctKr_bytes'])}  "
                      f"theta={human_bytes(d['theta_bytes'])}  "
                      f"(n={d['eval']['n']})")

    print("=" * width + "\n")

    # Per-trial eval breakdown
    print("Per-trial Eval time (ms):")
    print(f"  {'trial':>5}  {'packed':>10}  {'nopack':>10}")
    print("  " + "-" * 30)
    by_trial = {}
    for r in rows:
        by_trial.setdefault(int(r["trial"]), {})[r["method"]] = float(r["eval_ms"])
    for t in sorted(by_trial):
        pv = by_trial[t].get("packed")
        nv = by_trial[t].get("nopack")
        ps = f"{pv:10.1f}" if pv is not None else f"{'-':>10}"
        ns = f"{nv:10.1f}" if nv is not None else f"{'-':>10}"
        print(f"  {t:>5}  {ps}  {ns}")
    print()


if __name__ == "__main__":
    main()
