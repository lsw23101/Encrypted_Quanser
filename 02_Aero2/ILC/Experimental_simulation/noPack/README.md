# noPack — Packing vs. No-Packing benchmark on real Aero2 hardware

Measures the cost of **RGSW packing** in the encrypted ILC update `K_r × E`,
running the *actual* encrypted controller on the Quanser Aero2 (not a numerical
sim). This mirrors `Numerical_simulation/simulation_packed.go` vs.
`simulation_nopacking.go`, but on hardware. SVD / kernel variants are out of
scope here — only packing vs. no-packing is compared.

## Files

| File | What it is |
|------|-----------|
| `EncILC_packed.go` | Copy of `../EncILC.go` (packed ILC update) **+ timing/size logging**. Crypto unchanged. |
| `EncILC_nopack.go` | Same controller, but the ILC update uses **no packing** (`RGSW.Enc` / `RGSW.Mult` / `RLWE.Dec`). |
| `compare_pack.py`  | Reads `timing_results.csv` and prints a packed-vs-nopack table. |
| `timing_results.csv` | Auto-created; one row appended per ILC trial by either Go server. |

The originals under `../` are untouched — these are independent copies.

### What differs (packed → nopack)

Only the ILC learning step; the controller (`F,G,H,R,P,Nbar`) stays packed in both.

| Step | Packed | NoPacking |
|------|--------|-----------|
| Encrypt `K_r` | `RGSW.EncPack` → `pN` column-packed RGSW cts | `RGSW.Enc` → `nR × pN` RGSW matrix |
| ILC state `Theta` | 1 packed RLWE ct (`tauSVD` slots) | `nR` scalar RLWE cts |
| Update `K_r × E` | `RGSW.MultPack` (`pN` external products) | `RGSW.Mult` (`nR·pN` external products) |
| Decrypt `Theta` | `RLWE.DecUnpack` (1 decrypt) | `RLWE.Dec` (`nR` decrypts) |

## Run

Each Go server uses TCP `:5555` and the same protocol, so the **existing**
`../Plant_TCP.py` client drives both. Run one server at a time.

```bash
# Terminal 1 (packed)                # Terminal 2
go run EncILC_packed.go              python ../Plant_TCP.py     # >= 1 trial, Ctrl+C to end trial

# Terminal 1 (nopack)                # Terminal 2
go run EncILC_nopack.go              python ../Plant_TCP.py     # >= 1 trial

# Report
python compare_pack.py
```

Each server keeps its own encrypted ILC state under `noPack/ilc_state/`
(`sk.bin`, `theta_ilc.bin`). Delete that folder to reset between studies.
> Note: the packed and nopack servers store `theta_ilc.bin` in **different
> formats**. If you switch variants, the mismatched state is detected and the
> run simply restarts from zero feedforward — safe, but delete `ilc_state/`
> if you want a clean comparison from iteration 0.

## Metrics logged (`timing_results.csv`)

`method,trial,nR,pN,mN,enc_kr_ms,ctKr_bytes,eval_ms,dec_ms,theta_bytes`

- `enc_kr_ms`  — offline `K_r` encryption time (one-off per server start)
- `eval_ms`    — encrypted ILC update `K_r × E` (per trial) ← main comparison
- `dec_ms`     — `Theta` decryption (per trial)
- `ctKr_bytes` — serialized size of the encrypted gain `K_r`
- `theta_bytes`— serialized size of the encrypted ILC state
