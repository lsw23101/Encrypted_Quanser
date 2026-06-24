//go:build ignore

// 4th-order Numerical System: Fully Encrypted Observer-Based ILC Simulation
//   (n=4 states, m=2 inputs, p=1 output — matrices loaded from controller_data/)
//
// All control computations are in ciphertext domain:
//   u(k)   = H_*z(k) + N_bar*r(k)           (encrypted H_, N_bar, r)
//   z(k+1) = F_*z(k) + G_*y(k) + R_*u(k) + P_*r(k)   (all encrypted)
//
// Scaling:
//   Output  ciphertext at 1/(r·s²·L): rk encoded at state scale round(rk/(r·s))
//   State update ciphertext at 1/(r·s·L):  rk encoded at meas  scale round(rk/r)
//
// Run: go run simulation.go

package main

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"time"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ── Matrix helpers ─────────────────────────────────────────────────────────

func eye(n int) [][]float64 {
	M := make([][]float64, n)
	for i := range M {
		M[i] = make([]float64, n)
		M[i][i] = 1
	}
	return M
}

func matAdd(A, B [][]float64) [][]float64 {
	n, m := len(A), len(A[0])
	C := make([][]float64, n)
	for i := range C {
		C[i] = make([]float64, m)
		for j := 0; j < m; j++ {
			C[i][j] = A[i][j] + B[i][j]
		}
	}
	return C
}

func matMul(A, B [][]float64) [][]float64 {
	ra, k, cb := len(A), len(B), len(B[0])
	C := make([][]float64, ra)
	for i := range C {
		C[i] = make([]float64, cb)
		for j := 0; j < cb; j++ {
			for l := 0; l < k; l++ {
				C[i][j] += A[i][l] * B[l][j]
			}
		}
	}
	return C
}

func scalMat(s float64, A [][]float64) [][]float64 {
	n, m := len(A), len(A[0])
	B := make([][]float64, n)
	for i := range B {
		B[i] = make([]float64, m)
		for j := 0; j < m; j++ {
			B[i][j] = s * A[i][j]
		}
	}
	return B
}

func matTranspose(A [][]float64) [][]float64 {
	n, m := len(A), len(A[0])
	B := make([][]float64, m)
	for i := range B {
		B[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			B[i][j] = A[j][i]
		}
	}
	return B
}

// solveAXB solves A * X = B for X using Gauss-Jordan elimination
func solveAXB(A [][]float64, B [][]float64) [][]float64 {
	n := len(A)
	m := len(B[0])
	X := make([][]float64, n)
	for i := range X {
		X[i] = make([]float64, m)
	}

	aug := make([][]float64, n)
	for i := 0; i < n; i++ {
		aug[i] = make([]float64, n+m)
		for j := 0; j < n; j++ {
			aug[i][j] = A[i][j]
		}
		for j := 0; j < m; j++ {
			aug[i][n+j] = B[i][j]
		}
	}

	for i := 0; i < n; i++ {
		pivot := i
		maxVal := math.Abs(aug[i][i])
		for k := i + 1; k < n; k++ {
			if val := math.Abs(aug[k][i]); val > maxVal {
				maxVal = val
				pivot = k
			}
		}

		aug[i], aug[pivot] = aug[pivot], aug[i]

		pivotVal := aug[i][i]
		if pivotVal == 0 {
			fmt.Println("[Warning] Singular matrix in solveAXB")
			return X
		}
		for j := i; j < n+m; j++ {
			aug[i][j] /= pivotVal
		}

		for k := 0; k < n; k++ {
			if k != i {
				factor := aug[k][i]
				for j := i; j < n+m; j++ {
					aug[k][j] -= factor * aug[i][j]
				}
			}
		}
	}

	for i := 0; i < n; i++ {
		for j := 0; j < m; j++ {
			X[i][j] = aug[i][n+j]
		}
	}
	return X
}

// spectralRadiusSym computes the dominant eigenvalue of a symmetric matrix via power iteration
func spectralRadiusSym(A [][]float64) float64 {
	n := len(A)
	v := make([]float64, n)
	for i := range v {
		v[i] = 1.0
	}

	lambda := 0.0
	for iter := 0; iter < 100; iter++ {
		vNew := make([]float64, n)
		norm := 0.0
		for i := 0; i < n; i++ {
			sum := 0.0
			for j := 0; j < n; j++ {
				sum += A[i][j] * v[j]
			}
			vNew[i] = sum
			norm += sum * sum
		}
		norm = math.Sqrt(norm)
		if norm == 0 {
			break
		}
		for i := 0; i < n; i++ {
			v[i] = vNew[i] / norm
		}
		num := 0.0
		for i := 0; i < n; i++ {
			sum := 0.0
			for j := 0; j < n; j++ {
				sum += A[i][j] * v[j]
			}
			num += v[i] * sum
		}
		lambda = num
	}
	return math.Abs(lambda)
}

// zohDisc: ZOH discretisation via augmented matrix exponential (Taylor, 20 terms).
func zohDisc(Ac, Bc [][]float64, Ts float64) ([][]float64, [][]float64) {
	n, mi := len(Ac), len(Bc[0])
	N := n + mi
	M := make([][]float64, N)
	for i := range M {
		M[i] = make([]float64, N)
	}
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			M[i][j] = Ac[i][j] * Ts
		}
		for j := 0; j < mi; j++ {
			M[i][n+j] = Bc[i][j] * Ts
		}
	}
	result, power, fact := eye(N), eye(N), 1.0
	for k := 1; k <= 20; k++ {
		fact *= float64(k)
		power = matMul(power, M)
		result = matAdd(result, scalMat(1.0/fact, power))
	}
	Ad := make([][]float64, n)
	Bd := make([][]float64, n)
	for i := 0; i < n; i++ {
		Ad[i] = make([]float64, n)
		Bd[i] = make([]float64, mi)
		for j := 0; j < n; j++ {
			Ad[i][j] = result[i][j]
		}
		for j := 0; j < mi; j++ {
			Bd[i][j] = result[i][n+j]
		}
	}
	return Ad, Bd
}

// computeRef returns scalar reference for p=1 numerical system.
// Integer-multiple frequencies of f0=1/trajTime → r(0)=r(T)=0 (ILC periodic).
func computeRef(step int, Ts, trajTime float64) [1]float64 {
	t := float64(step) * Ts
	if t >= trajTime {
		return [1]float64{0}
	}
	f0 := 1.0 / trajTime
	// sin³(ωt) = (3/4)sin(ωt) - (1/4)sin(3ωt): zero value AND zero slope at t=0 and t=T
	ref1 := 1.0 * math.Pow(math.Sin(math.Pi*f0*t), 2) * (math.Sin(3*math.Pi*f0*t) - 0.5*math.Sin(2*math.Pi*f0*t))
	return [1]float64{ref1}
}

// computeDisturbance returns m=2 dimensional trial-invariant disturbance.
// Frequencies are integer multiples of f0=0.1 Hz → same period as reference.
func computeDisturbance(step int, Ts float64, m int) []float64 {
	t := float64(step) * Ts
	f0 := 0.1
	d := make([]float64, m)
	d[0] = 0.04*math.Sin(2*math.Pi*2*f0*t+0.5) + 0.02*math.Sin(2*math.Pi*4*f0*t)
	if m > 1 {
		d[1] = 0.03*math.Sin(2*math.Pi*3*f0*t+1.0) - 0.02*math.Cos(2*math.Pi*5*f0*t)
	}
	return d
}

func loadCSV(path string) ([][]float64, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	records, err := csv.NewReader(bufio.NewReader(f)).ReadAll()
	if err != nil {
		return nil, err
	}
	out := make([][]float64, len(records))
	for ri, row := range records {
		out[ri] = make([]float64, len(row))
		for ci, sv := range row {
			v, err2 := strconv.ParseFloat(sv, 64)
			if err2 != nil {
				return nil, fmt.Errorf("parse [%d,%d]: %w", ri, ci, err2)
			}
			out[ri][ci] = v
		}
	}
	return out, nil
}

func cloneCtSlice(src []*rlwe.Ciphertext) []*rlwe.Ciphertext {
	out := make([]*rlwe.Ciphertext, len(src))
	for i := range src {
		if src[i] != nil {
			out[i] = src[i].CopyNew()
		}
	}
	return out
}

func main() {
	_, thisFile, _, _ := runtime.Caller(0)
	baseDir := filepath.Dir(thisFile)

	// ── Encryption parameters ──────────────────────────────────────────────
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 9, LogQ: []int{56}, LogP: []int{51}, NTTFlag: true,
	})
	fmt.Println("N =", params.N(), "  logQ =", params.QBigInt().BitLen())

	// ── Load plant & controller from controller_data/ (generated by conversion.m) ──
	dataDir := filepath.Join(baseDir, "controller_data")
	mustLoad := func(name string) [][]float64 {
		mat, err := loadCSV(filepath.Join(dataDir, name))
		if err != nil {
			fmt.Printf("[ERROR] Cannot load %s/%s: %v\n", dataDir, name, err)
			fmt.Println("       Run conversion.m in MATLAB first, then retry.")
			os.Exit(1)
		}
		return mat
	}

	Ad := mustLoad("Ad.csv")
	Bd := mustLoad("Bd.csv")
	C := mustLoad("C.csv")
	F_ := mustLoad("F_.csv")
	G_ := mustLoad("G_.csv")
	R_ := mustLoad("R_.csv")
	H_ := mustLoad("H_.csv")
	P_ := mustLoad("P_.csv")
	Nbar := mustLoad("Nbar.csv")

	// F_ is nilpotent integer matrix; CSV introduces float noise → round to integers
	for i := range F_ {
		for j := range F_[i] {
			F_[i][j] = math.Round(F_[i][j])
		}
	}

	TsRow := mustLoad("Ts.csv")
	Ts := TsRow[0][0]
	n := len(Ad)    // state dim
	m := len(Bd[0]) // input dim
	p := len(C)     // output dim

	fmt.Printf("Loaded from %s/  →  n=%d  m=%d  p=%d  Ts=%.4f\n", dataDir, n, m, p, Ts)

	trajTime := 10.0
	nSteps := int(trajTime / Ts)

	// ── Quantisation ──────────────────────────────────────────────────────
	// s=0.1 keeps RGSW key entries (H_/s, etc.) below ~200 for this system.
	// The LQR gain H_ has entries up to ~19; at s=0.001 the RGSW key would
	// be ~19000, causing noise-driven divergence within ~3 observer steps.
	s := 1.0 / 1000.0
	L := 1.0 / 100.0
	r := 1.0 / 1000.0
	fmt.Printf("Scaling: 1/s=%v  1/r=%v  1/L=%v\n", 1/s, 1/r, 1/L)

	// ── RLWE/RGSW setup ────────────────────────────────────────────────────
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	ringQ := params.RingQ()

	// Packing widths are separated by vector space.
	// Online encrypted control only needs to pack z/y/r/u dimensions (4/2/2), so tauCtrl=4.
	// The lifted ILC vectors have mN=pN=400 elements, so tauILC=512.
	// Using tauILC=512 inside the online control loop makes every small-vector unpack
	// pay for the 512-slot rotation structure and looks like a hang after "Saving ciphertexts to".
	controlDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tauCtrl := int(math.Pow(2, math.Ceil(math.Log2(controlDim))))
	ilcDim := math.Max(float64(m*nSteps), float64(p*nSteps))
	tauILC := int(math.Pow(2, math.Ceil(math.Log2(ilcDim))))
	fmt.Printf("Packing: tauCtrl=%d  tauILC=%d  (same RLWE/RGSW params; packing width matched to vector dimension)\n", tauCtrl, tauILC)

	lognCtrl := int(math.Log2(float64(tauCtrl)))
	lognILC := int(math.Log2(float64(tauILC)))

	// monomialsCtrl is used only by the online control UnpackCt calls.
	// Those ciphertexts are packed with tauCtrl=4, so using tauILC monomials here
	// would be wasteful and, depending on the packing routine, may select the wrong
	// rotation schedule.
	galElsCtrl := make([]uint64, lognCtrl)
	monomialsCtrl := make([]ring.Poly, lognCtrl)
	for i := 0; i < lognCtrl; i++ {
		galElsCtrl[i] = uint64(tauCtrl/int(math.Pow(2, float64(i))) + 1)
		monomialsCtrl[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomialsCtrl[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomialsCtrl[i], monomialsCtrl[i])
		ringQ.NTT(monomialsCtrl[i], monomialsCtrl[i])
	}

	// Generate the union of Galois keys for both layouts. The current ILC update
	// uses a slice of scalar RLWE ciphertexts for E and therefore does not call
	// UnpackCt at tauILC, but keeping the tauILC keys available makes the key set
	// consistent if the ILC side is later changed to packed-E operations.
	galSet := make(map[uint64]bool)
	galEls := make([]uint64, 0, lognCtrl+lognILC)
	for _, tauNow := range []int{tauCtrl, tauILC} {
		lognNow := int(math.Log2(float64(tauNow)))
		for i := 0; i < lognNow; i++ {
			gal := uint64(tauNow/int(math.Pow(2, float64(i))) + 1)
			if !galSet[gal] {
				galSet[gal] = true
				galEls = append(galEls, gal)
			}
		}
	}

	ilcStateDir := filepath.Join(baseDir, "ilc_state_kernel")
	os.RemoveAll(ilcStateDir)
	if err := os.MkdirAll(ilcStateDir, 0755); err != nil {
		fmt.Println("mkdir error:", err)
	}

	kgen := rlwe.NewKeyGenerator(params)

	// IMPORTANT: v_ilc.bin is an RLWE ciphertext. It is only meaningful with the
	// same secret key that encrypted it. Reusing a ciphertext with a newly generated
	// key produces random-looking feedforward values of order Q * s*r*L.
	skPath := filepath.Join(ilcStateDir, "sk.bin")
	keyLoaded := false
	sk := kgen.GenSecretKeyNew()
	if skData, err := os.ReadFile(skPath); err == nil {
		if err2 := sk.UnmarshalBinary(skData); err2 != nil {
			fmt.Println("[KEY] WARN: could not parse sk.bin; generated a new key and reset ILC state:", err2)
			sk = kgen.GenSecretKeyNew()
			if out, err3 := sk.MarshalBinary(); err3 == nil {
				os.WriteFile(skPath, out, 0600)
			}
			os.Remove(filepath.Join(ilcStateDir, "v_ilc.bin"))
		} else {
			keyLoaded = true
			fmt.Println("[KEY] Loaded secret key:", skPath)
		}
	} else {
		if out, err2 := sk.MarshalBinary(); err2 == nil {
			os.WriteFile(skPath, out, 0600)
			fmt.Println("[KEY] Generated new secret key:", skPath)
		} else {
			fmt.Println("[KEY] WARN: could not save sk.bin:", err2)
		}
		// If an old ciphertext state exists but no matching key exists, it cannot be
		// decrypted correctly. Start the ILC state from zero.
		os.Remove(filepath.Join(ilcStateDir, "v_ilc.bin"))
	}

	rlk := kgen.GenRelinearizationKeyNew(sk)
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, kgen.GenGaloisKeysNew(galEls, sk)...)
	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	decryptorRLWE := rlwe.NewDecryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	// ── Offline: encrypt all controller matrices ───────────────────────────
	ctF := RGSW.EncPack(F_, tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(utils.ScalMatMult(1/s, G_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(utils.ScalMatMult(1/s, H_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(utils.ScalMatMult(1/s, R_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctNbar := RGSW.EncPack(utils.ScalMatMult(1/s, Nbar), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctP := RGSW.EncPack(utils.ScalMatMult(1/s, P_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)

	fmt.Printf("Steps: %d  (Ts=%.3fs, T=%.1fs)\n", nSteps, Ts, trajTime)

	// ── ILC Setup ─────────────────────────────────────────────────────────
	pN := p * nSteps // 400
	mN := m * nSteps // 400

	fmt.Println("[ILC] Calculating L_opt dynamically...")

	n_x := len(Ad)
	n_z := len(F_)
	n_aug := n_x + n_z

	A_aug := make([][]float64, n_aug)
	for i := range A_aug {
		A_aug[i] = make([]float64, n_aug)
	}
	B_aug := make([][]float64, n_aug)
	for i := range B_aug {
		B_aug[i] = make([]float64, m)
	}
	C_aug := make([][]float64, p)
	for i := range C_aug {
		C_aug[i] = make([]float64, n_aug)
	}

	BdH := matMul(Bd, H_)
	GC := matMul(G_, C)
	RH := matMul(R_, H_)
	F_plus_RH := matAdd(F_, RH)

	for i := 0; i < n_x; i++ {
		for j := 0; j < n_x; j++ {
			A_aug[i][j] = Ad[i][j]
		}
		for j := 0; j < n_z; j++ {
			A_aug[i][n_x+j] = BdH[i][j]
		}
		for j := 0; j < m; j++ {
			B_aug[i][j] = Bd[i][j]
		}
	}
	for i := 0; i < n_z; i++ {
		for j := 0; j < n_x; j++ {
			A_aug[n_x+i][j] = GC[i][j]
		}
		for j := 0; j < n_z; j++ {
			A_aug[n_x+i][n_x+j] = F_plus_RH[i][j]
		}
		for j := 0; j < m; j++ {
			B_aug[n_x+i][j] = R_[i][j]
		}
	}
	for i := 0; i < p; i++ {
		for j := 0; j < n_x; j++ {
			C_aug[i][j] = C[i][j]
		}
	}

	h := make([][][]float64, nSteps)
	Apow := eye(n_aug)
	for i := 0; i < nSteps; i++ {
		h[i] = matMul(C_aug, matMul(Apow, B_aug))
		Apow = matMul(A_aug, Apow)
	}

	GN := make([][]float64, pN)
	for i := range GN {
		GN[i] = make([]float64, mN)
	}
	for t := 1; t < nSteps; t++ {
		for s := 0; s < t; s++ {
			for i := 0; i < p; i++ {
				for j := 0; j < m; j++ {
					GN[t*p+i][s*m+j] = h[t-1-s][i][j]
				}
			}
		}
	}

	// ── Adjoint-based full-space ILC gain: L_opt = γ G_N^T ──────────────────
	//   Update:    V_{j+1} = V_j + γ G_N^T E_j
	//   Error dyn: E_{j+1} = (I - γ G_N G_N^T) E_j
	//   Stability: 0 < γ < 2 / λ_max(G_N^T G_N)
	//   Choice:    γ = 1 / λ_max  (no matrix inversion needed)

	GNT := matTranspose(GN)

	// λ_max(G_N^T G_N) via power iteration  (mN × mN, symmetric PSD)
	GNT_GN := matMul(GNT, GN)
	lambdaMax := spectralRadiusSym(GNT_GN)

	// γ = 1 / λ_max  satisfies 0 < γ < 2/λ_max
	gamma := 1.0 / lambdaMax

	// L_opt = γ G_N^T  (mN × pN)
	Lopt := scalMat(gamma, GNT)

	// Stability check in output space: ρ(I - γ G_N G_N^T)  (pN × pN)
	GN_GNT := matMul(GN, GNT)
	Phi := make([][]float64, pN)
	for i := 0; i < pN; i++ {
		Phi[i] = make([]float64, pN)
		for j := 0; j < pN; j++ {
			if i == j {
				Phi[i][j] = 1.0 - gamma*GN_GNT[i][j]
			} else {
				Phi[i][j] = -gamma * GN_GNT[i][j]
			}
		}
	}
	PhiT_Phi := matMul(matTranspose(Phi), Phi)
	rhoBound := math.Sqrt(spectralRadiusSym(PhiT_Phi))
	fmt.Printf("[ILC] L_opt: %d×%d   tauILC=%d\n", len(Lopt), len(Lopt[0]), tauILC)
	fmt.Printf("[ILC] γ=%.6f (1/λ_max, λ_max=%.4f), ρ(I - γ·G_N·G_N^T) = %.6f (stable: %v)\n",
		gamma, lambdaMax, rhoBound, rhoBound < 1.0)

	fmt.Println("[ILC] Encrypting L_opt as RGSW (offline) ...")
	tSetup := time.Now()
	ctL := RGSW.EncPack(utils.ScalMatMult(1/s, Lopt), tauILC, encryptorRGSW, levelQ, levelP, ringQ, params)
	fmt.Printf("[ILC] ctL ready in %v\n", time.Since(tSetup).Round(time.Millisecond))

	// VencCt: accumulated ILC correction, packed RLWE (tauILC=512, mN=400 elements).
	// Initialize with a real encryption of zero; do not decrypt an uninitialized/zero struct.
	zeroVBar := make([]int64, mN)
	VencCt := RLWE.EncPack(zeroVBar, tauILC, 1/L, *encryptorRLWE, ringQ, params)
	vIlcVec := make([]float64, mN)
	vPath := filepath.Join(ilcStateDir, "v_ilc.bin")
	if vData, err2 := os.ReadFile(vPath); err2 == nil {
		loadedV := rlwe.NewCiphertext(params, 1, params.MaxLevel())
		if err3 := loadedV.UnmarshalBinary(vData); err3 != nil {
			fmt.Println("[ILC] WARN: could not parse v_ilc.bin; starting from zero:", err3)
		} else if !keyLoaded {
			fmt.Println("[ILC] Existing v_ilc.bin has no matching persisted key; starting from zero.")
		} else {
			VencCt = loadedV
			vIlcVec = RLWE.DecUnpack(VencCt, mN, tauILC, *decryptorRLWE, s*r*L, ringQ, params)
			fmt.Println("[ILC] Loaded v_ilc.bin from previous iteration.")
		}
	} else {
		fmt.Println("[ILC] No compatible v_ilc.bin found — zero feedforward (iteration 0).")
	}
	fmt.Printf("[ILC] Feedforward ready: step0=[%.4f, %.4f] V\n", vIlcVec[0], vIlcVec[1])

	// ── 1) Plaintext simulation ────────────────────────────────────────────
	yPt := make([][]float64, nSteps)
	uPt := make([][]float64, nSteps)
	xpPt := make([][]float64, nSteps+1)
	xpPt[0] = make([]float64, n)

	xp := make([]float64, n)
	z := make([]float64, n)

	for k := 0; k < nSteps; k++ {
		ref := computeRef(k, Ts, trajTime)
		rk := []float64{ref[0]}
		y := utils.MatVecMult(C, xp)
		u := utils.VecAdd(utils.MatVecMult(H_, z), utils.MatVecMult(Nbar, rk))
		xp = utils.VecAdd(utils.MatVecMult(Ad, xp), utils.MatVecMult(Bd, u))
		z = utils.VecAdd(
			utils.VecAdd(utils.MatVecMult(F_, z), utils.MatVecMult(G_, y)),
			utils.VecAdd(utils.MatVecMult(R_, u), utils.MatVecMult(P_, rk)),
		)
		yPt[k] = y
		uPt[k] = u
		xpPt[k+1] = xp
	}

	// ── Simulation output dir ─────────────────────────────────────────────
	outDir := filepath.Join(baseDir, "result_kernel")
	os.RemoveAll(outDir)
	if err := os.MkdirAll(outDir, 0755); err != nil {
		fmt.Println("mkdir error:", err)
	}

	// ── ILC trial loop ────────────────────────────────────────────────────
	// Each trial: reset plant/observer → 200-step encrypted run → update V.
	// V accumulates across trials; plant state resets each trial.
	// A fixed repeating disturbance is applied to the plant every trial.
	//
	// Saved files:
	//   V_history.csv         : N_TRIALS rows × mN cols  (vIlcVec after each trial)
	//   error_rms.csv         : N_TRIALS rows × 2 cols   (pitchRMS_deg, yawRMS_deg)
	//   trial_NNN_{y,u}.csv   : full trajectory for select trials
	const N_TRIALS = 5000
	vHistory := make([][]float64, 0, N_TRIALS)
	errHistory := make([][]float64, 0, N_TRIALS)

	// Full (y, u) trajectory written for these 1-based trial numbers
	saveTrajAt := map[int]bool{1: true, N_TRIALS: true}

	for trial := 1; trial <= N_TRIALS; trial++ {
		fmt.Printf("\n=== ILC Trial %d / %d ===\n", trial, N_TRIALS)

		// Reset plant and observer state for this trial
		xpLoop := make([]float64, n)
		z0Bar := utils.RoundVec(utils.ScalVecMult(1.0/(r*s), make([]float64, n)))
		zCtPack := RLWE.EncPack(z0Bar, tauCtrl, 1/L, *encryptorRLWE, ringQ, params)

		yEncTrial := make([][]float64, nSteps)
		uEncTrial := make([][]float64, nSteps)
		period := make([][]float64, nSteps)
		yScalarStore := make([][]*rlwe.Ciphertext, nSteps)
		rScalarStore := make([][]*rlwe.Ciphertext, nSteps)

		// ── Encrypted control loop ─────────────────────────────────────────
		for k := 0; k < nSteps; k++ {
			ref := computeRef(k, Ts, trajTime)
			rk := []float64{ref[0]}

			y := utils.MatVecMult(C, xpLoop)
			tStart := time.Now()

			yMeasBar := utils.RoundVec(utils.ScalVecMult(1/r, y))
			rMeasBar := utils.RoundVec(utils.ScalVecMult(1/r, rk))
			rStateBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), rk))

			yCt := RLWE.Enc(yMeasBar, 1/L, *encryptorRLWE, ringQ, params)
			rkCt_meas := RLWE.Enc(rMeasBar, 1/L, *encryptorRLWE, ringQ, params)
			rkCt_state := RLWE.Enc(rStateBar, 1/L, *encryptorRLWE, ringQ, params)

			yScalarStore[k] = cloneCtSlice(yCt)
			rScalarStore[k] = cloneCtSlice(rkCt_meas)

			zCt := RLWE.UnpackCt(zCtPack, n, tauCtrl, evaluatorRLWE, ringQ, monomialsCtrl, params)

			uCtPack := RGSW.MultPack(zCt, ctH, evaluatorRGSW, ringQ, params)
			NbarRCt := RGSW.MultPack(rkCt_state, ctNbar, evaluatorRGSW, ringQ, params)
			ringQ.Add(uCtPack.Value[0], NbarRCt.Value[0], uCtPack.Value[0])
			ringQ.Add(uCtPack.Value[1], NbarRCt.Value[1], uCtPack.Value[1])

			u := RLWE.DecUnpack(uCtPack, m, tauCtrl, *decryptorRLWE, r*s*s*L, ringQ, params)
			for i := 0; i < m; i++ {
				u[i] += vIlcVec[m*k+i]
			}
			// if k < 5 {
			// 	fmt.Printf("[DBG k=%3d] ref=%.6f  u_plain=[%9.5f, %9.5f]  u_enc=[%9.5f, %9.5f]\n",
			// 		k, rk[0], uPt[k][0], uPt[k][1], u[0], u[1])
			// }
			uReEnc := RLWE.Enc(
				utils.RoundVec(utils.ScalVecMult(1/r, u)),
				1/L, *encryptorRLWE, ringQ, params)

			FzCt := RGSW.MultPack(zCt, ctF, evaluatorRGSW, ringQ, params)
			GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)
			RuCt := RGSW.MultPack(uReEnc, ctR, evaluatorRGSW, ringQ, params)
			PrCt := RGSW.MultPack(rkCt_meas, ctP, evaluatorRGSW, ringQ, params)
			zCtPack = RLWE.Add(FzCt, GyCt, RuCt, params)
			ringQ.Add(zCtPack.Value[0], PrCt.Value[0], zCtPack.Value[0])
			ringQ.Add(zCtPack.Value[1], PrCt.Value[1], zCtPack.Value[1])

			period[k] = []float64{float64(time.Since(tStart).Milliseconds())}
			yEncTrial[k] = y
			uEncTrial[k] = u

			xpLoop = utils.VecAdd(utils.MatVecMult(Ad, xpLoop), utils.MatVecMult(Bd, u))

			if k == 0 || (k+1)%50 == 0 {
				fmt.Printf("[CTRL] step %3d/%d  period=%.1f ms\n", k+1, nSteps, period[k][0])
			}
		}

		avg := utils.Average(utils.MatToVec(period))
		fmt.Printf("[CTRL] Avg period: %.1f ms\n", avg)

		// ── RMS tracking error ─────────────────────────────────────────────
		eSS := 0.0
		for t := 0; t < nSteps; t++ {
			ref := computeRef(t, Ts, trajTime)
			ep := ref[0] - yEncTrial[t][0]
			eSS += ep * ep
		}
		rmsE := math.Sqrt(eSS / float64(nSteps))
		errHistory = append(errHistory, []float64{rmsE})
		fmt.Printf("[ILC] Trial %2d  Output RMS=%.6f\n", trial, rmsE)

		// ── Save V applied this trial (before ILC update) ────────────────
		vCopy := make([]float64, mN)
		copy(vCopy, vIlcVec)
		vHistory = append(vHistory, vCopy)

		// ── Save trajectory for select trials ─────────────────────────────
		if saveTrajAt[trial] {
			label := fmt.Sprintf("trial_%04d", trial)
			utils.DataExport(yEncTrial, filepath.Join(outDir, label+"_y.csv"))
			utils.DataExport(uEncTrial, filepath.Join(outDir, label+"_u.csv"))
		}

		// ── Encrypted ILC update: V_{j+1} = V_j + L * (R - Y) ────────────
		ctE := make([]*rlwe.Ciphertext, pN)
		for t := 0; t < nSteps; t++ {
			for i := 0; i < p; i++ {
				j := p*t + i
				ctE[j] = rlwe.NewCiphertext(params, 1, rScalarStore[t][i].Level())
				ringQ.Sub(rScalarStore[t][i].Value[0], yScalarStore[t][i].Value[0], ctE[j].Value[0])
				ringQ.Sub(rScalarStore[t][i].Value[1], yScalarStore[t][i].Value[1], ctE[j].Value[1])
			}
		}
		tMult := time.Now()
		dUCt := RGSW.MultPack(ctE, ctL, evaluatorRGSW, ringQ, params)
		fmt.Printf("[ILC] MultPack: %v\n", time.Since(tMult).Round(time.Millisecond))

		ringQ.Add(VencCt.Value[0], dUCt.Value[0], VencCt.Value[0])
		ringQ.Add(VencCt.Value[1], dUCt.Value[1], VencCt.Value[1])

		vIlcNew := RLWE.DecUnpack(VencCt, mN, tauILC, *decryptorRLWE, s*r*L, ringQ, params)
		vIlcVec = vIlcNew

		// ── Incremental save every 10 trials (crash-safe) ─────────────────
		if trial%10 == 0 || trial == N_TRIALS {
			utils.DataExport(errHistory, filepath.Join(outDir, "error_rms.csv"))
			utils.DataExport(vHistory, filepath.Join(outDir, "V_history.csv"))
		}
	}

	// ── Persist final encrypted V for next run ────────────────────────────
	if vData, err := VencCt.MarshalBinary(); err == nil {
		os.WriteFile(vPath, vData, 0644)
		fmt.Println("[ILC] Saved v_ilc.bin →", vPath)
	}

	// ── Export histories ──────────────────────────────────────────────────
	utils.DataExport(vHistory, filepath.Join(outDir, "V_history.csv"))
	utils.DataExport(errHistory, filepath.Join(outDir, "error_rms.csv"))

	refs := make([][]float64, nSteps)
	for k := range refs {
		ref := computeRef(k, Ts, trajTime)
		refs[k] = []float64{ref[0]}
	}
	utils.DataExport(refs, filepath.Join(outDir, "ref.csv"))
	utils.DataExport(yPt, filepath.Join(outDir, "y_baseline.csv"))
	utils.DataExport(uPt, filepath.Join(outDir, "u_baseline.csv"))

	fmt.Printf("\nAll results saved to: %s\n", outDir)
	fmt.Printf("  V_history.csv : %d trials x %d values\n", N_TRIALS, mN)
	fmt.Printf("  error_rms.csv : %d trials x [outputRMS]\n", N_TRIALS)
	fmt.Println("Run: python plot_results.py")
}
