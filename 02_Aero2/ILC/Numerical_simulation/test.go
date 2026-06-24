//go:build ignore

// test.go — Encrypted ILC with SVD Reduced-Space Update (quick test, 20 trials)
//   (n=4 states, m=2 inputs, p=1 output — matrices loaded from controller_data/)
//
// Differs from simulation.go in the ILC learning phase:
//   - S  (mN × nR): orthonormal basis of Range(G_N^T),  nR = rank(G_N)
//   - G_r = G_N @ S   (pN × nR)
//   - K_r = (G_r^T G_r + λI)^{-1} G_r^T   (nR × pN)  ← much smaller than L_opt
//   - Encrypted state: Theta_j ∈ R^nR   (vs mN in simulation.go)
//   - Update:      Theta_{j+1} = Theta_j + K_r @ E_j   (encrypted, tauSVD slots)
//   - Reconstruct: V_{j+1} = S @ Theta_{j+1}           (plaintext projection)
//
// Benefits over simulation.go:
//   - No null-space accumulation (V stays in Range(G_N^T))
//   - Smaller encrypted state  (nR ≤ pN << mN)
//   - Faster encrypted multiply (K_r is nR×pN vs mN×pN for L_opt)
//
// Run: go run simulation_svd.go

package main

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"os/exec"
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

// computeS builds an orthonormal basis S (mN × nR) for Range(G_N^T)
// via Modified Gram-Schmidt on the rows of G_N.
func computeS(GN [][]float64, tol float64) ([][]float64, int) {
	pN, mN := len(GN), len(GN[0])
	vecs := make([][]float64, pN)
	for i := 0; i < pN; i++ {
		vecs[i] = make([]float64, mN)
		copy(vecs[i], GN[i])
	}
	qs := make([][]float64, 0, pN)
	for i := 0; i < pN; i++ {
		v := vecs[i]
		norm := 0.0
		for k := 0; k < mN; k++ {
			norm += v[k] * v[k]
		}
		norm = math.Sqrt(norm)
		if norm < tol {
			continue
		}
		q := make([]float64, mN)
		for k := 0; k < mN; k++ {
			q[k] = v[k] / norm
		}
		qs = append(qs, q)
		for j := i + 1; j < pN; j++ {
			dot := 0.0
			for k := 0; k < mN; k++ {
				dot += q[k] * vecs[j][k]
			}
			for k := 0; k < mN; k++ {
				vecs[j][k] -= dot * q[k]
			}
		}
	}
	nR := len(qs)
	S := make([][]float64, mN)
	for i := range S {
		S[i] = make([]float64, nR)
		for j := 0; j < nR; j++ {
			S[i][j] = qs[j][i]
		}
	}
	return S, nR
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

// computeRef: periodic reference matching conversion.m (r(0)=r(T)=0)
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

// computeDisturbance: trial-invariant time-varying disturbance (m=2)
func computeDisturbance(step int, Ts float64, m int) []float64 {
	t := float64(step) * Ts
	f0 := 0.1
	d := make([]float64, m)
	d[0] = 0.20*math.Sin(2*math.Pi*2*f0*t+0.5) + 0.10*math.Sin(2*math.Pi*4*f0*t)
	if m > 1 {
		d[1] = 0.15*math.Sin(2*math.Pi*3*f0*t+1.0) - 0.10*math.Cos(2*math.Pi*5*f0*t)
	}
	return d
}

func main() {
	_, thisFile, _, _ := runtime.Caller(0)
	baseDir := filepath.Dir(thisFile)

	// ── Encryption parameters ──────────────────────────────────────────────
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 9, LogQ: []int{60}, LogP: []int{60}, NTTFlag: true,
	})
	fmt.Println("N =", params.N(), "  logQ =", params.QBigInt().BitLen())

	// ── Load matrices from controller_data/ ───────────────────────────────
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

	n := len(Ad)
	m := len(Bd[0])
	p := len(C)

	fmt.Printf("Loaded from %s/  →  n=%d  m=%d  p=%d  Ts=%.4f\n", dataDir, n, m, p, Ts)

	trajTime := 10.0
	nSteps := int(trajTime / Ts)
	pN := p * nSteps
	mN := m * nSteps

	// ── Build lifted matrix G_N ────────────────────────────────────────────
	fmt.Println("[ILC] Building G_N...")
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
	F_plusRH := matAdd(F_, RH)

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
			A_aug[n_x+i][n_x+j] = F_plusRH[i][j]
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

	// ── SVD: Range(G_N^T) basis via Modified Gram-Schmidt ─────────────────
	fmt.Println("[SVD] Computing S = Range(G_N^T) basis via Gram-Schmidt...")
	maxRowNormSq := 0.0
	for i := 0; i < pN; i++ {
		norm := 0.0
		for k := 0; k < mN; k++ {
			norm += GN[i][k] * GN[i][k]
		}
		if norm > maxRowNormSq {
			maxRowNormSq = norm
		}
	}
	svdTol := math.Sqrt(maxRowNormSq) * 1e-10
	S, nR := computeS(GN, svdTol)
	fmt.Printf("[SVD] rank(G_N) = nR = %d  (pN=%d, mN=%d, compression=%.1f%%)\n",
		nR, pN, mN, 100.0*float64(nR)/float64(mN))

	// ── Quantisation ──────────────────────────────────────────────────────
	s := 1.0 / 500.0
	L := 1.0 / 10000.0
	r := 1.0 / 500.0
	fmt.Printf("Scaling: 1/s=%v  1/r=%v  1/L=%v\n", 1/s, 1/r, 1/L)

	// ── RLWE/RGSW setup ────────────────────────────────────────────────────
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	ringQ := params.RingQ()

	controlDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tauCtrl := int(math.Pow(2, math.Ceil(math.Log2(controlDim))))
	tauSVD := int(math.Pow(2, math.Ceil(math.Log2(float64(nR)))))
	fmt.Printf("Packing: tauCtrl=%d  tauSVD=%d  (nR=%d)\n", tauCtrl, tauSVD, nR)

	lognCtrl := int(math.Log2(float64(tauCtrl)))

	monomialsCtrl := make([]ring.Poly, lognCtrl)
	for i := 0; i < lognCtrl; i++ {
		monomialsCtrl[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomialsCtrl[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomialsCtrl[i], monomialsCtrl[i])
		ringQ.NTT(monomialsCtrl[i], monomialsCtrl[i])
	}

	galSet := make(map[uint64]bool)
	galEls := make([]uint64, 0)
	for _, tau := range []int{tauCtrl, tauSVD} {
		logn := int(math.Log2(float64(tau)))
		for i := 0; i < logn; i++ {
			gal := uint64(tau/int(math.Pow(2, float64(i))) + 1)
			if !galSet[gal] {
				galSet[gal] = true
				galEls = append(galEls, gal)
			}
		}
	}

	ilcStateDir := filepath.Join(baseDir, "ilc_state_test")
	os.RemoveAll(ilcStateDir)
	if err := os.MkdirAll(ilcStateDir, 0755); err != nil {
		fmt.Println("mkdir error:", err)
	}

	kgen := rlwe.NewKeyGenerator(params)
	skPath := filepath.Join(ilcStateDir, "sk.bin")
	keyLoaded := false
	sk := kgen.GenSecretKeyNew()
	if skData, err := os.ReadFile(skPath); err == nil {
		if err2 := sk.UnmarshalBinary(skData); err2 != nil {
			fmt.Println("[KEY] WARN: could not parse sk.bin; generating new key:", err2)
			sk = kgen.GenSecretKeyNew()
			if out, err3 := sk.MarshalBinary(); err3 == nil {
				os.WriteFile(skPath, out, 0600)
			}
			os.Remove(filepath.Join(ilcStateDir, "theta_svd.bin"))
		} else {
			keyLoaded = true
			fmt.Println("[KEY] Loaded secret key:", skPath)
		}
	} else {
		if out, err2 := sk.MarshalBinary(); err2 == nil {
			os.WriteFile(skPath, out, 0600)
			fmt.Println("[KEY] Generated new secret key:", skPath)
		}
		os.Remove(filepath.Join(ilcStateDir, "theta_svd.bin"))
	}

	rlk := kgen.GenRelinearizationKeyNew(sk)
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, kgen.GenGaloisKeysNew(galEls, sk)...)
	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	decryptorRLWE := rlwe.NewDecryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	// ── Offline: encrypt controller matrices ──────────────────────────────
	ctF := RGSW.EncPack(F_, tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(utils.ScalMatMult(1/s, G_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(utils.ScalMatMult(1/s, H_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(utils.ScalMatMult(1/s, R_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctNbar := RGSW.EncPack(utils.ScalMatMult(1/s, Nbar), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctP := RGSW.EncPack(utils.ScalMatMult(1/s, P_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)

	fmt.Printf("Steps: %d  (Ts=%.3fs, T=%.1fs)\n", nSteps, Ts, trajTime)

	// ── Adjoint-based ILC gain: K_r = γ G_r^T ───────────────────────────────
	//   Update:    Theta_{j+1} = Theta_j + γ G_r^T E_j
	//   Error dyn: E_{j+1}    = (I - γ G_r G_r^T) E_j
	//   Stability: 0 < γ < 2 / λ_max(G_r^T G_r)
	//   Choice:    γ = 1 / λ_max  (no matrix inversion needed)

	// G_r = G_N @ S  (pN × nR)
	Gr := matMul(GN, S)
	GrT := matTranspose(Gr)

	// λ_max(G_r^T G_r) via power iteration  (nR × nR, symmetric PSD)
	GrTGr := matMul(GrT, Gr)
	lambdaMax := spectralRadiusSym(GrTGr)

	// γ = 1 / λ_max  satisfies 0 < γ < 2/λ_max
	gamma := 1.0 / lambdaMax

	// K_r = γ G_r^T  (nR × pN)
	Kr := scalMat(gamma, GrT)

	// Stability check: ρ(I - γ G_r^T G_r)  (nR × nR)
	ATheta := make([][]float64, nR)
	for i := 0; i < nR; i++ {
		ATheta[i] = make([]float64, nR)
		for j := 0; j < nR; j++ {
			if i == j {
				ATheta[i][j] = 1.0 - gamma*GrTGr[i][j]
			} else {
				ATheta[i][j] = -gamma * GrTGr[i][j]
			}
		}
	}
	rhoSVD := math.Sqrt(spectralRadiusSym(matMul(matTranspose(ATheta), ATheta)))
	fmt.Printf("[SVD-ILC] γ=%.6f  (1/λ_max, λ_max=%.4f)  nR=%d  K_r: %d×%d\n",
		gamma, lambdaMax, nR, len(Kr), len(Kr[0]))
	fmt.Printf("[SVD-ILC] ρ(I - γ·G_r^T·G_r) = %.6f  (stable: %v)\n", rhoSVD, rhoSVD < 1.0)

	// Encrypt K_r (offline, tauSVD slots)
	fmt.Println("[SVD-ILC] Encrypting K_r (offline)...")
	tSetup := time.Now()
	ctKr := RGSW.EncPack(utils.ScalMatMult(1/s, Kr), tauSVD, encryptorRGSW, levelQ, levelP, ringQ, params)
	fmt.Printf("[SVD-ILC] ctKr ready in %v\n", time.Since(tSetup).Round(time.Millisecond))

	// ── Init encrypted ILC state: Theta_j (nR entries) ────────────────────
	zeroThetaBar := make([]int64, nR)
	ThetaEncCt := RLWE.EncPack(zeroThetaBar, tauSVD, 1/L, *encryptorRLWE, ringQ, params)
	thetaVec := make([]float64, nR)
	vIlcVec := make([]float64, mN) // V = S @ Theta (plaintext feedforward)

	thetaPath := filepath.Join(ilcStateDir, "theta_svd.bin")
	if tData, err2 := os.ReadFile(thetaPath); err2 == nil {
		loadedT := rlwe.NewCiphertext(params, 1, params.MaxLevel())
		if err3 := loadedT.UnmarshalBinary(tData); err3 != nil {
			fmt.Println("[SVD-ILC] WARN: could not parse theta_svd.bin; starting from zero:", err3)
		} else if !keyLoaded {
			fmt.Println("[SVD-ILC] Existing theta_svd.bin has no matching key; starting from zero.")
		} else {
			ThetaEncCt = loadedT
			thetaVec = RLWE.DecUnpack(ThetaEncCt, nR, tauSVD, *decryptorRLWE, s*r*L, ringQ, params)
			vIlcVec = utils.MatVecMult(S, thetaVec)
			fmt.Println("[SVD-ILC] Loaded theta_svd.bin from previous iteration.")
		}
	} else {
		fmt.Println("[SVD-ILC] No theta_svd.bin found — zero feedforward (iteration 0).")
	}
	fmt.Printf("[SVD-ILC] Feedforward ready: step0=[%.4f, %.4f]\n", vIlcVec[0], vIlcVec[1])

	// ── Plaintext baseline simulation ─────────────────────────────────────
	yPt := make([][]float64, nSteps)
	uPt := make([][]float64, nSteps)
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
	}

	// ── Output directory ──────────────────────────────────────────────────
	outDir := filepath.Join(baseDir, "test")
	os.RemoveAll(outDir)
	if err := os.MkdirAll(outDir, 0755); err != nil {
		fmt.Println("mkdir error:", err)
	}

	// ── ILC trial loop ────────────────────────────────────────────────────
	const N_TRIALS = 20
	vHistory := make([][]float64, 0, N_TRIALS)
	thetaHistory := make([][]float64, 0, N_TRIALS)
	errHistory := make([][]float64, 0, N_TRIALS)
	saveTrajAt := map[int]bool{1: true, N_TRIALS: true}

	for trial := 1; trial <= N_TRIALS; trial++ {
		fmt.Printf("\n=== SVD-ILC Trial %d / %d ===\n", trial, N_TRIALS)

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

			yCtPack := RLWE.EncPack(yMeasBar, tauCtrl, 1/L, *encryptorRLWE, ringQ, params)
			yCt := RLWE.UnpackCt(yCtPack, p, tauCtrl, evaluatorRLWE, ringQ, monomialsCtrl, params)
			rkCt_meas := RLWE.Enc(rMeasBar, 1/L, *encryptorRLWE, ringQ, params)
			rkCt_state := RLWE.Enc(rStateBar, 1/L, *encryptorRLWE, ringQ, params)

			yScalarStore[k] = cloneCtSlice(yCt)
			rScalarStore[k] = cloneCtSlice(rkCt_meas)

			zCt := RLWE.UnpackCt(zCtPack, n, tauCtrl, evaluatorRLWE, ringQ, monomialsCtrl, params)

			uCtPack := RGSW.MultPack(zCt, ctH, evaluatorRGSW, ringQ, params)
			NbarRCt := RGSW.MultPack(rkCt_state, ctNbar, evaluatorRGSW, ringQ, params)
			ringQ.Add(uCtPack.Value[0], NbarRCt.Value[0], uCtPack.Value[0])
			ringQ.Add(uCtPack.Value[1], NbarRCt.Value[1], uCtPack.Value[1])

			// Decrypt u, add plaintext feedforward V = S @ Theta
			u := RLWE.DecUnpack(uCtPack, m, tauCtrl, *decryptorRLWE, r*s*s*L, ringQ, params)
			for i := 0; i < m; i++ {
				u[i] += vIlcVec[m*k+i]
			}

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
		fmt.Printf("[SVD-ILC] Trial %2d  Output RMS=%.6f\n", trial, rmsE)

		// ── Save V and Theta applied this trial (before ILC update) ──────────
		vCopy := make([]float64, mN)
		copy(vCopy, vIlcVec)
		vHistory = append(vHistory, vCopy)
		thetaHistory = append(thetaHistory, append([]float64(nil), thetaVec...))

		if saveTrajAt[trial] {
			label := fmt.Sprintf("trial_%04d", trial)
			utils.DataExport(yEncTrial, filepath.Join(outDir, label+"_y.csv"))
			utils.DataExport(uEncTrial, filepath.Join(outDir, label+"_u.csv"))
		}

		// ── Encrypted SVD-ILC update: Theta += K_r × (R - Y) ─────────────
		// ctE: pN scalar ciphertexts of error signal
		fmt.Println("[SVD-ILC] Encrypted update (reduced space)...")
		tILC := time.Now()

		ctE := make([]*rlwe.Ciphertext, pN)
		for t := 0; t < nSteps; t++ {
			for i := 0; i < p; i++ {
				j := p*t + i
				ctE[j] = rlwe.NewCiphertext(params, 1, rScalarStore[t][i].Level())
				ringQ.Sub(rScalarStore[t][i].Value[0], yScalarStore[t][i].Value[0], ctE[j].Value[0])
				ringQ.Sub(rScalarStore[t][i].Value[1], yScalarStore[t][i].Value[1], ctE[j].Value[1])
			}
		}

		// dTheta = K_r × E  (nR entries, tauSVD slots)
		dThetaCt := RGSW.MultPack(ctE, ctKr, evaluatorRGSW, ringQ, params)
		ringQ.Add(ThetaEncCt.Value[0], dThetaCt.Value[0], ThetaEncCt.Value[0])
		ringQ.Add(ThetaEncCt.Value[1], dThetaCt.Value[1], ThetaEncCt.Value[1])
		fmt.Printf("[SVD-ILC] MultPack done in %v\n", time.Since(tILC).Round(time.Millisecond))

		// Decrypt Theta → reconstruct V = S @ Theta (plaintext)
		thetaVec = RLWE.DecUnpack(ThetaEncCt, nR, tauSVD, *decryptorRLWE, s*r*L, ringQ, params)
		vIlcVec = utils.MatVecMult(S, thetaVec)
		fmt.Printf("[SVD-ILC] Next feedforward: step0=[%.4f, %.4f]\n", vIlcVec[0], vIlcVec[1])

		// Persist Theta ciphertext
		if tData, err := ThetaEncCt.MarshalBinary(); err == nil {
			os.WriteFile(thetaPath, tData, 0644)
			fmt.Println("[SVD-ILC] Saved theta_svd.bin →", thetaPath)
		}

		if trial%10 == 0 || trial == N_TRIALS {
			utils.DataExport(errHistory, filepath.Join(outDir, "error_rms.csv"))
			utils.DataExport(vHistory, filepath.Join(outDir, "V_history.csv"))
		}
	}

	// ── Persist final Theta ciphertext ────────────────────────────────────
	if tData, err := ThetaEncCt.MarshalBinary(); err == nil {
		os.WriteFile(thetaPath, tData, 0644)
	}

	// ── Export histories ──────────────────────────────────────────────────
	utils.DataExport(vHistory, filepath.Join(outDir, "V_history.csv"))
	utils.DataExport(errHistory, filepath.Join(outDir, "error_rms.csv"))
	utils.DataExport(thetaHistory, filepath.Join(outDir, "theta_history.csv"))

	refs := make([][]float64, nSteps)
	for k := range refs {
		ref := computeRef(k, Ts, trajTime)
		refs[k] = []float64{ref[0]}
	}
	utils.DataExport(refs, filepath.Join(outDir, "ref.csv"))
	utils.DataExport(yPt, filepath.Join(outDir, "y_baseline.csv"))
	utils.DataExport(uPt, filepath.Join(outDir, "u_baseline.csv"))

	fmt.Printf("\nAll results saved to: %s\n", outDir)
	fmt.Printf("  V_history.csv     : %d trials x %d values\n", N_TRIALS, mN)
	fmt.Printf("  theta_history.csv : %d trials x %d values\n", N_TRIALS, nR)
	fmt.Printf("  error_rms.csv     : %d trials x [outputRMS]\n", N_TRIALS)
	fmt.Println("\nLaunching plot_test.py ...")
	cmd := exec.Command("python", "plot_test.py")
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		fmt.Println("[PLOT] failed:", err)
	}
}
