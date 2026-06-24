//go:build ignore

// EncILC_svd.go — Encrypted ILC Controller with SVD-based reduced-space update
//
// Differs from EncILC.go in the ILC learning phase:
//   - ILC state in ciphertext: Theta_j = S^T V_j  (n_r entries, n_r = rank(G_N))
//   - Reduced gain: K_r = (G_r^T G_r + λI)^{-1} G_r^T  where G_r = G_N @ S
//   - Update: Theta_{j+1} = Theta_j + K_r @ E_j  (encrypted)
//   - Reconstruct: V_{j+1} = S @ Theta_{j+1}  (plaintext at plant side)
// This restricts updates to Range(G_N^T) and avoids null-space accumulation.
//
// Binary protocol (little-endian, per step):
//   client→server: [0x01][y0 y1 r0 r1 : 4×float64] = 33 bytes
//   server→client: [u0 u1 : 2×float64]              = 16 bytes
// End-of-trial:
//   client→server: [0x02]                            =  1 byte
//   server→client: [rmsP_rad rmsY_rad : 2×float64]  = 16 bytes
//
// Startup: go run EncILC_svd.go
// Then:    python TCP.py

package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"os"
	"path/filepath"
	"runtime"
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

func matVecMul(A [][]float64, x []float64) []float64 {
	n, m := len(A), len(A[0])
	y := make([]float64, n)
	for i := 0; i < n; i++ {
		for j := 0; j < m; j++ {
			y[i] += A[i][j] * x[j]
		}
	}
	return y
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

// computeS builds an orthonormal basis S (mN x nR) for Range(G_N^T)
// by running Modified Gram-Schmidt on the rows of G_N.
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
		LogN: 12, LogQ: []int{56}, LogP: []int{51}, NTTFlag: true,
	})
	fmt.Println("N =", params.N(), "  logQ =", params.QBigInt().BitLen())

	// ── Aero2 continuous-time model ────────────────────────────────────────
	Jp, Jy := 0.0219, 0.0220
	Dp, Dy := 0.00711, 0.0220
	Mg := 0.0375
	Kpp, Kpy := 0.0011, 0.0021
	Kyp, Kyy := -0.0027, 0.0022

	Ac := [][]float64{
		{0, 0, 1, 0},
		{0, 0, 0, 1},
		{-Mg / Jp, 0, -Dp / Jp, 0},
		{0, 0, 0, -Dy / Jy},
	}
	Bc := [][]float64{
		{0, 0},
		{0, 0},
		{Kpp / Jp, Kpy / Jp},
		{Kyp / Jy, Kyy / Jy},
	}
	C := [][]float64{
		{1, 0, 0, 0},
		{0, 1, 0, 0},
	}

	Ts := 0.05 // 20 Hz
	Ad, Bd := zohDisc(Ac, Bc, Ts)

	// ── Transformed controller matrices (from conversion.m) ────────────────
	F_ := [][]float64{
		{0, 0, 1, 0},
		{0, 0, 0, 1},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
	}
	G_ := [][]float64{
		{2.0305, 0.2638},
		{-0.0466, 3.4913},
		{-1.9532, -0.2350},
		{0.0477, -3.1567},
	}
	R_ := [][]float64{
		{-0.0247, -0.0387},
		{0.0283, -0.0195},
		{0.0092, 0.0143},
		{-0.0102, 0.0070},
	}
	H_ := [][]float64{
		{-17.0981, 28.4792, 0.0000, 0.0000},
		{-24.0144, -17.2800, 0.0000, 0.0000},
	}
	P_ := [][]float64{
		{4.8159, 0.7575},
		{-0.0850, 8.4213},
		{-1.8661, -0.2879},
		{0.0335, -3.1789},
	}
	Nbar := [][]float64{
		{54.6598, -150.3106},
		{73.6847, 108.3416},
	}

	nState := len(F_) // 4
	m := len(H_)      // 2
	p := len(C)       // 2

	nSteps := 200 // 10s × 20Hz
	pN := p * nSteps
	mN := m * nSteps

	// ── Build augmented closed-loop system and lifted matrix G_N ──────────
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
		for s_idx := 0; s_idx < t; s_idx++ {
			for i := 0; i < p; i++ {
				for j := 0; j < m; j++ {
					GN[t*p+i][s_idx*m+j] = h[t-1-s_idx][i][j]
				}
			}
		}
	}

	// ── SVD: compute S = orthonormal basis of Range(G_N^T) ────────────────
	fmt.Println("[ILC] Computing S via Modified Gram-Schmidt on rows of G_N...")
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
	fmt.Printf("[ILC] rank(G_N) = n_r = %d  (pN=%d, mN=%d)\n", nR, pN, mN)

	// ── Quantisation ──────────────────────────────────────────────────────
	s := 1.0 / 1000.0
	L := 1.0 / 100000.0
	r := 1.0 / 1000.0
	fmt.Printf("Scaling: 1/s=%v  1/r=%v  1/L=%v\n", 1/s, 1/r, 1/L)

	// ── RLWE/RGSW setup ───────────────────────────────────────────────────
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	ringQ := params.RingQ()

	controlDim := math.Max(math.Max(float64(nState), float64(m)), float64(p))
	tauCtrl := int(math.Pow(2, math.Ceil(math.Log2(controlDim))))
	tauSVD := int(math.Pow(2, math.Ceil(math.Log2(float64(nR)))))
	fmt.Printf("Packing: tauCtrl=%d  tauSVD=%d\n", tauCtrl, tauSVD)

	lognCtrl := int(math.Log2(float64(tauCtrl)))
	lognSVD := int(math.Log2(float64(tauSVD)))

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
	_ = galElsCtrl

	galSet := make(map[uint64]bool)
	galEls := make([]uint64, 0, lognCtrl+lognSVD)
	for _, tauNow := range []int{tauCtrl, tauSVD} {
		lognNow := int(math.Log2(float64(tauNow)))
		for i := 0; i < lognNow; i++ {
			gal := uint64(tauNow/int(math.Pow(2, float64(i))) + 1)
			if !galSet[gal] {
				galSet[gal] = true
				galEls = append(galEls, gal)
			}
		}
	}

	ilcStateDir := filepath.Join(baseDir, "ilc_state")
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
			os.Remove(filepath.Join(ilcStateDir, "theta_ilc.bin"))
		} else {
			keyLoaded = true
			fmt.Println("[KEY] Loaded secret key:", skPath)
		}
	} else {
		if out, err2 := sk.MarshalBinary(); err2 == nil {
			os.WriteFile(skPath, out, 0600)
			fmt.Println("[KEY] Generated new secret key:", skPath)
		}
		os.Remove(filepath.Join(ilcStateDir, "theta_ilc.bin"))
	}

	rlk := kgen.GenRelinearizationKeyNew(sk)
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, kgen.GenGaloisKeysNew(galEls, sk)...)
	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	decryptorRLWE := rlwe.NewDecryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	// ── Encrypt controller matrices (offline, once) ────────────────────────
	fmt.Println("[SETUP] Encrypting controller matrices...")
	ctF := RGSW.EncPack(F_, tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(utils.ScalMatMult(1/s, G_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(utils.ScalMatMult(1/s, H_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(utils.ScalMatMult(1/s, R_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctNbar := RGSW.EncPack(utils.ScalMatMult(1/s, Nbar), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctP := RGSW.EncPack(utils.ScalMatMult(1/s, P_), tauCtrl, encryptorRGSW, levelQ, levelP, ringQ, params)

	// ── ILC: compute reduced gain K_r from G_r = G_N @ S ─────────────────
	ILC_LAMBDA := 0.001

	// G_r = G_N @ S  (pN × n_r)
	Gr := matMul(GN, S)

	// K_r = (G_r^T G_r + λI)^{-1} G_r^T  (n_r × pN)
	GrT := matTranspose(Gr)
	GrT_Gr := matMul(GrT, Gr)
	for i := 0; i < nR; i++ {
		GrT_Gr[i][i] += ILC_LAMBDA
	}
	Kr := solveAXB(GrT_Gr, GrT)

	// ── Stability check: A_Theta = I - K_r G_r  and  A_E = I - G_r K_r ──────────
	Kr_Gr := matMul(Kr, Gr)
	A_Theta := make([][]float64, nR)
	for i := 0; i < nR; i++ {
		A_Theta[i] = make([]float64, nR)
		for j := 0; j < nR; j++ {
			if i == j {
				A_Theta[i][j] = 1.0 - Kr_Gr[i][j]
			} else {
				A_Theta[i][j] = -Kr_Gr[i][j]
			}
		}
	}
	// ρ(A_Theta): largest singular value via power iteration on symmetric A_Theta^T A_Theta
	AThetaT_ATheta := matMul(matTranspose(A_Theta), A_Theta)
	rhoTheta := math.Sqrt(spectralRadiusSym(AThetaT_ATheta))
	// ρ(A_E)|_Range(G_r): by Sylvester's theorem, nonzero eigs of G_r K_r equal
	// nonzero eigs of K_r G_r, so eigs of A_E on Range(G_r) equal eigs of A_Theta.
	rhoE := rhoTheta
	fmt.Printf("[ILC] λ=%.4f  n_r=%d\n", ILC_LAMBDA, nR)
	fmt.Printf("[Stability] ρ(I - K_r@G_r)             = %.6f  (Schur stable: %v)\n", rhoTheta, rhoTheta < 1.0)
	fmt.Printf("[Stability] ρ(I - G_r@K_r)|_Range(G_r)  = %.6f  (Schur stable: %v)\n", rhoE, rhoE < 1.0)

	// ── Encrypt K_r (offline, once) ───────────────────────────────────────
	fmt.Println("[ILC] Encrypting K_r (offline)...")
	tSetup := time.Now()
	ctKr := RGSW.EncPack(utils.ScalMatMult(1/s, Kr), tauSVD, encryptorRGSW, levelQ, levelP, ringQ, params)
	fmt.Printf("[ILC] ctKr ready in %v\n", time.Since(tSetup).Round(time.Millisecond))

	// ── Load ILC state: Theta_j (n_r entries, persistent across trials) ───
	zeroThetaBar := make([]int64, nR)
	ThetaEncCt := RLWE.EncPack(zeroThetaBar, tauSVD, 1/L, *encryptorRLWE, ringQ, params)
	thetaVec := make([]float64, nR)
	vIlcVec := make([]float64, mN) // V = S @ Theta, plaintext feedforward

	thetaPath := filepath.Join(ilcStateDir, "theta_ilc.bin")
	if tData, err := os.ReadFile(thetaPath); err == nil {
		loadedT := rlwe.NewCiphertext(params, 1, params.MaxLevel())
		if err2 := loadedT.UnmarshalBinary(tData); err2 != nil {
			fmt.Println("[ILC] WARN: could not parse theta_ilc.bin; starting from zero:", err2)
		} else if !keyLoaded {
			fmt.Println("[ILC] Existing theta_ilc.bin has no matching key; starting from zero.")
		} else {
			ThetaEncCt = loadedT
			thetaVec = RLWE.DecUnpack(ThetaEncCt, nR, tauSVD, *decryptorRLWE, s*r*L, ringQ, params)
			vIlcVec = matVecMul(S, thetaVec) // V = S @ Theta
			fmt.Println("[ILC] Loaded theta_ilc.bin from previous iteration.")
		}
	} else {
		fmt.Println("[ILC] No theta_ilc.bin found — zero feedforward (iteration 0).")
	}
	fmt.Printf("[ILC] Current feedforward: step0=[%.4f, %.4f] V\n", vIlcVec[0], vIlcVec[1])

	// ── TCP server ────────────────────────────────────────────────────────
	ln, err := net.Listen("tcp", ":5555")
	if err != nil {
		fmt.Println("[SERVER] Listen error:", err)
		return
	}
	defer ln.Close()
	fmt.Println("\n=== EncILC-SVD server ready on :5555 ===")
	fmt.Println("Waiting for TCP.py to connect...")

	iterNum := 0

	// ── Trial loop: one TCP connection = one hardware trial ───────────────
	for {
		conn, err := ln.Accept()
		if err != nil {
			fmt.Println("[SERVER] Accept error:", err)
			continue
		}
		iterNum++
		fmt.Printf("\n=== Trial %d: connection from %s ===\n", iterNum, conn.RemoteAddr())

		// Reset per-trial encrypted observer state to zero
		z0Bar := make([]int64, nState)
		zCtPack := RLWE.EncPack(z0Bar, tauCtrl, 1/L, *encryptorRLWE, ringQ, params)

		yScalarStore := make([][]*rlwe.Ciphertext, nSteps)
		rScalarStore := make([][]*rlwe.Ciphertext, nSteps)
		yLog := make([][]float64, nSteps)
		rLog := make([][]float64, nSteps)

		step := 0
		typeBuf := make([]byte, 1)
		dataBuf := make([]byte, 32)

		// ── Online control loop ────────────────────────────────────────────
		for step < nSteps {
			if _, err := io.ReadFull(conn, typeBuf); err != nil {
				fmt.Printf("[SERVER] Connection lost at step %d: %v\n", step, err)
				break
			}
			if typeBuf[0] == 0x02 {
				break // END signal from Python
			}
			if typeBuf[0] != 0x01 {
				continue // unknown type, skip
			}
			if _, err := io.ReadFull(conn, dataBuf); err != nil {
				fmt.Printf("[SERVER] Read error at step %d: %v\n", step, err)
				break
			}

			y0 := math.Float64frombits(binary.LittleEndian.Uint64(dataBuf[0:8]))
			y1 := math.Float64frombits(binary.LittleEndian.Uint64(dataBuf[8:16]))
			r0 := math.Float64frombits(binary.LittleEndian.Uint64(dataBuf[16:24]))
			r1 := math.Float64frombits(binary.LittleEndian.Uint64(dataBuf[24:32]))

			y := []float64{y0, y1}
			rk := []float64{r0, r1}
			yLog[step] = y
			rLog[step] = rk

			// Quantise and encrypt y, r
			yMeasBar := utils.RoundVec(utils.ScalVecMult(1/r, y))
			rMeasBar := utils.RoundVec(utils.ScalVecMult(1/r, rk))
			rStateBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), rk))

			yCt := RLWE.Enc(yMeasBar, 1/L, *encryptorRLWE, ringQ, params)
			rkCt_meas := RLWE.Enc(rMeasBar, 1/L, *encryptorRLWE, ringQ, params)
			rkCt_state := RLWE.Enc(rStateBar, 1/L, *encryptorRLWE, ringQ, params)

			yScalarStore[step] = cloneCtSlice(yCt)
			rScalarStore[step] = cloneCtSlice(rkCt_meas)

			// Encrypted control: u = H×z + Nbar×r
			zCt := RLWE.UnpackCt(zCtPack, nState, tauCtrl, evaluatorRLWE, ringQ, monomialsCtrl, params)

			uCtPack := RGSW.MultPack(zCt, ctH, evaluatorRGSW, ringQ, params)
			NbarRCt := RGSW.MultPack(rkCt_state, ctNbar, evaluatorRGSW, ringQ, params)
			ringQ.Add(uCtPack.Value[0], NbarRCt.Value[0], uCtPack.Value[0])
			ringQ.Add(uCtPack.Value[1], NbarRCt.Value[1], uCtPack.Value[1])

			// Decrypt u, add plaintext ILC feedforward V = S @ Theta
			u := RLWE.DecUnpack(uCtPack, m, tauCtrl, *decryptorRLWE, r*s*s*L, ringQ, params)
			u[0] += vIlcVec[2*step]
			u[1] += vIlcVec[2*step+1]

			// Re-encrypt u (before clipping) for encrypted state update
			uReEnc := RLWE.Enc(
				utils.RoundVec(utils.ScalVecMult(1/r, u)),
				1/L, *encryptorRLWE, ringQ, params)

			// Clip u for hardware output
			for i := range u {
				if u[i] > 20 {
					u[i] = 20
				}
				if u[i] < -20 {
					u[i] = -20
				}
			}

			// Encrypted observer state update: z = F×z + G×y + R×u + P×r
			FzCt := RGSW.MultPack(zCt, ctF, evaluatorRGSW, ringQ, params)
			GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)
			RuCt := RGSW.MultPack(uReEnc, ctR, evaluatorRGSW, ringQ, params)
			PrCt := RGSW.MultPack(rkCt_meas, ctP, evaluatorRGSW, ringQ, params)
			zCtPack = RLWE.Add(FzCt, GyCt, RuCt, params)
			ringQ.Add(zCtPack.Value[0], PrCt.Value[0], zCtPack.Value[0])
			ringQ.Add(zCtPack.Value[1], PrCt.Value[1], zCtPack.Value[1])

			// Send u to Python client
			var ubuf [16]byte
			binary.LittleEndian.PutUint64(ubuf[0:8], math.Float64bits(u[0]))
			binary.LittleEndian.PutUint64(ubuf[8:16], math.Float64bits(u[1]))
			if _, err := conn.Write(ubuf[:]); err != nil {
				fmt.Printf("[SERVER] Write error at step %d: %v\n", step, err)
				break
			}

			if step == 0 || (step+1)%50 == 0 {
				fmt.Printf("[CTRL] step %3d/%d  u=[%.3f, %.3f] V\n", step+1, nSteps, u[0], u[1])
			}
			step++
		}

		nValid := step
		fmt.Printf("[SERVER] Trial %d: %d/%d steps completed\n", iterNum, nValid, nSteps)

		// Zero out missing steps (partial trial handling)
		zeroCt := RLWE.Enc([]int64{0, 0}, 1/L, *encryptorRLWE, ringQ, params)
		for t := nValid; t < nSteps; t++ {
			yScalarStore[t] = cloneCtSlice(zeroCt)
			rScalarStore[t] = cloneCtSlice(zeroCt)
		}

		// ── Encrypted ILC update: Theta += K_r × (R - Y) ─────────────────
		// E_j stacked as pN scalar ciphertexts (same as before)
		fmt.Println("[ILC] Computing encrypted ILC update (SVD reduced space)...")
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

		// dTheta = K_r × E  (packed, n_r entries in tauSVD slots)
		dThetaCt := RGSW.MultPack(ctE, ctKr, evaluatorRGSW, ringQ, params)
		ringQ.Add(ThetaEncCt.Value[0], dThetaCt.Value[0], ThetaEncCt.Value[0])
		ringQ.Add(ThetaEncCt.Value[1], dThetaCt.Value[1], ThetaEncCt.Value[1])
		fmt.Printf("[ILC] Update done in %v\n", time.Since(tILC).Round(time.Millisecond))

		// Save updated Theta ciphertext
		if tData, err := ThetaEncCt.MarshalBinary(); err == nil {
			os.WriteFile(thetaPath, tData, 0644)
			fmt.Println("[ILC] Saved theta_ilc.bin →", thetaPath)
		}

		// Decrypt Theta and reconstruct V = S @ Theta  (plaintext)
		thetaVec = RLWE.DecUnpack(ThetaEncCt, nR, tauSVD, *decryptorRLWE, s*r*L, ringQ, params)
		vIlcVec = matVecMul(S, thetaVec)
		fmt.Printf("[ILC] Next feedforward: step0=[%.4f, %.4f] V\n", vIlcVec[0], vIlcVec[1])

		// Compute plaintext error statistics from logged data
		rmsP, rmsY := 0.0, 0.0
		nStat := nValid
		if nStat == 0 {
			nStat = 1
		}
		for t := 0; t < nValid; t++ {
			ep := rLog[t][0] - yLog[t][0]
			ey := rLog[t][1] - yLog[t][1]
			rmsP += ep * ep
			rmsY += ey * ey
		}
		rmsP = math.Sqrt(rmsP / float64(nStat))
		rmsY = math.Sqrt(rmsY / float64(nStat))
		fmt.Printf("[ILC] E_rms: Pitch=%.4f rad (%.2f°)  Yaw=%.4f rad (%.2f°)\n",
			rmsP, rmsP*180/math.Pi, rmsY, rmsY*180/math.Pi)

		// Send stats back to Python
		var statsBuf [16]byte
		binary.LittleEndian.PutUint64(statsBuf[0:8], math.Float64bits(rmsP))
		binary.LittleEndian.PutUint64(statsBuf[8:16], math.Float64bits(rmsY))
		conn.Write(statsBuf[:])

		conn.Close()
		fmt.Printf("=== Trial %d complete. Waiting for next connection... ===\n\n", iterNum)
	}
}
