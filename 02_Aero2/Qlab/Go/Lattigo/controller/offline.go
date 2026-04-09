package main // <--- [중요] 실행 파일은 무조건 main 이어야 합니다!

import (
	"fmt"
	"math"
	"path/filepath"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"

	fileutils "github.com/CDSL-EncryptedControl/CDSL/Lattigo/utils"

	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

func main() {
	fmt.Println("--- Offline Phase (Aero2) ---")

	// RLWE params
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    12,
		LogQ:    []int{60},
		LogP:    []int{60},
		NTTFlag: true,
	})

	// Quantization params
	// y [rad], u [V] → 각각 1/r, 1/s 로 스케일
	// xBar = round(x / (r*s)),  GBar = round(G/s),  HBar = round(H/s)
	// RBar = round(R/s),        PBar = round(P/s)
	// N_bar 는 평문으로 처리 (암호화 불필요)
	s := 1 / 1000.0
	L := 1 / 1000000.0
	r := 1 / 1000.0
	fmt.Printf("Params: LogN=%d, 1/r=%.0f, 1/s=%.0f, 1/L=%.0f\n", params.LogN(), 1/r, 1/s, 1/L)

	// ── Aero2 controller matrices ──────────────────────────────────────────
	// Dimensions: state n=4, input m=2, output p=2
	// z(k+1) = F*z + G*y + R*u + P*r
	// u(k)   = H*z          (N_bar*r 는 평문으로 plant 또는 controller에서 가산)

	// F: 4×4 nilpotent integer (스케일링 불필요)
	F := [][]float64{
		{0, 0, 1, 0},
		{0, 0, 0, 1},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
	}

	// GBar = round(G / s) = round(G × 1000)  [4×2]
	GBar := [][]float64{
		{2232, 341},
		{220, 3159},
		{-2166, -319},
		{-202, -3024},
	}

	// HBar = round(H / s) = round(H × 1000)  [2×4]
	HBar := [][]float64{
		{-48570, 31205, 0, 0},
		{-30380, -22353, 0, 0},
	}

	// RBar = round(R / s) = round(R × 1000)  [4×2]
	RBar := [][]float64{
		{-17, -22},
		{21, -33},
		{6, 8},
		{-8, 12},
	}

	// PBar = round(P / s) = round(P × 1000)  [4×2]
	PBar := [][]float64{
		{2363, 576},
		{455, 3422},
		{-915, -220},
		{-174, -1292},
	}

	// Controller Initial State
	x_ini := []float64{0.0, 0.0, 0.0, 0.0}

	// Dimensions
	n := 4
	m := 2
	p := 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))
	fmt.Printf("n=%d, m=%d, p=%d → tau=%d\n", n, m, p, tau)

	// Galois Elements
	logn := int(math.Log2(float64(tau)))
	galEls := make([]uint64, logn)
	for i := 0; i < logn; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// Key Generation
	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	gks := kgen.GenGaloisKeysNew(galEls, sk)

	// Encryptors
	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	ringQ := params.RingQ()
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1

	// RGSW Encryption
	fmt.Println(">> Encrypting matrices...")
	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctP := RGSW.EncPack(PBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	// Initial State Encryption
	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))
	xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

	// Save directory
	saveDir := "enc_data"
	if err := fileutils.EnsureDir(saveDir); err != nil {
		panic(err)
	}
	fmt.Printf(">> Saving data to '%s'...\n", saveDir)

	// Keys
	if err := fileutils.WriteWT(filepath.Join(saveDir, "sk.dat"), sk); err != nil {
		panic(err)
	}
	if err := fileutils.WriteWT(filepath.Join(saveDir, "rlk.dat"), rlk); err != nil {
		panic(err)
	}
	if err := fileutils.SaveGaloisKeys(saveDir, gks, galEls); err != nil {
		panic(err)
	}

	// Encrypted matrices
	if err := fileutils.SaveRGSWPack(saveDir, "ctF", ctF); err != nil {
		panic(err)
	}
	if err := fileutils.SaveRGSWPack(saveDir, "ctG", ctG); err != nil {
		panic(err)
	}
	if err := fileutils.SaveRGSWPack(saveDir, "ctH", ctH); err != nil {
		panic(err)
	}
	if err := fileutils.SaveRGSWPack(saveDir, "ctR", ctR); err != nil {
		panic(err)
	}
	if err := fileutils.SaveRGSWPack(saveDir, "ctP", ctP); err != nil {
		panic(err)
	}

	// Initial State
	if err := fileutils.WriteWT(filepath.Join(saveDir, "xCtPack.dat"), xCtPack); err != nil {
		panic(err)
	}

	fmt.Println("--- Offline Phase Complete! ---")
}
