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
	fmt.Println("--- Offline Phase ---")

	// RLWE params
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    12,
		LogQ:    []int{60},
		LogP:    []int{60},
		NTTFlag: true,
	})

	// Quantization params
	s := 1 / 1000.0
	L := 1 / 1000000.0
	r := 1 / 1000.0
	fmt.Printf("Params: LogN=%d, 1/r=%.1f, 1/s=%.1f, 1/L=%.1f\n", params.LogN(), 1/r, 1/s, 1/L)

	// Contoller
	// F (integer matrix !)
	F := [][]float64{
		{0.0000, -0.0000, -0.0000, -0.0000},
		{1.0000, 0.0000, 0.0000, -2.0000},
		{-0.0000, 1.0000, 0.0000, 1.0000},
		{0.0000, -0.0000, 1.0000, 2.0000},
	}

	GBar := [][]float64{
		{1000, -3016},
		{-0, -13073},
		{0, -6867},
		{0, 6544},
	}

	HBar := [][]float64{
		{-18810, 7081, -39702, -25616},
	}

	RBar := [][]float64{
		{-25},
		{1507},
		{769},
		{-729},
	}

	// Controller Initial State
	x_ini := []float64{0.0, 0.0, 0.0, 0.0}

	// Dimensions input:m output:p state:n
	n := 4
	m := 1
	p := 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

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

	// Scaling (CDSL library)
	// GBar := utils.ScalMatMult(1/s, G)
	// HBar := utils.ScalMatMult(1/s, H)
	// RBar := utils.ScalMatMult(1/s, R)

	// RGSW Encryption
	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	// Initial State Encryption
	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))
	xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

	// save directory for offline data
	saveDir := "enc_data"

	// [변경] 여기서부터는 fileutils (우리가 만든 파일 유틸)를 사용합니다.
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

	// Ciphertexts
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

	// Initial State
	if err := fileutils.WriteWT(filepath.Join(saveDir, "xCtPack.dat"), xCtPack); err != nil {
		panic(err)
	}

	fmt.Println("--- Offline Phase Complete! ---")
}
