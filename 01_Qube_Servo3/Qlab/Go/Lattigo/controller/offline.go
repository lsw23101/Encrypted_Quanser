package main // <--- [중요] 실행 파일은 무조건 main 이어야 합니다!

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"

	fileutils "github.com/CDSL-EncryptedControl/CDSL/Lattigo/utils"

	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

// CryptoConfig는 모든 파일(controller.go, lib_crypto.go, plant/*.py)이
// 공유하는 암호문 파라미터입니다. offline.go 에서만 수정하면 됩니다.
type CryptoConfig struct {
	LogN    int     `json:"logN"`
	LogQ    []int   `json:"logQ"`
	LogP    []int   `json:"logP"`
	NTTFlag bool    `json:"nttFlag"`
	S       float64 `json:"s"`
	L       float64 `json:"L"`
	R       float64 `json:"r"`
	N       int     `json:"n"`
	M       int     `json:"m"`
	P       int     `json:"p"`
}

// scalRoundMat 은 행렬의 각 원소에 scalar 를 곱하고 반올림합니다 (양자화).
func scalRoundMat(scalar float64, m [][]float64) [][]float64 {
	result := make([][]float64, len(m))
	for i, row := range m {
		result[i] = make([]float64, len(row))
		for j, v := range row {
			result[i][j] = math.Round(scalar * v)
		}
	}
	return result
}

func main() {
	fmt.Println("--- Offline Phase ---")

	// ============================================================
	// [1] 파라미터 정의 (여기만 수정하면 모든 파일에 반영됩니다)
	// ============================================================
	cfg := CryptoConfig{
		LogN:    10,
		LogQ:    []int{60},
		LogP:    []int{60},
		NTTFlag: true,
		S:       1.0 / 1000.0,
		L:       1.0 / 1000000.0,
		R:       1.0 / 1000.0,
		N:       4,
		M:       1,
		P:       2,
	}

	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    cfg.LogN,
		LogQ:    cfg.LogQ,
		LogP:    cfg.LogP,
		NTTFlag: cfg.NTTFlag,
	})
	s, L, r := cfg.S, cfg.L, cfg.R
	n, m, p := cfg.N, cfg.M, cfg.P

	fmt.Printf("Params: LogN=%d, 1/r=%.1f, 1/s=%.1f, 1/L=%.1f\n", params.LogN(), 1/r, 1/s, 1/L)

	// ============================================================
	// [2] Controller 행렬 (MATLAB 설계값, 양자화 전 실수 행렬)
	// F 는 정수 행렬이므로 그대로 사용합니다.
	// G, H, R 은 MATLAB 값을 그대로 넣으면 s 에 맞춰 자동 양자화됩니다.
	// ============================================================
	F := [][]float64{
		{0.0000, -0.0000, -0.0000, -0.0000},
		{1.0000, 0.0000, 0.0000, -2.0000},
		{-0.0000, 1.0000, 0.0000, 1.0000},
		{0.0000, -0.0000, 1.0000, 2.0000},
	}

	// ↓↓↓ MATLAB 에서 구한 실수 행렬을 여기에 입력하세요 ↓↓↓
	G := [][]float64{
		{1.0, -3.016},
		{0.0, -13.073},
		{0.0, -6.867},
		{0.0, 6.544},
	}
	H := [][]float64{
		{-18.810, 7.081, -39.702, -25.616},
	}
	R := [][]float64{
		{-0.025},
		{1.507},
		{0.769},
		{-0.729},
	}
	// ↑↑↑ ↑↑↑ ↑↑↑

	// 양자화: Bar = round(원본 / s)
	GBar := scalRoundMat(1/s, G)
	HBar := scalRoundMat(1/s, H)
	RBar := scalRoundMat(1/s, R)
	fmt.Printf("GBar[0] = %v\n", GBar[0])

	x_ini := []float64{0.0, 0.0, 0.0, 0.0}

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	logn := int(math.Log2(float64(tau)))
	galEls := make([]uint64, logn)
	for i := 0; i < logn; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	gks := kgen.GenGaloisKeysNew(galEls, sk)

	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	ringQ := params.RingQ()
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1

	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))
	xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

	// ============================================================
	// [3] 저장
	// ============================================================
	saveDir := "enc_data"
	if err := fileutils.EnsureDir(saveDir); err != nil {
		panic(err)
	}
	fmt.Printf(">> Saving data to '%s'...\n", saveDir)

	// [3a] params.json 저장 (controller.go, lib_crypto.go, plant/*.py 가 읽음)
	jsonData, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		panic(err)
	}
	if err := os.WriteFile(filepath.Join(saveDir, "params.json"), jsonData, 0644); err != nil {
		panic(err)
	}
	fmt.Println(">> Saved enc_data/params.json")

	// [3b] 키
	if err := fileutils.WriteWT(filepath.Join(saveDir, "sk.dat"), sk); err != nil {
		panic(err)
	}
	if err := fileutils.WriteWT(filepath.Join(saveDir, "rlk.dat"), rlk); err != nil {
		panic(err)
	}
	if err := fileutils.SaveGaloisKeys(saveDir, gks, galEls); err != nil {
		panic(err)
	}

	// [3c] 암호문
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
	if err := fileutils.WriteWT(filepath.Join(saveDir, "xCtPack.dat"), xCtPack); err != nil {
		panic(err)
	}

	// ============================================================
	// [4] crypto 공유 라이브러리 재빌드
	// ============================================================
	fmt.Println(">> Rebuilding crypto shared library...")

	wd, err := os.Getwd()
	if err != nil {
		panic(err)
	}
	// offline.go 는 controller/ 에서 실행됩니다. 부모(Lattigo/)의 crypto/ 폴더를 빌드합니다.
	cryptoDir := filepath.Join(filepath.Dir(wd), "crypto")

	outFile := "client_crypto.so"
	if runtime.GOOS == "windows" {
		outFile = "client_crypto.dll"
	}

	buildCmd := exec.Command("go", "build", "-o", outFile, "-buildmode=c-shared", "lib_crypto.go")
	buildCmd.Dir = cryptoDir
	buildCmd.Stdout = os.Stdout
	buildCmd.Stderr = os.Stderr
	if err := buildCmd.Run(); err != nil {
		panic("Crypto library build failed: " + err.Error())
	}
	fmt.Printf(">> Crypto library rebuilt: %s/%s\n", cryptoDir, outFile)

	fmt.Println("--- Offline Phase Complete! ---")
}
