package main // <--- [중요] 실행 파일은 무조건 main 이어야 합니다!

import (
	"fmt"
	"math"
	"path/filepath"

	// 1. 기존 CDSL 라이브러리 유틸 (수학 연산 등) -> "utils"라는 이름으로 사용
	utils "github.com/CDSL-EncryptedControl/CDSL/utils"

	// 2. [추가] 우리가 방금 만든 파일 저장용 유틸 -> "fileutils"라는 별명으로 사용!
	// (폴더 경로가 python_test/utils 라고 가정)
	fileutils "github.com/CDSL-EncryptedControl/CDSL/python_test/utils"

	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

func main() {
	fmt.Println("--- [Offline Phase] Generating Keys & Encrypting Matrices ---")

	// 1. 파라미터 설정
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    12,
		LogQ:    []int{56},
		LogP:    []int{51},
		NTTFlag: true,
	})

	// Quantization parameters
	s := 1 / 1000.0
	L := 1 / 10000.0
	r := 1 / 1000.0
	fmt.Printf("Params: LogN=%d, 1/r=%.1f, 1/s=%.1f, 1/L=%.1f\n", params.LogN(), 1/r, 1/s, 1/L)

	// 2. 제어기 모델
	// F Matrix (4x4) - 기존과 동일
	F := [][]float64{
		{0.0000, -0.0000, -0.0000, -0.0000},
		{1.0000, 0.0000, 0.0000, -2.0000},
		{-0.0000, 1.0000, 0.0000, 1.0000},
		{0.0000, -0.0000, 1.0000, 2.0000},
	}

	G := [][]float64{
		{1.0000, -2.5334},
		{0.0000, -9.9067},
		{-0.0000, -5.2223},
		{0.0000, 4.9602},
	}

	H := [][]float64{
		{32.6937, -12.3145, 69.0290, 44.5420},
	}

	R := [][]float64{
		{0.0173},
		{-1.0184},
		{-0.5241},
		{0.4951},
	}

	// Initial State (4x1) - plant.py의 초기값과 맞춰줌 (예: 각도만 0.1)
	x_ini := []float64{0.0, 0.0, 0.0, 0.0}

	// 차원 계산
	// n := len(F)
	// m := len(H)
	// p := len(G[0])
	n := 4
	m := 1 // <-- 여기가 1이어야 함
	p := 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// Galois Elements 계산
	logn := int(math.Log2(float64(tau)))
	galEls := make([]uint64, logn)
	for i := 0; i < logn; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// 3. 키 생성
	fmt.Println(">> Generating Keys...")
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

	// 4. 행렬 암호화
	fmt.Println(">> Encrypting Matrices (This may take a while)...")

	// Scaling Matrices (여기 utils는 CDSL 라이브러리)
	GBar := utils.ScalMatMult(1/s, G)
	HBar := utils.ScalMatMult(1/s, H)
	RBar := utils.ScalMatMult(1/s, R)

	// RGSW Encryption
	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	// 5. 초기 상태 암호화
	fmt.Println(">> Encrypting Initial State...")
	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))
	xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

	// 6. 파일 저장
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
