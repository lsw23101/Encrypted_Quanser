package main

import (
	"Encrypted_Cartpole/com_utils"
	"fmt"
	"log"
	"math"
	"path/filepath"
	"time"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

func main() {
	// ================= 1) Encryption parameters =================
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    11,
		LogQ:    []int{28},
		LogP:    []int{28},
		NTTFlag: true,
	})
	fmt.Println("Degree of polynomials:", params.N())
	fmt.Println("Ciphertext modulus:", params.QBigInt())
	fmt.Println("Special modulus:", params.PBigInt())
	fmt.Println("Secret key distribution (Ternary):", params.Xs())
	fmt.Println("Error distribution (Discrete Gaussian):", params.Xe())

	// ================= Controller (PID-based) =================
	const (
		Kp = 32.0
		Ki = 2.5
		Kd = 40.0

		Lp = 30.0
		Li = 0.1
		Ld = 3.0
	)

	F := [][]float64{
		{1, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 0},
	}
	G := [][]float64{
		{1, 0},
		{1, 0},
		{0, 1},
		{0, 1},
	}
	H := [][]float64{
		{Ki, -Kd, Li, -Ld},
	}
	R := [][]float64{
		{0, 0},
	}
	J := [][]float64{
		{Kp + Ki + Kd, Lp + Li + Ld},
	}

	// Controller initial state
	x_ini := []float64{0, 0, 0, 0}

	// Dimensions
	n := len(F)
	m := len(H)
	p := len(G[0])

	// ================= 2) Quantization parameters =================
	s := 1 / 5.0
	L := 1 / 300.0
	r := 1 / 50.0
	fmt.Printf("Scaling parameters 1/L: %v, 1/s: %v, 1/r: %v\n", 1/L, 1/s, 1/r)

	// ================= 3) Rings / aux =================
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	_ = levelQ
	_ = levelP

	ringQ := params.RingQ()

	// tau, monomials (for RLWE pack/unpack)
	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))
	logn := int(math.Log2(float64(tau)))

	monomials := make([]ring.Poly, logn)
	for i := 0; i < logn; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}

	// Galois elements (for rotations used in Unpack)
	galEls := make([]uint64, logn)
	for i := 0; i < logn; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// ================= 4) KeyGen & encryptors/evaluators =================
	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	gks := kgen.GenGaloisKeysNew(galEls, sk)

	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	// decryptorRLWE := rlwe.NewDecryptor(params, sk) // <- 사용 안 하므로 제거
	encryptorRGSW := rgsw.NewEncryptor(params, sk)

	// ================= 5) Encrypt controller matrices =================
	GBar := utils.ScalMatMult(1/s, G)
	HBar := utils.ScalMatMult(1/s, H)
	RBar := utils.ScalMatMult(1/s, R)
	JBar := utils.ScalMatMult(1/(s*s), J)

	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params) // 사용 안 하지만 저장은 함
	ctJ := RGSW.EncPack(JBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	_ = ctR

	// initial state x (packed RLWE)
	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))
	xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

	// Zero ct
	zeroCt := rlwe.NewCiphertext(params, 1)

	// ================= 6) SAVE all artifacts =================
	base := filepath.Join("enc_data", "rgsw_for_N11")
	if err := com_utils.EnsureDir(base); err != nil {
		log.Fatal(err)
	}

	// ciphertexts
	if err := com_utils.WriteWT(filepath.Join(base, "xCtPack.dat"), xCtPack); err != nil {
		log.Fatalf("save xCtPack failed: %v", err)
	}
	if err := com_utils.SaveRGSWPack(base, "ctF", ctF); err != nil {
		log.Fatal(err)
	}
	if err := com_utils.SaveRGSWPack(base, "ctG", ctG); err != nil {
		log.Fatal(err)
	}
	if err := com_utils.SaveRGSWPack(base, "ctH", ctH); err != nil {
		log.Fatal(err)
	}
	if err := com_utils.SaveRGSWPack(base, "ctR", ctR); err != nil {
		log.Fatal(err)
	}
	if err := com_utils.SaveRGSWPack(base, "ctJ", ctJ); err != nil {
		log.Fatal(err)
	}

	// keys
	if err := com_utils.WriteWT(filepath.Join(base, "rlk.dat"), rlk); err != nil {
		log.Fatalf("save rlk failed: %v", err)
	}
	for i, gk := range gks {
		fn := filepath.Join(base, fmt.Sprintf("gk_%d.dat", galEls[i]))
		if err := com_utils.WriteWT(fn, gk); err != nil {
			log.Fatalf("save gk(%d) failed: %v", galEls[i], err)
		}
	}
	if err := com_utils.WriteWT(filepath.Join(base, "sk.dat"), sk); err != nil {
		log.Fatalf("save sk failed: %v", err)
	}
	fmt.Println("[SAVE] saved to", base)

	// ================= 7) LOAD artifacts as recovered_* =================
	var err error

	recoveredX := new(rlwe.Ciphertext)
	if err = com_utils.ReadRT(filepath.Join(base, "xCtPack.dat"), recoveredX); err != nil {
		log.Fatalf("load xCtPack failed: %v", err)
	}

	var recoveredF, recoveredG, recoveredH, recoveredJ []*rgsw.Ciphertext

	recoveredF, err = com_utils.LoadRGSWPack(base, "ctF")
	if err != nil {
		log.Fatalf("load ctF failed: %v", err)
	}

	recoveredG, err = com_utils.LoadRGSWPack(base, "ctG")
	if err != nil {
		log.Fatalf("load ctG failed: %v", err)
	}

	recoveredH, err = com_utils.LoadRGSWPack(base, "ctH")
	if err != nil {
		log.Fatalf("load ctH failed: %v", err)
	}

	// R은 사용 안 하므로 읽지 않아도 됨 (필요하면 아래처럼 버리기)
	// _, _ = com_utils.LoadRGSWPack(base, "ctR")

	recoveredJ, err = com_utils.LoadRGSWPack(base, "ctJ")
	if err != nil {
		log.Fatalf("load ctJ failed: %v", err)
	}

	recoveredRlk := new(rlwe.RelinearizationKey)
	if err = com_utils.ReadRT(filepath.Join(base, "rlk.dat"), recoveredRlk); err != nil {
		log.Fatalf("load rlk failed: %v", err)
	}
	recoveredGks, err := com_utils.LoadGaloisKeys(base)
	if err != nil {
		log.Fatalf("load gk_* failed: %v", err)
	}

	recoveredSk := new(rlwe.SecretKey)
	if err = com_utils.ReadRT(filepath.Join(base, "sk.dat"), recoveredSk); err != nil {
		log.Fatalf("load sk failed: %v", err)
	}

	// ================= 8) Rebuild evaluators/cryptors from recovered keys =================
	evkRGSW2 := rlwe.NewMemEvaluationKeySet(recoveredRlk)
	evkRLWE2 := rlwe.NewMemEvaluationKeySet(recoveredRlk, recoveredGks...)
	evaluatorRGSW2 := rgsw.NewEvaluator(params, evkRGSW2)
	evaluatorRLWE2 := rlwe.NewEvaluator(params, evkRLWE2)
	encryptorRLWE2 := rlwe.NewEncryptor(params, recoveredSk)
	decryptorRLWE2 := rlwe.NewDecryptor(params, recoveredSk)

	// ================= 9) Simulation: baseline (unencrypted) =================
	iter := 500
	fmt.Printf("Number of iterations: %v\n", iter)

	yUnenc := [][]float64{}
	uUnenc := [][]float64{}
	xcUnenc := [][]float64{}

	x := append([]float64(nil), x_ini...)
	for i := 0; i < iter; i++ {
		y := []float64{-0.5, 0.5}
		u := utils.VecAdd(
			utils.MatVecMult(H, x),
			utils.MatVecMult(J, y),
		)
		x = utils.VecAdd(utils.MatVecMult(F, x), utils.MatVecMult(G, y))

		yUnenc = append(yUnenc, y)
		uUnenc = append(uUnenc, u)
		xcUnenc = append(xcUnenc, x)
	}

	// ================= 10) Simulation: encrypted with recovered_* =================
	yEnc := [][]float64{}
	uEnc := [][]float64{}
	period := make([][]float64, iter)
	startPeriod := make([]time.Time, iter)

	// start from recovered packed state
	// xCtPack := recoveredX

	for i := 0; i < iter; i++ {
		y := []float64{-0.5, 0.5}
		startPeriod[i] = time.Now()

		// Encrypt y with recovered secret key
		yBar := utils.RoundVec(utils.ScalVecMult(1/r, y))
		yCtPack := RLWE.EncPack(yBar, tau, 1/L, *encryptorRLWE2, ringQ, params)

		// Unpack state and input
		xCt := RLWE.UnpackCt(xCtPack, n, tau, evaluatorRLWE2, ringQ, monomials, params)
		yCt := RLWE.UnpackCt(yCtPack, p, tau, evaluatorRLWE2, ringQ, monomials, params)

		// Controller output: u = Hx + Jy
		uCtPack := RGSW.MultPack(xCt, recoveredH, evaluatorRGSW2, ringQ, params)
		JyCt := RGSW.MultPack(yCt, recoveredJ, evaluatorRGSW2, ringQ, params)
		uCtPack = RLWE.Add(uCtPack, JyCt, zeroCt, params)

		// Decrypt output
		u := RLWE.DecUnpack(uCtPack, m, tau, *decryptorRLWE2, r*s*s*L, ringQ, params)

		// State update: x = F*x + G*y
		FxCt := RGSW.MultPack(xCt, recoveredF, evaluatorRGSW2, ringQ, params)
		GyCt := RGSW.MultPack(yCt, recoveredG, evaluatorRGSW2, ringQ, params)
		xCtPack = RLWE.Add(FxCt, GyCt, zeroCt, params)

		period[i] = []float64{float64(time.Since(startPeriod[i]).Nanoseconds()) / 1e6}

		yEnc = append(yEnc, y)
		uEnc = append(uEnc, u)
	}

	avgPeriod := utils.Average(utils.MatToVec(period))
	fmt.Println("Average elapsed time for a control period (recovered):", avgPeriod, "ms")

	// ================= 11) Compare =================
	uDiff := make([][]float64, iter)
	for i := range uDiff {
		uDiff[i] = []float64{utils.Vec2Norm(utils.VecSub(uUnenc[i], uEnc[i]))}
	}

	// // Optional CSV export
	utils.DataExport(uUnenc, "./uUnenc.csv")
	utils.DataExport(uEnc, "./uEnc.csv")
	utils.DataExport(uDiff, "./uDiff.csv")
	// utils.DataExport(period, "./period.csv")
}
