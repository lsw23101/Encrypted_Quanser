package main

import (
	"fmt"
	"math"
	"time"

	"github.com/CDSL-EncryptedControl/CDSL/utils"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
	"github.com/tuneinsight/lattigo/v6/schemes/bgv"
)

func main() {
	// *****************************************************************
	// ************************* User's choice *************************
	// *****************************************************************
	// ============== Encryption parameters ==============
	// Refer to ``Homomorphic encryption standard''

	// log2 of polynomial degree
	logN := 12
	// Choose the size of plaintext modulus (2^ptSize)
	ptSize := uint64(50)
	// Choose the size of ciphertext modulus (2^ctSize)
	ctSize := int(120)

	// ============== Plant model ==============
	A := [][]float64{
		{1, 0.0301, 0.02, 0.0002},
		{0,    1.0528,   -0.0000,    0.0204},
		{0,    3.0375,    0.9998,    0.0301},
		{0,    5.3236,   -0.0002,    1.0528},
	}
	B := [][]float64{
		{0.0100},
		{0.0099},
		{1.0043},
		{1.0001},
	}
	C := [][]float64{
		{1, 0, 0, 0},
		{0, 1, 0, 0},
	}

	// Plant initial state
	xp0 := []float64{
		0.1,
		0.1,
		0.1,
		0.1,
	}

	// ============== Pre-designed controller ==============
	// ============== Pre-designed controller ==============
	F := [][]float64{
		{-0.2886, -0.2902, 0.0318, -0.0202},
		{ 0.0393, -0.6374, 0.0117,  0.0001},
		{-17.9294, -27.1040, 2.1893, -2.0234},
		{ 4.1026, -50.3645, 1.1844, -0.9922},
	}
	G := [][]float64{
		{1.3303, 0.0732},
		{0.0021, 1.4450},
		{22.1180, 5.3075},
		{0.0685, 30.9571},
	}
	H := [][]float64{
		{4.1707, -24.7282, 1.1845, -2.0448},
	}


	// input-output representation of controller obtained by conversion.m
	// transpose of vecHu, vecHy from conversion.m
	Hu := [][]float64{
    	{-0.0498, 0},
    	{ 0.2513, 0},
    	{-0.2869, 0},
    	{ 0.2710, 0},
	}

	Hy := [][]float64{
		{-3.1796,  7.7947},
		{22.8571, -57.1848},
		{-50.4053, 132.1963},
		{31.5550, -92.4417},
	}
	// Controller initial state
	xc0 := []float64{
		0,
		0,
		0,
		0,
	}
	// transpose of Yini from conversion.m
	yy0 := [][]float64{
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
	}
	// transpose of Uini from conversion.m
	uu0 := [][]float64{
		{0},
		{0},
		{0},
		{0},
	}
	// ============== Quantization parameters ==============
	r := 0.00020
	s := 0.00010
	fmt.Println("Scaling parameters 1/r:", 1/r, "1/s:", 1/s)
	// *****************************************************************
	// *****************************************************************

	// ============== Encryption settings ==============
	// Search a proper prime to set plaintext modulus
	primeGen := ring.NewNTTFriendlyPrimesGenerator(ptSize, uint64(math.Pow(2, float64(logN)+1)))
	ptModulus, _ := primeGen.NextAlternatingPrime()
	fmt.Println("Plaintext modulus:", ptModulus)

	// Create a chain of ciphertext modulus
	logQ := []int{int(math.Floor(float64(ctSize) * 0.5)), int(math.Ceil(float64(ctSize) * 0.5))}

	// Parameters satisfying 128-bit security
	// BGV scheme is used
	params, _ := bgv.NewParametersFromLiteral(bgv.ParametersLiteral{
		LogN:             logN,
		LogQ:             logQ,
		PlaintextModulus: ptModulus,
	})
	fmt.Println("Ciphertext modulus:", params.QBigInt())
	fmt.Println("Degree of polynomials:", params.N())

	// Generate secret key
	kgen := bgv.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()

	encryptor := bgv.NewEncryptor(params, sk)
	decryptor := bgv.NewDecryptor(params, sk)
	encoder := bgv.NewEncoder(params)
	eval := bgv.NewEvaluator(params, nil)

	bredparams := ring.GenBRedConstant(params.PlaintextModulus())

	// ==============  Encryption of controller ==============
	// dimensions
	n := len(A)
	l := len(C)
	m := len(B[0])
	h := int(math.Max(float64(l), float64(m)))

	// duplicate
	yy0vec := make([][]float64, n)
	uu0vec := make([][]float64, n)
	for i := 0; i < n; i++ {
		yy0vec[i] = utils.VecDuplicate(yy0[i], m, h)
		uu0vec[i] = utils.VecDuplicate(uu0[i], m, h)
	}

	// Plaintext of past inputs and outputs
	ptY := make([]*rlwe.Plaintext, n)
	ptU := make([]*rlwe.Plaintext, n)
	// Plaintext of control parameters
	ptHy := make([]*rlwe.Plaintext, n)
	ptHu := make([]*rlwe.Plaintext, n)
	// Ciphertext of past inputs and outputs
	ctY := make([]*rlwe.Ciphertext, n)
	ctU := make([]*rlwe.Ciphertext, n)
	// Ciphertext of control parameters
	ctHy := make([]*rlwe.Ciphertext, n)
	ctHu := make([]*rlwe.Ciphertext, n)

	// Quantization - packing - encryption
	for i := 0; i < n; i++ {
		ptY[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, yy0vec[i])), params.PlaintextModulus()), ptY[i])
		ctY[i], _ = encryptor.EncryptNew(ptY[i])

		ptU[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, uu0vec[i])), params.PlaintextModulus()), ptU[i])
		ctU[i], _ = encryptor.EncryptNew(ptU[i])

		ptHy[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/s, Hy[i])), params.PlaintextModulus()), ptHy[i])
		ctHy[i], _ = encryptor.EncryptNew(ptHy[i])

		ptHu[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/s, Hu[i])), params.PlaintextModulus()), ptHu[i])
		ctHu[i], _ = encryptor.EncryptNew(ptHu[i])
	}

	// ============== Simulation ==============
	// Number of simulation steps
	iter := 500
	fmt.Printf("Number of iterations: %v\n", iter)

	// 1) Plant + unencrypted (original) controller
	// Data storage
	yUnenc := [][]float64{}
	uUnenc := [][]float64{}
	xpUnenc := [][]float64{}
	xcUnenc := [][]float64{}

	xpUnenc = append(xpUnenc, xp0)
	xcUnenc = append(xcUnenc, xc0)

	// Plant state
	xp := xp0
	// Controller state
	xc := xc0

	for i := 0; i < iter; i++ {
		y := utils.MatVecMult(C, xp)
		u := utils.MatVecMult(H, xc)
		xp = utils.VecAdd(utils.MatVecMult(A, xp), utils.MatVecMult(B, u))
		xc = utils.VecAdd(utils.MatVecMult(F, xc), utils.MatVecMult(G, y))

		yUnenc = append(yUnenc, y)
		uUnenc = append(uUnenc, u)
		xpUnenc = append(xpUnenc, xp)
		xcUnenc = append(xcUnenc, xc)
	}

	// 2) Plant + encrypted controller

	// To save data
	yEnc := [][]float64{}
	uEnc := [][]float64{}
	xpEnc := [][]float64{}
	xpEnc = append(xpEnc, xp0)

	// Plant state
	xp = xp0

	// For time check
	period := make([][]float64, iter)
	startPeriod := make([]time.Time, iter)

	for i := 0; i < iter; i++ {
		// **** Sensor ****
		// Plant output
		Y := utils.MatVecMult(C, xp) // [][]float64

		startPeriod[i] = time.Now()

		// Quantize and duplicate
		Ysens := utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(Y, m, h))), params.PlaintextModulus())
		Ypacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(Ysens, Ypacked)
		Ycin, _ := encryptor.EncryptNew(Ypacked)

		// **** Encrypted controller ****
		Uout, _ := eval.MulNew(ctHy[0], ctY[0])
		eval.MulThenAdd(ctHu[0], ctU[0], Uout)
		for j := 1; j < n; j++ {
			eval.MulThenAdd(ctHy[j], ctY[j], Uout)
			eval.MulThenAdd(ctHu[j], ctU[j], Uout)
		}

		// **** Actuator ****
		// Plant input
		U := make([]float64, m)
		// Unpacked and re-scaled u at actuator
		Uact := make([]uint64, params.N())
		// u after inner sum
		Usum := make([]uint64, m)
		encoder.Decode(decryptor.DecryptNew(Uout), Uact)
		// Generate plant input
		for k := 0; k < m; k++ {
			Usum[k] = utils.VecSumUint(Uact[k*h:(k+1)*h], params.PlaintextModulus(), bredparams)
			U[k] = float64(r * s * utils.SignFloat(float64(Usum[k]), params.PlaintextModulus()))
		}
		// Re-encryption
		Upacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(U, m, h))), params.PlaintextModulus()), Upacked)
		Ucin, _ := encryptor.EncryptNew(Upacked)

		// **** Encrypted Controller ****
		// State update
		ctY = append(ctY[1:], Ycin)
		ctU = append(ctU[1:], Ucin)

		period[i] = []float64{float64(time.Since(startPeriod[i]).Microseconds()) / 1000}

		// **** Plant ****
		// State update
		xp = utils.VecAdd(utils.MatVecMult(A, xp), utils.MatVecMult(B, U))

		// Save data
		yEnc = append(yEnc, Y)
		uEnc = append(uEnc, U)
		xpEnc = append(xpEnc, xp)
	}

	avgPeriod := utils.Average(utils.MatToVec(period))
	fmt.Println("Average elapsed time for a control period:", avgPeriod, "ms")

	// Compare plant input between 1) and 2)
	uDiff := make([][]float64, iter)
	for i := range uDiff {
		uDiff[i] = []float64{utils.Vec2Norm(utils.VecSub(uUnenc[i], uEnc[i]))}
	}

	// =========== Export data ===========

	// Plant state equipped with encrypted controller
	utils.DataExport(xpEnc, "./RLWE_data/state.csv")

	// Plant intput from encrypted controller
	utils.DataExport(uEnc, "./RLWE_data/uEnc.csv")

	// Plant output with encrypted controller
	utils.DataExport(yEnc, "./RLWE_data/yEnc.csv")

	// Performance of encrypted controller
	utils.DataExport(uDiff, "./RLWE_data/uDiff.csv")

	// Elapsed time
	utils.DataExport(period, "./RLWE_data/period.csv")

}
