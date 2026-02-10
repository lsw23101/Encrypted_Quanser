package main

import (
	"fmt"
	"log"
	"math"

	"Quanser_Enc_test/com_utils"

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
	ptSize := uint64(28)
	// Choose the size of ciphertext modulus (2^ctSize)
	ctSize := int(74)

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

	encoder := bgv.NewEncoder(params)

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

	// 저장
	for i := 0; i < n; i++ {
		err := com_utils.WriteToFile(ctHu[i], fmt.Sprintf("ctHu[%d].dat", i))
		if err != nil {
			log.Fatalf("ctHu 저장 실패: %v", err)
		}
		err = com_utils.WriteToFile(ctHy[i], fmt.Sprintf("ctHy[%d].dat", i))
		if err != nil {
			log.Fatalf("ctHy 저장 실패: %v", err)
		}
		err = com_utils.WriteToFile(ctU[i], fmt.Sprintf("ctU[%d].dat", i))
		if err != nil {
			log.Fatalf("ctU 저장 실패: %v", err)
		}
		err = com_utils.WriteToFile(ctY[i], fmt.Sprintf("ctY[%d].dat", i))
		if err != nil {
			log.Fatalf("ctY 저장 실패: %v", err)
		}
	}

	// 비밀키 저장
	err := com_utils.WriteToFile(sk, "sk.dat")
	if err != nil {
		log.Fatalf("비밀키 저장 실패: %v", err)
	}

}
