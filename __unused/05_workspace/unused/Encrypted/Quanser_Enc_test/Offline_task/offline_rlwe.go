package main

import (
	"fmt"
	"log"
	"math"

	"Encrypted_Cartpole/com_utils"

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
	ctSize := int(90)

	// ============== Plant model ==============
	A := [][]float64{
		{0.998406460921939, 0, 0.00417376927758289, 0},
		{0, 0.998893625478993, 0, -0.00332671872292611},
		{0, 0, 0.995822899329324, 0},
		{0, 0, 0, 0.996671438596397},
	}
	B := [][]float64{
		{0.00831836513049678, 9.99686131895421e-06},
		{-5.19664522845810e-06, 0.00627777465144397},
		{0, 0.00477571210746992},
		{0.00311667643652227, 0},
	}
	C := [][]float64{
		{0.500000000000000, 0, 0, 0},
		{0, 0.500000000000000, 0, 0},
	}

	// input-output representation of controller obtained by conversion.m
	// transpose of vecHu, vecHy from conversion.m
	Hy := [][]float64{
		{0.334883269997112, -0.0993726952581632, 0.109105860257554, 0.340141173304891},
		{0.340715074862138, -0.101693452659005, 0.111263681570879, 0.346096102431116},
		{0.0212757993084255, -0.00721494759029773, 0.00717571762620109, 0.0215259945842975},
		{-0.705323732730193, 0.209355413587286, -0.230615165512593, -0.715776671026420},
	}
	Hu := [][]float64{
		{-0.285602015399616, -0.000307101965816320, 0.00106747945670671, -0.286337872976116},
		{0.183962668144521, -0.000156850543232820, 0.000585408816047406, 0.183342919294642},
		{0.464731844320360, -0.000717550250832144, 0.000183250207538066, 0.464698956437188},
		{0.631884279880355, -0.00124460838502882, -0.000477508261005455, 0.632382252336539},
	}

	// transpose of Yini from conversion.m
	yy0 := [][]float64{
		{-168.915339084001, 152.553129120773},
		{0, 0},
		{0, 0},
		{37.1009230518511, -33.8787596718866},
	}
	// transpose of Uini from conversion.m
	uu0 := [][]float64{
		{0, 0},
		{151.077820919228, -70.2395320362580},
		{90.8566491021641, -42.4186053244263},
		{54.6591007720606, -25.4768092703056},
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
