package main

// 데이터 받고 암호화 해서 보내기

import (
	"Quanser_Enc_test/com_utils"
	"fmt"
	"math"
	"net"
	"os"
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

	// // input-output representation of controller obtained by conversion.m
	// // transpose of vecHu, vecHy from conversion.m
	// Hu := [][]float64{
    // 	{-0.0498, 0},
    // 	{ 0.2513, 0},
    // 	{-0.2869, 0},
    // 	{ 0.2710, 0},
	// }

	// Hy := [][]float64{
	// 	{-3.1796,  7.7947},
	// 	{22.8571, -57.1848},
	// 	{-50.4053, 132.1963},
	// 	{31.5550, -92.4417},
	// }



	// Plant initial state
	xp0 := []float64{
		0.01,
		0.01,
		0.01,
		0.01,
	}

	// transpose of Yini from conversion.m
	// 초기값 시퀀스 이 부분도 암호화 해서 넘겨줬기 때문에 정리 가능해보임

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

	// 미리 만들어둔 sk 를 사용할 예정이므로 여기서 sk 생성은 주석처리
	// Generate secret key
	// kgen := bgv.NewKeyGenerator(params)
	// sk := kgen.GenSecretKeyNew()

	// 비어있는 sk 객체 생성 후 데이터 읽기
	sk := rlwe.NewSecretKey(params)
	com_utils.ReadFromFile("sk.dat", sk)

	// 암호화, 패킹에 필요한 객체
	encryptor := bgv.NewEncryptor(params, sk)
	decryptor := bgv.NewDecryptor(params, sk)
	encoder := bgv.NewEncoder(params)

	bredparams := ring.GenBRedConstant(params.PlaintextModulus())

	// ARX controller 와 관련하여 vecterize 등의 작업
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

	// Ciphertext of past inputs and outputs
	ctY := make([]*rlwe.Ciphertext, n)
	ctU := make([]*rlwe.Ciphertext, n)

	for i := 0; i < n; i++ {
		ptY[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, yy0vec[i])), params.PlaintextModulus()), ptY[i])
		ctY[i], _ = encryptor.EncryptNew(ptY[i])

		ptU[i] = bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, uu0vec[i])), params.PlaintextModulus()), ptU[i])
		ctU[i], _ = encryptor.EncryptNew(ptU[i])

	}

	// 통신 소켓 설정

	// listen, err := net.Listen("tcp", "192.168.0.30:8080") // 연구실 라즈베리파이 ip
	listen, err := net.Listen("tcp", "127.0.0.1:8080") //
	if err != nil {
		fmt.Println("서버 소켓 설정 실패:", err)
		os.Exit(1)
	}

	defer listen.Close()
	fmt.Println("플랜트 서버 실행 중...")

	// 클라이언트와 연결 수락
	conn, err := listen.Accept()
	if err != nil {
		fmt.Println("연결 수락 실패:", err)
		os.Exit(1)
	}
	defer conn.Close()
	fmt.Println("컨트롤러와 연결됨:", conn.RemoteAddr())

	// ============== Simulation ==============
	// Number of simulation steps
	iter := 500
	fmt.Printf("Number of iterations: %v\n", iter)

	// Plant state
	xp := xp0

	for i := 0; i < iter; i++ {
		fmt.Println(i+1, "번째 이터레이션")

		///////
		// 플랜트는 Y를 암호화해서 Ycin 보내고
		// Uout을 받아 복호화하여 U를 받고
		// 재암호화 하여 U cin을 다시 보냄
		//////

		// 시간 측정 시작 (전체 루프)
		startLoop := time.Now()

		// 플랜트 출력 계산
		// 실제 실험 장비에서는 이 부분이 아두이노의 센서 값이 될 예정
		Y := utils.MatVecMult(C, xp) // [][]float64

		// Quantize and duplicate
		// ARX 형태로 패킹하여 암호화
		Ysens := utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(Y, m, h))), params.PlaintextModulus())
		Ypacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(Ysens, Ypacked)
		Ycin, _ := encryptor.EncryptNew(Ypacked)

		// Ycin 송신
		serialized_Ycin, err := Ycin.MarshalBinary() // 이런 식으로

		// fmt.Println("Y 크기.", len(serialized_Ycin)) // >> 131406

		_, err = conn.Write([]byte(serialized_Ycin)) // 리스트 값을 문자열로 전송
		if err != nil {
			fmt.Println("출력값 전송 실패:", err)
			break
		}
		// 송신 시간 계산
		fmt.Println("Ycin 송신 후 시간:", time.Since(startLoop))

		// 여기서 제어 입력 받는 부분
		Uout := rlwe.NewCiphertext(params, params.MaxLevel())
		// 데이터 수신 버퍼 설정 버퍼 설정과 관련해서는 좀 더 논의가 필요

		fmt.Println("U out 수신 직전:", time.Since(startLoop))

		totalData, err := com_utils.ReadFullData(conn, 196966)
		if err != nil {
			fmt.Println("Uout 데이터 수신 실패:", err)
			return
		}

		fmt.Println("U out 수신 후 :", time.Since(startLoop))

		err = Uout.UnmarshalBinary(totalData[:196966])
		if err != nil {
			// 오류 로그 출력
			fmt.Println("Ciphertext 역직렬화 실패:", err)
			return
		}

		// **** Actuator ****

		// ARX 형태를 U로 바꾸는 작업
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

		fmt.Println("재암호화 하기 직전:", time.Since(startLoop))
		// Re-encryption 재 암호화
		Upacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(U, m, h))), params.PlaintextModulus()), Upacked)
		Ucin, _ := encryptor.EncryptNew(Upacked)

		// 직렬화 후 송신
		serialized_reenc_U, err := Ucin.MarshalBinary() // 이런 식으로
		fmt.Println("재입력 U 크기.", len(serialized_reenc_U))

		_, err = conn.Write([]byte(serialized_reenc_U)) // 리스트 값을 문자열로 전송
		if err != nil {
			fmt.Println("출력값 전송 실패:", err)
			break
		}

		// State update
		xp = utils.VecAdd(utils.MatVecMult(A, xp), utils.MatVecMult(B, U))

		fmt.Println("xp 업데이트: ")
		for i := 0; i < len(xp); i++ {
			fmt.Printf("%v\n", xp[i])
		}

		// 루프 끝난 후 수신 과정 한번 해주어야 오류 안생김
		ackBuf := make([]byte, 3)
		_, err = conn.Read(ackBuf)
		if err != nil || string(ackBuf) != "ACK" {
			fmt.Println("ACK 수신 실패:", err)
			return
		}

		// 시간 측정 끝 (전체 루프)
		fmt.Println("전체 루프 시간:", time.Since(startLoop))
	}

}
