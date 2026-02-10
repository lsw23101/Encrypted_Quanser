package main

import (
	"Quanser_Enc_test/com_utils"
	"fmt"
	"math"
	"net"
	"time"

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

	// 컨트롤러는 evaluator만 가짐

	eval := bgv.NewEvaluator(params, nil)

	// ==============  Encryption of controller ==============
	// dimensions
	n := 4

	// ctHy랑 ctHu, 입출력 초기값 파일 읽기

	// 암호문 읽고 복원하기 + u랑 y 시퀀스 초기값도 읽어와야됨됨
	recovered_ctHu := make([]*rlwe.Ciphertext, 4) // 크기 4로 초기화
	recovered_ctHy := make([]*rlwe.Ciphertext, 4) // 크기 4로 초기화
	// Ciphertext of past inputs and outputs
	recovered_ctY := make([]*rlwe.Ciphertext, n)
	recovered_ctU := make([]*rlwe.Ciphertext, n)

	// 각 인덱스에 새로운 Ciphertext 객체 생성
	// nil로 두지 않기 위해서 1 인자 넣고 초기화

	for i := 0; i < 4; i++ {
		recovered_ctHu[i] = rlwe.NewCiphertext(params, 1)
		recovered_ctHy[i] = rlwe.NewCiphertext(params, 1)
		recovered_ctU[i] = rlwe.NewCiphertext(params, 1)
		recovered_ctY[i] = rlwe.NewCiphertext(params, 1)
	}

	for i := 0; i < 4; i++ {
		filename_Hu := fmt.Sprintf("ctHu[%d].dat", i)
		if err := com_utils.ReadFromFile(filename_Hu, recovered_ctHu[i]); err != nil {
			fmt.Println("파일 읽기 실패:", filename_Hu, err)
			return
		}

		filename_Hy := fmt.Sprintf("ctHy[%d].dat", i)
		if err := com_utils.ReadFromFile(filename_Hy, recovered_ctHy[i]); err != nil {
			fmt.Println("파일 읽기 실패:", filename_Hy, err)
			return
		}

		filename_u := fmt.Sprintf("ctU[%d].dat", i)
		if err := com_utils.ReadFromFile(filename_u, recovered_ctU[i]); err != nil {
			fmt.Println("파일 읽기 실패:", filename_u, err)
			return
		}

		filename_y := fmt.Sprintf("ctY[%d].dat", i)
		if err := com_utils.ReadFromFile(filename_y, recovered_ctY[i]); err != nil {
			fmt.Println("파일 읽기 실패:", filename_y, err)
			return
		}
	}

	// 이건 읽으면 안됨
	// com_utils.ReadFromFile("sk.dat", recovered_sk)

	// 컨트롤러 소켓 연결설정

	conn, err := net.Dial("tcp", "127.0.0.1:8080") // 서버에서 설정한 ip
	// conn, err := net.Dial("tcp", "192.168.0.30:8080") // 연구실 라즈베리파이 ip
	if err != nil {
		fmt.Println("서버에 연결 실패:", err)
		return
	}
	defer conn.Close()
	fmt.Println("컨트롤러와 연결됨:", conn.RemoteAddr())

	// ============== Simulation ==============
	// Number of simulation steps
	iter := 500
	fmt.Printf("Number of iterations: %v\n", iter)

	for i := 0; i < iter; i++ {
		fmt.Println(i+1, "번째 이터레이션")

		///////
		// 컨트롤러는 Ycin을 받고 Uout을 보내주며 Ucin 재암호화 값을 받는다 ////
		//////

		// 이터레이션 시간 측정
		loop_start := time.Now()

		// 플랜트 출력을 담을 cipher message
		Ycin := rlwe.NewCiphertext(params, params.MaxLevel())

		// 데이터 수신을 위한 누적된 결과 저장
		var totalData []byte

		// fmt.Println("첫번째 통신 시작 지점 ")

		// 여기서 131406은 파라미터로 설정한 q값에 따른 ct의 바이너리 크기
		totalData, err := com_utils.ReadFullData(conn, 131406)
		if err != nil {
			fmt.Println("Ycin 데이터 수신 실패:", err)
			return
		}

		// 마샬링 시간 계산
		fmt.Println("첫 통신 Ycin 받는데 걸리는 시간:", time.Since(loop_start))

		err = Ycin.UnmarshalBinary(totalData[:131406])
		if err != nil {
			// 오류 로그 출력
			fmt.Println("Ciphertext 역직렬화 실패:", err)
			return
		}

		// fmt.Println("Ycin 직렬화 걸리는 시간:", time.Since(loop_start))

		// 여기서 컨트롤러 입력 U을 계산

		compute_u := time.Now()

		Uout, _ := eval.MulNew(recovered_ctHy[0], recovered_ctY[0])

		eval.MulThenAdd(recovered_ctHu[0], recovered_ctU[0], Uout)
		for j := 1; j < n; j++ {
			eval.MulThenAdd(recovered_ctHy[j], recovered_ctY[j], Uout)
			eval.MulThenAdd(recovered_ctHu[j], recovered_ctU[j], Uout)
		}

		fmt.Println("제어기 암호 연산 시간:", time.Since(compute_u))

		serialized_Uout, err := Uout.MarshalBinary() // 이런 식으로

		// 컨트롤러의 제어 입력 계산 값은 바이너리 사이즈가 131406보다 커짐 196966
		fmt.Println("Uout 크기.", len(serialized_Uout))

		_, err = conn.Write([]byte(serialized_Uout))
		if err != nil {
			fmt.Println("출력값 전송 실패:", err)
			break
		}

		fmt.Println("Uout 연산 후 전송 시간", time.Since(compute_u))

		// 재암호화 값 받기

		Ucin := rlwe.NewCiphertext(params, params.MaxLevel())

		get_reenc := time.Now()

		fmt.Println("재암호화 받기 전:", time.Since(loop_start))
		totalData_reenc, err := com_utils.ReadFullData(conn, 131406)
		if err != nil {
			fmt.Println("Ucin 데이터 수신 실패:", err)
			return
		}

		fmt.Println("재암호화 받는데 걸리는 시간 : ", time.Since(get_reenc))

		err = Ucin.UnmarshalBinary(totalData_reenc[:131406])
		if err != nil {
			// 오류 로그 출력
			fmt.Println("Ciphertext 역직렬화 실패:", err)
			return
		}

		// State update
		recovered_ctY = append(recovered_ctY[1:], Ycin)
		recovered_ctU = append(recovered_ctU[1:], Ucin)

		// ACK 송신 과정이 없으면 수신 후 바로 수신하는 부분에서 에러 발생

		conn.Write([]byte("ACK"))
		fmt.Println("한 루프 걸리는 시간:", time.Since(loop_start))
	}

}
