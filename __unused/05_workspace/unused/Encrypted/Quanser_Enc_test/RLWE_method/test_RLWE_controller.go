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
	// ================= Encryption parameters =================
	logN := 12
	ptSize := uint64(28)
	ctSize := int(74)

	primeGen := ring.NewNTTFriendlyPrimesGenerator(ptSize, uint64(math.Pow(2, float64(logN)+1)))
	ptModulus, _ := primeGen.NextAlternatingPrime()
	fmt.Println("Plaintext modulus:", ptModulus)

	logQ := []int{int(math.Floor(float64(ctSize) * 0.5)), int(math.Ceil(float64(ctSize) * 0.5))}
	params, err := bgv.NewParametersFromLiteral(bgv.ParametersLiteral{
		LogN:             logN,
		LogQ:             logQ,
		PlaintextModulus: ptModulus,
	})
	if err != nil {
		fmt.Println("BGV 파라미터 생성 실패:", err)
		return
	}
	fmt.Println("Poly degree (N):", params.N())

	// 컨트롤러는 evaluator만 가짐
	eval := bgv.NewEvaluator(params, nil)

	// ================= Load encrypted controller & I/O history =================
	n := 4
	recovered_ctHu := make([]*rlwe.Ciphertext, n)
	recovered_ctHy := make([]*rlwe.Ciphertext, n)
	recovered_ctU := make([]*rlwe.Ciphertext, n)
	recovered_ctY := make([]*rlwe.Ciphertext, n)

	for i := 0; i < n; i++ {
		recovered_ctHu[i] = rlwe.NewCiphertext(params, 1)
		recovered_ctHy[i] = rlwe.NewCiphertext(params, 1)
		recovered_ctU[i]  = rlwe.NewCiphertext(params, 1)
		recovered_ctY[i]  = rlwe.NewCiphertext(params, 1)

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

	// ================= Listen (SERVER) =================
	addr := "127.0.0.1:8080" // 필요 시 변경
	ln, err := net.Listen("tcp", addr)
	if err != nil {
		fmt.Println("서버 소켓 생성 실패:", err)
		return
	}
	defer ln.Close()
	fmt.Println("컨트롤러 서버 대기 중 @", addr)

	conn, err := ln.Accept()
	if err != nil {
		fmt.Println("연결 수락 실패:", err)
		return
	}
	defer conn.Close()
	fmt.Println("플랜트와 연결됨:", conn.RemoteAddr())

	// ================= Protocol sizes (상대와 합의된 값) =================
	const (
		bytesYcin  = 131406 // Ycin 크기
		bytesUout  = 196966 // Uout 크기 (곱셈 깊이로 커짐)
		bytesUcin  = 131406 // Ucin 크기
	)

	// ================= Simulation loop =================
	iter := 500
	fmt.Printf("Number of iterations: %d\n", iter)

	for i := 0; i < iter; i++ {
		fmt.Println(i+1, "번째 이터레이션")
		loopStart := time.Now()

		// ---- 1) Ycin 수신 ----
		Ycin := rlwe.NewCiphertext(params, params.MaxLevel())
		totalDataY, err := com_utils.ReadFullData(conn, bytesYcin)
		if err != nil {
			fmt.Println("Ycin 데이터 수신 실패:", err)
			return
		}
		if err := Ycin.UnmarshalBinary(totalDataY[:bytesYcin]); err != nil {
			fmt.Println("Ycin 역직렬화 실패:", err)
			return
		}
		fmt.Println("Ycin 수신 완료:", time.Since(loopStart))

		// ---- 2) Uout = sum(Hy_j*Y_j) + sum(Hu_j*U_j) ----
		tComp := time.Now()
		Uout, _ := eval.MulNew(recovered_ctHy[0], recovered_ctY[0])
		eval.MulThenAdd(recovered_ctHu[0], recovered_ctU[0], Uout)
		for j := 1; j < n; j++ {
			eval.MulThenAdd(recovered_ctHy[j], recovered_ctY[j], Uout)
			eval.MulThenAdd(recovered_ctHu[j], recovered_ctU[j], Uout)
		}
		fmt.Println("제어기 암호 연산 시간:", time.Since(tComp))

		// ---- 3) Uout 송신 ----
		serUout, err := Uout.MarshalBinary()
		if err != nil {
			fmt.Println("Uout 직렬화 실패:", err)
			return
		}
		fmt.Println("Uout 직렬화 길이:", len(serUout)) // 기대: bytesUout
		if _, err = conn.Write(serUout); err != nil {
			fmt.Println("Uout 송신 실패:", err)
			return
		}
		fmt.Println("Uout 송신 완료:", time.Since(tComp))

		// ---- 4) Ucin 수신 ----
		Ucin := rlwe.NewCiphertext(params, params.MaxLevel())
		tReenc := time.Now()
		totalDataUcin, err := com_utils.ReadFullData(conn, bytesUcin)
		if err != nil {
			fmt.Println("Ucin 데이터 수신 실패:", err)
			return
		}
		if err := Ucin.UnmarshalBinary(totalDataUcin[:bytesUcin]); err != nil {
			fmt.Println("Ucin 역직렬화 실패:", err)
			return
		}
		fmt.Println("Ucin 수신 완료:", time.Since(tReenc))

		// ---- 5) 상태 시퀀스 업데이트 ----
		recovered_ctY = append(recovered_ctY[1:], Ycin)
		recovered_ctU = append(recovered_ctU[1:], Ucin)

		// ---- 6) ACK 송신 ----
		if _, err := conn.Write([]byte("ACK")); err != nil {
			fmt.Println("ACK 송신 실패:", err)
			return
		}

		fmt.Println("한 루프 총 시간:", time.Since(loopStart))
	}

	fmt.Println("시뮬레이션 종료")
}
