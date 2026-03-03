package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"path/filepath"
	"time"

	// 1. 파일 유틸
	fileutils "github.com/CDSL-EncryptedControl/CDSL/python_test/utils"

	// 2. 라이브러리
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// --- 네트워크 통신 헬퍼 함수 (길이 + 데이터) ---

// 데이터 전송: [Length(4bytes)][Payload...]
func writeCiphertext(conn net.Conn, ct *rlwe.Ciphertext) error {
	data, err := ct.MarshalBinary()
	if err != nil {
		return err
	}
	length := uint32(len(data))

	// 1. 길이 전송
	lenBuf := make([]byte, 4)
	binary.LittleEndian.PutUint32(lenBuf, length)
	if _, err := conn.Write(lenBuf); err != nil {
		return err
	}

	// 2. 데이터 전송
	if _, err := conn.Write(data); err != nil {
		return err
	}
	return nil
}

// 데이터 수신: [Length(4bytes)] 읽고 -> [Payload] 읽기
func readCiphertext(conn net.Conn) (*rlwe.Ciphertext, error) {
	// 1. 길이 읽기
	lenBuf := make([]byte, 4)
	if _, err := io.ReadFull(conn, lenBuf); err != nil {
		return nil, err
	}
	length := binary.LittleEndian.Uint32(lenBuf)

	// 2. 데이터 읽기
	data := make([]byte, length)
	if _, err := io.ReadFull(conn, data); err != nil {
		return nil, err
	}

	// 3. 언마샬링
	ct := new(rlwe.Ciphertext)
	if err := ct.UnmarshalBinary(data); err != nil {
		return nil, err
	}
	return ct, nil
}

func main() {
	// *****************************************************************
	// 1. 설정 및 데이터 불러오기
	// *****************************************************************
	fmt.Println("--- [Go Controller] Loading FHE Context... ---")
	loadDir := "enc_data"

	// 파라미터 로드
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 12, LogQ: []int{56}, LogP: []int{51}, NTTFlag: true,
	})
	ringQ := params.RingQ()

	// 차원 설정
	// 이거 차원 맞춰주기 시스템 바뀌면 입력 m 출력 p
	n, m, p := 4, 1, 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// Monomials 생성
	logn := int(math.Log2(float64(tau)))
	monomials := make([]ring.Poly, logn)
	for i := 0; i < logn; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}

	// 키 로드
	rlk := new(rlwe.RelinearizationKey)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "rlk.dat"), rlk); err != nil {
		panic(err)
	}

	gks, err := fileutils.LoadGaloisKeys(loadDir)
	if err != nil {
		panic(err)
	}

	// 암호화된 행렬 로드
	fmt.Println(">> Loading Matrices...")
	ctF, _ := fileutils.LoadRGSWPack(loadDir, "ctF")
	ctG, _ := fileutils.LoadRGSWPack(loadDir, "ctG")
	ctH, _ := fileutils.LoadRGSWPack(loadDir, "ctH")
	ctR, _ := fileutils.LoadRGSWPack(loadDir, "ctR")

	// 초기 상태 로드
	xCtPackTemplate := new(rlwe.Ciphertext)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "xCtPack.dat"), xCtPackTemplate); err != nil {
		panic(err)
	}

	// Evaluator 구성
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, gks...)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	fmt.Println("--- Ready for Fully Encrypted TCP ---")

	// *****************************************************************
	// 2. TCP 서버 시작
	// *****************************************************************
	HOST, PORT := "localhost", "8000"
	listener, err := net.Listen("tcp", HOST+":"+PORT)
	if err != nil {
		panic(err)
	}
	defer listener.Close()

	fmt.Printf("Waiting for Plant at %s:%s...\n", HOST, PORT)

	for {
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("Error accepting:", err)
			continue
		}
		fmt.Println(">> Plant Connected!")

		// 초기 상태 복사
		xCtPack := xCtPackTemplate.CopyNew()

		// 핸들러 실행
		go handleEncryptedControl(conn, xCtPack, ctF, ctG, ctH, ctR,
			evaluatorRGSW, evaluatorRLWE, ringQ, monomials, params, n, m, p, tau)
	}
}

func handleEncryptedControl(
	conn net.Conn,
	xCtPack *rlwe.Ciphertext, // Encrypted State
	ctF, ctG, ctH, ctR []*rgsw.Ciphertext,
	evaluatorRGSW *rgsw.Evaluator,
	evaluatorRLWE *rlwe.Evaluator,
	ringQ *ring.Ring,
	monomials []ring.Poly,
	params rlwe.Parameters,
	n, m, p, tau int,
) {
	defer conn.Close()
	iter := 0

	for {
		start := time.Now()

		// ---------------------------------------------------------
		// Step 1. Receive Encrypted y from Plant
		// ---------------------------------------------------------
		yCtPack, err := readCiphertext(conn)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read y Error:", err)
			}
			return
		}

		// ---------------------------------------------------------
		// Step 2. Compute Encrypted u = H * x
		// ---------------------------------------------------------
		// Unpack State & Input
		xCt := RLWE.UnpackCt(xCtPack, n, tau, evaluatorRLWE, ringQ, monomials, params)

		// Unpack y (for State Update later)
		yCt := RLWE.UnpackCt(yCtPack, p, tau, evaluatorRLWE, ringQ, monomials, params)

		// u = Hx
		uCtPack := RGSW.MultPack(xCt, ctH, evaluatorRGSW, ringQ, params)

		// ---------------------------------------------------------
		// Step 3. Send Encrypted u to Plant
		// ---------------------------------------------------------
		if err := writeCiphertext(conn, uCtPack); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// ---------------------------------------------------------
		// Step 4. Receive Re-Encrypted u from Plant
		// ---------------------------------------------------------
		uReEnc, err := readCiphertext(conn)
		if err != nil {
			fmt.Println("Read ReEnc u Error:", err)
			return
		}

		// ---------------------------------------------------------
		// Step 5. Update State: x_next = Fx + Gy + Ru
		// ---------------------------------------------------------
		// Fx
		FxCt := RGSW.MultPack(xCt, ctF, evaluatorRGSW, ringQ, params)
		// Gy
		GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)

		// Ru 계산을 위해 uReEnc도 Unpack 수행! (여기가 수정됨)
		// u의 차원은 m 입니다.
		uCt := RLWE.UnpackCt(uReEnc, m, tau, evaluatorRLWE, ringQ, monomials, params)

		// Ru (Unpacked u 사용)
		RuCt := RGSW.MultPack(uCt, ctR, evaluatorRGSW, ringQ, params)

		// x_next 합산
		xCtPack = RLWE.Add(FxCt, GyCt, RuCt, params)

		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: Cycle Time %d ms (Fully Encrypted)\n", iter, elapsed)
		}
		iter++
	}
}
