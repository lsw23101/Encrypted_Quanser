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
	fileutils "github.com/CDSL-EncryptedControl/CDSL/Lattigo/utils"

	// 2. 라이브러리
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// Lattigo 데이터 송수신 함수들...

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
	// 1. 오프라인 암호문 읽기
	fmt.Println("--- [Go Controller] Loading FHE Context... ---")
	loadDir := "enc_data"

	// RWLE 파라미터 (128-bit 조금 부족)
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 12, LogQ: []int{60}, LogP: []int{60}, NTTFlag: true,
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

	// 키 로드 (비밀키는 로드 x)
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

	// 제어기 상태 초기값 로드
	xCtPackTemplate := new(rlwe.Ciphertext)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "xCtPack.dat"), xCtPackTemplate); err != nil {
		panic(err)
	}

	// Evaluator 구성
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, gks...)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	fmt.Println("--- Ready ---")

	// 2. TCP 서버 시작
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
	xCtPack *rlwe.Ciphertext,
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

	fmt.Println(">> Controller Loop Started (Optimized Flow - No Rotate)")

	for {
		start := time.Now()
		// 1. Compute Encrypted u = H * x & Send
		xCt := RLWE.UnpackCt(xCtPack, n, tau, evaluatorRLWE, ringQ, monomials, params)
		uCtPack := RGSW.MultPack(xCt, ctH, evaluatorRGSW, ringQ, params)
		// Send
		if err := writeCiphertext(conn, uCtPack); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// 2. Receive Packed Enc(y), ReEnc(u) (i.e. [y | u] )
		combinedCt, err := readCiphertext(conn)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read Combined CT Error:", err)
			}
			return
		}

		// 3. Unpack
		// totalUnpacked = [y_0, y_1, u_0]
		totalUnpacked := RLWE.UnpackCt(combinedCt, p+m, tau, evaluatorRLWE, ringQ, monomials, params)
		yCt := totalUnpacked[:p]                 // 앞쪽 p개 (0 ~ 1)
		uReEncUnpacked := totalUnpacked[p : p+m] // 뒤쪽 m개 (2 ~ 2)

		// 4. State update
		FxCt := RGSW.MultPack(xCt, ctF, evaluatorRGSW, ringQ, params)            // Fx
		GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)            // Gy
		RuCt := RGSW.MultPack(uReEncUnpacked, ctR, evaluatorRGSW, ringQ, params) // Ru
		xCtPack = RLWE.Add(FxCt, GyCt, RuCt, params)

		// Time log
		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: Cycle Time %d ms (1-Round Trip)\n", iter, elapsed)
		}
		iter++
	}
}
