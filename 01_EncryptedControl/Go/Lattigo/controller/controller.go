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

// 데이터 전송
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

// 데이터 수신
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
	// 1. RLWE Params
	fmt.Println("--- RLWE params ---")
	loadDir := "enc_data"

	// RWLE 파라미터 (128-bit 조금 부족)
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 12, LogQ: []int{60}, LogP: []int{60}, NTTFlag: true,
	})
	ringQ := params.RingQ()

	// 차원 설정 input:m output:p state:n
	n, m, p := 4, 1, 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// params
	fmt.Println("Degree of polynomials:", params.N())
	fmt.Println("Ciphertext modulus:", params.QBigInt())
	fmt.Println("Special modulus:", params.PBigInt())
	// Default secret key distribution
	// Each coefficient in the polynomial is uniformly sampled in [-1, 0, 1]
	fmt.Println("Secret key distribution (Ternary):", params.Xs())
	// Default error distribution
	// Each coefficient in the polynomial is sampled according to a
	// discrete Gaussian distribution with standard deviation 3.2 and bound 19.2
	fmt.Println("Error distribution (Discrete Gaussian):", params.Xe())

	// Monomials 생성 for unpack
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

	fmt.Println(">> Controller Loop Started ")

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
		yCt := totalUnpacked[:p]                 // y
		uReEncUnpacked := totalUnpacked[p : p+m] // u

		// 4. State update
		FxCt := RGSW.MultPack(xCt, ctF, evaluatorRGSW, ringQ, params)            // Fx
		GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)            // Gy
		RuCt := RGSW.MultPack(uReEncUnpacked, ctR, evaluatorRGSW, ringQ, params) // Ru
		xCtPack = RLWE.Add(FxCt, GyCt, RuCt, params)

		// Time log
		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: Cycle Time %d ms\n", iter, elapsed)
		}
		iter++
	}
}
