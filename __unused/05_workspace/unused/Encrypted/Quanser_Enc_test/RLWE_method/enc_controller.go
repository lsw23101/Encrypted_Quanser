package main

import (
	"Quanser_Enc_test/com_utils"
	"bufio"
	"fmt"
	"log"
	"math"
	"net"
	"path/filepath"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ===== Encryption params & quantization (플랜트와 동일) =====
const (
	logN = 12
	logQ = 56
	logP = 51

	n = 4 // (state dim; 여기선 미사용)
	m = 1 // output dim (u)
	p = 2 // input dim (y=[theta,alpha])

	// 플랜트 쪽과 동일 스케일
	s  = 1.0 / 10.0
	L  = 1.0 / 10000.0
	rQ = 1.0 / 1000.0
)

const addr = "127.0.0.1:9100"

func main() {
	// ===== 1) RLWE parameters & rings =====
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    logN,
		LogQ:    []int{logQ},
		LogP:    []int{logP},
		NTTFlag: true,
	})
	ringQ := params.RingQ()

	// tau = 2^ceil(log2(max(n,m,p)))
	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := 1
	for tau < int(maxDim) {
		tau <<= 1
	}

	// Unpack용 monomials
	logn := int(math.Log2(float64(tau)))
	monomials := make([]ring.Poly, logn)
	for i := 0; i < logn; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}

	// Galois elements (Unpack에 필요)
	galEls := make([]uint64, logn)
	for i := 0; i < logn; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// ===== 2) 키 생성 (동형연산용 평가키) =====
	kgen := rlwe.NewKeyGenerator(params)
	base := filepath.Join("..","Offline_task", "enc_data", "rgsw_for_N12")
	sk := new(rlwe.SecretKey)
	if err := com_utils.ReadRT(filepath.Join(base, "sk.dat"), sk); err != nil {
		log.Fatalf("load sk: %v", err)
	}
	rlk := kgen.GenRelinearizationKeyNew(sk)
	gks := kgen.GenGaloisKeysNew(galEls, sk)

	// Evaluators
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, gks...)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	// RGSW encryptor (행렬 J 암호화를 위해 필요)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)

	// ===== 3) J = [1 1] 를 RGSW로 암호화 (u = y0 + y1) =====
	// 스케일 주의:
	//  - 플랜트는 yBar = round(y / rQ), EncPack(..., 1/L) 로 보냄
	//  - 플랜트 복호화는 DecUnpack(..., rQ*s*s*L) 를 사용
	//  - 여기서 JBar 스케일을 1/s (또는 1/(s*s))로 선택해 최종 복원값을 맞춘다.
	J := [][]float64{{1.0, 1.0}}
	JBar := utils.ScalMatMult(1.0/s, J)     // 기본값: 1/s
	// JBar := utils.ScalMatMult(1.0/(s*s), J) // 필요시 이 라인으로 바꿔보며 튜닝
	ctJ := RGSW.EncPack(JBar, tau, encryptorRGSW, params.QCount()-1, params.PCount()-1, ringQ, params)

	// ===== 4) TCP 서버 (플랜트와 연결) =====
	ln, err := net.Listen("tcp", addr)
	if err != nil {
		log.Fatal(err)
	}
	defer ln.Close()
	fmt.Println("[EncController] Listening on", addr, "...")

	conn, err := ln.Accept()
	if err != nil {
		log.Fatal(err)
	}
	defer conn.Close()

	if tcp, ok := conn.(*net.TCPConn); ok {
		tcp.SetNoDelay(true)
		tcp.SetKeepAlive(true)
		tcp.SetReadBuffer(1 << 20)
		tcp.SetWriteBuffer(1 << 20)
	}
	rbuf := bufio.NewReaderSize(conn, 256<<10)
	wbuf := bufio.NewWriterSize(conn, 256<<10)

	// zeroCt := rlwe.NewCiphertext(params, 1) // Add 시에 사용 (형 맞춤)

	fmt.Println("[EncController] Connected. Waiting for encrypted y...")

	for {
		// 5) 플랜트가 보낸 y 암호문 수신 (EncPack 형태)
		yCtPack := new(rlwe.Ciphertext)
		if _, err := yCtPack.ReadFrom(rbuf); err != nil {
			log.Println("[EncController] Read yCtPack err:", err)
			return
		}

		// 6) Unpack: p=2 개의 RLWE 암호문 벡터로 분해
		yCt := RLWE.UnpackCt(yCtPack, p, tau, evaluatorRLWE, ringQ, monomials, params)

		// 7) u = J * y  (여기서 J=[1 1])  → RGSW MultPack
		uCtPack := RGSW.MultPack(yCt, ctJ, evaluatorRGSW, ringQ, params)
		// (필요 시 다른 항과 합성할 때) uCtPack = RLWE.Add(uCtPack, other, zeroCt, params)

		// 8) 결과 암호문 반환
		if _, err := uCtPack.WriteTo(wbuf); err != nil {
			log.Println("[EncController] Write uCtPack err:", err)
			return
		}
		if err := wbuf.Flush(); err != nil {
			log.Println("[EncController] Flush err:", err)
			return
		}
	}
}
